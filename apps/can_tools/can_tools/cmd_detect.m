/**
 * @file cmd_detect.m
 * @brief detect: bus type detection (receive-only unless --probe)
 *
 * Moved from can_reader/main.m. The adapter setup is the PASSIVE init profile;
 * ATRV and the --probe commands (ATCSM0, ATSP0, 0100) stay here.
 *
 * cli_detect_pcan: the passive listen from a PCAN-USB (listen-only); no
 * ATRV, and --probe is refused before this is reached (main.m).
 */

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <IOBluetooth/IOBluetooth.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <limits.h>
#include <ctype.h>

#include "elm327.h"
#include "stn2255.h"
#include "cli_util.h"
#include "frame_source.h"
#include "frame_source_pcan.h"
#include "cli_pcan.h"
#import "obd_adapter.h"
#import "cli_common.h"
#include "commands.h"

/**
 * CLI: detect <device>
 * Auto-detect D-CAN (OBD port) vs PT-CAN (direct bus tap).
 *
 * 3-state detection:
 *   1. Silent mode passive listen 3 seconds
 *   2. If frames received → "ptcan" (confirmed)
 *   3. If no frames → probe query (0100) to verify D-CAN
 *   4. If probe response → "dcan" (confirmed)
 *   5. If no probe response → "unknown" (IGN OFF or not connected)
 */
int cli_detect(const char *device, bool allow_probe) {
    /* Receive-only init: CAN 500k, silent mode (no ACK), raw frames, headers */
    int matchIdx = cli_ptcan_connect(device, NULL, 0);
    char resp[512];

    /* Battery voltage via ATRV */
    double voltage = 0.0;
    cli_log("Checking battery voltage (ATRV)...");
    if ([g_ble sendCommand:"ATRV" response:resp maxLen:sizeof(resp)]) {
        elm327_parse_voltage(resp, &voltage);
        cli_log("Battery voltage: %.1fV", voltage);
    }

    /* === Phase 1: Passive listen (3 seconds) === */
    cli_log("Phase 1: Passive listening for 3 seconds...");

    frame_buffer_reset(&g_cli_capture);
    [g_ble startMonitorWithCallback:monitor_frame_callback context:NULL];

    const char *monitor_cmd = g_stn_detected ? "STMA" : "ATMA";
    char monitor_str[16];
    snprintf(monitor_str, sizeof(monitor_str), "%s\r", monitor_cmd);
    [g_ble sendRawString:monitor_str];

    NSDate *endTime = [NSDate dateWithTimeIntervalSinceNow:3.0];
    while ([[NSDate date] compare:endTime] == NSOrderedAscending &&
           g_ble.isConnected && !g_interrupt) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }

    /* Stop monitoring */
    [g_ble stopMonitor];
    [g_ble sendRawString:"\r"];
    run_loop_for(0.2);

    int frames = g_cli_capture.count;
    const char *mode;
    bool probe_responded = false;

    if (frames > 0) {
        cli_log("Phase 1 result: CAN frames detected (%d)", frames);
    } else {
        cli_log("Phase 1 result: No CAN frames");
    }

    /* === Phase 2: Probe query (only with --probe) ===
     * The probe transmits (ACK on, ATSP0 auto search, 0100). On a direct tap of a
     * vehicle bus such as PT-CAN nothing may be sent, and detect cannot tell a
     * tap from an OBD port, so it stays receive-only unless asked. */
    bool probe_sent = false;
    if (!allow_probe) {
        cli_log("Phase 2 skipped: receive-only (pass --probe on an OBD port to send 0100)");
    } else {
        cli_log("Phase 2: Probing with 0100 (transmits)...");
        probe_sent = true;

        /* Switch to active mode for probe */
        [g_ble sendCommand:"ATCSM0" response:resp maxLen:sizeof(resp)];
        [g_ble sendCommand:"ATSP0" response:resp maxLen:sizeof(resp)];

        char probe_resp[256];
        if ([g_ble sendCommand:"0100" response:probe_resp maxLen:sizeof(probe_resp)] &&
            !elm327_is_error_response(probe_resp)) {
            probe_responded = true;
            cli_log("Phase 2 result: ECU responded");
        } else {
            cli_log("Phase 2 result: No ECU response");
        }
    }

    /*  Phase 1 (frames)  |  Phase 2 (probe)  |  Result
     *  ------------------+-------------------+------------------
     *  YES               |  YES              |  dcan (broadcast)
     *  YES               |  NO               |  ptcan
     *  NO                |  YES              |  dcan
     *  NO                |  NO               |  unknown
     */
    if (probe_responded) {
        mode = "dcan";
        cli_log("Result: D-CAN (OBD-II port%s)", frames > 0 ? ", with broadcast" : "");
    } else if (frames > 0) {
        mode = probe_sent ? "ptcan" : "broadcast";
        cli_log(probe_sent ? "Result: PT-CAN (direct bus tap)"
                           : "Result: broadcast traffic (direct tap or OBD port with broadcast)");
    } else {
        mode = "unknown";
        cli_log(probe_sent ? "Result: undetermined (IGN OFF?)"
                           : "Result: no traffic (IGN OFF, or an OBD port behind a gateway: try --probe)");
    }

    /* Count unique CAN IDs */
    int unique_ids = 0;
    uint32_t seen_ids[512];
    for (int i = 0; i < frames && unique_ids < 512; i++) {
        bool found = false;
        for (int j = 0; j < unique_ids; j++) {
            if (seen_ids[j] == g_cli_capture.frames[i].can_id) {
                found = true;
                break;
            }
        }
        if (!found) {
            seen_ids[unique_ids++] = g_cli_capture.frames[i].can_id;
        }
    }

    /* JSON output */
    printf("{\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"mode\":");
    json_print_string(stdout, mode);
    printf(",\"voltage\":%.1f", voltage);
    printf(",\"broadcast\":%s", (frames > 0) ? "true" : "false");
    printf(",\"frames_received\":%d", frames);
    printf(",\"probe_sent\":%s", probe_sent ? "true" : "false");
    printf(",\"probe_responded\":%s", probe_responded ? "true" : "false");
    printf(",\"unique_can_ids\":%d", unique_ids);
    if (unique_ids > 0) {
        printf(",\"can_ids\":[");
        for (int i = 0; i < unique_ids; i++) {
            if (i > 0) printf(",");
            printf("\"%03X\"", seen_ids[i]);
        }
        printf("]");
    }
    printf("}\n");

    [g_ble disconnect];
    g_interrupt = NO;
    return 0;
}


/*******************************************************************************
 * detect pcan: 3 s listen-only (nothing is ever sent)
 ******************************************************************************/

static bool detect_cancelled(void) {
    return !g_running || g_interrupt;
}

int cli_detect_pcan(const char *device, const pcan_bitrate_t *bitrate) {
    pcan_channel_t ch;
    bool autodetected = false;
    if (!cli_pcan_open(device, bitrate, detect_cancelled, &ch, &autodetected)) return 1;
    char label[64];
    snprintf(label, sizeof(label), "%s", ch.label);
    int rate = ch.bitrate;
    frame_source_t *src = frame_source_pcan_create(&ch);
    if (!src) {
        pcan_close(&ch);
        fprintf(stderr, "[ERROR] Out of memory\n");
        return 1;
    }

    cli_log("Phase 1: Passive listening for 3 seconds (PCAN listen-only %s)...", pcan_bitrate_name(rate));
    frame_buffer_reset(&g_cli_capture);
    g_cli_pcan_source = src;
    frame_source_start(src, monitor_frame_callback, NULL);
    int64_t end_us = pcan_mono_us() + 3000000;
    while (frame_source_is_open(src) && !g_interrupt && g_running) {
        int64_t left = end_us - pcan_mono_us();
        if (left <= 0) break;
        frame_source_pump(src, left < 50000 ? (double)left / 1e6 : 0.05);
    }
    frame_source_stop(src);
    g_cli_pcan_source = NULL;
    bool lost = !frame_source_is_open(src);

    int frames = g_cli_capture.count;
    if (frames > 0) cli_log("Phase 1 result: CAN frames detected (%d)", frames);
    else cli_log("Phase 1 result: No CAN frames");
    if (lost) fprintf(stderr, "[ERROR] PCAN %s stopped answering during the listen\n", label);
    cli_log("Phase 2 skipped: PCAN is listen-only (no 0100 probe)");

    const char *mode = frames > 0 ? "broadcast" : "unknown";
    cli_log(frames > 0 ? "Result: broadcast traffic (direct tap or OBD port with broadcast)"
                       : "Result: no traffic (IGN OFF, wrong bitrate, or an OBD port behind a gateway)");

    int unique_ids = 0;
    uint32_t seen_ids[512];
    for (int i = 0; i < frames && unique_ids < 512; i++) {
        bool found = false;
        for (int j = 0; j < unique_ids && !found; j++) found = seen_ids[j] == g_cli_capture.frames[i].can_id;
        if (!found) seen_ids[unique_ids++] = g_cli_capture.frames[i].can_id;
    }

    printf("{\"device\":");
    json_print_string(stdout, label);
    printf(",\"interface\":\"pcan\",\"library_version\":");
    json_print_string(stdout, pcan_api_version());
    printf(",\"bitrate\":%d,\"bitrate_auto\":%s", rate, autodetected ? "true" : "false");
    printf(",\"mode\":");
    json_print_string(stdout, mode);
    printf(",\"voltage\":null");
    printf(",\"broadcast\":%s", (frames > 0) ? "true" : "false");
    printf(",\"frames_received\":%d", frames);
    printf(",\"probe_sent\":false");
    printf(",\"probe_responded\":false");
    if (lost) printf(",\"device_lost\":true");
    printf(",\"unique_can_ids\":%d", unique_ids);
    if (unique_ids > 0) {
        printf(",\"can_ids\":[");
        for (int i = 0; i < unique_ids; i++) {
            if (i > 0) printf(",");
            printf("\"%03X\"", seen_ids[i]);
        }
        printf("]");
    }
    printf("}\n");

    frame_source_free(src);   /* CAN_Uninitialize */
    g_interrupt = NO;
    return lost ? 1 : 0;
}
