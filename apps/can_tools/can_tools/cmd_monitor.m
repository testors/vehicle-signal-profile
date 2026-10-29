/**
 * @file cmd_monitor.m
 * @brief monitor: passive CAN monitor (receive-only)
 *
 * Moved unchanged from can_reader/main.m. PASSIVE init profile with optional
 * hardware filters (cli_ptcan_connect).
 *
 * cli_monitor_pcan: the same capture from a PCAN-USB opened listen-only;
 * the whole bus is received and the optional IDs filter in software.
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
 * Emit an operator cue after Bluetooth/CAN setup is complete and immediately
 * before the monitor command is sent. This keeps live test timing aligned with
 * the actual capture window.
 */
static void cli_monitor_cue_before_start(void) {
    const char *label = getenv("CAN_TOOLS_MONITOR_LABEL");
    if (!label || !*label) return;

    int countdown = 0;
    const char *countdown_env = getenv("CAN_TOOLS_MONITOR_COUNTDOWN");
    if (countdown_env && *countdown_env) {
        countdown = atoi(countdown_env);
        if (countdown < 0) countdown = 0;
        if (countdown > 30) countdown = 30;
    }

    fprintf(stderr, "[CUE] %s: ready\n", label);
    fflush(stderr);
    for (int n = countdown; n > 0; n--) {
        fprintf(stderr, "[CUE] %s: start in %d\n", label, n);
        fflush(stderr);
        run_loop_for(1.0);
    }

    fprintf(stderr, "\a[CUE] %s: START NOW\n", label);
    fflush(stderr);
}

static void cli_monitor_cue_after_stop(void) {
    const char *label = getenv("CAN_TOOLS_MONITOR_LABEL");
    if (!label || !*label) return;
    fprintf(stderr, "\a[CUE] %s: STOP\n", label);
    fflush(stderr);
}

/**
 * CLI: monitor <device> [seconds] [CAN_IDs...]
 * Passive CAN monitor (silent mode — no ACK on bus).
 * Optional CAN ID filters via STFAP.
 */
int cli_monitor(const char *device, int seconds,
                const char *filter_argv[], int filter_argc) {
    /* Parse filter CAN IDs */
    uint32_t filter_ids[16];
    int filter_count = 0;
    for (int i = 0; i < filter_argc && filter_count < 16; i++) {
        unsigned int val;
        if (sscanf(filter_argv[i], "%x", &val) == 1 && val <= 0x7FF) {
            filter_ids[filter_count++] = val;
        } else {
            fprintf(stderr, "[ERROR] Invalid CAN ID: %s (must be 000-7FF hex)\n", filter_argv[i]);
            return 1;
        }
    }

    int matchIdx = cli_ptcan_connect(device, filter_ids, filter_count);

    const char *monitor_cmd = g_stn_detected ? "STMA" : "ATMA";
    cli_log("Starting CAN monitor (%s, silent) for %d seconds...", monitor_cmd, seconds);
    if (filter_count > 0) {
        fprintf(stderr, "[INFO] Filters: ");
        for (int i = 0; i < filter_count; i++) {
            fprintf(stderr, "%s%03X", i > 0 ? " " : "", filter_ids[i]);
        }
        fprintf(stderr, "\n");
    }

    cli_monitor_cue_before_start();

    frame_buffer_reset(&g_cli_capture);
    /* Monitor parsing on (also clears response state) */
    [g_ble startMonitorWithCallback:monitor_frame_callback context:NULL];

    /* Send monitor command */
    char monitor_str[16];
    snprintf(monitor_str, sizeof(monitor_str), "%s\r", monitor_cmd);
    [g_ble sendRawString:monitor_str];

    /* Collect for specified duration */
    NSDate *endTime = [NSDate dateWithTimeIntervalSinceNow:seconds];

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

    cli_log("CAN monitor stopped. Frames: %d", g_cli_capture.count);
    if (g_cli_capture.dropped > 0) {
        fprintf(stderr, "[WARN] Capture buffer full: %d frames dropped\n",
                g_cli_capture.dropped);
    }
    cli_monitor_cue_after_stop();

    /* JSON output */
    FILE *json_out = stdout;
    const char *monitor_out_path = g_cli_monitor_out_path;
    if (!monitor_out_path || !*monitor_out_path) {
        monitor_out_path = getenv("CAN_TOOLS_MONITOR_OUT");
    }
    if (monitor_out_path && *monitor_out_path) {
        json_out = fopen(monitor_out_path, "w");
        if (!json_out) {
            fprintf(stderr, "[ERROR] Failed to open monitor output: %s\n", monitor_out_path);
            [g_ble disconnect];
            g_interrupt = NO;
            return 1;
        }
    }

    fprintf(json_out, "{\"device\":");
    json_print_string(json_out, g_ble.discoveredNames[matchIdx].UTF8String);
    fprintf(json_out, ",\"mode\":\"silent\",\"duration_s\":%d", seconds);
    if (filter_count > 0) {
        fprintf(json_out, ",\"filters\":[");
        for (int i = 0; i < filter_count; i++) {
            if (i > 0) fprintf(json_out, ",");
            fprintf(json_out, "\"%03X\"", filter_ids[i]);
        }
        fprintf(json_out, "]");
    }
    fprintf(json_out, ",\"frames\":[\n");

    for (int i = 0; i < g_cli_capture.count; i++) {
        if (i > 0) fprintf(json_out, ",\n");
        fprintf(json_out, "  {\"timestamp_ms\":%lld,\"id\":\"%03X\",\"is_extended\":%s,\"dlc\":%d,\"data\":\"",
                (long long)g_cli_capture.frames[i].timestamp_ms,
                g_cli_capture.frames[i].can_id,
                g_cli_capture.frames[i].is_extended ? "true" : "false",
                g_cli_capture.frames[i].dlc);
        for (int j = 0; j < g_cli_capture.frames[i].dlc; j++) {
            if (j > 0) fprintf(json_out, " ");
            fprintf(json_out, "%02X", g_cli_capture.frames[i].data[j]);
        }
        fprintf(json_out, "\"}");
    }

    if (g_cli_capture.count > 0) fprintf(json_out, "\n");
    fprintf(json_out, "]}\n");

    if (json_out != stdout) {
        fclose(json_out);
        cli_log("Wrote monitor JSON: %s", monitor_out_path);
    }

    [g_ble disconnect];
    g_interrupt = NO;
    return 0;
}


/*******************************************************************************
 * monitor pcan: listen-only capture from a PCAN-USB
 ******************************************************************************/

typedef struct {
    const uint32_t *ids;
    int             count;
} pcan_monitor_filter_t;

/* Software ID filter in front of the capture buffer (the bus is received whole). */
static void pcan_monitor_sink(const can_frame_t *frame, void *ctx) {
    const pcan_monitor_filter_t *flt = (const pcan_monitor_filter_t *)ctx;
    if (flt->count > 0) {
        bool keep = false;
        for (int i = 0; i < flt->count && !keep; i++) keep = flt->ids[i] == frame->can_id;
        if (!keep) return;
    }
    monitor_frame_callback(frame, NULL);
}

static bool cli_cancelled(void) {
    return !g_running || g_interrupt;
}

int cli_monitor_pcan(const char *device, int seconds, const char *filter_argv[], int filter_argc,
                     const pcan_bitrate_t *bitrate) {
    uint32_t filter_ids[64];
    int filter_count = 0;
    for (int i = 0; i < filter_argc && filter_count < 64; i++) {
        char *end = NULL;
        unsigned long val = strtoul(filter_argv[i], &end, 16);
        if (!end || *end != '\0' || end == filter_argv[i] || val > 0x1FFFFFFFUL) {
            fprintf(stderr, "[ERROR] Invalid CAN ID: %s (hex, 000-7FF or a 29-bit ID)\n", filter_argv[i]);
            return 1;
        }
        filter_ids[filter_count++] = (uint32_t)val;
    }

    pcan_channel_t ch;
    bool autodetected = false;
    if (!cli_pcan_open(device, bitrate, cli_cancelled, &ch, &autodetected)) return 1;
    char label[64];
    snprintf(label, sizeof(label), "%s", ch.label);
    int rate = ch.bitrate;
    frame_source_t *src = frame_source_pcan_create(&ch);
    if (!src) {
        pcan_close(&ch);
        fprintf(stderr, "[ERROR] Out of memory\n");
        return 1;
    }

    cli_log("Starting CAN monitor (PCAN listen-only %s, hardware timestamps) for %d seconds...",
            pcan_bitrate_name(rate), seconds);
    if (filter_count > 0) {
        fprintf(stderr, "[INFO] Software filter: ");
        for (int i = 0; i < filter_count; i++) {
            fprintf(stderr, "%s%03X", i > 0 ? " " : "", filter_ids[i]);
        }
        fprintf(stderr, "\n");
    }

    cli_monitor_cue_before_start();

    frame_buffer_reset(&g_cli_capture);
    pcan_monitor_filter_t flt = { .ids = filter_ids, .count = filter_count };
    g_cli_pcan_source = src;
    frame_source_start(src, pcan_monitor_sink, &flt);
    int64_t end_us = pcan_mono_us() + (int64_t)seconds * 1000000;
    while (frame_source_is_open(src) && !g_interrupt && g_running) {
        int64_t left = end_us - pcan_mono_us();
        if (left <= 0) break;
        frame_source_pump(src, left < 50000 ? (double)left / 1e6 : 0.05);
    }
    frame_source_stop(src);
    g_cli_pcan_source = NULL;
    bool lost = !frame_source_is_open(src);
    frame_source_pcan_stats_t st;
    frame_source_pcan_stats(src, &st);

    cli_log("CAN monitor stopped. Frames: %d", g_cli_capture.count);
    if (lost) fprintf(stderr, "[ERROR] PCAN %s stopped answering during the capture\n", label);
    if (g_cli_capture.dropped > 0) {
        fprintf(stderr, "[WARN] Capture buffer full: %d frames dropped\n", g_cli_capture.dropped);
    }
    cli_monitor_cue_after_stop();

    FILE *json_out = stdout;
    const char *monitor_out_path = g_cli_monitor_out_path;
    if (!monitor_out_path || !*monitor_out_path) monitor_out_path = getenv("CAN_TOOLS_MONITOR_OUT");
    if (monitor_out_path && *monitor_out_path) {
        json_out = fopen(monitor_out_path, "w");
        if (!json_out) {
            fprintf(stderr, "[ERROR] Failed to open monitor output: %s\n", monitor_out_path);
            frame_source_free(src);
            g_interrupt = NO;
            return 1;
        }
    }

    fprintf(json_out, "{\"device\":");
    json_print_string(json_out, label);
    fprintf(json_out, ",\"interface\":\"pcan\",\"library_version\":");
    json_print_string(json_out, pcan_api_version());
    fprintf(json_out, ",\"bitrate\":%d,\"bitrate_auto\":%s", rate, autodetected ? "true" : "false");
    fprintf(json_out, ",\"mode\":\"listen-only\",\"timestamps\":\"hardware\",\"duration_s\":%d", seconds);
    if (filter_count > 0) {
        fprintf(json_out, ",\"filters\":[");
        for (int i = 0; i < filter_count; i++) {
            if (i > 0) fprintf(json_out, ",");
            fprintf(json_out, "\"%03X\"", filter_ids[i]);
        }
        fprintf(json_out, "]");
    }
    fprintf(json_out, ",\"not_delivered\":{\"rtr\":%llu,\"status\":%llu,\"error_frames\":%llu,\"other\":%llu}",
            (unsigned long long)st.counts.rtr, (unsigned long long)st.counts.status,
            (unsigned long long)st.counts.error, (unsigned long long)st.counts.other);
    fprintf(json_out, ",\"overruns\":%llu,\"read_errors\":%llu", (unsigned long long)st.overruns,
            (unsigned long long)st.read_errors);
    if (st.have_drift) fprintf(json_out, ",\"clock_drift_us\":%lld", (long long)st.drift_us);
    if (lost) fprintf(json_out, ",\"device_lost\":true");
    fprintf(json_out, ",\"frames\":[\n");

    for (int i = 0; i < g_cli_capture.count; i++) {
        const can_frame_t *f = &g_cli_capture.frames[i];
        if (i > 0) fprintf(json_out, ",\n");
        fprintf(json_out, "  {\"timestamp_ms\":%lld", (long long)f->timestamp_ms);
        if (f->hw_timestamp) fprintf(json_out, ",\"timestamp_us\":%lld", (long long)can_frame_time_us(f));
        fprintf(json_out, ",\"id\":\"%03X\",\"is_extended\":%s,\"dlc\":%d,\"data\":\"", f->can_id,
                f->is_extended ? "true" : "false", f->dlc);
        for (int j = 0; j < f->dlc; j++) {
            if (j > 0) fprintf(json_out, " ");
            fprintf(json_out, "%02X", f->data[j]);
        }
        fprintf(json_out, "\"}");
    }
    if (g_cli_capture.count > 0) fprintf(json_out, "\n");
    fprintf(json_out, "]}\n");

    if (json_out != stdout) {
        fclose(json_out);
        cli_log("Wrote monitor JSON: %s", monitor_out_path);
    }

    frame_source_free(src);   /* CAN_Uninitialize */
    g_interrupt = NO;
    return 0;
}
