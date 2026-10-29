/**
 * @file cmd_scan.m
 * @brief scan: list BLE/Classic OBD adapters (JSON)
 *
 * Moved unchanged from can_reader/main.m. Bluetooth, plus the PCAN-USB
 * channels (transport "usb") when the PCBUSB library loads. No CAN traffic:
 * PCAN channels are only asked for their condition and name, never opened.
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
#import "obd_adapter.h"
#import "cli_common.h"
#include "commands.h"
#include "pcan.h"

/* PCAN channels; false (with a note on stderr) when the library is missing. */
static bool scan_pcan(pcan_channel_info_t *out, int *count) {
    char err[768];
    *count = 0;
    const pcan_api_t *api = pcan_api_load(err, sizeof(err));
    if (!api) {
        fprintf(stderr, "[INFO] PCAN not listed: %s\n", err);
        return false;
    }
    *count = pcan_enumerate(api, out, PCAN_MAX_CHANNELS);
    cli_log("PCAN library %s (version \"%s\"): %d channel(s)", pcan_api_path(), pcan_api_version(), *count);
    return true;
}

static void print_pcan_rows(const pcan_channel_info_t *pc, int npc, int first_index, bool first_row) {
    for (int i = 0; i < npc; i++) {
        char label[64], channel[16];
        pcan_channel_label(pc[i].index, pc[i].hw_name, label, sizeof(label));
        snprintf(channel, sizeof(channel), "pcan%d", pc[i].index);
        printf("%s\n  {\"index\":%d,\"name\":", (first_row && i == 0) ? "" : ",", first_index + i);
        json_print_string(stdout, label);
        printf(",\"transport\":\"usb\",\"channel\":\"%s\",\"condition\":", channel);
        json_print_string(stdout, pcan_condition_text(pc[i].condition));
        if (pc[i].has_device_id) printf(",\"device_id\":%u", pc[i].device_id);
        if (pc[i].firmware[0]) {
            printf(",\"firmware\":");
            json_print_string(stdout, pc[i].firmware);
        }
        printf("}");
    }
}

/** CLI: scan - BLE scan and output JSON device list */
int cli_scan(void) {
    pcan_channel_info_t pc[PCAN_MAX_CHANNELS];
    int npc = 0;

    /* Wait for Bluetooth */
    cli_log("Waiting for Bluetooth...");
    if (![g_ble isBleReady]) {
        [g_ble waitForBleReady:5.0];
    }
    if (![g_ble isBleReady]) {
        fprintf(stderr, "[ERROR] Bluetooth not available\n");
        /* without the PCAN library this is the old result */
        if (!scan_pcan(pc, &npc)) return 1;
        printf("[");
        print_pcan_rows(pc, npc, 0, true);
        if (npc > 0) printf("\n");
        printf("]\n");
        return npc > 0 ? 0 : 1;
    }

    cli_log("Scanning for devices...");
    [g_ble startScan];
    run_loop_for(5.0);
    [g_ble stopScan];

    cli_log("Found %lu device(s)", (unsigned long)g_ble.discoveredNames.count);
    scan_pcan(pc, &npc);

    /* JSON output */
    printf("[");
    for (int i = 0; i < (int)g_ble.discoveredNames.count; i++) {
        if (i > 0) printf(",");
        OBDAdapterTransport t = (OBDAdapterTransport)[g_ble.discoveredTransports[i] intValue];
        printf("\n  {\"index\":%d,\"name\":", i);
        json_print_string(stdout, g_ble.discoveredNames[i].UTF8String);
        printf(",\"rssi\":%d,\"transport\":\"%s\"}",
               g_ble.discoveredRSSIs[i].intValue,
               t == OBD_ADAPTER_TRANSPORT_CLASSIC ? "classic" : "ble");
    }
    print_pcan_rows(pc, npc, (int)g_ble.discoveredNames.count, g_ble.discoveredNames.count == 0);
    if (g_ble.discoveredNames.count > 0 || npc > 0) printf("\n");
    printf("]\n");

    return 0;
}

