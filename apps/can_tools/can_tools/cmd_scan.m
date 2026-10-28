/**
 * @file cmd_scan.m
 * @brief scan: list BLE/Classic OBD adapters (JSON)
 *
 * Moved unchanged from can_reader/main.m. Bluetooth only, no CAN traffic.
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

/** CLI: scan - BLE scan and output JSON device list */
int cli_scan(void) {
    /* Wait for Bluetooth */
    cli_log("Waiting for Bluetooth...");
    if (![g_ble isBleReady]) {
        [g_ble waitForBleReady:5.0];
    }
    if (![g_ble isBleReady]) {
        fprintf(stderr, "[ERROR] Bluetooth not available\n");
        return 1;
    }

    cli_log("Scanning for devices...");
    [g_ble startScan];
    run_loop_for(5.0);
    [g_ble stopScan];

    cli_log("Found %lu device(s)", (unsigned long)g_ble.discoveredNames.count);

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
    if (g_ble.discoveredNames.count > 0) printf("\n");
    printf("]\n");

    return 0;
}

