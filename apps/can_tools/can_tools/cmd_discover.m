/**
 * @file cmd_discover.m
 * @brief discover: guided PT-CAN signal discovery (receive-only)
 *
 * cli_discover moved from can_reader/main.m (can_discover/main.m had the same
 * body as run_discover); cmd_discover_main is can_discover's main. PASSIVE
 * init profile, no filters; a NULL device prefix takes the first adapter.
 * A PCAN device ("pcan", "pcan1".."pcan8") runs the same session over the
 * PCAN frame source (listen-only); no Bluetooth object is created then.
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
#include "discover_session.h"
#import "frame_source_live.h"
#include "frame_source_pcan.h"
#include "cli_pcan.h"

/*******************************************************************************
 * CLI: discover <device>
 * Interactive PT-CAN signal auto-discovery.
 * Guides user through capture phases with skip/confirm for each signal.
 ******************************************************************************/

static void disconnect_adapter(void *ctx) {
    [(OBDAdapter *)ctx disconnect];
}

int cli_discover(const char *device, const char *dump_dir_override) {
    int matchIdx = cli_ptcan_connect(device, NULL, 0);
    const char *connected_name = NULL;
    if (matchIdx >= 0 && matchIdx < (int)g_ble.discoveredNames.count) {
        connected_name = g_ble.discoveredNames[matchIdx].UTF8String;
    }
    const char *monitor_cmd = g_stn_detected ? "STMA" : "ATMA";

    frame_source_t *source = frame_source_live_create(g_ble, monitor_cmd);
    if (!source) {
        fprintf(stderr, "[ERROR] Out of memory\n");
        return 1;
    }
    disc_cli_config_t cfg = {
        .source = source,
        .device_prefix = device,
        .connected_name = connected_name,
        .monitor_cmd = monitor_cmd,
        .dump_dir_override = dump_dir_override,
        .running = &g_running,
        .interrupt = &g_interrupt,
        .disconnect = disconnect_adapter,
        .disconnect_ctx = g_ble,
    };
    int ret = disc_cli_run(&cfg);
    frame_source_free(source);
    return ret;
}

/*******************************************************************************
 * discover pcan: the guided session over a listen-only PCAN channel
 ******************************************************************************/

static bool discover_cancelled(void) {
    return !g_running || g_interrupt;
}

int cli_discover_pcan(const char *device, const char *dump_dir_override, const pcan_bitrate_t *bitrate) {
    pcan_channel_t ch;
    if (!cli_pcan_open(device, bitrate, discover_cancelled, &ch, NULL)) return 1;
    char label[64], monitor_cmd[48];
    snprintf(label, sizeof(label), "%s", ch.label);
    snprintf(monitor_cmd, sizeof(monitor_cmd), "PCAN listen-only %s", pcan_bitrate_name(ch.bitrate));
    frame_source_t *source = frame_source_pcan_create(&ch);
    if (!source) {
        pcan_close(&ch);
        fprintf(stderr, "[ERROR] Out of memory\n");
        return 1;
    }
    g_cli_pcan_source = source;
    disc_cli_config_t cfg = {
        .source = source,
        .device_prefix = device,
        .connected_name = label,
        .monitor_cmd = monitor_cmd,
        .dump_dir_override = dump_dir_override,
        .running = &g_running,
        .interrupt = &g_interrupt,
        .disconnect = NULL,      /* frame_source_free closes the channel */
        .disconnect_ctx = NULL,
    };
    int ret = disc_cli_run(&cfg);
    g_cli_pcan_source = NULL;
    frame_source_free(source);
    return ret;
}

/*******************************************************************************
 * can_discover / can_tools discover: arguments (device prefix optional)
 ******************************************************************************/

int cmd_discover_main(int argc, const char *argv[]) {

    @autoreleasepool {
        const char *device_prefix = NULL;
        const char *dump_dir_override = NULL;
        const char *bitrate_arg = NULL;

        if (argc >= 2) {
            for (int i = 1; i < argc; i++) {
                if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0 ||
                    strcmp(argv[i], "help") == 0) {
                    fprintf(stderr, "Usage: canbus_discover [device_prefix] [--dump-dir DIR] [--bitrate RATE|auto]\n");
                    fprintf(stderr, "\n");
                    fprintf(stderr, "  PT-CAN signal auto-discovery tool.\n");
                    fprintf(stderr, "  Connects via BLE/Classic BT (or a PCAN-USB), captures CAN traffic in 6 phases,\n");
                    fprintf(stderr, "  and identifies steering/RPM/throttle/brake/gear/wheel speed signals.\n");
                    fprintf(stderr, "\n");
                    fprintf(stderr, "  device_prefix: BLE name prefix (e.g., vLinker, OBDLink)\n");
                    fprintf(stderr, "                 If omitted, connects to first discovered device.\n");
                    fprintf(stderr, "                 pcan / pcan1..pcan8: PEAK PCAN-USB, listen-only (PCBUSB library)\n");
                    fprintf(stderr, "  --bitrate RATE: PCAN only: auto or %s (default 500k)\n",
                            pcan_bitrate_list());
                    fprintf(stderr, "  --dump-dir DIR: Override the default capture dump directory.\n");
                    fprintf(stderr, "                  Default: captures/YYYYMMDD_HHMMSS\n");
                    fprintf(stderr, "\n");
                    fprintf(stderr, "  JSON output on stdout, logs on stderr.\n");
                    fprintf(stderr, "  Also writes phase dumps, result.json, manifest.json,\n");
                    fprintf(stderr, "  draft DBC, and draft VSP into the dump directory.\n");
                    fprintf(stderr, "  Use 2>/dev/null to get clean JSON.\n");
                    return 0;
                } else if (strcmp(argv[i], "--dump-dir") == 0) {
                    if (i + 1 >= argc) {
                        fprintf(stderr, "[ERROR] --dump-dir requires a directory path\n");
                        return 1;
                    }
                    dump_dir_override = argv[++i];
                } else if (strncmp(argv[i], "--dump-dir=", 11) == 0) {
                    dump_dir_override = argv[i] + 11;
                } else if (strcmp(argv[i], "--bitrate") == 0) {
                    if (i + 1 >= argc) {
                        fprintf(stderr, "[ERROR] --bitrate requires a value\n");
                        return 1;
                    }
                    bitrate_arg = argv[++i];
                } else if (strncmp(argv[i], "--bitrate=", 10) == 0) {
                    bitrate_arg = argv[i] + 10;
                } else if (!device_prefix) {
                    device_prefix = argv[i];
                } else {
                    fprintf(stderr, "[ERROR] Unexpected argument: %s\n", argv[i]);
                    fprintf(stderr, "Usage: canbus_discover [device_prefix] [--dump-dir DIR] [--bitrate RATE|auto]\n");
                    return 1;
                }
            }
        }

        signal(SIGINT, signal_handler);

        if (device_prefix && pcan_is_device(device_prefix)) {
            pcan_bitrate_t br;
            if (!cli_pcan_parse_bitrate(bitrate_arg, &br)) return 1;
            return cli_discover_pcan(device_prefix, dump_dir_override, &br);
        }
        if (bitrate_arg) {
            fprintf(stderr, "[ERROR] --bitrate applies to PCAN devices only (pcan, pcan1..pcan8)\n");
            return 1;
        }

        cli_create_adapter();

        int ret = cli_discover(device_prefix, dump_dir_override);
        cli_release_adapter();

        return ret;
    }
}
