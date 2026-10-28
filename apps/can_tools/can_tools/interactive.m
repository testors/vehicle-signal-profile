/**
 * @file interactive.m
 * @brief Interactive menu (can_tools / can_reader without arguments)
 *
 * Moved unchanged from can_reader/main.m (menu actions and the menu loop).
 * Menu 3 and 7 keep their own adapter commands. Replaced in Step 3 by the TUI.
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

/*******************************************************************************
 * Menu Actions
 ******************************************************************************/

static void action_scan(void) {
    [g_ble startScan];
    printf("  Scanning for 5 seconds...\n\n");
    run_loop_for(5.0);
    [g_ble stopScan];
    for (NSUInteger i = 0; i < g_ble.discoveredNames.count; i++) {
        if ([g_ble.discoveredTransports[i] intValue] == OBD_ADAPTER_TRANSPORT_CLASSIC) {
            printf("  [%lu] %s (Classic BT)\n", (unsigned long)i,
                   g_ble.discoveredNames[i].UTF8String);
        } else {
            printf("  [%lu] %s (BLE, RSSI: %d dBm)\n", (unsigned long)i,
                   g_ble.discoveredNames[i].UTF8String,
                   g_ble.discoveredRSSIs[i].intValue);
        }
    }
    printf("\n  Found %lu device(s).\n", (unsigned long)g_ble.discoveredNames.count);
}

static void action_connect(void) {
    if (g_ble.discoveredNames.count == 0) {
        printf("  No devices found. Scan first.\n");
        return;
    }

    printf("  Enter device number [0-%lu]: ",
           (unsigned long)(g_ble.discoveredNames.count - 1));
    fflush(stdout);

    char input[16];
    if (!fgets(input, sizeof(input), stdin)) return;
    int idx = atoi(input);

    if ([g_ble connectToDevice:idx]) {
        /* Wait for connection + service discovery */
        printf("  Waiting for connection...\n");
        BOOL connected = NO;

        NSDate *timeout = [NSDate dateWithTimeIntervalSinceNow:10.0];
        while (!g_ble.serviceDiscoveryDone &&
               [[NSDate date] compare:timeout] == NSOrderedAscending) {
            @autoreleasepool {
                [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                         beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
            }
        }

        connected = g_ble.serviceDiscoveryDone;
        if (connected) {
            printf("  Connection successful.\n");
        } else {
            printf("  Connection/service discovery timeout.\n");
        }
    }
}

static void action_init_elm327(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    printf("  Initializing ELM327...\n\n");

    char cmds[ELM327_MAX_INIT_CMDS][ELM327_CMD_MAX_LEN];
    int cmd_count;
    elm327_build_init_cmds(cmds, &cmd_count);

    char resp[512];

    for (int i = 0; i < cmd_count; i++) {
        printf("  >> %s\n", cmds[i]);

        if ([g_ble sendCommand:cmds[i] response:resp maxLen:sizeof(resp)]) {
            printf("  << %s\n\n", resp);

            /* Parse specific responses */
            if (strcmp(cmds[i], "ATZ") == 0) {
                /* After reset, add extra delay */
                run_loop_for(0.5);
            } else if (strcmp(cmds[i], "ATRV") == 0) {
                double voltage;
                if (elm327_parse_voltage(resp, &voltage)) {
                    printf("  Battery Voltage: %.1f V\n\n", voltage);
                }
            } else if (strcmp(cmds[i], "ATDPN") == 0) {
                const char *proto = elm327_parse_protocol(resp);
                if (proto) {
                    printf("  Protocol: %s\n\n", proto);
                }
            }
        } else {
            printf("  << (timeout)\n\n");
        }
    }

    printf("  ELM327 initialization complete.\n");

    /* Try STN detection */
    printf("\n  Detecting STN chip...\n");
    char sti_resp[256];
    printf("  >> STI\n");
    if ([g_ble sendCommand:"STI" response:sti_resp maxLen:sizeof(sti_resp)]) {
        printf("  << %s\n", sti_resp);

        if (stn2255_detect(resp, sti_resp, &g_stn_info)) {
            g_stn_detected = true;
            printf("\n  STN detected: %s\n", g_stn_info.device_id);

            /* Send STDI for hardware ID */
            char stdi_resp[256];
            printf("  >> STDI\n");
            if ([g_ble sendCommand:"STDI" response:stdi_resp maxLen:sizeof(stdi_resp)]) {
                printf("  << %s\n", stdi_resp);
                /* Copy hardware ID */
                const char *s = stdi_resp;
                while (*s == ' ' || *s == '\r' || *s == '\n') s++;
                strncpy(g_stn_info.hardware_id, s, sizeof(g_stn_info.hardware_id) - 1);
            }

            /* Send STN optimization commands */
            char stn_cmds[STN2255_MAX_INIT_CMDS][STN2255_CMD_MAX_LEN];
            int stn_cmd_count;
            stn2255_build_init_cmds(stn_cmds, &stn_cmd_count);

            printf("\n  Applying STN optimizations...\n\n");
            for (int i = 0; i < stn_cmd_count; i++) {
                printf("  >> %s\n", stn_cmds[i]);
                char stn_resp[256];
                if ([g_ble sendCommand:stn_cmds[i] response:stn_resp maxLen:sizeof(stn_resp)]) {
                    printf("  << %s\n\n", stn_resp);
                } else {
                    printf("  << (timeout)\n\n");
                }
            }

            printf("  STN initialization complete.\n");
        } else {
            printf("  Not an STN chip (standard ELM327).\n");
        }
    } else {
        printf("  << (timeout) - STI not supported, standard ELM327.\n");
    }
}

static void action_query_supported_pids(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    bool supported[256];
    memset(supported, 0, sizeof(supported));

    printf("  Querying supported PIDs...\n\n");

    /* Query PID 0x00 (supported PIDs 01-20) */
    static const uint8_t query_pids[] = { 0x00, 0x20, 0x40, 0x60 };
    static const char *query_labels[] = {
        "PIDs 01-20", "PIDs 21-40", "PIDs 41-60", "PIDs 61-80"
    };

    for (int q = 0; q < 4; q++) {
        char cmd[16];
        elm327_build_pid_query(query_pids[q], cmd, sizeof(cmd));

        char resp[256];
        printf("  >> %s (%s)\n", cmd, query_labels[q]);

        if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)]) {
            if (!elm327_is_error_response(resp)) {
                printf("  << %s\n", resp);
                elm327_parse_supported_pids(resp, query_pids[q], supported);
            } else {
                printf("  << %s (not supported)\n", resp);
                break; /* Stop if range not supported */
            }
        } else {
            printf("  << (timeout)\n");
            break;
        }
    }

    /* Print supported PIDs */
    printf("\n  Supported PIDs:\n");
    int count = 0;
    for (int i = 1; i < 256; i++) {
        if (supported[i]) {
            const elm327_pid_info_t *info = elm327_get_pid_info(i);
            if (info) {
                printf("    0x%02X  %-25s [%s]\n", i, info->name, info->unit);
            } else {
                printf("    0x%02X  (unknown)\n", i);
            }
            count++;
        }
    }
    printf("\n  Total: %d supported PIDs.\n", count);
}

static void action_poll_single_pid(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    printf("  Enter PID hex (e.g., 0C for RPM): ");
    fflush(stdout);

    char input[16];
    if (!fgets(input, sizeof(input), stdin)) return;

    /* Parse hex PID */
    unsigned int pid_val;
    if (sscanf(input, "%x", &pid_val) != 1 || pid_val > 0xFF) {
        printf("  Invalid PID.\n");
        return;
    }
    uint8_t pid = (uint8_t)pid_val;

    const elm327_pid_info_t *info = elm327_get_pid_info(pid);
    printf("  Querying PID 0x%02X (%s)...\n",
           pid, info ? info->name : "unknown");

    char cmd[16];
    elm327_build_pid_query(pid, cmd, sizeof(cmd));

    char resp[256];
    printf("  >> %s\n", cmd);

    if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)]) {
        printf("  << %s\n", resp);

        if (!elm327_is_error_response(resp)) {
            uint8_t parsed_pid;
            uint8_t data[ELM327_MAX_DATA_BYTES];
            int data_len;

            if (elm327_parse_pid_response(resp, &parsed_pid, data, &data_len)) {
                double value = elm327_convert_pid_value(parsed_pid, data, data_len);
                printf("\n  Result: %.2f %s\n",
                       value, info ? info->unit : "");
            } else {
                printf("  Failed to parse response.\n");
            }
        } else {
            printf("  Error response from adapter.\n");
        }
    } else {
        printf("  Command timeout.\n");
    }
}

static void action_poll_continuous(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    /* Motorsports basic PIDs: RPM, Throttle, Speed */
    static const uint8_t poll_pids[] = { 0x0C, 0x11, 0x0D };
    static const int poll_count = 3;

    printf("  Continuous polling: RPM, Throttle, Speed\n");
    if (g_stn_detected) {
        printf("  Mode: batch query (1 round-trip)\n");
    } else {
        printf("  Mode: sequential query (%d round-trips)\n", poll_count);
    }
    printf("  Press Ctrl+C to stop.\n\n");

    g_interrupt = NO;

    while (!g_interrupt && g_ble.isConnected) {
        @autoreleasepool {
            printf("\r  ");

            if (g_stn_detected) {
                /* Batch query: single round-trip */
                char cmd[64];
                stn2255_build_batch_query(poll_pids, poll_count, cmd, sizeof(cmd));

                char resp[512];
                if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] &&
                    !elm327_is_error_response(resp)) {

                    stn2255_batch_result_t batch;
                    int parsed = stn2255_parse_batch_response(resp, &batch);

                    if (parsed > 0) {
                        for (int i = 0; i < batch.count; i++) {
                            if (!batch.results[i].valid) continue;
                            const elm327_pid_info_t *info = elm327_get_pid_info(batch.results[i].pid);
                            if (info) {
                                printf("%-15s: %7.1f %-5s  ",
                                       info->name, batch.results[i].value, info->unit);
                            }
                            g_sample_count++;
                        }

                        /* Fallback: if fewer PIDs returned than requested, poll missing ones */
                        if (batch.count < poll_count) {
                            for (int i = 0; i < poll_count && !g_interrupt; i++) {
                                bool found = false;
                                for (int j = 0; j < batch.count; j++) {
                                    if (batch.results[j].pid == poll_pids[i] && batch.results[j].valid) {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found) {
                                    /* Sequential fallback for missing PID */
                                    char single_cmd[16];
                                    elm327_build_pid_query(poll_pids[i], single_cmd, sizeof(single_cmd));
                                    char single_resp[256];
                                    if ([g_ble sendCommand:single_cmd response:single_resp maxLen:sizeof(single_resp)] &&
                                        !elm327_is_error_response(single_resp)) {
                                        uint8_t pid;
                                        uint8_t data[ELM327_MAX_DATA_BYTES];
                                        int data_len;
                                        if (elm327_parse_pid_response(single_resp, &pid, data, &data_len)) {
                                            double value = elm327_convert_pid_value(pid, data, data_len);
                                            const elm327_pid_info_t *info = elm327_get_pid_info(pid);
                                            if (info) {
                                                printf("%-15s: %7.1f %-5s  ",
                                                       info->name, value, info->unit);
                                            }
                                            g_sample_count++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                /* Sequential query: N round-trips */
                for (int i = 0; i < poll_count && !g_interrupt; i++) {
                    char cmd[16];
                    elm327_build_pid_query(poll_pids[i], cmd, sizeof(cmd));

                    char resp[256];
                    if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] &&
                        !elm327_is_error_response(resp)) {

                        uint8_t pid;
                        uint8_t data[ELM327_MAX_DATA_BYTES];
                        int data_len;

                        if (elm327_parse_pid_response(resp, &pid, data, &data_len)) {
                            double value = elm327_convert_pid_value(pid, data, data_len);
                            const elm327_pid_info_t *info = elm327_get_pid_info(pid);

                            if (info) {
                                printf("%-15s: %7.1f %-5s  ",
                                       info->name, value, info->unit);
                            }

                            g_sample_count++;
                        }
                    }
                }
            }

            fflush(stdout);
        }
    }

    printf("\n\n  Polling stopped. (samples: %u)\n",
           g_sample_count);
    g_interrupt = NO;
}

static void action_can_monitor(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    char resp[256];

    /* Passive init: CAN 500k, Silent Mode, raw formatting */
    [g_ble sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCSM1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];

    const char *monitor_cmd = g_stn_detected ? "STMA" : "ATMA";
    printf("  Starting CAN bus monitor (%s, Silent Mode)...\n", monitor_cmd);
    printf("  Press Ctrl+C to stop.\n\n");
    printf("  %-8s %-5s %s\n", "CAN ID", "DLC", "Data");
    printf("  %-8s %-5s %s\n", "------", "---", "------------------------");

    g_interrupt = NO;
    /* Monitor parsing on (clears response state), then send monitor command */
    [g_ble startMonitorWithCallback:monitor_frame_callback context:NULL];

    char monitor_str[16];
    snprintf(monitor_str, sizeof(monitor_str), "%s\r", monitor_cmd);
    [g_ble sendRawString:monitor_str];

    /* Pump run loop until interrupted */
    while (!g_interrupt && g_ble.isConnected) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }

    /* Stop monitoring: send CR to interrupt */
    [g_ble stopMonitor];
    [g_ble sendRawString:"\r"];
    run_loop_for(0.2);

    printf("\n  CAN monitor stopped. (frames: %u)\n",
           g_can_frame_count);
    g_interrupt = NO;
}

static void action_disconnect(void) {
    if (!g_ble.isConnected) {
        printf("  Not connected.\n");
        return;
    }
    [g_ble disconnect];
    g_stn_detected = false;
    memset(&g_stn_info, 0, sizeof(g_stn_info));
    printf("  Disconnected.\n");
}

/*******************************************************************************
 * Menu
 ******************************************************************************/

static void print_menu(void) {
    printf("\n");
    printf("=== macOS OBD Reader ===\n");

    if (g_ble.isConnected) {
        printf("  Connected: %s", g_ble.connectedDeviceName.UTF8String ?: "unknown");
        if (g_stn_detected) {
            printf(" [STN: %s]", g_stn_info.device_id);
        }
        printf("\n");
    }

    printf("\n");
    printf("  1. Scan for devices\n");
    printf("  2. Connect to device\n");
    printf("  3. Initialize adapter (ELM327/STN)\n");
    printf("  4. Query supported PIDs\n");
    printf("  5. Poll PID (single)\n");
    printf("  6. Poll PIDs (continuous - motorsports basic)\n");
    printf("  7. Monitor CAN bus (%s)\n", g_stn_detected ? "STMA" : "ATMA");
    printf("  8. Send raw AT command\n");
    printf("  9. Disconnect\n");
    printf("  0. Quit\n");
    printf("\n");
    printf("  Select: ");
    fflush(stdout);
}

static void action_raw_command(void) {
    if (!g_ble.isConnected || !g_ble.serviceDiscoveryDone) {
        printf("  Not connected.\n");
        return;
    }

    printf("  Enter command: ");
    fflush(stdout);

    char input[128];
    if (!fgets(input, sizeof(input), stdin)) return;

    /* Strip newline */
    size_t len = strlen(input);
    while (len > 0 && (input[len - 1] == '\n' || input[len - 1] == '\r')) {
        input[--len] = '\0';
    }
    if (len == 0) return;

    char resp[512];
    printf("  >> %s\n", input);

    if ([g_ble sendCommand:input response:resp maxLen:sizeof(resp)]) {
        printf("  << %s\n", resp);
    } else {
        printf("  << (timeout)\n");
    }
}

/*******************************************************************************
 * Interactive main (no arguments)
 ******************************************************************************/

int interactive_main(void) {
    /* Interactive mode (original behavior) */
    printf("=========================================\n");
    printf("  CAN Reader\n");
    printf("=========================================\n\n");

    /* Setup signal handler for Ctrl+C */
    signal(SIGINT, signal_handler);

    /* Initialize adapter (OBD name hints, errors on stderr) */
    g_ble = [[OBDAdapter alloc] initWithNameHints:[OBDAdapter defaultNameHints]
                                            quiet:NO];
    g_ble.logCommandErrors = YES;

    /* Wait for Bluetooth to power on */
    printf("  Waiting for Bluetooth...\n");
    if (![g_ble isBleReady]) {
        [g_ble waitForBleReady:5.0];
    }

    if (![g_ble isBleReady]) {
        printf("  Bluetooth initialization timeout.\n");
        printf("  Make sure Bluetooth is enabled in System Settings.\n");
    }

    /* Main menu loop */
    while (g_running) {
        @autoreleasepool {
            print_menu();

            /* Read input while pumping run loop */
            char input[16] = {0};

            /* Use select() to check for stdin while running the event loop */
            BOOL gotInput = NO;
            while (!gotInput && g_running) {
                @autoreleasepool {
                    /* Pump the run loop briefly */
                    [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                            beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];

                    /* Check if stdin has data */
                    fd_set fds;
                    FD_ZERO(&fds);
                    FD_SET(STDIN_FILENO, &fds);
                    struct timeval tv = { .tv_sec = 0, .tv_usec = 0 };

                    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
                        if (fgets(input, sizeof(input), stdin)) {
                            gotInput = YES;
                        }
                    }
                }
            }

            if (!g_running) break;

            int choice = atoi(input);

            switch (choice) {
                case 1: action_scan(); break;
                case 2: action_connect(); break;
                case 3: action_init_elm327(); break;
                case 4: action_query_supported_pids(); break;
                case 5: action_poll_single_pid(); break;
                case 6: action_poll_continuous(); break;
                case 7: action_can_monitor(); break;
                case 8: action_raw_command(); break;
                case 9: action_disconnect(); break;
                case 0:
                    g_running = NO;
                    break;
                default:
                    printf("  Invalid selection.\n");
                    break;
            }
        }
    }

    /* Cleanup */
    [g_ble disconnect];

    printf("\nGoodbye.\n");
    cli_release_adapter();
    return 0;
}
