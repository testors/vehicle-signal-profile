/**
 * @file cmd_query.m
 * @brief query / supported / raw: OBD-II requests (transmit)
 *
 * Moved unchanged from can_reader/main.m. Connect with the ACTIVE init
 * profile (cli_auto_connect); OBD port only.
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

/** CLI: query <device> <pid_hex...> */
int cli_query(const char *device, int pid_argc, const char *pid_argv[]) {
    if (pid_argc < 1) {
        fprintf(stderr, "[ERROR] Usage: query <device> <pid_hex> [pid_hex...]\n");
        return 1;
    }

    /* Parse PID hex values */
    uint8_t pids[STN2255_MAX_BATCH_PIDS];
    int pid_count = 0;
    for (int i = 0; i < pid_argc && pid_count < STN2255_MAX_BATCH_PIDS; i++) {
        unsigned int val;
        if (sscanf(pid_argv[i], "%x", &val) == 1 && val <= 0xFF) {
            pids[pid_count++] = (uint8_t)val;
        } else {
            fprintf(stderr, "[ERROR] Invalid PID hex: %s\n", pid_argv[i]);
            return 1;
        }
    }

    int matchIdx = cli_auto_connect(device);

    cli_log("Querying PIDs: %s%s%s",
            pid_argc >= 1 ? pid_argv[0] : "",
            pid_argc >= 2 ? " " : "", pid_argc >= 2 ? pid_argv[1] : "");

    /* Build JSON output */
    printf("{\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"stn\":%s,\"results\":[\n", g_stn_detected ? "true" : "false");

    int result_count = 0;

    if (g_stn_detected && pid_count > 1) {
        /* Batch query */
        char cmd[64];
        stn2255_build_batch_query(pids, pid_count, cmd, sizeof(cmd));
        char resp[512];

        if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] &&
            !elm327_is_error_response(resp)) {

            stn2255_batch_result_t batch;
            stn2255_parse_batch_response(resp, &batch);

            for (int i = 0; i < batch.count; i++) {
                if (!batch.results[i].valid) continue;
                if (result_count > 0) printf(",\n");

                const elm327_pid_info_t *info = elm327_get_pid_info(batch.results[i].pid);
                printf("  {\"pid\":\"%02X\",\"name\":", batch.results[i].pid);
                json_print_string(stdout, info ? info->name : "Unknown");
                printf(",\"value\":%.2f,\"unit\":", batch.results[i].value);
                json_print_string(stdout, info ? info->unit : "");

                /* Raw hex */
                printf(",\"raw\":\"");
                for (int j = 0; j < batch.results[i].data_len; j++) {
                    if (j > 0) printf(" ");
                    printf("%02X", batch.results[i].data[j]);
                }
                printf("\"}");
                result_count++;
            }

            /* Fallback: poll PIDs missing from batch */
            for (int i = 0; i < pid_count; i++) {
                bool found = false;
                for (int j = 0; j < batch.count; j++) {
                    if (batch.results[j].pid == pids[i] && batch.results[j].valid) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    char single_cmd[16];
                    elm327_build_pid_query(pids[i], single_cmd, sizeof(single_cmd));
                    char single_resp[256];
                    if ([g_ble sendCommand:single_cmd response:single_resp maxLen:sizeof(single_resp)] &&
                        !elm327_is_error_response(single_resp)) {
                        uint8_t pid;
                        uint8_t data[ELM327_MAX_DATA_BYTES];
                        int data_len;
                        if (elm327_parse_pid_response(single_resp, &pid, data, &data_len)) {
                            double value = elm327_convert_pid_value(pid, data, data_len);
                            const elm327_pid_info_t *info = elm327_get_pid_info(pid);
                            if (result_count > 0) printf(",\n");
                            printf("  {\"pid\":\"%02X\",\"name\":", pid);
                            json_print_string(stdout, info ? info->name : "Unknown");
                            printf(",\"value\":%.2f,\"unit\":", value);
                            json_print_string(stdout, info ? info->unit : "");
                            printf(",\"raw\":\"");
                            for (int j = 0; j < data_len; j++) {
                                if (j > 0) printf(" ");
                                printf("%02X", data[j]);
                            }
                            printf("\"}");
                            result_count++;
                        }
                    }
                }
            }
        } else {
            fprintf(stderr, "[WARN] Batch query failed, falling back to sequential\n");
            goto sequential;
        }
    } else {
sequential:
        /* Sequential query */
        for (int i = 0; i < pid_count; i++) {
            char cmd[16];
            elm327_build_pid_query(pids[i], cmd, sizeof(cmd));
            char resp[256];

            if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] &&
                !elm327_is_error_response(resp)) {
                uint8_t pid;
                uint8_t data[ELM327_MAX_DATA_BYTES];
                int data_len;

                if (elm327_parse_pid_response(resp, &pid, data, &data_len)) {
                    double value = elm327_convert_pid_value(pid, data, data_len);
                    const elm327_pid_info_t *info = elm327_get_pid_info(pid);

                    if (result_count > 0) printf(",\n");
                    printf("  {\"pid\":\"%02X\",\"name\":", pid);
                    json_print_string(stdout, info ? info->name : "Unknown");
                    printf(",\"value\":%.2f,\"unit\":", value);
                    json_print_string(stdout, info ? info->unit : "");
                    printf(",\"raw\":\"");
                    for (int j = 0; j < data_len; j++) {
                        if (j > 0) printf(" ");
                        printf("%02X", data[j]);
                    }
                    printf("\"}");
                    result_count++;
                }
            } else {
                cli_log("PID %02X: no data or error", pids[i]);
            }
        }
    }

    printf("\n]}\n");

    [g_ble disconnect];
    return 0;
}

/** CLI: supported <device> */
int cli_supported(const char *device) {
    int matchIdx = cli_auto_connect(device);

    bool supported[256];
    memset(supported, 0, sizeof(supported));

    cli_log("Querying supported PIDs...");

    static const uint8_t query_pids[] = { 0x00, 0x20, 0x40, 0x60 };
    for (int q = 0; q < 4; q++) {
        char cmd[16];
        elm327_build_pid_query(query_pids[q], cmd, sizeof(cmd));
        char resp[256];

        if ([g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)]) {
            if (!elm327_is_error_response(resp)) {
                elm327_parse_supported_pids(resp, query_pids[q], supported);
            } else {
                break;
            }
        } else {
            break;
        }
    }

    /* JSON output */
    printf("{\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"supported\":[\n");

    int count = 0;
    for (int i = 1; i < 256; i++) {
        if (!supported[i]) continue;
        const elm327_pid_info_t *info = elm327_get_pid_info(i);
        if (count > 0) printf(",\n");
        printf("  {\"pid\":\"%02X\",\"name\":", i);
        json_print_string(stdout, info ? info->name : "Unknown");
        printf(",\"unit\":");
        json_print_string(stdout, info ? info->unit : "");
        printf("}");
        count++;
    }

    printf("\n]}\n");
    cli_log("Total: %d supported PIDs", count);

    [g_ble disconnect];
    return 0;
}

/** CLI: raw <device> <command> */
int cli_raw(const char *device, const char *command) {
    cli_auto_connect(device);

    cli_log("Sending: %s", command);

    char resp[512];
    if ([g_ble sendCommand:command response:resp maxLen:sizeof(resp)]) {
        printf("%s\n", resp);
    } else {
        fprintf(stderr, "[ERROR] Command timeout\n");
        [g_ble disconnect];
        return 1;
    }

    [g_ble disconnect];
    return 0;
}

