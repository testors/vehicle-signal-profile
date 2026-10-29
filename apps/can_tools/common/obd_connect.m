/**
 * @file obd_connect.m
 * @brief The one scan / connect / adapter-init path every command uses
 *
 * Moved from can_reader (cli_ble_connect, cli_auto_connect, cli_ptcan_connect),
 * can_discover (first-device auto-select) and vsp_probe (init_basic_adapter,
 * the STFAP part of setup_passive_can). Compiled without ARC.
 */

#import "obd_connect.h"

#include <stdio.h>
#include <string.h>

#include "cli_util.h"
#include "elm327.h"

void obd_link_init(obd_link_t *link) {
    if (!link) return;
    memset(link, 0, sizeof(*link));
    link->device_index = -1;
}

const char *obd_monitor_cmd(const obd_link_t *link) {
    return (link && link->stn_detected) ? "STMA" : "ATMA";
}

void obd_scan(OBDAdapter *adapter, double seconds) {
    [adapter startScan];
    [adapter runLoopFor:seconds];
    [adapter stopScan];
}

bool obd_connect_device(OBDAdapter *adapter, const char *device_prefix, obd_link_t *link) {
    if (!adapter || !link) return false;

    /* BLE may be unavailable while Classic RFCOMM still works. Scan anyway so
       Classic adapters like vLinker MS can be selected. */
    cli_log("Waiting for Bluetooth...");
    if (![adapter isBleReady]) {
        [adapter waitForBleReady:10.0];
    }
    if (![adapter isBleReady]) {
        fprintf(stderr, "[WARN] BLE not available; scanning paired/Classic devices only\n");
    }

    /* Scan */
    cli_log("Scanning for devices...");
    obd_scan(adapter, 8.0);

    if (adapter.discoveredNames.count == 0) {
        fprintf(stderr, "[WARN] No OBD devices found on first pass; retrying Classic scan\n");
        obd_scan(adapter, 8.0);
    }

    if (adapter.discoveredNames.count == 0) {
        fprintf(stderr, "[ERROR] No OBD devices found\n");
        return false;
    }

    int matchIdx = -1;
    if (device_prefix && device_prefix[0]) {
        /* Match device by name prefix (case-insensitive) */
        NSString *prefix = [[NSString stringWithUTF8String:device_prefix] uppercaseString];
        for (int i = 0; i < (int)adapter.discoveredNames.count; i++) {
            NSString *name = [adapter.discoveredNames[i] uppercaseString];
            if ([name containsString:prefix]) {
                matchIdx = i;
                cli_log("Found: %s (RSSI: %d)", adapter.discoveredNames[i].UTF8String,
                        adapter.discoveredRSSIs[i].intValue);
                break;
            }
        }

        if (matchIdx < 0) {
            fprintf(stderr, "[ERROR] No device matching '%s' found. Available:\n", device_prefix);
            for (int i = 0; i < (int)adapter.discoveredNames.count; i++) {
                fprintf(stderr, "  [%d] %s\n", i, adapter.discoveredNames[i].UTF8String);
            }
            return false;
        }
    } else {
        /* No prefix — connect to first discovered device */
        matchIdx = 0;
        cli_log("Auto-selecting: %s (RSSI: %d)", adapter.discoveredNames[0].UTF8String,
                adapter.discoveredRSSIs[0].intValue);
    }

    OBDAdapterTransport transport =
        (OBDAdapterTransport)[adapter.discoveredTransports[matchIdx] intValue];
    if (transport == OBD_ADAPTER_TRANSPORT_BLE && ![adapter isBleReady]) {
        fprintf(stderr, "[ERROR] Matched device requires BLE, but BLE is not available\n");
        return false;
    }

    /* Connect */
    cli_log("Connecting...");
    if (![adapter connectToDevice:matchIdx]) {
        fprintf(stderr, "[ERROR] Failed to initiate connection\n");
        return false;
    }

    /* Wait for service discovery */
    NSDate *timeout = [NSDate dateWithTimeIntervalSinceNow:10.0];
    while (!adapter.serviceDiscoveryDone &&
           [[NSDate date] compare:timeout] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.05]];
        }
    }

    if (!adapter.serviceDiscoveryDone) {
        fprintf(stderr, "[ERROR] Connection/service discovery timeout\n");
        return false;
    }

    link->device_index = matchIdx;
    snprintf(link->device_name, sizeof(link->device_name), "%s",
             adapter.discoveredNames[matchIdx].UTF8String ?: "");
    return true;
}

/* STI; on an STN chip fills link->stn_info. `ati_resp` is the previous
 * response buffer, passed through as before. */
static bool detect_stn(OBDAdapter *adapter, const char *ati_resp, obd_link_t *link, bool log) {
    char sti_resp[256] = "";
    if ([adapter sendCommand:"STI" response:sti_resp maxLen:sizeof(sti_resp)]) {
        if (stn2255_detect(ati_resp, sti_resp, &link->stn_info)) {
            link->stn_detected = true;
            if (log) cli_log("STN detected: %s", link->stn_info.device_id);
        }
    }
    return link->stn_detected;
}

/* Standard OBD-II init: ELM327 init commands + STN detection and options.
 * Transmits (ATSP0 auto protocol search on the first request). */
static bool init_active(OBDAdapter *adapter, obd_link_t *link) {
    cli_log("Initializing ELM327...");
    char cmds[ELM327_MAX_INIT_CMDS][ELM327_CMD_MAX_LEN];
    int cmd_count;
    elm327_build_init_cmds(cmds, &cmd_count);

    char resp[512] = "";
    for (int i = 0; i < cmd_count; i++) {
        if ([adapter sendCommand:cmds[i] response:resp maxLen:sizeof(resp)]) {
            if (strcmp(cmds[i], "ATZ") == 0) {
                [adapter runLoopFor:0.5];
            } else if (strcmp(cmds[i], "ATI") == 0) {
                cli_log("ELM327 initialized: %s", resp);
            }
        }
    }

    if (detect_stn(adapter, resp, link, true)) {
        /* STDI for hardware ID */
        char stdi_resp[256];
        if ([adapter sendCommand:"STDI" response:stdi_resp maxLen:sizeof(stdi_resp)]) {
            const char *s = stdi_resp;
            while (*s == ' ' || *s == '\r' || *s == '\n') s++;
            strncpy(link->stn_info.hardware_id, s, sizeof(link->stn_info.hardware_id) - 1);
        }

        /* STN optimization commands */
        char stn_cmds[STN2255_MAX_INIT_CMDS][STN2255_CMD_MAX_LEN];
        int stn_cmd_count;
        stn2255_build_init_cmds(stn_cmds, &stn_cmd_count);
        for (int i = 0; i < stn_cmd_count; i++) {
            char stn_resp[256];
            [adapter sendCommand:stn_cmds[i] response:stn_resp maxLen:sizeof(stn_resp)];
        }
    }
    return true;
}

/* Reset + echo/linefeeds off + spaces on + STN detection; no protocol. */
static bool init_basic(OBDAdapter *adapter, obd_link_t *link) {
    char resp[512] = "";
    const char *init[] = { "ATZ", "ATE0", "ATL0", "ATS1" };
    for (int i = 0; i < 4; i++) {
        [adapter sendCommand:init[i] response:resp maxLen:sizeof(resp) timeout:3.0];
        if (strcmp(init[i], "ATZ") == 0) [adapter runLoopFor:0.5];
    }
    detect_stn(adapter, resp, link, false);
    return true;
}

/* Receive-only CAN monitor setup: silent mode (no ACK), raw frames, headers.
 * No OBD protocol negotiation; nothing is sent on the bus. */
static bool init_passive(OBDAdapter *adapter, const obd_init_config_t *cfg, obd_link_t *link) {
    char resp[512] = "";

    /* Minimal init: reset + echo off + linefeeds off */
    cli_log("Initializing PT-CAN passive mode...");

    static const char *init_cmds[] = { "ATZ", "ATE0", "ATL0" };
    for (int i = 0; i < 3; i++) {
        [adapter sendCommand:init_cmds[i] response:resp maxLen:sizeof(resp)];
        if (strcmp(init_cmds[i], "ATZ") == 0) {
            [adapter runLoopFor:0.5];
        }
    }

    /* STN detection (for STFAP/STM support) */
    detect_stn(adapter, resp, link, true);

    /* Force protocol: ISO 15765-4 CAN 11-bit 500kbps */
    cli_log("Setting protocol: CAN 11-bit 500k (ATSP6)");
    if (![adapter sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)]) {
        fprintf(stderr, "[ERROR] ATSP6 failed\n");
        return false;
    }

    /* CAN Silent Monitoring — adapter will NOT send ACK on CAN bus. The
       monitor starts only once the adapter answered OK: a timeout or "?"
       (unsupported) leaves the ACK behaviour unknown. */
    cli_log("Enabling silent mode (ATCSM1) — no ACK on CAN bus");
    resp[0] = '\0';
    bool replied = [adapter sendCommand:"ATCSM1" response:resp maxLen:sizeof(resp)];
    if (!replied || !strstr(resp, "OK")) {
        fprintf(stderr, "[ERROR] silent mode not confirmed: ATCSM1 %s%s%s; not starting the monitor\n",
                replied ? "answered \"" : "timed out", replied ? resp : "", replied ? "\"" : "");
        return false;
    }
    cli_log("Silent mode on (ATCSM1: OK)");

    /* Spaces ON for readable output */
    [adapter sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];

    /* Raw CAN formatting: no ISO-TP interpretation */
    [adapter sendCommand:"ATCAF0" response:resp maxLen:sizeof(resp)];
    [adapter sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];

    /* Hardware CAN filters */
    if (cfg->filters) {
        /* Clear existing filters first */
        if (link->stn_detected) {
            [adapter sendCommand:"STFAC" response:resp maxLen:sizeof(resp)];
        }

        for (int i = 0; i < cfg->filter_count; i++) {
            const obd_can_filter_t *f = &cfg->filters[i];
            char filter_cmd[64];
            if (link->stn_detected) {
                /* STN: STFAP pass filter */
                snprintf(filter_cmd, sizeof(filter_cmd), "STFAP %03X,%03X", f->id, f->mask);
            } else if (cfg->apply_elm_filters) {
                /* ELM327: ATCF/ATCM pair (one filter in the chip) */
                snprintf(filter_cmd, sizeof(filter_cmd), "ATCF %03X", f->id);
                [adapter sendCommand:filter_cmd response:resp maxLen:sizeof(resp)];
                snprintf(filter_cmd, sizeof(filter_cmd), "ATCM %03X", f->mask);
            } else {
                continue;
            }
            cli_log("Filter: %s", filter_cmd);
            [adapter sendCommand:filter_cmd response:resp maxLen:sizeof(resp)];
        }
    }

    cli_log("PT-CAN passive mode ready (silent, no ACK)");
    return true;
}

bool obd_init_adapter(OBDAdapter *adapter, const obd_init_config_t *cfg, obd_link_t *link) {
    if (!adapter || !cfg || !link) return false;
    link->stn_detected = false;
    memset(&link->stn_info, 0, sizeof(link->stn_info));

    switch (cfg->profile) {
        case OBD_INIT_NONE:    return true;
        case OBD_INIT_BASIC:   return init_basic(adapter, link);
        case OBD_INIT_PASSIVE: return init_passive(adapter, cfg, link);
        case OBD_INIT_ACTIVE:  return init_active(adapter, link);
    }
    return false;
}

bool obd_open(OBDAdapter *adapter, const char *device_prefix,
              const obd_init_config_t *cfg, obd_link_t *link) {
    if (!obd_connect_device(adapter, device_prefix, link)) return false;
    return obd_init_adapter(adapter, cfg, link);
}
