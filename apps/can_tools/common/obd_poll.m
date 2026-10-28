/**
 * @file obd_poll.m
 * @brief Poll-mode OBD requests (moved from vsp_probe's poll phase; MRC)
 */

#import "obd_poll.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "obd_connect.h"

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int extract_line_hex(const char *line, char *hexOut, size_t hexOutLen) {
    size_t j = 0;
    if (!line || !hexOut || hexOutLen == 0) return 0;
    for (size_t i = 0; line[i] && j + 1 < hexOutLen; i++) {
        if (hex_value(line[i]) >= 0) hexOut[j++] = (char)toupper((unsigned char)line[i]);
    }
    hexOut[j] = '\0';
    return (int)j;
}

static bool parse_hex_byte_c(const char *hex, uint8_t *out) {
    int hi = hex_value(hex[0]);
    int lo = hex_value(hex[1]);
    if (hi < 0 || lo < 0) return false;
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static void send_best_effort(OBDAdapter *adapter, const char *cmd) {
    char resp[512] = "";
    [adapter sendCommand:cmd response:resp maxLen:sizeof(resp)];
}

bool obd_poll_setup(OBDAdapter *adapter) {
    /* Reset + STN detection (common/obd_connect BASIC profile) */
    obd_init_config_t cfg = { .profile = OBD_INIT_BASIC };
    obd_link_t link;
    obd_link_init(&link);
    obd_init_adapter(adapter, &cfg, &link);

    send_best_effort(adapter, "ATSP6");
    send_best_effort(adapter, "ATCSM0");
    send_best_effort(adapter, "ATAT0");
    send_best_effort(adapter, "ATST 10");
    send_best_effort(adapter, "ATAL");
    send_best_effort(adapter, "ATS1");
    send_best_effort(adapter, "ATH1");
    send_best_effort(adapter, "ATCAF1");
    send_best_effort(adapter, "ATCFC1");
    send_best_effort(adapter, "ATFCSD300001");
    send_best_effort(adapter, "ATFCSM1");
    return true;
}

bool obd_poll_parse_response_line(const char *line, can_frame_t *frame) {
    char hex[256];
    int hexLen = extract_line_hex(line, hex, sizeof(hex));
    if (hexLen < 5) return false;
    unsigned int idVal = 0;
    if (sscanf(hex, "%3x", &idVal) != 1) return false;
    frame->can_id = idVal;
    frame->dlc = 0;
    for (int i = 3; i + 1 < hexLen && frame->dlc < 8; i += 2) {
        if (!parse_hex_byte_c(hex + i, &frame->data[frame->dlc])) break;
        frame->dlc++;
    }
    return frame->dlc > 0;
}

int obd_poll_query(OBDAdapter *adapter, uint32_t tx_id, uint32_t rx_id,
                   const char *request_bare_hex, double timeout_s,
                   can_frame_t *frames, int max_frames) {
    char cmd[64];
    char resp[4096] = "";

    snprintf(cmd, sizeof(cmd), "ATSH%03X", tx_id);
    send_best_effort(adapter, cmd);
    snprintf(cmd, sizeof(cmd), "ATFCSH%03X", tx_id);
    send_best_effort(adapter, cmd);
    if (rx_id != 0) {
        snprintf(cmd, sizeof(cmd), "ATCRA%03X", rx_id);
        send_best_effort(adapter, cmd);
    }

    BOOL ok = [adapter sendCommand:request_bare_hex
                          response:resp
                            maxLen:sizeof(resp)
                           timeout:timeout_s];
    if (!ok || elm327_is_error_response(resp)) return -1;

    /* With ATH1 every response line is one CAN frame "ID PCI DATA..." */
    int n = 0;
    char buf[sizeof(resp)];
    snprintf(buf, sizeof(buf), "%s", resp);
    for (char *line = strtok(buf, "\r\n"); line && n < max_frames; line = strtok(NULL, "\r\n")) {
        can_frame_t frame = {0};
        if (!obd_poll_parse_response_line(line, &frame)) continue;
        frames[n++] = frame;
    }
    return n;
}
