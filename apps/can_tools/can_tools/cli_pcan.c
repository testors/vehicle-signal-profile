/**
 * @file cli_pcan.c
 * @brief PCAN-USB helpers for the can_tools commands (Pure C)
 */

#include "cli_pcan.h"

#include <stdio.h>
#include <string.h>

#include "cli_util.h"

int cli_pcan_refuse(const char *what) {
    fprintf(stderr, "[ERROR] PCAN is listen-only: %s would transmit\n", what);
    fprintf(stderr, "[ERROR] Use a Bluetooth OBD adapter on the OBD port for that; PCAN only listens.\n");
    return 1;
}

bool cli_pcan_parse_bitrate(const char *value, pcan_bitrate_t *out) {
    if (!pcan_parse_bitrate(value, out)) {
        fprintf(stderr, "[ERROR] Bad --bitrate %s (use auto or one of: %s)\n", value, pcan_bitrate_list());
        return false;
    }
    return true;
}

bool cli_pcan_open(const char *device, const pcan_bitrate_t *bitrate, bool (*cancelled)(void),
                   pcan_channel_t *out, bool *autodetected) {
    char err[768];
    if (autodetected) *autodetected = false;
    if (pcan_parse_device(device) == PCAN_DEVICE_BAD) {
        fprintf(stderr, "[ERROR] %s: PCAN channels are pcan1..pcan8\n", device);
        return false;
    }
    const pcan_api_t *api = pcan_api_load(err, sizeof(err));
    if (!api) {
        fprintf(stderr, "[ERROR] %s\n", err);
        return false;
    }
    cli_log("PCAN library: %s, version \"%s\" (tested with %s)", pcan_api_path(), pcan_api_version(),
            PCAN_TESTED_VERSION);
    int index = pcan_resolve_device(api, device, err, sizeof(err));
    if (!index) {
        fprintf(stderr, "[ERROR] PCAN: %s\n", err);
        return false;
    }

    pcan_bitrate_t want = { .autodetect = false, .bitrate = PCAN_DEFAULT_BITRATE };
    if (bitrate) want = *bitrate;
    int rate = want.bitrate;
    if (want.autodetect) {
        cli_log("pcan%d: detecting the bitrate, listen-only (%.1f s per rate at most)...", index,
                PCAN_AUTOBAUD_ATTEMPT_S);
        pcan_autobaud_t ab;
        pcan_autobaud_begin(&ab, api, index, 0);
        while (!pcan_autobaud_step(&ab, 0.1)) {
            if (cancelled && cancelled()) {
                pcan_autobaud_cancel(&ab);
                break;
            }
        }
        char report[512];
        pcan_autobaud_report(&ab, report, sizeof(report));
        if (!ab.bitrate) {
            fprintf(stderr, "[ERROR] pcan%d: bitrate detection failed: %s (tried: %s)\n", index, ab.error,
                    report[0] ? report : "nothing");
            return false;
        }
        cli_log("pcan%d: bitrate %s detected (tried: %s)", index, pcan_bitrate_name(ab.bitrate), report);
        rate = ab.bitrate;
        if (autodetected) *autodetected = true;
    }

    if (pcan_open(api, index, rate, out, err, sizeof(err)) != PCAN_OPEN_OK) {
        fprintf(stderr, "[ERROR] %s\n", err);
        return false;
    }
    cli_log("Opened %s at %s, listen-only verified (no ACK, no error frames, nothing sent)%s%s",
            out->label, pcan_bitrate_name(out->bitrate), out->firmware[0] ? "; firmware " : "",
            out->firmware);
    return true;
}
