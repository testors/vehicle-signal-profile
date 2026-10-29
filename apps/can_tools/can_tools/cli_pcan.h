/**
 * @file cli_pcan.h
 * @brief PCAN-USB helpers for the can_tools commands (Pure C)
 *
 * Device names "pcan" / "pcan1".."pcan8" select a PEAK PCAN-USB. It is
 * receive-only in this tool: commands that transmit refuse it with
 * cli_pcan_refuse() before any adapter object is created or the library is
 * loaded; the receive commands open it with cli_pcan_open() (listen-only,
 * verified, see common/pcan.h).
 */

#pragma once

#include <stdbool.h>

#include "pcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/** "[ERROR] PCAN is listen-only: <what> would transmit" on stderr; returns 1. */
int  cli_pcan_refuse(const char *what);

/** Parse a --bitrate value (NULL = the 500k default); logs and returns false on a bad one. */
bool cli_pcan_parse_bitrate(const char *value, pcan_bitrate_t *out);

/**
 * Load the library, pick the channel for device, detect (bitrate->autodetect)
 * or take the bitrate, and open the channel listen-only. Progress and errors
 * go to stderr. cancelled (may be NULL) is asked while detecting (Ctrl-C).
 * *autodetected (may be NULL) tells whether the bitrate was detected.
 */
bool cli_pcan_open(const char *device, const pcan_bitrate_t *bitrate, bool (*cancelled)(void),
                   pcan_channel_t *out, bool *autodetected);

#ifdef __cplusplus
}
#endif
