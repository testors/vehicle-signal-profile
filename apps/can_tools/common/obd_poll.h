/**
 * @file obd_poll.h
 * @brief Poll-mode OBD requests over an ELM327/STN adapter (moved from
 *        vsp_probe so the TUI's TX mode uses the same code)
 *
 * TRANSMITS: obd_poll_setup turns silent monitoring off (ATCSM0) and every
 * obd_poll_query sends a request on the bus. Only for the car's OBD port.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#import "obd_adapter.h"
#include "elm327.h"

/** BASIC init (ATZ, ATE0, ATL0, ATS1, STI), then ATSP6, ATCSM0, ATAT0,
 *  ATST 10, ATAL, ATS1, ATH1, ATCAF1, ATCFC1, ATFCSD300001, ATFCSM1. */
bool obd_poll_setup(OBDAdapter *adapter);

/** One ELM327 response line with headers on: 3 hex digits of 11-bit ID,
 *  then the data bytes. */
bool obd_poll_parse_response_line(const char *line, can_frame_t *frame);

/** ATSH<tx>, ATFCSH<tx>, ATCRA<rx> (rx != 0), then the request. Returns -1
 *  when the adapter did not answer or answered with an error, else the
 *  number of response frames written to frames[] (at most max_frames;
 *  timestamps left 0). */
int obd_poll_query(OBDAdapter *adapter, uint32_t tx_id, uint32_t rx_id,
                   const char *request_bare_hex, double timeout_s,
                   can_frame_t *frames, int max_frames);
