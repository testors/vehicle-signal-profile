/**
 * @file frame_source_pcan.h
 * @brief Frame source over an open, listen-only PCAN channel (Pure C)
 *
 * start: drain and drop what queued up before the start, then deliver
 * pump:  drain CAN_Read until the queue is empty, select() on the receive
 *        event up to `seconds`, drain again
 * open:  false after repeated read errors (device unplugged)
 * stop:  stop delivering (the channel stays open; frames queued meanwhile
 *        are dropped at the next start)
 * destroy: logs the clock drift, CAN_Uninitialize
 *
 * Frames carry controller timestamps mapped to the wall clock (see
 * pcan_clock_t): timestamp_ms / timestamp_us / hw_timestamp. RTR, status,
 * error and FD/echo messages are not delivered but counted; queue overruns
 * and read errors are counted and logged (rate-limited) on stderr.
 * This source never transmits: it only calls CAN_Read, CAN_GetStatus and
 * CAN_Uninitialize.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "frame_source.h"
#include "pcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Read errors in a row that count as an unplugged device. */
#define FRAME_SOURCE_PCAN_DEAD_READS 3

/** Takes over the open channel *ch (ch->open is false afterwards; the source
 *  closes it). NULL if ch is not open or on allocation failure (ch is then
 *  left as it was). */
frame_source_t *frame_source_pcan_create(pcan_channel_t *ch);

typedef struct {
    pcan_counts_t counts;        /* every message read while running */
    uint64_t      delivered;     /* data frames given to the sink */
    uint64_t      dropped_at_start; /* messages queued before a start */
    uint64_t      overruns;      /* reads that reported OVERRUN / QOVERRUN */
    uint64_t      read_errors;
    uint32_t      bus_status;    /* last PCAN_ERROR_* bus bits seen */
    bool          anchored;
    bool          have_drift;
    int64_t       drift_us;      /* host minus mapped time, relative to the anchor */
    double        drift_after_s; /* seconds since anchoring when the drift was measured */
    int           reanchors;
} frame_source_pcan_stats_t;

/** True for sources made by frame_source_pcan_create(). */
bool frame_source_is_pcan(const frame_source_t *src);
/** The channel (label, bitrate, ...); NULL for other sources. */
const pcan_channel_t *frame_source_pcan_channel(const frame_source_t *src);
bool frame_source_pcan_stats(const frame_source_t *src, frame_source_pcan_stats_t *out);

#ifdef __cplusplus
}
#endif
