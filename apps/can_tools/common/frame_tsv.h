/**
 * @file frame_tsv.h
 * @brief Capture dump (*.frames.tsv) reader (Pure C)
 *
 * Reads the per-phase frame dump that the discover session writes:
 *
 *   timestamp_ms \t can_id \t is_extended \t dlc \t data_hex
 *   1712560212345\t0x0A5\t0\t8\t0011223344556677
 *
 * Hardware-stamped frames (PCAN) add a 6th column, the full epoch time in
 * microseconds; column 1 stays floor(timestamp_us / 1000), so a reader of the
 * first 5 columns sees the same capture:
 *
 *   timestamp_ms \t can_id \t is_extended \t dlc \t data_hex \t timestamp_us
 *   1712560212345\t0x0A5\t0\t8\t0011223344556677\t1712560212345678
 *
 * Shared by the discover session (writer), replay, the replay frame source and
 * the TUI recorder.
 */

#pragma once

#include <stdbool.h>
#include <stdio.h>
#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_TSV_LINE_MAX          1024
#define FRAME_TSV_MAX_ROW_WARNINGS  5

/** Growable frame list filled by frame_tsv_load(). */
typedef struct {
    can_frame_t *frames;
    int          count;
    int          capacity;
    int          malformed;     /**< Rows rejected by the parser */
} frame_list_t;

/** Append a frame (doubles the capacity on demand). False if out of memory. */
bool frame_list_push(frame_list_t *list, const can_frame_t *frame);

/** Write the column header line (timestamp_ms, can_id, is_extended, dlc, data_hex). */
void frame_tsv_write_header(FILE *f);

/** Header line; with_us adds the 6th name, timestamp_us (hardware-stamped source). */
void frame_tsv_write_header_ex(FILE *f, bool with_us);

/** Write one data row: "1712560212345\t0xA5\t0\t8\t0011223344556677\n".
 *  A hw_timestamp frame gets the 6th column: "...\t1712560212345678\n". */
void frame_tsv_write_row(FILE *f, const can_frame_t *frame);

/** Free the frames and zero the list. */
void frame_list_free(frame_list_t *list);

/**
 * Parse one data row: timestamp_ms \t 0xID \t is_extended \t dlc \t data_hex
 * [\t timestamp_us] (the format disc_write_phase_dump() writes). The optional
 * 6th column must fall within the millisecond of column 1; it sets
 * timestamp_us and hw_timestamp. The line is modified in place.
 */
bool frame_tsv_parse_row(char *line, can_frame_t *out);

/**
 * Load a phase dump. Returns 1 if loaded, 0 if the file does not exist,
 * -1 on I/O or allocation error. Malformed rows are skipped and counted in
 * list->malformed; the first FRAME_TSV_MAX_ROW_WARNINGS are reported on
 * stderr as "<display_name>:<line>".
 */
int frame_tsv_load(const char *path, const char *display_name, frame_list_t *list);

#ifdef __cplusplus
}
#endif
