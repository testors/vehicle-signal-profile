/**
 * @file tui_recorder.h
 * @brief Write the frame stream to <session>/live.frames.tsv (Pure C)
 *
 * Same row format as the discover phase dumps (frame_tsv_write_row), so a
 * recording replays with frame_tsv_load / can_tools tui --replay. Each
 * re-enable starts a new segment: live.frames.tsv, live_02.frames.tsv, ...
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUI_RECORDER_FLUSH_MS 1000

typedef struct {
    FILE    *f;
    char     dir[512];
    char     path[640];
    int      segment;        /* segments opened so far */
    int64_t  last_flush_ms;
    uint64_t frames;         /* frames in the open segment */
} tui_recorder_t;

void tui_recorder_init(tui_recorder_t *r, const char *dir);
/** Open the next segment and write the header; false with a message in err. */
bool tui_recorder_open(tui_recorder_t *r, int64_t now_ms, char *err, size_t err_len);
void tui_recorder_write(tui_recorder_t *r, const can_frame_t *frame, int64_t now_ms);
/** Flush when TUI_RECORDER_FLUSH_MS passed since the last flush. */
void tui_recorder_tick(tui_recorder_t *r, int64_t now_ms);
void tui_recorder_close(tui_recorder_t *r);
bool tui_recorder_is_open(const tui_recorder_t *r);

#ifdef __cplusplus
}
#endif
