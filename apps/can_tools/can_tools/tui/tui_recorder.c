/**
 * @file tui_recorder.c
 * @brief Write the frame stream to <session>/live.frames.tsv (Pure C)
 */

#include "tui_recorder.h"
#include "frame_tsv.h"

#include <errno.h>
#include <string.h>

void tui_recorder_init(tui_recorder_t *r, const char *dir) {
    memset(r, 0, sizeof(*r));
    snprintf(r->dir, sizeof(r->dir), "%s", dir ? dir : ".");
}

bool tui_recorder_open(tui_recorder_t *r, int64_t now_ms, char *err, size_t err_len) {
    if (r->f) return true;
    int segment = r->segment + 1;
    if (segment == 1) snprintf(r->path, sizeof(r->path), "%s/live.frames.tsv", r->dir);
    else snprintf(r->path, sizeof(r->path), "%s/live_%02d.frames.tsv", r->dir, segment);
    r->f = fopen(r->path, "w");
    if (!r->f) {
        if (err) snprintf(err, err_len, "cannot write %s: %s", r->path, strerror(errno));
        return false;
    }
    r->segment = segment;
    r->frames = 0;
    r->last_flush_ms = now_ms;
    frame_tsv_write_header_ex(r->f, r->us_column);
    return true;
}

void tui_recorder_write(tui_recorder_t *r, const can_frame_t *frame, int64_t now_ms) {
    if (!r->f) return;
    frame_tsv_write_row(r->f, frame);
    r->frames++;
    tui_recorder_tick(r, now_ms);
}

void tui_recorder_tick(tui_recorder_t *r, int64_t now_ms) {
    if (r->f && now_ms - r->last_flush_ms >= TUI_RECORDER_FLUSH_MS) {
        fflush(r->f);
        r->last_flush_ms = now_ms;
    }
}

void tui_recorder_close(tui_recorder_t *r) {
    if (!r->f) return;
    fclose(r->f);
    r->f = NULL;
}

bool tui_recorder_is_open(const tui_recorder_t *r) {
    return r->f != NULL;
}
