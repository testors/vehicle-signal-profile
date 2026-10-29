/**
 * @file tui_replay.h
 * @brief Load a capture for TUI replay (Pure C)
 *
 * PATH is a *.frames.tsv file or a session folder: its live*.frames.tsv files
 * in name order if any of them has frames, else its NN_*.frames.tsv files in
 * name order (a file without a frame row in its first lines is skipped).
 * Consecutive files are concatenated; a gap of more than 1 s between files is
 * shortened to 1 s by shifting the later timestamps.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "frame_tsv.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUI_REPLAY_MAX_GAP_MS 1000

typedef struct {
    frame_list_t frames;      /* concatenated, owned */
    int          files;
    int          malformed;
    char         name[256];   /* file or folder name for the header */
} tui_replay_data_t;

/** 0 on success; -1 with a message in err. */
int  tui_replay_load(const char *path, tui_replay_data_t *out, char *err, size_t err_len);
void tui_replay_free(tui_replay_data_t *data);
/** True when PATH would replay at least one frame (the list's check: reads
 *  only the first lines of each file); false with the reason in err. */
bool tui_replay_has_frames(const char *path, char *err, size_t err_len);
/** The "profile" path the live TUI recorded in session.json: PATH's own
 *  session.json when PATH is a folder, else the one next to the file. False
 *  when there is none, it cannot be parsed, or the profile is null. */
bool tui_replay_session_profile(const char *path, char *out, size_t len);

#define TUI_REPLAY_LIST_MAX 512

/** One replayable capture: a session folder with live*.frames.tsv or
 *  NN_*.frames.tsv dumps, or a *.frames.tsv file directly under the captures
 *  root. */
typedef struct {
    char    path[1024];       /* full path to pass to tui_replay_load */
    char    name[256];        /* folder or file name */
    char    when[32];         /* "YYYY-MM-DD HH:MM" */
    char    size[16];         /* human-readable size of the replayed files */
    int     files;            /* frame files that will be replayed */
    int64_t mtime;            /* newest first */
} tui_replay_entry_t;

typedef struct {
    tui_replay_entry_t *entries;
    int                 count;
    int                 cap;
    char                root[512];
} tui_replay_list_t;

/** Scan root for replayable captures (at least one frame), the newest
 *  TUI_REPLAY_LIST_MAX, newest first. The list must be zero-initialized (or
 *  freed first). 0 on success (count may be 0); -1 with a message in err
 *  when root cannot be read. */
int  tui_replay_list_scan(tui_replay_list_t *list, const char *root, char *err, size_t err_len);
void tui_replay_list_free(tui_replay_list_t *list);

/** Append src to dst, shifting src so that the gap between dst's last frame
 *  and src's first frame is at most max_gap_ms. Returns the shift applied. */
int64_t tui_replay_append(frame_list_t *dst, const frame_list_t *src, int64_t max_gap_ms);

#ifdef __cplusplus
}
#endif
