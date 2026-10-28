/**
 * @file tui_capture.h
 * @brief Redirect stderr into a file while the TUI owns the terminal, and
 *        read it back line by line for the Log panel (C)
 *
 * A file, not a pipe: a pipe can fill up during a blocking AT command and
 * deadlock the single thread. Before a session folder exists the file is a
 * temporary one; tui_capture_switch moves to <session>/tui.log (the earlier
 * content is copied over).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool   active;
    int    saved_fd;        /* the original fd 2 */
    int    read_fd;
    bool   temp;            /* path is a temporary file (unlinked at the end) */
    char   path[1024];
    char   partial[2048];   /* incomplete last line */
    size_t plen;
} tui_capture_t;

typedef void (*tui_capture_line_fn)(const char *line, void *ctx);

/** fd 2 -> a new temporary file. False (and nothing changed) on failure. */
bool tui_capture_begin(tui_capture_t *c);
/** fd 2 -> path (created / appended); a temporary file's content is copied
 *  first and the temporary file removed. Lines already read stay read. */
bool tui_capture_switch(tui_capture_t *c, const char *path);
/** Back to a fresh temporary file (after a session closed). */
bool tui_capture_switch_temp(tui_capture_t *c);
/** Deliver complete new lines (NSLog prefixes stripped). Returns the count. */
int  tui_capture_poll(tui_capture_t *c, tui_capture_line_fn fn, void *ctx);
/** Restore fd 2 (idempotent; safe from atexit). */
void tui_capture_end(tui_capture_t *c);

/** "2026-10-28 03:01:02.123 can_tools[12:34] text" -> "text" (in place). */
void tui_capture_strip_nslog(char *line);

#ifdef __cplusplus
}
#endif
