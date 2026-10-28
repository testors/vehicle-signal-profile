/**
 * @file tui_live.h
 * @brief Live TUI: connect screen, adapter connect + PASSIVE init, the live
 *        dashboard loop and the TX host (Objective-C, MRC)
 *
 * Safety (step 3 brief section 0): connects only on an explicit Enter or an
 * explicit non-empty --device; initializes with OBD_INIT_PASSIVE only (no
 * ATSP0, no OBD_INIT_ACTIVE); the only transmitting calls are the TX host
 * callbacks, which tui_app reaches only through its TX guard.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tui_app.h"
#include "tui_capture.h"

typedef struct {
    const char *device;          /* non-empty --device, or NULL (connect screen) */
    const char *profile;         /* --profile path, "none", or NULL (picker) */
    const char *profiles_dir;
    const char *captures_root;
    const char *state_dir;       /* ~/.can_tools */
    const char *exe_path;
    bool        no_tx;
    bool        no_rec;
    bool        ascii;
    int         colors;          /* 256 / 16 / 8 / 0 */
    tui_capture_t *capture;      /* active stderr capture, or NULL */
    volatile int  *stop;         /* set by SIGINT/SIGTERM/SIGHUP */

    /* headless test hook (CAN_TOOLS_TUI_HEADLESS=1 with --snapshot-at):
       wall-clock times from the start, no terminal */
    bool                   headless;
    const int64_t         *snap_times;
    int                    nsnap;
    const tui_timed_key_t *keys;
    int                    nkeys;
    int                    cols, rows;
    bool                   color_map;
} tui_live_options_t;

/** Run until quit. If the user opened a capture (o), its path is written to
 *  replay_out and 3 is returned; else 0 (or 1 on a setup error). */
int tui_live_run(const tui_live_options_t *o, char *replay_out, size_t replay_len);
