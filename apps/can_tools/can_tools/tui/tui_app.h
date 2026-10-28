/**
 * @file tui_app.h
 * @brief TUI application state, frame fan-out, overlays and key dispatch (C)
 *
 * Single-threaded. The caller pumps the frame source (frames arrive through
 * tui_app_on_frame), then calls tui_app_tick, feeds keys, and renders.
 *
 * Live mode talks to the adapter only through tui_host_t. Safety (step 3
 * brief section 0): the app sends nothing itself; TX mode is the only path
 * to host->tx_setup / tx_query, and it is guarded by tui_app_tx_lock_reason.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "frame_source.h"
#include "tui_grid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Keys (printable ASCII is passed as itself) */
enum {
    TUI_KEY_NONE      = -1,
    TUI_KEY_CTRL_N    = 14,
    TUI_KEY_TAB       = '\t',
    TUI_KEY_ENTER     = '\n',
    TUI_KEY_ESC       = 27,
    TUI_KEY_BACKSPACE = 127,
    TUI_KEY_UP        = 0x1001,
    TUI_KEY_DOWN,
    TUI_KEY_PGUP,
    TUI_KEY_PGDN,
    TUI_KEY_RESIZE,
};

/** A key injected at a time (snapshot / headless runs). */
typedef struct {
    int64_t at_ms;
    int     key;
} tui_timed_key_t;

/** Key name ("enter", "esc", "tab", "space", "up", "down", "pgup", "pgdn",
 *  "backspace", "ctrl-n", or one character) -> key code; TUI_KEY_NONE if
 *  unknown. */
int tui_key_from_name(const char *name);

typedef struct {
    const char *profile_path;    /* NULL: picker (live) / none (replay); "none" = no profile */
    const char *profiles_dir;    /* --profiles, NULL = search */
    const char *exe_path;        /* argv[0], for the profile folder search */
    const char *captures_root;   /* session folders, default "captures" */
    const char *state_dir;       /* folder of tui_last; NULL = never read or write it */
    double      speed;           /* replay speed */
    bool        no_tx;
    bool        no_rec;
    bool        ascii;
    bool        headless;        /* snapshot run: no wall clock, count-based indexing */
    int         colors;          /* terminal colors: 256, 16, 8, or 0 (none); set it */
} tui_app_options_t;

/** Live adapter operations (implemented in tui_live.m). */
typedef struct {
    void       *ctx;
    const char *device_name;
    const char *device_prefix;
    const char *transport;      /* "BLE" / "Classic" */
    const char *stn;            /* STN device id, NULL = plain ELM327 */
    const char *monitor_cmd;    /* "STMA" / "ATMA" */
    bool (*is_connected)(void *ctx);
    /** Monitor already stopped: configure the adapter for OBD requests. */
    bool (*tx_setup)(void *ctx);
    /** One request; response frames into frames[] with *ts_ms. -1 = error. */
    int  (*tx_query)(void *ctx, uint32_t tx_id, uint32_t rx_id, const char *request_bare_hex,
                     can_frame_t *frames, int max_frames, int64_t *ts_ms);
    /** obd_init_adapter(PASSIVE): ATZ ... ATCSM1, receive only. */
    bool (*passive_init)(void *ctx);
} tui_host_t;

typedef struct tui_app tui_app_t;

/** Replay app over src (already created, not started). source_name is shown
 *  in the header. Returns NULL with a message in err. */
tui_app_t *tui_app_create_replay(const tui_app_options_t *opt, frame_source_t *src,
                                 const char *source_name, int cols, int rows,
                                 char *err, size_t err_len);
/** Live app over a connected, PASSIVE-initialized adapter's source (not
 *  started). session_dir exists already (recording, tui.log, discover).
 *  preface: Log lines from the connect (oldest first), logged before the
 *  profile loads. */
tui_app_t *tui_app_create_live(const tui_app_options_t *opt, frame_source_t *src,
                               const tui_host_t *host, const char *session_dir,
                               const char *const *preface, int npreface,
                               int cols, int rows, char *err, size_t err_len);
void tui_app_free(tui_app_t *app);

/** frame_sink_fn for the source (ctx = app). */
void tui_app_on_frame(const can_frame_t *frame, void *ctx);
/** Start the source with the app as its sink (live: the monitor command). */
bool tui_app_start(tui_app_t *app);

void tui_app_tick(tui_app_t *app);
void tui_app_key(tui_app_t *app, int key);
void tui_app_resize(tui_app_t *app, int cols, int rows);
void tui_app_render(tui_app_t *app, tui_grid_t *grid);
bool tui_app_quit_requested(const tui_app_t *app);
/** The capture the user picked in the replay list, NULL when none. The app
 *  quits after setting it (after a y while TX mode or discover runs); the
 *  caller opens that capture as a new replay. */
const char *tui_app_replay_request(const tui_app_t *app);
/** Disconnected and the user pressed c: back to the connect screen. */
bool tui_app_wants_connect_screen(const tui_app_t *app);
/** Before stopping the source: leave TX mode (PASSIVE re-init, rule 0.6),
 *  abort a running discover, close the recording, finish session.json. */
void tui_app_shutdown(tui_app_t *app);
/** Add a message to the Log panel (lines; [INFO]/[WARN]/[ERROR] prefixes). */
void tui_app_log(tui_app_t *app, const char *text);
/** Why TX mode cannot be switched on now, NULL when it can (rule 0.2
 *  except the confirmation). */
const char *tui_app_tx_lock_reason(const tui_app_t *app);
bool tui_app_tx_on(const tui_app_t *app);
/** Live: the monitor command is running (false in TX mode or after a disconnect). */
bool tui_app_monitor_running(const tui_app_t *app);

/** ~/.can_tools/tui_last: "device=", "profile=", "date=" lines. */
bool tui_last_read(const char *state_dir, char *device, size_t dlen, char *profile, size_t plen,
                   char *date, size_t datelen);
bool tui_last_write(const char *state_dir, const char *device, const char *profile);

#ifdef __cplusplus
}
#endif
