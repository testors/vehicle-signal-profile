/**
 * @file tui_screens.h
 * @brief Modal overlays and the connect screen (Pure C, draws into a grid)
 *
 * Each screen takes a plain view struct the app fills; nothing here owns
 * state beyond the scroll positions passed in.
 */

#pragma once

#include <stdbool.h>

#include "tui_grid.h"
#include "tui_render.h"
#include "tui_replay.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Centered rect of at most want_w x want_h inside cols x rows (keeps a
 *  2-column, 1-row margin when the screen is large enough). */
tui_rect_t tui_modal_rect(int cols, int rows, int want_w, int want_h);
/** Every cell dim (the dashboard behind a modal keeps updating). */
void tui_dim_grid(tui_grid_t *g);
/** Clear r, draw its frame with title / right text, a separator above the
 *  last inner row and the hints on it. Returns the content rect (inside the
 *  frame, above the separator). */
tui_rect_t tui_draw_modal(tui_grid_t *g, tui_rect_t r, const char *title, const char *right,
                          const tui_hint_t *hints, int nhints, bool ascii);
/** Wrap text into at most max_lines lines of width cols (UTF-8, at spaces).
 *  Writes NUL-terminated lines of up to line_len bytes; returns the count. */
int tui_wrap_text(const char *text, int cols, char lines[][256], int max_lines);

/*******************************************************************************
 * Discover overlay
 ******************************************************************************/

typedef enum {
    TUI_PHASE_PENDING = 0,
    TUI_PHASE_ACTIVE,
    TUI_PHASE_DONE,
    TUI_PHASE_SKIPPED,
} tui_phase_state_t;

typedef struct {
    const char       *name;
    int               duration_s;
    tui_phase_state_t state;
    bool              next;         /* the phase Enter starts */
    bool              skippable;
    double            elapsed_s;    /* active */
    int               frames;       /* active / done */
    int               ids;          /* done */
    const char       *note;         /* e.g. "bus: DIRECT" on the baseline */
} tui_discover_row_t;

typedef struct {
    int                count;
    tui_discover_row_t rows[8];
    int                current;          /* phase named in the title, -1 = none */
    const char        *instruction;      /* of the active / next phase */
    const char        *warning;
    const char        *live_text;        /* "live" line, NULL = none */
    const char        *live_spark;       /* sparkline after live_text, NULL = none */
    const char        *status;           /* replaces the live line, e.g. "analyzing…" */
    bool               session_active;   /* esc does not close */
    bool               capturing;
} tui_discover_view_t;

void tui_draw_discover(tui_grid_t *g, int cols, int rows, const tui_discover_view_t *v, bool ascii);

/*******************************************************************************
 * Discover result
 ******************************************************************************/

typedef struct {
    char signal[16];
    char id[16];
    char layout[32];
    char hz[16];
    char conf[16];
    char range[40];
    char prior[24];
    bool found;
    char missing[24];   /* "skipped", "not found" */
} tui_result_row_t;

typedef struct {
    const char      *dir;          /* title: session folder */
    tui_result_row_t rows[8];
    int              count;
    char             bus[32];      /* "bus: DIRECT" */
    char             written[400]; /* "written: a · b · c" */
    bool             can_load;     /* a draft VSP exists */
} tui_result_view_t;

void tui_draw_result(tui_grid_t *g, int cols, int rows, const tui_result_view_t *v, bool ascii);

/*******************************************************************************
 * Text view (result.json, help) and confirm modal
 ******************************************************************************/

/** Scrollable lines; *scroll is clamped. want_w = preferred box width. */
void tui_draw_text_view(tui_grid_t *g, int cols, int rows, const char *title, const char *right,
                        const char *const *lines, int nlines, int *scroll, int want_w,
                        const tui_hint_t *hints, int nhints, bool ascii);

/** Small modal sized to its lines. role applies to every line. */
void tui_draw_confirm(tui_grid_t *g, int cols, int rows, const char *title,
                      const char *const *lines, int nlines, char role,
                      const tui_hint_t *hints, int nhints, bool ascii);

/*******************************************************************************
 * Profile picker
 ******************************************************************************/

typedef struct {
    const char *make;
    const char *model;
    const char *gen;
    const char *years;      /* "2019–", "2009–2014", "—" */
    const char *file;       /* file name (profiles) or path (drafts) */
    int         signals;
    int         can;
    int         obd;
    bool        current;    /* the loaded profile */
    int         suggest;    /* rank among the profiles the traffic matches (1..5), 0 = none */
} tui_picker_row_t;

typedef struct {
    const char             *filter;
    int                     total;      /* profiles found */
    int                     indexed;
    bool                    indexing;
    const char             *dir;        /* profile directory, NULL = none found */
    const tui_picker_row_t *rows;       /* filtered profiles */
    int                     nrows;
    const tui_picker_row_t *drafts;
    int                     ndrafts;
    int                     selected;   /* rows first, then drafts */
    int                    *scroll;
    bool                    obd_dim;    /* DIRECT bus: obd signals cannot be live */
} tui_picker_view_t;

void tui_draw_picker(tui_grid_t *g, int cols, int rows, const tui_picker_view_t *v, bool ascii);
/** Profile rows the picker list shows at this size (for paging). */
int  tui_picker_list_rows(int cols, int rows, int nrows, int ndrafts);

/*******************************************************************************
 * Replay list
 ******************************************************************************/

typedef struct {
    const tui_replay_entry_t *rows;
    int                       nrows;
    int                       selected;
    int                      *scroll;
    const char               *root;      /* captures folder, for the empty message */
} tui_replay_view_t;

void tui_draw_replay_list(tui_grid_t *g, int cols, int rows, const tui_replay_view_t *v, bool ascii);
/** Capture rows the replay list shows at this size (for paging). */
int  tui_replay_list_rows(int cols, int rows);

/*******************************************************************************
 * Connect screen (full screen)
 ******************************************************************************/

typedef struct {
    const char *name;
    const char *transport;   /* "BLE" / "Classic" */
    char        detail[24];  /* "-58 dBm" / "paired" */
} tui_connect_row_t;

typedef struct {
    const char              *bt_state;      /* right of the title: "BT ready", "BT off", "BT …" */
    const char              *bt_note;       /* e.g. "Bluetooth off — paired Classic adapters only" */
    const tui_connect_row_t *rows;
    int                      nrows;
    int                      selected;      /* 0..nrows-1, nrows = the "last:" line */
    const char              *last_text;     /* NULL = no tui_last */
    const char              *status;        /* "scanning… 3 s", "connecting…", errors */
    char                     status_role;
    const char              *prompt_label;  /* o: "capture path: ", NULL = no prompt */
    const char              *prompt_text;
    bool                     busy;          /* connecting / initializing */
} tui_connect_view_t;

void tui_draw_connect(tui_grid_t *g, int cols, int rows, const tui_connect_view_t *v, bool ascii);

#ifdef __cplusplus
}
#endif
