/**
 * @file tui_render.h
 * @brief Layout and drawing of the TUI dashboard into a cell grid (Pure C)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tui_gauges.h"
#include "tui_grid.h"
#include "tui_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TUI_PANEL_GAUGES = 0,
    TUI_PANEL_SIGNALS,
    TUI_PANEL_RAW,
    TUI_PANEL_LOG,
    TUI_PANEL_COUNT
} tui_panel_t;

#define TUI_GAUGES_HEIGHT 5

typedef struct {
    int x, y, w, h;
} tui_rect_t;

typedef struct {
    bool       narrow;                    /* < 80 cols or < 24 rows: tabs */
    tui_rect_t body;                      /* inside the outer frame */
    tui_rect_t tabs;                      /* narrow only */
    tui_rect_t panel[TUI_PANEL_COUNT];
    bool       visible[TUI_PANEL_COUNT];
} tui_layout_t;

/** has_signals: a profile is loaded (Signals panel exists); has_gauges: the
 *  Gauges panel exists (5 rows above Signals). In narrow mode only `tab` is
 *  visible; a split that would leave Signals with fewer than 3 data rows
 *  (or all of them, when there are fewer) also falls back to tabs. */
void tui_layout_compute(tui_layout_t *L, int cols, int rows, bool has_signals,
                        int signal_count, bool has_gauges, tui_panel_t tab);
/** Rows of content a panel shows (inside borders, below its column header). */
int  tui_layout_content_rows(const tui_layout_t *L, tui_panel_t p);

/** Everything the renderer needs besides the model; filled by the app. */
typedef struct {
    int         cols, rows;
    bool        ascii;
    int64_t     now_ms;

    /* header */
    const char *device;          /* adapter name, or the replay name */
    bool        live;            /* adapter connected (shows "silent" / "TX ON") */
    const char *live_mode;       /* replaces "silent" (PCAN: "listen-only 500k"), NULL = "silent" */
    bool        tx_on;
    bool        disconnected;
    bool        replay;
    double      speed;
    bool        paused;
    bool        ended;
    int64_t     replay_pos_ms;
    int64_t     replay_total_ms;
    bool        recording;
    const char *profile;         /* short name, NULL = none */

    /* body */
    tui_panel_t focus;
    tui_panel_t tab;             /* narrow mode */
    int         scroll[TUI_PANEL_COUNT];
    tui_sort_t  sort;
    const char *filter;          /* "" = none */
    bool        filter_editing;
    const char *tx_hint;         /* history text for never-polled diagnostic signals */

    /* footer */
    bool        rec_available;   /* live: r toggles recording */
    const char *tx_footer;       /* x hint text ("TX mode [locked: DIRECT]"), NULL = none */
    bool        monitor_paused;  /* TX mode: Raw CAN title says so */

    /* gauges and colors */
    const tui_gauges_t *gauges;  /* NULL = no Gauges panel */
    int         colors;          /* 256, 8 (basic), 0 (none: bar tracks "·") */
} tui_view_t;

/** Key hint for footers: "key desc" (role 0 = default for desc). */
typedef struct {
    const char *key;
    const char *desc;
    char        role;
} tui_hint_t;

/* Drawing helpers shared with tui_screens.c */
int  tui_draw_text(tui_grid_t *g, int x, int y, const char *s, char role, int max_cols);
void tui_draw_fit(tui_grid_t *g, int x, int y, const char *s, char role, int w, bool ascii);
void tui_draw_right(tui_grid_t *g, int x, int y, int w, const char *s, char role, bool ascii);
void tui_draw_hline(tui_grid_t *g, int x, int y, int n, bool ascii, char role);
void tui_draw_box(tui_grid_t *g, tui_rect_t r, const char *title, const char *right,
                  char title_role, bool ascii);
void tui_draw_clip_marks(tui_grid_t *g, tui_rect_t r, int above, int below, bool ascii);
/** Whole hints only, separated by two spaces; returns columns used. */
int  tui_draw_hints(tui_grid_t *g, int x, int y, int limit, const tui_hint_t *hints, int n, bool ascii);

/** Draw the dashboard. Clamps v->scroll to the visible content. */
void tui_render_dashboard(tui_grid_t *g, const tui_model_t *m, tui_view_t *v);

/** Signals that pass the filter (name substring, case-insensitive). */
bool tui_signal_matches(const tui_signal_t *s, const char *filter);

#ifdef __cplusplus
}
#endif
