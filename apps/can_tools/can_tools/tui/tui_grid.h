/**
 * @file tui_grid.h
 * @brief Cell grid the TUI draws into (Pure C, no ncurses)
 *
 * One cell per terminal column: a UTF-8 character, its display width and a
 * style role. A wide character (width 2) occupies its cell and a continuation
 * cell. The terminal backend maps roles to colors; snapshot mode prints the
 * text and the roles.
 */

#pragma once

#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Style roles; the letter is what --color-map prints. */
typedef enum {
    TUI_ROLE_DEFAULT = '.',
    TUI_ROLE_DIM     = 'd',   /**< secondary text, unchanged bytes, silent rows */
    TUI_ROLE_BRIGHT  = 'b',   /**< recently changed bytes, focused panel title */
    TUI_ROLE_STALE   = 'g',   /**< value older than 3 periods */
    TUI_ROLE_ALERT   = 'r',   /**< REC, TX ON, DISCONNECTED, errors */
    TUI_ROLE_HEADER  = 'h',   /**< header segments */
    TUI_ROLE_WARN    = 'y',   /**< warnings */
    TUI_ROLE_SELECT  = 's',   /**< selection / active tab (reverse) */
} tui_role_t;

typedef struct {
    char    ch[5];     /**< UTF-8, NUL-terminated; "" for a continuation cell */
    unsigned char width; /**< 1, 2 (wide lead) or 0 (continuation) */
    char    role;
    unsigned char fg;  /**< xterm-256 foreground index, 0 = the role's color */
} tui_cell_t;

typedef struct {
    int         cols;
    int         rows;
    tui_cell_t *cells;
} tui_grid_t;

tui_grid_t *tui_grid_new(int cols, int rows);
void tui_grid_free(tui_grid_t *g);
bool tui_grid_resize(tui_grid_t *g, int cols, int rows);
void tui_grid_clear(tui_grid_t *g);
const tui_cell_t *tui_grid_cell(const tui_grid_t *g, int x, int y);

/** Write text at (x, y), clipped at max_cols columns and the grid edge.
 *  A wide character that does not fit becomes a space. Returns columns used. */
int tui_grid_put(tui_grid_t *g, int x, int y, const char *utf8, char role, int max_cols);

/** Repeat one character (UTF-8) n columns. */
void tui_grid_fill(tui_grid_t *g, int x, int y, int n, const char *utf8, char role);

/** Set the role of n cells without changing their text. */
void tui_grid_set_role(tui_grid_t *g, int x, int y, int n, char role);

/** Every write (put / fill) resets the cells' fg to 0. These set it. */
int  tui_grid_put_fg(tui_grid_t *g, int x, int y, const char *utf8, char role, int fg, int max_cols);
void tui_grid_set_fg(tui_grid_t *g, int x, int y, int n, int fg);
/** Some cell has an fg index. */
bool tui_grid_has_fg(const tui_grid_t *g);
/** Two hex digits per cell ("..": none), one line per row. */
void tui_grid_dump_fg(const tui_grid_t *g, FILE *f);

/** Nearest basic color of an xterm-256 index, as an xterm index 1..8:
 *  1 red, 2 green, 3 yellow, 4 blue, 6 cyan, 7 white, 8 gray (dim). The
 *  gauge and timeline palettes map by table, so zone meaning survives
 *  (cold -> blue, ok -> green, hot -> red). */
int  tui_color_basic(int index);
/** Adapt fg indices to the terminal: 256 keep, 8/16 -> tui_color_basic,
 *  0 (no colors) -> cleared. */
void tui_grid_map_colors(tui_grid_t *g, int colors);

/** Print the grid as UTF-8 text, one line per row (full width). */
void tui_grid_dump_text(const tui_grid_t *g, FILE *f);
/** Print the role letters, one line per row. */
void tui_grid_dump_roles(const tui_grid_t *g, FILE *f);

/** Display width of a UTF-8 string (wide East Asian characters count 2). */
int tui_utf8_width(const char *s);
/** Decode one UTF-8 character; returns bytes consumed (>= 1), sets *cp. */
int tui_utf8_decode(const char *s, unsigned int *cp);
/** Display width of a code point: 0 control, 2 wide, else 1. */
int tui_codepoint_width(unsigned int cp);

/** Copy src into dst cut to at most cols display columns. With ellipsis the
 *  last column becomes the ellipsis glyph when the text is cut. */
void tui_text_fit(char *dst, size_t dst_len, const char *src, int cols, bool ellipsis, bool ascii);

/** Glyphs with an ASCII fallback. */
typedef enum {
    TUI_G_H, TUI_G_V, TUI_G_TL, TUI_G_TR, TUI_G_BL, TUI_G_BR,
    TUI_G_T_DOWN, TUI_G_T_UP, TUI_G_T_RIGHT, TUI_G_T_LEFT,
    TUI_G_ARROW,      /* ▶ */
    TUI_G_PROMPT,     /* ▸ */
    TUI_G_REC_ON,     /* ● */
    TUI_G_REC_OFF,    /* ○ */
    TUI_G_ELLIPSIS,   /* … */
    TUI_G_DASH,       /* — */
    TUI_G_FLAT,       /* ━ */
    TUI_G_CHECK,      /* ✔ */
    TUI_G_DOT,        /* · */
    TUI_G_ENTER,      /* ⏎ */
    TUI_G_UPDOWN,     /* ↑↓ */
    TUI_G_MORE_UP,    /* ▲ */
    TUI_G_MORE_DOWN,  /* ▼ */
    TUI_G_BAR_FULL,   /* █ */
    TUI_G_BAR_EMPTY,  /* ░ */
    TUI_G_SKIP,       /* – */
    TUI_G_COUNT
} tui_glyph_t;

const char *tui_glyph(tui_glyph_t g, bool ascii);
/** Sparkline level 0..7 (▁..█, ASCII " _.-~=*#"). */
const char *tui_spark(int level, bool ascii);

#ifdef __cplusplus
}
#endif
