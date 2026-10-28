/**
 * @file tui_term.h
 * @brief ncurses backend: draws a tui_grid_t, maps style roles to colors,
 *        reads keys (C)
 */

#pragma once

#include <stdbool.h>

#include "tui_grid.h"

#ifdef __cplusplus
extern "C" {
#endif

/** setlocale, initscr, colors. ascii is set when --ascii was given or the
 *  locale is not UTF-8. mono: --mono or NO_COLOR or no color support. */
bool tui_term_open(bool force_mono, bool force_ascii, bool *ascii_out);
/** Restore the terminal (idempotent, safe from atexit). */
void tui_term_close(void);
bool tui_term_is_open(void);
/** 256, 16, 8, or 0 (mono / NO_COLOR / no color support). */
int  tui_term_colors(void);
void tui_term_size(int *cols, int *rows);
void tui_term_draw(const tui_grid_t *g);
/** Next key (TUI_KEY_* / ASCII) or TUI_KEY_NONE; never blocks. */
int  tui_term_key(void);

#ifdef __cplusplus
}
#endif
