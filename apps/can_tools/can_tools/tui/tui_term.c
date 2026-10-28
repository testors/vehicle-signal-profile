/**
 * @file tui_term.c
 * @brief ncurses backend (C)
 */

#define _XOPEN_SOURCE_EXTENDED 1

#include "tui_term.h"
#include "tui_app.h"

#include <curses.h>
#include <term.h>
#include <langinfo.h>
#include <locale.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

static bool g_open = false;
static bool g_mono = false;

enum { PAIR_DIM = 1, PAIR_BRIGHT, PAIR_STALE, PAIR_ALERT, PAIR_HEADER, PAIR_WARN, PAIR_FG_FIRST = 16 };

/* Per-cell fg colors: one pair per used index, allocated on first use. */
static short g_fg_pair[256];
static int   g_next_pair = PAIR_FG_FIRST;

static short fg_pair(int fg) {
    if (fg <= 0 || fg > 255) return 0;
    if (g_fg_pair[fg]) return g_fg_pair[fg] > 0 ? g_fg_pair[fg] : 0;
    if (g_next_pair >= COLOR_PAIRS || g_next_pair > 32767) {
        g_fg_pair[fg] = -1;            /* out of pairs: the role color */
        return 0;
    }
    int color = fg;
    if (COLORS < 256 && fg == 8) color = COLORS >= 16 ? 8 : COLOR_WHITE;   /* gray */
    if (init_pair((short)g_next_pair, (short)color, -1) == ERR) {
        g_fg_pair[fg] = -1;
        return 0;
    }
    g_fg_pair[fg] = (short)g_next_pair++;
    return g_fg_pair[fg];
}

static attr_t role_attr(char role, short *pair) {
    *pair = 0;
    if (g_mono) {
        switch (role) {
            case TUI_ROLE_DIM:    return A_DIM;
            case TUI_ROLE_BRIGHT: return A_BOLD;
            case TUI_ROLE_STALE:  return A_DIM;
            case TUI_ROLE_ALERT:  return A_BOLD | A_UNDERLINE;
            case TUI_ROLE_HEADER: return A_BOLD;
            case TUI_ROLE_WARN:   return A_BOLD;
            case TUI_ROLE_SELECT: return A_REVERSE;
            default:              return A_NORMAL;
        }
    }
    switch (role) {
        case TUI_ROLE_DIM:    *pair = PAIR_DIM;    return COLORS >= 256 ? A_NORMAL : A_DIM;
        case TUI_ROLE_BRIGHT: *pair = PAIR_BRIGHT; return A_BOLD;
        case TUI_ROLE_STALE:  *pair = PAIR_STALE;  return COLORS >= 256 ? A_NORMAL : A_BOLD;
        case TUI_ROLE_ALERT:  *pair = PAIR_ALERT;  return A_BOLD;
        case TUI_ROLE_HEADER: *pair = PAIR_HEADER; return A_BOLD;
        case TUI_ROLE_WARN:   *pair = PAIR_WARN;   return A_NORMAL;
        case TUI_ROLE_SELECT: return A_REVERSE;
        default:              return A_NORMAL;
    }
}

static void setup_colors(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_DIM, 245, -1);
        init_pair(PAIR_BRIGHT, 231, -1);
        init_pair(PAIR_STALE, 240, -1);
        init_pair(PAIR_ALERT, 196, -1);
        init_pair(PAIR_HEADER, 117, -1);
        init_pair(PAIR_WARN, 214, -1);
    } else {
        init_pair(PAIR_DIM, COLOR_WHITE, -1);
        init_pair(PAIR_BRIGHT, COLOR_WHITE, -1);
        init_pair(PAIR_STALE, COLOR_BLACK, -1);
        init_pair(PAIR_ALERT, COLOR_RED, -1);
        init_pair(PAIR_HEADER, COLOR_CYAN, -1);
        init_pair(PAIR_WARN, COLOR_YELLOW, -1);
    }
}

bool tui_term_open(bool force_mono, bool force_ascii, bool *ascii_out) {
    setlocale(LC_ALL, "");
    const char *cs = nl_langinfo(CODESET);
    bool utf8 = cs && (strstr(cs, "UTF-8") || strstr(cs, "utf8") || strstr(cs, "UTF8"));
    if (ascii_out) *ascii_out = force_ascii || !utf8;
    if (!initscr()) return false;
    g_open = true;
    /* macOS ncurses 6.0 sends runs of one character with the terminfo "rep"
       capability, even for multibyte characters. Terminals whose rep starts
       with %p1%c (xterm-ghostty: "%p1%c\E[%p2%{1}%-%db") then get an empty
       string for U+2500 etc.: the run is dropped and the cursor does not
       advance. Without repeat_char ncurses writes every character. */
    repeat_char = NULL;
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    set_escdelay(25);
    const char *no_color = getenv("NO_COLOR");
    g_mono = force_mono || (no_color && no_color[0]) || !has_colors();
    if (!g_mono) setup_colors();
    return true;
}

void tui_term_close(void) {
    if (!g_open) return;
    g_open = false;
    curs_set(1);
    endwin();
}

bool tui_term_is_open(void) {
    return g_open;
}

int tui_term_colors(void) {
    if (!g_open || g_mono) return 0;
    if (COLORS >= 256) return 256;
    return COLORS >= 16 ? 16 : 8;
}

void tui_term_size(int *cols, int *rows) {
    int y = 0, x = 0;
    getmaxyx(stdscr, y, x);
    if (cols) *cols = x;
    if (rows) *rows = y;
}

void tui_term_draw(const tui_grid_t *g) {
    int rows = 0, cols = 0;
    getmaxyx(stdscr, rows, cols);
    for (int y = 0; y < g->rows && y < rows; y++) {
        for (int x = 0; x < g->cols && x < cols; x++) {
            const tui_cell_t *c = tui_grid_cell(g, x, y);
            if (!c || c->width == 0) continue;
            unsigned int cp = ' ';
            tui_utf8_decode(c->ch[0] ? c->ch : " ", &cp);
            wchar_t wstr[2] = { (wchar_t)cp, 0 };
            short pair = 0;
            attr_t attr = role_attr(c->role, &pair);
            if (c->fg && !g_mono) {
                int fg = c->fg;
                if (COLORS < 256 && fg >= 16) fg = tui_color_basic(fg);
                short p = fg_pair(fg);
                if (p) {
                    pair = p;
                    if (COLORS < 16 && fg == 8) attr |= A_DIM;   /* gray on 8 colors */
                }
            }
            cchar_t cc;
            setcchar(&cc, wstr, attr, pair, NULL);
            mvadd_wch(y, x, &cc);
        }
    }
    refresh();
}

int tui_term_key(void) {
    wint_t ch;
    int rc = get_wch(&ch);
    if (rc == ERR) return TUI_KEY_NONE;
    if (rc == KEY_CODE_YES) {
        switch (ch) {
            case KEY_UP:        return TUI_KEY_UP;
            case KEY_DOWN:      return TUI_KEY_DOWN;
            case KEY_PPAGE:     return TUI_KEY_PGUP;
            case KEY_NPAGE:     return TUI_KEY_PGDN;
            case KEY_BACKSPACE: return TUI_KEY_BACKSPACE;
            case KEY_ENTER:     return TUI_KEY_ENTER;
            case KEY_RESIZE:    return TUI_KEY_RESIZE;
            case KEY_BTAB:      return TUI_KEY_TAB;
            default:            return TUI_KEY_NONE;
        }
    }
    if (ch == '\r' || ch == '\n') return TUI_KEY_ENTER;
    if (ch == 8 || ch == 127) return TUI_KEY_BACKSPACE;
    if (ch == 27) return TUI_KEY_ESC;
    if (ch == '\t') return TUI_KEY_TAB;
    if (ch >= 32 && ch < 127) return (int)ch;
    return TUI_KEY_NONE;
}
