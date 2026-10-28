/**
 * @file tui_grid.c
 * @brief Cell grid the TUI draws into (Pure C, no ncurses)
 */

#include "tui_grid.h"

#include <stdlib.h>
#include <string.h>

tui_grid_t *tui_grid_new(int cols, int rows) {
    tui_grid_t *g = (tui_grid_t *)calloc(1, sizeof(*g));
    if (!g) return NULL;
    if (!tui_grid_resize(g, cols, rows)) {
        free(g);
        return NULL;
    }
    return g;
}

void tui_grid_free(tui_grid_t *g) {
    if (!g) return;
    free(g->cells);
    free(g);
}

bool tui_grid_resize(tui_grid_t *g, int cols, int rows) {
    if (cols < 1) cols = 1;
    if (rows < 1) rows = 1;
    tui_cell_t *cells = (tui_cell_t *)calloc((size_t)cols * (size_t)rows, sizeof(*cells));
    if (!cells) return false;
    free(g->cells);
    g->cells = cells;
    g->cols = cols;
    g->rows = rows;
    tui_grid_clear(g);
    return true;
}

void tui_grid_clear(tui_grid_t *g) {
    for (int i = 0; i < g->cols * g->rows; i++) {
        g->cells[i].ch[0] = ' ';
        g->cells[i].ch[1] = '\0';
        g->cells[i].width = 1;
        g->cells[i].role = TUI_ROLE_DEFAULT;
        g->cells[i].fg = 0;
    }
}

const tui_cell_t *tui_grid_cell(const tui_grid_t *g, int x, int y) {
    if (!g || x < 0 || y < 0 || x >= g->cols || y >= g->rows) return NULL;
    return &g->cells[y * g->cols + x];
}

static tui_cell_t *cell_at(tui_grid_t *g, int x, int y) {
    return &g->cells[y * g->cols + x];
}

static void set_space(tui_cell_t *c, char role) {
    c->ch[0] = ' ';
    c->ch[1] = '\0';
    c->width = 1;
    c->role = role;
    c->fg = 0;
}

/* Overwriting part of a wide character blanks its other half. */
static void unlink_wide(tui_grid_t *g, int x, int y) {
    tui_cell_t *c = cell_at(g, x, y);
    if (c->width == 0 && x > 0) set_space(cell_at(g, x - 1, y), cell_at(g, x - 1, y)->role);
    if (c->width == 2 && x + 1 < g->cols) set_space(cell_at(g, x + 1, y), cell_at(g, x + 1, y)->role);
}

int tui_utf8_decode(const char *s, unsigned int *cp) {
    const unsigned char *u = (const unsigned char *)s;
    if (u[0] < 0x80) { *cp = u[0]; return 1; }
    if ((u[0] & 0xE0) == 0xC0 && (u[1] & 0xC0) == 0x80) {
        *cp = ((unsigned int)(u[0] & 0x1F) << 6) | (u[1] & 0x3F);
        return 2;
    }
    if ((u[0] & 0xF0) == 0xE0 && (u[1] & 0xC0) == 0x80 && (u[2] & 0xC0) == 0x80) {
        *cp = ((unsigned int)(u[0] & 0x0F) << 12) | ((unsigned int)(u[1] & 0x3F) << 6) | (u[2] & 0x3F);
        return 3;
    }
    if ((u[0] & 0xF8) == 0xF0 && (u[1] & 0xC0) == 0x80 && (u[2] & 0xC0) == 0x80 &&
        (u[3] & 0xC0) == 0x80) {
        *cp = ((unsigned int)(u[0] & 0x07) << 18) | ((unsigned int)(u[1] & 0x3F) << 12) |
              ((unsigned int)(u[2] & 0x3F) << 6) | (u[3] & 0x3F);
        return 4;
    }
    *cp = 0xFFFD;   /* invalid byte: one replacement character */
    return 1;
}

int tui_codepoint_width(unsigned int cp) {
    if (cp < 0x20 || (cp >= 0x7F && cp < 0xA0)) return 0;
    if ((cp >= 0x1100 && cp <= 0x115F) || (cp >= 0x2E80 && cp <= 0xA4CF && cp != 0x303F) ||
        (cp >= 0xAC00 && cp <= 0xD7A3) || (cp >= 0xF900 && cp <= 0xFAFF) ||
        (cp >= 0xFE30 && cp <= 0xFE4F) || (cp >= 0xFF00 && cp <= 0xFF60) ||
        (cp >= 0xFFE0 && cp <= 0xFFE6) || (cp >= 0x1F300 && cp <= 0x1F64F) ||
        (cp >= 0x1F900 && cp <= 0x1F9FF) || (cp >= 0x20000 && cp <= 0x3FFFD)) {
        return 2;
    }
    return 1;
}

int tui_utf8_width(const char *s) {
    int w = 0;
    while (s && *s) {
        unsigned int cp;
        s += tui_utf8_decode(s, &cp);
        w += tui_codepoint_width(cp);
    }
    return w;
}

int tui_grid_put(tui_grid_t *g, int x, int y, const char *utf8, char role, int max_cols) {
    if (!g || !utf8 || y < 0 || y >= g->rows || x >= g->cols) return 0;
    int limit = g->cols - x;
    if (max_cols >= 0 && max_cols < limit) limit = max_cols;
    int used = 0;
    const char *p = utf8;
    while (*p && used < limit) {
        unsigned int cp;
        int n = tui_utf8_decode(p, &cp);
        int w = tui_codepoint_width(cp);
        if (w == 0) { p += n; continue; }   /* drop control characters */
        int cx = x + used;
        if (cx < 0) { used += w; p += n; continue; }
        if (w == 2 && used + 2 > limit) {
            unlink_wide(g, cx, y);
            set_space(cell_at(g, cx, y), role);
            used++;
            break;
        }
        unlink_wide(g, cx, y);
        tui_cell_t *c = cell_at(g, cx, y);
        memset(c->ch, 0, sizeof(c->ch));
        if (cp == 0xFFFD && n == 1) memcpy(c->ch, "?", 1);
        else memcpy(c->ch, p, (size_t)n);
        c->width = (unsigned char)w;
        c->role = role;
        c->fg = 0;
        if (w == 2) {
            unlink_wide(g, cx + 1, y);
            tui_cell_t *cont = cell_at(g, cx + 1, y);
            cont->ch[0] = '\0';
            cont->width = 0;
            cont->role = role;
            cont->fg = 0;
        }
        used += w;
        p += n;
    }
    return used;
}

void tui_grid_fill(tui_grid_t *g, int x, int y, int n, const char *utf8, char role) {
    for (int i = 0; i < n; i++) tui_grid_put(g, x + i, y, utf8, role, 1);
}

void tui_grid_set_role(tui_grid_t *g, int x, int y, int n, char role) {
    if (!g || y < 0 || y >= g->rows) return;
    for (int i = 0; i < n; i++) {
        if (x + i < 0 || x + i >= g->cols) continue;
        cell_at(g, x + i, y)->role = role;
    }
}

int tui_grid_put_fg(tui_grid_t *g, int x, int y, const char *utf8, char role, int fg, int max_cols) {
    int n = tui_grid_put(g, x, y, utf8, role, max_cols);
    tui_grid_set_fg(g, x, y, n, fg);
    return n;
}

void tui_grid_set_fg(tui_grid_t *g, int x, int y, int n, int fg) {
    if (!g || y < 0 || y >= g->rows) return;
    for (int i = 0; i < n; i++) {
        if (x + i < 0 || x + i >= g->cols) continue;
        cell_at(g, x + i, y)->fg = (unsigned char)(fg < 0 || fg > 255 ? 0 : fg);
    }
}

bool tui_grid_has_fg(const tui_grid_t *g) {
    for (int i = 0; i < g->cols * g->rows; i++) {
        if (g->cells[i].fg) return true;
    }
    return false;
}

void tui_grid_dump_fg(const tui_grid_t *g, FILE *f) {
    for (int y = 0; y < g->rows; y++) {
        for (int x = 0; x < g->cols; x++) {
            int fg = g->cells[y * g->cols + x].fg;
            if (fg) fprintf(f, "%02x", fg);
            else fputs("..", f);
        }
        fputc('\n', f);
    }
}

enum { BASIC_RED = 1, BASIC_GREEN = 2, BASIC_YELLOW = 3, BASIC_BLUE = 4, BASIC_CYAN = 6, BASIC_WHITE = 7,
       BASIC_GRAY = 8 };

int tui_color_basic(int index) {
    /* the TUI's palettes, by table */
    static const struct { unsigned char idx, basic; } kTable[] = {
        /* greens: throttle, ok zone, rpm low */
        { 34, BASIC_GREEN }, { 40, BASIC_GREEN }, { 46, BASIC_GREEN }, { 83, BASIC_GREEN },
        { 120, BASIC_GREEN }, { 41, BASIC_GREEN }, { 77, BASIC_GREEN }, { 113, BASIC_GREEN },
        /* yellows: rpm middle */
        { 149, BASIC_YELLOW }, { 185, BASIC_YELLOW }, { 221, BASIC_YELLOW }, { 215, BASIC_YELLOW },
        { 214, BASIC_YELLOW },
        /* reds: brake, hot zone, rpm top, alerts */
        { 124, BASIC_RED }, { 160, BASIC_RED }, { 196, BASIC_RED }, { 197, BASIC_RED }, { 203, BASIC_RED },
        { 209, BASIC_RED }, { 210, BASIC_RED },
        /* blues: cold zone */
        { 25, BASIC_BLUE }, { 26, BASIC_BLUE }, { 27, BASIC_BLUE }, { 33, BASIC_BLUE }, { 39, BASIC_BLUE },
        /* cyans: speed, steering, generic timeline, over-range marker */
        { 24, BASIC_CYAN }, { 31, BASIC_CYAN }, { 38, BASIC_CYAN }, { 45, BASIC_CYAN }, { 51, BASIC_CYAN },
        { 87, BASIC_CYAN }, { 123, BASIC_CYAN }, { 117, BASIC_CYAN },
        /* whites and grays */
        { 231, BASIC_WHITE }, { 252, BASIC_WHITE }, { 255, BASIC_WHITE },
        { 237, BASIC_GRAY }, { 239, BASIC_GRAY }, { 240, BASIC_GRAY }, { 245, BASIC_GRAY },
    };
    for (size_t i = 0; i < sizeof(kTable) / sizeof(kTable[0]); i++) {
        if (kTable[i].idx == index) return kTable[i].basic;
    }
    if (index <= 0) return 0;
    if (index < 16) {
        static const unsigned char kLow[16] = { BASIC_GRAY, 1, 2, 3, 4, BASIC_RED, 6, 7,
                                                BASIC_GRAY, 1, 2, 3, 4, BASIC_RED, 6, 7 };
        return kLow[index];
    }
    int r, gr, b;
    if (index >= 232) {
        return index >= 250 ? BASIC_WHITE : BASIC_GRAY;
    }
    static const int kLevels[6] = { 0, 95, 135, 175, 215, 255 };
    int c = index - 16;
    r = kLevels[c / 36];
    gr = kLevels[(c / 6) % 6];
    b = kLevels[c % 6];
    int mx = r > gr ? (r > b ? r : b) : (gr > b ? gr : b);
    int mn = r < gr ? (r < b ? r : b) : (gr < b ? gr : b);
    if (mx - mn < 40) return mx >= 200 ? BASIC_WHITE : BASIC_GRAY;
    /* hue in degrees */
    double h;
    double d = (double)(mx - mn);
    if (mx == r) h = 60.0 * (double)(gr - b) / d;
    else if (mx == gr) h = 60.0 * (2.0 + (double)(b - r) / d);
    else h = 60.0 * (4.0 + (double)(r - gr) / d);
    if (h < 0) h += 360.0;
    if (h < 25 || h >= 330) return BASIC_RED;
    if (h < 70) return BASIC_YELLOW;
    if (h < 160) return BASIC_GREEN;
    if (h < 200) return BASIC_CYAN;
    if (h < 270) return BASIC_BLUE;
    return BASIC_RED;   /* magenta has no slot: closest alert color */
}

void tui_grid_map_colors(tui_grid_t *g, int colors) {
    if (colors >= 256) return;
    for (int i = 0; i < g->cols * g->rows; i++) {
        if (!g->cells[i].fg) continue;
        g->cells[i].fg = colors <= 0 ? 0 : (unsigned char)tui_color_basic(g->cells[i].fg);
    }
}

void tui_grid_dump_text(const tui_grid_t *g, FILE *f) {
    for (int y = 0; y < g->rows; y++) {
        for (int x = 0; x < g->cols; x++) {
            const tui_cell_t *c = &g->cells[y * g->cols + x];
            if (c->width == 0) continue;
            fputs(c->ch, f);
        }
        fputc('\n', f);
    }
}

void tui_grid_dump_roles(const tui_grid_t *g, FILE *f) {
    for (int y = 0; y < g->rows; y++) {
        for (int x = 0; x < g->cols; x++) fputc(g->cells[y * g->cols + x].role, f);
        fputc('\n', f);
    }
}

void tui_text_fit(char *dst, size_t dst_len, const char *src, int cols, bool ellipsis, bool ascii) {
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!src || cols <= 0) return;
    if (tui_utf8_width(src) <= cols) {
        snprintf(dst, dst_len, "%s", src);
        return;
    }
    int budget = ellipsis ? cols - 1 : cols;
    size_t out = 0;
    int used = 0;
    const char *p = src;
    while (*p) {
        unsigned int cp;
        int n = tui_utf8_decode(p, &cp);
        int w = tui_codepoint_width(cp);
        if (used + w > budget || out + (size_t)n + 1 >= dst_len) break;
        memcpy(dst + out, p, (size_t)n);
        out += (size_t)n;
        used += w;
        p += n;
    }
    dst[out] = '\0';
    if (ellipsis) {
        const char *e = tui_glyph(TUI_G_ELLIPSIS, ascii);
        size_t el = strlen(e);
        if (out + el + 1 <= dst_len) memcpy(dst + out, e, el + 1);
    }
}

static const char *kGlyphs[TUI_G_COUNT][2] = {
    [TUI_G_H] = { "─", "-" },        [TUI_G_V] = { "│", "|" },
    [TUI_G_TL] = { "┌", "+" },       [TUI_G_TR] = { "┐", "+" },
    [TUI_G_BL] = { "└", "+" },       [TUI_G_BR] = { "┘", "+" },
    [TUI_G_T_DOWN] = { "┬", "+" },   [TUI_G_T_UP] = { "┴", "+" },
    [TUI_G_T_RIGHT] = { "├", "+" },  [TUI_G_T_LEFT] = { "┤", "+" },
    [TUI_G_ARROW] = { "▶", ">" },    [TUI_G_PROMPT] = { "▸", ">" },
    [TUI_G_REC_ON] = { "●", "*" },   [TUI_G_REC_OFF] = { "○", "o" },
    [TUI_G_ELLIPSIS] = { "…", "." }, [TUI_G_DASH] = { "—", "-" },
    [TUI_G_FLAT] = { "━", "=" },     [TUI_G_CHECK] = { "✔", "v" },
    [TUI_G_DOT] = { "·", "." },      [TUI_G_ENTER] = { "⏎", "Enter" },
    [TUI_G_UPDOWN] = { "↑↓", "up/dn" },
    [TUI_G_MORE_UP] = { "▲", "^" },  [TUI_G_MORE_DOWN] = { "▼", "v" },
    [TUI_G_BAR_FULL] = { "█", "#" }, [TUI_G_BAR_EMPTY] = { "░", "." },
    [TUI_G_SKIP] = { "–", "-" },
};

const char *tui_glyph(tui_glyph_t g, bool ascii) {
    if ((int)g < 0 || g >= TUI_G_COUNT) return "?";
    return kGlyphs[g][ascii ? 1 : 0];
}

const char *tui_spark(int level, bool ascii) {
    static const char *kBlocks[8] = { "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█" };
    static const char *kAscii[8] = { " ", "_", ".", "-", "~", "=", "*", "#" };
    if (level < 0) level = 0;
    if (level > 7) level = 7;
    return ascii ? kAscii[level] : kBlocks[level];
}
