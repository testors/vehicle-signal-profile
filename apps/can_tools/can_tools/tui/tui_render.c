/**
 * @file tui_render.c
 * @brief Layout and drawing of the TUI dashboard into a cell grid (Pure C)
 */

#include "tui_render.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

/*******************************************************************************
 * Layout
 ******************************************************************************/

void tui_layout_compute(tui_layout_t *L, int cols, int rows, bool has_signals,
                        int signal_count, bool has_gauges, tui_panel_t tab) {
    memset(L, 0, sizeof(*L));
    L->body = (tui_rect_t){ 1, 1, cols - 2, rows - 4 };
    L->narrow = cols < 80 || rows < 24;
    if (!has_signals) has_gauges = false;
    if (!L->narrow && has_gauges) {
        /* the gauges take their rows from the Signals / Raw-Log split */
        int rest = L->body.h - TUI_GAUGES_HEIGHT;
        int need = signal_count + 3;
        int sh = need < rest * 55 / 100 ? need : rest * 55 / 100;
        int want = 3 + (signal_count < 3 ? signal_count : 3);
        if (sh < want) L->narrow = true;
    }

    if (L->narrow) {
        L->tabs = (tui_rect_t){ L->body.x, L->body.y, L->body.w, 1 };
        tui_rect_t p = { L->body.x, L->body.y + 1, L->body.w, L->body.h - 1 };
        for (int i = 0; i < TUI_PANEL_COUNT; i++) {
            L->panel[i] = p;
            L->visible[i] = (int)tab == i && (i != TUI_PANEL_SIGNALS || has_signals) &&
                            (i != TUI_PANEL_GAUGES || has_gauges);
        }
        return;
    }

    int top = L->body.y;
    int rest_h = L->body.h;
    if (has_gauges) {
        L->panel[TUI_PANEL_GAUGES] = (tui_rect_t){ L->body.x, top, L->body.w, TUI_GAUGES_HEIGHT };
        L->visible[TUI_PANEL_GAUGES] = true;
        top += TUI_GAUGES_HEIGHT;
        rest_h -= TUI_GAUGES_HEIGHT;
    }
    if (has_signals) {
        int need = signal_count + 3;
        int max_h = rest_h * 55 / 100;
        int sh = need < max_h ? need : max_h;
        if (sh < 4) sh = 4;
        L->panel[TUI_PANEL_SIGNALS] = (tui_rect_t){ L->body.x, top, L->body.w, sh };
        L->visible[TUI_PANEL_SIGNALS] = true;
        top += sh;
        rest_h -= sh;
    }
    int raw_w = L->body.w * 55 / 100;
    L->panel[TUI_PANEL_RAW] = (tui_rect_t){ L->body.x, top, raw_w, rest_h };
    L->panel[TUI_PANEL_LOG] = (tui_rect_t){ L->body.x + raw_w, top, L->body.w - raw_w, rest_h };
    L->visible[TUI_PANEL_RAW] = true;
    L->visible[TUI_PANEL_LOG] = true;
}

int tui_layout_content_rows(const tui_layout_t *L, tui_panel_t p) {
    if (!L->visible[p]) return 0;
    int n = L->panel[p].h - (p == TUI_PANEL_LOG || p == TUI_PANEL_GAUGES ? 2 : 3);
    return n > 0 ? n : 0;
}

/*******************************************************************************
 * Drawing helpers
 ******************************************************************************/

static int put(tui_grid_t *g, int x, int y, const char *s, char role, int max_cols) {
    return tui_grid_put(g, x, y, s, role, max_cols);
}

static void put_fit(tui_grid_t *g, int x, int y, const char *s, char role, int w, bool ascii) {
    char buf[512];
    tui_text_fit(buf, sizeof(buf), s, w, true, ascii);
    put(g, x, y, buf, role, w);
}

static void put_right(tui_grid_t *g, int x, int y, int w, const char *s, char role, bool ascii) {
    char buf[512];
    tui_text_fit(buf, sizeof(buf), s, w, true, ascii);
    int sw = tui_utf8_width(buf);
    put(g, x + (w - sw), y, buf, role, w);
}

static void hline(tui_grid_t *g, int x, int y, int n, bool ascii, char role) {
    tui_grid_fill(g, x, y, n, tui_glyph(TUI_G_H, ascii), role);
}

static void box(tui_grid_t *g, tui_rect_t r, const char *title, const char *right,
                char title_role, bool ascii) {
    if (r.w < 2 || r.h < 2) return;
    put(g, r.x, r.y, tui_glyph(TUI_G_TL, ascii), TUI_ROLE_DEFAULT, 1);
    hline(g, r.x + 1, r.y, r.w - 2, ascii, TUI_ROLE_DEFAULT);
    put(g, r.x + r.w - 1, r.y, tui_glyph(TUI_G_TR, ascii), TUI_ROLE_DEFAULT, 1);
    for (int y = r.y + 1; y < r.y + r.h - 1; y++) {
        put(g, r.x, y, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
        put(g, r.x + r.w - 1, y, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
    }
    put(g, r.x, r.y + r.h - 1, tui_glyph(TUI_G_BL, ascii), TUI_ROLE_DEFAULT, 1);
    hline(g, r.x + 1, r.y + r.h - 1, r.w - 2, ascii, TUI_ROLE_DEFAULT);
    put(g, r.x + r.w - 1, r.y + r.h - 1, tui_glyph(TUI_G_BR, ascii), TUI_ROLE_DEFAULT, 1);

    int right_w = 0;
    if (right && right[0]) {
        right_w = tui_utf8_width(right) + 2;
        if (right_w > r.w - 4) right_w = 0;
    }
    if (title && title[0]) {
        int avail = r.w - 4 - (right_w ? right_w + 1 : 0);
        if (avail > 2) {
            char buf[256];
            tui_text_fit(buf, sizeof(buf), title, avail - 2, true, ascii);
            put(g, r.x + 1, r.y, " ", TUI_ROLE_DEFAULT, 1);
            int n = put(g, r.x + 2, r.y, buf, title_role, avail - 2);
            put(g, r.x + 2 + n, r.y, " ", TUI_ROLE_DEFAULT, 1);
        }
    }
    if (right_w) {
        int x = r.x + r.w - 2 - right_w;
        put(g, x, r.y, " ", TUI_ROLE_DEFAULT, 1);
        put(g, x + 1, r.y, right, TUI_ROLE_DIM, right_w - 2);
        put(g, x + right_w - 1, r.y, " ", TUI_ROLE_DEFAULT, 1);
    }
}

/* "▲ n" (rows above) and "▼ n more" (rows below) on a panel's bottom border. */
static void clip_marks(tui_grid_t *g, tui_rect_t r, int above, int below, bool ascii) {
    if (above <= 0 && below <= 0) return;
    char text[96] = "";
    size_t n = 0;
    if (above > 0) n += (size_t)snprintf(text + n, sizeof(text) - n, "%s %d", tui_glyph(TUI_G_MORE_UP, ascii), above);
    if (below > 0) n += (size_t)snprintf(text + n, sizeof(text) - n, "%s%s %d more", above > 0 ? "  " : "",
                                         tui_glyph(TUI_G_MORE_DOWN, ascii), below);
    int w = tui_utf8_width(text) + 2;
    if (w > r.w - 4) return;
    int x = r.x + r.w - 2 - w;
    int y = r.y + r.h - 1;
    put(g, x, y, " ", TUI_ROLE_DEFAULT, 1);
    put(g, x + 1, y, text, TUI_ROLE_BRIGHT, w - 2);
    put(g, x + w - 1, y, " ", TUI_ROLE_DEFAULT, 1);
}

/* Exported helpers for the other screens (tui_screens.c). */
int tui_draw_text(tui_grid_t *g, int x, int y, const char *s, char role, int max_cols) {
    return put(g, x, y, s, role, max_cols);
}
void tui_draw_fit(tui_grid_t *g, int x, int y, const char *s, char role, int w, bool ascii) {
    put_fit(g, x, y, s, role, w, ascii);
}
void tui_draw_right(tui_grid_t *g, int x, int y, int w, const char *s, char role, bool ascii) {
    put_right(g, x, y, w, s, role, ascii);
}
void tui_draw_hline(tui_grid_t *g, int x, int y, int n, bool ascii, char role) {
    hline(g, x, y, n, ascii, role);
}
void tui_draw_box(tui_grid_t *g, tui_rect_t r, const char *title, const char *right,
                  char title_role, bool ascii) {
    box(g, r, title, right, title_role, ascii);
}
void tui_draw_clip_marks(tui_grid_t *g, tui_rect_t r, int above, int below, bool ascii) {
    clip_marks(g, r, above, below, ascii);
}
int tui_draw_hints(tui_grid_t *g, int x, int y, int limit, const tui_hint_t *hints, int n, bool ascii) {
    (void)ascii;
    int x0 = x;
    for (int i = 0; i < n; i++) {
        int w = (i > 0 ? 2 : 0) + tui_utf8_width(hints[i].key) + 1 + tui_utf8_width(hints[i].desc);
        if (x + w > limit) break;
        if (i > 0) x += put(g, x, y, "  ", TUI_ROLE_DEFAULT, limit - x);
        x += put(g, x, y, hints[i].key, TUI_ROLE_BRIGHT, limit - x);
        x += put(g, x, y, " ", TUI_ROLE_DEFAULT, limit - x);
        x += put(g, x, y, hints[i].desc, hints[i].role ? hints[i].role : TUI_ROLE_DEFAULT, limit - x);
    }
    return x - x0;
}

static int max_width_of(int a, const char *s) {
    int w = tui_utf8_width(s);
    return w > a ? w : a;
}

static int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/*******************************************************************************
 * Header and footer
 ******************************************************************************/

static void format_speed(char *out, size_t len, double speed) {
    if (speed < 1.0) snprintf(out, len, "%.2gx", speed);
    else snprintf(out, len, "%.1fx", speed);
}

static void draw_header(tui_grid_t *g, const tui_model_t *m, const tui_view_t *v) {
    bool ascii = v->ascii;
    int cols = v->cols;
    const char *dot = tui_glyph(TUI_G_DOT, ascii);

    /* seg1 = device · bus [· silent | TX ON] */
    char bus[64];
    if (m->cls.bus == TUI_BUS_DETECTING) snprintf(bus, sizeof(bus), "detecting%s", tui_glyph(TUI_G_ELLIPSIS, ascii));
    else snprintf(bus, sizeof(bus), "%s", tui_bus_text(m->cls.bus));
    const char *tx = v->live ? (v->tx_on ? "TX ON" : "silent") : NULL;
    char tail1[128];
    if (tx) snprintf(tail1, sizeof(tail1), " %s %s %s %s", dot, bus, dot, tx);
    else snprintf(tail1, sizeof(tail1), " %s %s", dot, bus);
    const char *device = v->device ? v->device : "";

    /* seg3 = fr/s · elapsed */
    char count[32], hms[32], s3_full[96], s3_short[64];
    tui_format_count(count, sizeof(count), (long long)(tui_model_frames_per_s(m, v->now_ms) + 0.5));
    tui_format_hms(hms, sizeof(hms), v->now_ms - m->origin_ms);
    snprintf(s3_full, sizeof(s3_full), "%s fr/s %s %s", count, dot, hms);
    snprintf(s3_short, sizeof(s3_short), "%s fr/s", count);

    /* seg4 = REC / replay state */
    char s4_full[96], s4_short[96];
    char role4 = TUI_ROLE_HEADER;
    if (v->replay) {
        char pos[16], total[16], sp[16];
        tui_format_mmss(pos, sizeof(pos), v->replay_pos_ms);
        tui_format_mmss(total, sizeof(total), v->replay_total_ms);
        format_speed(sp, sizeof(sp), v->speed);
        if (v->ended) snprintf(s4_full, sizeof(s4_full), "END %s/%s", pos, total);
        else if (v->paused) snprintf(s4_full, sizeof(s4_full), "PAUSED %s/%s", pos, total);
        else snprintf(s4_full, sizeof(s4_full), "REPLAY %s %s/%s", sp, pos, total);
        if (v->ended) snprintf(s4_short, sizeof(s4_short), "END");
        else if (v->paused) snprintf(s4_short, sizeof(s4_short), "PAUSED %s", pos);
        else snprintf(s4_short, sizeof(s4_short), "%s %s", sp, pos);
    } else if (v->disconnected) {
        snprintf(s4_full, sizeof(s4_full), "DISCONNECTED");
        snprintf(s4_short, sizeof(s4_short), "DISCONNECTED");
        role4 = TUI_ROLE_ALERT;
    } else if (v->recording) {
        snprintf(s4_full, sizeof(s4_full), "%s REC", tui_glyph(TUI_G_REC_ON, ascii));
        snprintf(s4_short, sizeof(s4_short), "%s", s4_full);
        role4 = TUI_ROLE_ALERT;
    } else {
        snprintf(s4_full, sizeof(s4_full), "%s rec", tui_glyph(TUI_G_REC_OFF, ascii));
        snprintf(s4_short, sizeof(s4_short), "%s", s4_full);
        role4 = TUI_ROLE_DIM;
    }

    /* Shorten step by step: drop the profile, cut the device name to 12
       columns, short fr/s, short replay state, then cut the device name
       further. The bus classification always stays. */
    const char *s2 = (v->profile && v->profile[0]) ? v->profile : NULL;
    const char *s3 = s3_full;
    const char *s4 = s4_full;
    int w_tail1 = tui_utf8_width(tail1);
    int w_dev = tui_utf8_width(device);
#define HEADER_FIXED() (2 + w_tail1 + 1 + 1 + 3 + (s2 ? tui_utf8_width(s2) + 5 : 0) + \
                        tui_utf8_width(s3) + 4 + tui_utf8_width(s4) + 2)
    int min_dev = w_dev < 12 ? w_dev : 12;
    if (s2 && HEADER_FIXED() + w_dev > cols) s2 = NULL;
    if (HEADER_FIXED() + min_dev > cols) s3 = s3_short;
    if (HEADER_FIXED() + min_dev > cols) s4 = s4_short;
    int dev_room = cols - HEADER_FIXED();
#undef HEADER_FIXED
    if (dev_room > w_dev) dev_room = w_dev;
    if (dev_room < 1) dev_room = 1;

    int x = 0;
    x += put(g, x, 0, tui_glyph(TUI_G_TL, ascii), TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    char dev[256];
    tui_text_fit(dev, sizeof(dev), device, dev_room, true, ascii);
    x += put(g, x, 0, dev, TUI_ROLE_HEADER, dev_room);
    if (tx) {
        char t[96];
        snprintf(t, sizeof(t), " %s %s %s ", dot, bus, dot);
        x += put(g, x, 0, t, TUI_ROLE_HEADER, -1);
        x += put(g, x, 0, tx, v->tx_on ? TUI_ROLE_ALERT : TUI_ROLE_HEADER, -1);
    } else {
        x += put(g, x, 0, tail1, TUI_ROLE_HEADER, -1);
    }
    x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    int tail = 3 + (s2 ? tui_utf8_width(s2) + 5 : 0) + tui_utf8_width(s3) + 4 + tui_utf8_width(s4) + 2;
    int fill = cols - x - tail;
    if (fill < 1) fill = 1;
    hline(g, x, 0, fill, ascii, TUI_ROLE_DEFAULT);
    x += fill;
    x += put(g, x, 0, tui_glyph(TUI_G_T_DOWN, ascii), TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, tui_glyph(TUI_G_H, ascii), TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    if (s2) {
        x += put(g, x, 0, s2, TUI_ROLE_HEADER, -1);
        x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
        x += put(g, x, 0, tui_glyph(TUI_G_H, ascii), TUI_ROLE_DEFAULT, 1);
        x += put(g, x, 0, tui_glyph(TUI_G_T_DOWN, ascii), TUI_ROLE_DEFAULT, 1);
        x += put(g, x, 0, tui_glyph(TUI_G_H, ascii), TUI_ROLE_DEFAULT, 1);
        x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    }
    x += put(g, x, 0, s3, TUI_ROLE_HEADER, -1);
    x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, tui_glyph(TUI_G_H, ascii), TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, tui_glyph(TUI_G_T_DOWN, ascii), TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
    x += put(g, x, 0, s4, role4, cols - 1 - x);
    if (x < cols - 1) {
        put(g, x, 0, " ", TUI_ROLE_DEFAULT, 1);
        x++;
    }
    if (x < cols - 1) hline(g, x, 0, cols - 1 - x, ascii, TUI_ROLE_DEFAULT);
    put(g, cols - 1, 0, tui_glyph(TUI_G_TR, ascii), TUI_ROLE_DEFAULT, 1);
}

static void draw_footer(tui_grid_t *g, const tui_view_t *v, bool narrow) {
    bool ascii = v->ascii;
    int y = v->rows - 2;
    int x = 2;
    int limit = v->cols - 3;
    if (v->filter_editing) {
        char buf[160];
        snprintf(buf, sizeof(buf), "/%s_", v->filter ? v->filter : "");
        x += put(g, x, y, "filter ", TUI_ROLE_DIM, limit - x);
        x += put(g, x, y, buf, TUI_ROLE_BRIGHT, limit - x);
        x += put(g, x, y, "   ", TUI_ROLE_DEFAULT, limit - x);
        x += put(g, x, y, tui_glyph(TUI_G_ENTER, ascii), TUI_ROLE_BRIGHT, limit - x);
        x += put(g, x, y, " keep  ", TUI_ROLE_DEFAULT, limit - x);
        x += put(g, x, y, "esc", TUI_ROLE_BRIGHT, limit - x);
        put(g, x, y, " clear", TUI_ROLE_DEFAULT, limit - x);
        return;
    }
    tui_hint_t hints[16];
    int n = 0;
    hints[n++] = (tui_hint_t){ "tab", narrow ? "switch" : "panel", 0 };
    hints[n++] = (tui_hint_t){ "/", (v->filter && v->filter[0]) ? "filter (esc clears)" : "filter", 0 };
    hints[n++] = (tui_hint_t){ "s", "sort", 0 };
    hints[n++] = (tui_hint_t){ "d", "discover", 0 };
    if (v->replay) {
        hints[n++] = (tui_hint_t){ "space", v->paused ? "play" : "pause", 0 };
        hints[n++] = (tui_hint_t){ "[ ]", "speed", 0 };
    }
    if (v->rec_available) hints[n++] = (tui_hint_t){ "r", "rec", 0 };
    hints[n++] = (tui_hint_t){ "p", "profile", 0 };
    if (v->tx_footer) hints[n++] = (tui_hint_t){ "x", v->tx_footer, 0 };
    if (v->replay || v->live || v->disconnected) hints[n++] = (tui_hint_t){ "R", "replay list", 0 };
    hints[n++] = (tui_hint_t){ tui_glyph(TUI_G_UPDOWN, ascii), "scroll", 0 };
    /* "? help" and "q quit" always show; the others while they fit */
    const tui_hint_t tail[2] = { { "?", "help", 0 }, { "q", "quit", 0 } };
    int tail_w = 0;
    for (int i = 0; i < 2; i++) tail_w += 2 + tui_utf8_width(tail[i].key) + 1 + tui_utf8_width(tail[i].desc);
    tui_hint_t shown[18];
    int ns = 0, used = 0;
    for (int i = 0; i < n; i++) {
        int w = (ns > 0 ? 2 : 0) + tui_utf8_width(hints[i].key) + 1 + tui_utf8_width(hints[i].desc);
        if (x + used + w + tail_w > limit) break;
        shown[ns++] = hints[i];
        used += w;
    }
    shown[ns++] = tail[0];
    shown[ns++] = tail[1];
    tui_draw_hints(g, x, y, limit, shown, ns, ascii);
}

static void draw_frame(tui_grid_t *g, const tui_view_t *v) {
    bool ascii = v->ascii;
    int cols = v->cols, rows = v->rows;
    for (int y = 1; y < rows - 3; y++) {
        put(g, 0, y, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
        put(g, cols - 1, y, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
    }
    put(g, 0, rows - 3, tui_glyph(TUI_G_T_RIGHT, ascii), TUI_ROLE_DEFAULT, 1);
    hline(g, 1, rows - 3, cols - 2, ascii, TUI_ROLE_DEFAULT);
    put(g, cols - 1, rows - 3, tui_glyph(TUI_G_T_LEFT, ascii), TUI_ROLE_DEFAULT, 1);
    put(g, 0, rows - 2, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
    put(g, cols - 1, rows - 2, tui_glyph(TUI_G_V, ascii), TUI_ROLE_DEFAULT, 1);
    put(g, 0, rows - 1, tui_glyph(TUI_G_BL, ascii), TUI_ROLE_DEFAULT, 1);
    hline(g, 1, rows - 1, cols - 2, ascii, TUI_ROLE_DEFAULT);
    put(g, cols - 1, rows - 1, tui_glyph(TUI_G_BR, ascii), TUI_ROLE_DEFAULT, 1);
}

static void draw_tabs(tui_grid_t *g, const tui_layout_t *L, const tui_view_t *v, bool has_signals) {
    static const char *names[TUI_PANEL_COUNT] = { "Gauges", "Signals", "Raw CAN", "Log" };
    int x = L->tabs.x + 1;
    int limit = L->tabs.x + L->tabs.w;
    bool first = true;
    for (int i = 0; i < TUI_PANEL_COUNT; i++) {
        if (i == TUI_PANEL_SIGNALS && !has_signals) continue;
        if (i == TUI_PANEL_GAUGES && !(has_signals && v->gauges)) continue;
        if (!first) x += put(g, x, L->tabs.y, tui_glyph(TUI_G_V, v->ascii), TUI_ROLE_DIM, limit - x);
        first = false;
        char buf[32];
        snprintf(buf, sizeof(buf), " %s ", names[i]);
        x += put(g, x, L->tabs.y, buf, (int)v->tab == i ? TUI_ROLE_SELECT : TUI_ROLE_DIM, limit - x);
    }
}

/*******************************************************************************
 * Panels
 ******************************************************************************/

bool tui_signal_matches(const tui_signal_t *s, const char *filter) {
    if (!filter || !filter[0]) return true;
    size_t n = strlen(filter);
    for (const char *p = s->name; *p; p++) {
        if (strncasecmp(p, filter, n) == 0) return true;
    }
    return false;
}

static void title_with_filter(char *out, size_t len, const char *title, const tui_view_t *v) {
    if (v->filter && v->filter[0]) snprintf(out, len, "%s %s /%s", title, tui_glyph(TUI_G_DOT, v->ascii), v->filter);
    else snprintf(out, len, "%s", title);
}

static char state_role(tui_sig_state_t st) {
    switch (st) {
        case TUI_SIG_LIVE:  return TUI_ROLE_DEFAULT;
        case TUI_SIG_STALE: return TUI_ROLE_STALE;
        case TUI_SIG_DEAD:  return TUI_ROLE_STALE;
        case TUI_SIG_NEVER: break;
    }
    return TUI_ROLE_DIM;
}

static void draw_signals(tui_grid_t *g, const tui_model_t *m, tui_view_t *v,
                         const tui_layout_t *L) {
    tui_rect_t r = L->panel[TUI_PANEL_SIGNALS];
    bool ascii = v->ascii;
    int64_t now = v->now_ms;
    const char *dash = tui_glyph(TUI_G_DASH, ascii);

    int live = 0;
    int *rows = (int *)calloc((size_t)(m->signal_count + 1), sizeof(int));
    int nrows = 0;
    int name_w = 6, unit_w = 4, src_w = 3;
    for (int i = 0; i < m->signal_count; i++) {
        const tui_signal_t *s = &m->signals[i];
        tui_sig_state_t st = tui_signal_state(s, now);
        if (st == TUI_SIG_LIVE || st == TUI_SIG_STALE) live++;
        name_w = max_width_of(name_w, s->name);
        unit_w = max_width_of(unit_w, s->unit);
        src_w = max_width_of(src_w, s->src);
        if (rows && tui_signal_matches(s, v->filter)) rows[nrows++] = i;
    }
    name_w = clampi(name_w, 6, 24);
    unit_w = clampi(unit_w, 4, 8);
    src_w = clampi(src_w, 3, 10);

    char title[128], t2[160];
    snprintf(title, sizeof(title), "Signals (%d/%d live)", live, m->signal_count);
    title_with_filter(t2, sizeof(t2), title, v);
    box(g, r, t2, NULL, v->focus == TUI_PANEL_SIGNALS && !L->narrow ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT, ascii);

    /* Column plan. The history column takes the remaining width; below 8
       cells it becomes min and max columns, making room by narrowing the
       name and value columns and, if needed, dropping src. */
    int x0 = r.x + 1;
    int iw = r.w - 2;
    int value_w = 10, hz_w = 5, age_w = 6, mm_w = 9;
    bool show_src = true;
    int x_name = 0, x_value = 0, x_unit = 0, x_hz = 0, x_age = 0, x_hist = 0, x_src = 0, hist_w = 0;
    bool spark = false, minmax = false, minmax_mode = false;
    for (int attempt = 0; attempt < 12; attempt++) {
        x_name = x0 + 1;
        x_value = x_name + name_w + 1;
        x_unit = x_value + value_w + 2;
        x_hz = x_unit + unit_w + 1;
        x_age = x_hz + hz_w + 1;
        x_hist = x_age + age_w + 3;
        x_src = show_src ? x0 + iw - 1 - src_w : x0 + iw;
        hist_w = x_src - 1 - x_hist;
        if (!minmax_mode) {
            if (hist_w >= 8) { spark = true; break; }
            if (name_w > 10) { name_w = name_w - 2 > 10 ? name_w - 2 : 10; continue; }
            minmax_mode = true;
            mm_w = 7;
        }
        if (hist_w >= 2 * mm_w + 1) { minmax = true; break; }
        if (show_src) { show_src = false; continue; }
        if (name_w > 8) { name_w = 8; continue; }
        if (value_w > 8) { value_w = 8; continue; }
        break;
    }
    if (!spark && !minmax && !show_src) x_src = x0 + iw;

    int hy = r.y + 1;
    put(g, x_name, hy, "name", TUI_ROLE_DIM, name_w);
    put_right(g, x_value, hy, value_w, "value", TUI_ROLE_DIM, ascii);
    put(g, x_unit, hy, "unit", TUI_ROLE_DIM, unit_w);
    put_right(g, x_hz, hy, hz_w, "hz", TUI_ROLE_DIM, ascii);
    put_right(g, x_age, hy, age_w, "age", TUI_ROLE_DIM, ascii);
    if (spark) put(g, x_hist, hy, "last 10 s", TUI_ROLE_DIM, hist_w);
    if (minmax) {
        put_right(g, x_hist, hy, mm_w, "min", TUI_ROLE_DIM, ascii);
        put_right(g, x_hist + mm_w + 1, hy, mm_w, "max", TUI_ROLE_DIM, ascii);
    }
    if (show_src) put(g, x_src, hy, "src", TUI_ROLE_DIM, src_w);

    int visible = tui_layout_content_rows(L, TUI_PANEL_SIGNALS);
    int max_scroll = nrows - visible > 0 ? nrows - visible : 0;
    v->scroll[TUI_PANEL_SIGNALS] = clampi(v->scroll[TUI_PANEL_SIGNALS], 0, max_scroll);
    int first = v->scroll[TUI_PANEL_SIGNALS];

    for (int row = 0; row < visible && first + row < nrows; row++) {
        const tui_signal_t *s = &m->signals[rows[first + row]];
        int y = r.y + 2 + row;
        tui_sig_state_t st = tui_signal_state(s, now);
        char role = state_role(st);
        bool blank = st == TUI_SIG_NEVER || st == TUI_SIG_DEAD;

        put_fit(g, x_name, y, s->name, TUI_ROLE_DEFAULT, name_w, ascii);

        char val[64];
        /* live values bold; coolant / oil / gearbox temperatures in their zone color */
        char vrole = st == TUI_SIG_LIVE ? TUI_ROLE_BRIGHT : role;
        int vfg = 0;
        if (st == TUI_SIG_LIVE && s->palette == TUI_PAL_TEMP && s->has_value && !s->value_null) {
            vfg = tui_zone_text_fg(tui_temp_zone(s->temp, tui_to_celsius(s->value, s->unit)));
        }
        if (blank) {
            put_right(g, x_value, y, value_w, dash, role, ascii);
        } else if (s->label) {
            put_fit(g, x_value, y, s->label, vrole, value_w, ascii);
        } else if (s->has_value) {
            tui_format_signal_value(val, sizeof(val), s->value, s->decimals);
            put_right(g, x_value, y, value_w, val, vrole, ascii);
            if (vfg) tui_grid_set_fg(g, x_value, y, value_w, vfg);
        } else {
            put_right(g, x_value, y, value_w, dash, role, ascii);
        }

        put_fit(g, x_unit, y, s->unit, TUI_ROLE_DIM, unit_w, ascii);

        char hz[16], age[16];
        if (st == TUI_SIG_NEVER) {
            snprintf(hz, sizeof(hz), "%s", dash);
            snprintf(age, sizeof(age), "%s", dash);
        } else {
            /* a stale signal is not arriving: its rate is 0 */
            if (st == TUI_SIG_LIVE) tui_format_hz(hz, sizeof(hz), tui_signal_hz(s, now), ascii);
            else snprintf(hz, sizeof(hz), "0");
            tui_format_age(age, sizeof(age), now - s->last_ms);
        }
        put_right(g, x_hz, y, hz_w, hz, role, ascii);
        put_right(g, x_age, y, age_w, age, role, ascii);

        if (spark || minmax) {
            char hist[1024];
            char hrole = role;
            if (st == TUI_SIG_NEVER) {
                if (s->diagnostic) snprintf(hist, sizeof(hist), "%s %s %s", s->request,
                                            tui_glyph(TUI_G_DOT, ascii), v->tx_hint ? v->tx_hint : "");
                else snprintf(hist, sizeof(hist), "no frame yet");
                hrole = TUI_ROLE_DIM;
                put_fit(g, x_hist, y, hist, hrole, spark ? hist_w : 2 * mm_w + 1, ascii);
            } else if (st == TUI_SIG_DEAD) {
                put(g, x_hist, y, "stale", TUI_ROLE_STALE, spark ? hist_w : 2 * mm_w + 1);
            } else if (minmax) {
                double mn = 0, mx = 0;
                char a[32], b[32];
                if (tui_signal_minmax(s, now, &mn, &mx)) {
                    tui_format_signal_value(a, sizeof(a), mn, s->decimals);
                    tui_format_signal_value(b, sizeof(b), mx, s->decimals);
                } else {
                    snprintf(a, sizeof(a), "%s", dash);
                    snprintf(b, sizeof(b), "%s", dash);
                }
                put_right(g, x_hist, y, mm_w, a, hrole, ascii);
                put_right(g, x_hist + mm_w + 1, y, mm_w, b, hrole, ascii);
            } else if (s->discrete) {
                tui_signal_discrete_history(s, now, hist_w, hist, sizeof(hist));
                put(g, x_hist, y, hist, hrole, hist_w);
            } else if (ascii) {
                tui_signal_sparkline(s, now, hist_w, ascii, hist, sizeof(hist));
                put(g, x_hist, y, hist, hrole, hist_w);
            } else {
                /* btop-style braille: 2 samples per cell, colored by height */
                size_t blen = (size_t)hist_w * 4 + 1;
                char *br = (char *)malloc(blen);
                unsigned char *fg = (unsigned char *)calloc((size_t)hist_w + 1, 1);
                if (br && fg) {
                    int nc = tui_signal_braille(s, now, hist_w, br, blen, fg);
                    put(g, x_hist, y, br, hrole, hist_w);
                    if (st == TUI_SIG_LIVE) {
                        for (int i = 0; i < nc; i++) tui_grid_set_fg(g, x_hist + i, y, 1, fg[i]);
                    }
                }
                free(br);
                free(fg);
            }
        }
        if (show_src) put(g, x_src, y, s->src, TUI_ROLE_DIM, src_w);
    }
    {
        int shown = nrows - first < visible ? nrows - first : visible;
        clip_marks(g, r, first, nrows - first - shown, ascii);
    }
    free(rows);
}

static const char *sort_name(tui_sort_t s) {
    switch (s) {
        case TUI_SORT_ID:      return "sort: id";
        case TUI_SORT_CHANGED: return "sort: changed";
        default:               return "sort: hz";
    }
}

static void draw_raw(tui_grid_t *g, const tui_model_t *m, tui_view_t *v, const tui_layout_t *L) {
    tui_rect_t r = L->panel[TUI_PANEL_RAW];
    bool ascii = v->ascii;
    int64_t now = v->now_ms;

    char title[96], t2[160];
    if (v->monitor_paused) snprintf(title, sizeof(title), "Raw CAN (monitor paused: TX mode)");
    else snprintf(title, sizeof(title), "Raw CAN (%d IDs)", m->raw.count);
    title_with_filter(t2, sizeof(t2), title, v);
    box(g, r, t2, sort_name(v->sort),
        v->focus == TUI_PANEL_RAW && !L->narrow ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT, ascii);

    int n = m->raw.count;
    const tui_raw_entry_t **list = (const tui_raw_entry_t **)calloc((size_t)(n + 1), sizeof(*list));
    if (!list) return;
    int count = tui_raw_sorted(&m->raw, v->sort, now, v->filter, list, n);
    bool any_ext = false;
    for (int i = 0; i < count; i++) any_ext = any_ext || list[i]->ext;

    /* Column plan: data bytes never get cut while the panel can hold them.
       Free width by dropping dlc (the byte count shows it), narrowing hz to
       its widest current value, then tightening the padding. */
    int x0 = r.x + 1;
    int limit = r.x + r.w - 1;
    int id_w = any_ext ? 10 : 5;
    int max_dlc = 0, hz_text_w = 2;
    for (int i = 0; i < count; i++) {
        char t[16];
        snprintf(t, sizeof(t), "%.1f", tui_raw_hz(list[i], now));
        int w = (int)strlen(t);
        if (w > hz_text_w) hz_text_w = w;
        if (list[i]->dlc > max_dlc) max_dlc = list[i]->dlc;
    }
    int data_w = max_dlc > 0 ? max_dlc * 3 - 1 : 4;
    bool show_dlc = true;
    int hz_w = 6, pad = 1, gap = 3;
    int x_id = 0, x_hz = 0, x_dlc = 0, x_data = 0;
    for (int step = 0; step < 4; step++) {
        x_id = x0 + pad;
        x_hz = x_id + id_w;
        x_dlc = x_hz + hz_w + 1;
        x_data = show_dlc ? x_dlc + 3 + gap : x_hz + hz_w + gap - 1;
        if (x_data + data_w <= limit) break;
        if (step == 0) show_dlc = false;
        else if (step == 1) hz_w = hz_text_w + 1;
        else if (step == 2) { pad = 0; gap = 2; }
    }

    int hy = r.y + 1;
    put(g, x_id, hy, "id", TUI_ROLE_DIM, limit - x_id);
    if (x_hz + hz_w <= limit) put_right(g, x_hz, hy, hz_w, "hz", TUI_ROLE_DIM, ascii);
    if (show_dlc && x_dlc + 3 <= limit) put_right(g, x_dlc, hy, 3, "dlc", TUI_ROLE_DIM, ascii);
    if (x_data < limit) put(g, x_data, hy, "data", TUI_ROLE_DIM, limit - x_data);

    int visible = tui_layout_content_rows(L, TUI_PANEL_RAW);
    int max_scroll = count - visible > 0 ? count - visible : 0;
    v->scroll[TUI_PANEL_RAW] = clampi(v->scroll[TUI_PANEL_RAW], 0, max_scroll);
    int first = v->scroll[TUI_PANEL_RAW];

    for (int row = 0; row < visible && first + row < count; row++) {
        const tui_raw_entry_t *e = list[first + row];
        int y = r.y + 2 + row;
        bool silent = now - e->last_ms > 5000;
        char base = silent ? TUI_ROLE_DIM : TUI_ROLE_DEFAULT;
        char id[24], hz[16], dlc[8];
        tui_format_can_id(id, sizeof(id), e->id, e->ext);
        snprintf(hz, sizeof(hz), "%.1f", tui_raw_hz(e, now));
        snprintf(dlc, sizeof(dlc), "%u", e->dlc);
        put(g, x_id, y, id, base, limit - x_id);
        if (x_hz + hz_w <= limit) put_right(g, x_hz, y, hz_w, hz, base, ascii);
        if (show_dlc && x_dlc + 3 <= limit) put_right(g, x_dlc, y, 3, dlc, base, ascii);
        for (int b = 0; b < e->dlc; b++) {
            int x = x_data + b * 3;
            if (x + 2 > limit) break;
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X", e->data[b]);
            char role = silent ? TUI_ROLE_DIM
                               : (now - e->byte_change_ms[b] < 1000 ? TUI_ROLE_BRIGHT : TUI_ROLE_DIM);
            put(g, x, y, hex, role, 2);
        }
    }
    {
        int shown = count - first < visible ? count - first : visible;
        clip_marks(g, r, first, count - first - shown, ascii);
    }
    free(list);
}

static void draw_log(tui_grid_t *g, const tui_model_t *m, tui_view_t *v, const tui_layout_t *L) {
    tui_rect_t r = L->panel[TUI_PANEL_LOG];
    bool ascii = v->ascii;
    box(g, r, "Log", NULL, v->focus == TUI_PANEL_LOG && !L->narrow ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT, ascii);
    int visible = tui_layout_content_rows(L, TUI_PANEL_LOG);
    int max_scroll = m->log.count - visible > 0 ? m->log.count - visible : 0;
    v->scroll[TUI_PANEL_LOG] = clampi(v->scroll[TUI_PANEL_LOG], 0, max_scroll);
    int first = v->scroll[TUI_PANEL_LOG];
    int x0 = r.x + 2;
    int w = r.w - 3;
    for (int row = 0; row < visible; row++) {
        const tui_log_line_t *line = tui_log_get(&m->log, first + row);
        if (!line) break;
        int y = r.y + 1 + row;
        char t[16];
        tui_format_mmss(t, sizeof(t), line->ms - m->origin_ms);
        int used = put(g, x0, y, t, TUI_ROLE_DIM, w);
        used += put(g, x0 + used, y, " ", TUI_ROLE_DEFAULT, w - used);
        char role = line->level == TUI_LOG_ERROR ? TUI_ROLE_ALERT
                  : line->level == TUI_LOG_WARN ? TUI_ROLE_WARN : TUI_ROLE_DEFAULT;
        put_fit(g, x0 + used, y, line->text, role, w - used, ascii);
    }
    {
        int shown = m->log.count - first < visible ? m->log.count - first : visible;
        if (shown < 0) shown = 0;
        clip_marks(g, r, first, m->log.count - first - shown, ascii);
    }
}

/*******************************************************************************
 * Gauges
 ******************************************************************************/

static void put_cells(tui_grid_t *g, int x, int y, const tui_bar_cell_t *cells, int n, int limit) {
    for (int i = 0; i < n && x + i < limit; i++) tui_grid_put_fg(g, x + i, y, cells[i].ch, cells[i].role, cells[i].fg, 1);
}

/* "scale  rpm 9k · speed 300 · brake 100 bar", items dropped from the right
   until it fits next to the title. */
static void gauge_scale_label(char *out, size_t len, const tui_gauges_t *gs, const tui_model_t *m,
                              int max_w, bool ascii) {
    const char *dot = tui_glyph(TUI_G_DOT, ascii);
    char items[3][32];
    int n = 0;
    if (gs->g[TUI_GAUGE_RPM].ncand > 0) {
        snprintf(items[n++], sizeof(items[0]), "rpm %.0fk", gs->g[TUI_GAUGE_RPM].scale / 1000.0);
    }
    if (gs->g[TUI_GAUGE_SPEED].ncand > 0) {
        snprintf(items[n++], sizeof(items[0]), "speed %.0f", gs->g[TUI_GAUGE_SPEED].scale);
    }
    const tui_signal_t *b = tui_gauge_signal(gs, m, TUI_GAUGE_BRAKE);
    if (b && tui_brake_mode(b->canonical) == TUI_BRAKE_PRESSURE) {
        snprintf(items[n++], sizeof(items[0]), "brake %.0f bar", gs->g[TUI_GAUGE_BRAKE].scale);
    }
    out[0] = '\0';
    for (int k = n; k > 0; k--) {
        size_t o = (size_t)snprintf(out, len, "scale ");
        for (int i = 0; i < k && o < len; i++) {
            o += (size_t)snprintf(out + o, len - o, "%s%s", i ? (dot[0] == '.' ? " . " : " · ") : " ", items[i]);
        }
        if (tui_utf8_width(out) <= max_w) return;
    }
    out[0] = '\0';
}

typedef enum { BAR_METER, BAR_STEER, BAR_GEAR, BAR_NONE } bar_kind_t;

static void draw_one_gauge(tui_grid_t *g, const tui_model_t *m, const tui_view_t *v, tui_gauge_id_t id,
                           int x, int y, int colw, bool wide) {
    const tui_gauges_t *gs = v->gauges;
    bool ascii = v->ascii;
    tui_bar_style_t st = { v->colors, ascii };
    int limit = x + colw;
    const tui_signal_t *s = tui_gauge_signal(gs, m, id);
    bool bound = gs->g[id].ncand > 0;     /* without candidates: label, "—", empty track, no unit */
    tui_sig_state_t state = s ? tui_signal_state(s, v->now_ms) : TUI_SIG_NEVER;
    bool stale = state == TUI_SIG_STALE;
    bool shown = s && (state == TUI_SIG_LIVE || stale);
    bool number = shown && s->has_value && !s->value_null;
    const char *dash = tui_glyph(TUI_G_DASH, ascii);

    int lab_w = wide ? 9 : 5;
    int val_x = wide ? x + 10 : x + 6, val_w = wide ? 6 : 7;
    int unit_x = x + 17;
    int bar_x = wide ? x + 22 : x + 14;
    int bar_w = wide ? colw - 23 : 10;
    if (bar_w > 15) bar_w = 15;
    if (bar_w < 6) bar_w = 6;

    put(g, x + 1, y, tui_gauge_label(id, !wide), TUI_ROLE_DIM, lab_w);

    char val[40] = "", unit[12] = "";
    char vrole = stale ? TUI_ROLE_STALE : TUI_ROLE_BRIGHT;
    int vfg = 0;
    double frac = 0;
    const unsigned char *stops = TUI_GRAD_CYAN;
    int nstops = 7;
    bar_kind_t kind = BAR_METER;
    double steer = 0;
    char gear[16] = "";

    switch (id) {
        case TUI_GAUGE_THROTTLE: {
            bool deg = s && tui_unit_is_degrees(s->unit);
            snprintf(unit, sizeof(unit), "%s", deg ? "°" : "%");
            if (number) {
                tui_format_fixed(val, sizeof(val), s->value, 0);
                frac = s->value / (deg ? 90.0 : 100.0);
            }
            stops = TUI_GRAD_THR;
            nstops = 5;
            break;
        }
        case TUI_GAUGE_BRAKE: {
            tui_brake_mode_t mode = s ? tui_brake_mode(s->canonical) : TUI_BRAKE_PRESSURE;
            stops = TUI_GRAD_BRK;
            nstops = 5;
            if (mode == TUI_BRAKE_PEDAL) {
                snprintf(unit, sizeof(unit), "%%");
                if (number) {
                    tui_format_fixed(val, sizeof(val), s->value, 0);
                    frac = s->value / 100.0;
                }
            } else if (mode == TUI_BRAKE_SWITCH) {
                if (number) {
                    bool on = fabs(s->value) > 1e-9;
                    snprintf(val, sizeof(val), "%s", on ? "ON" : "OFF");
                    frac = on ? 1.0 : 0.0;
                }
            } else {
                snprintf(unit, sizeof(unit), "bar");
                if (number) {
                    double bar = tui_gauge_convert(id, s->unit, s->value);
                    tui_format_fixed(val, sizeof(val), bar, 1);
                    frac = bar / gs->g[id].scale;
                }
            }
            break;
        }
        case TUI_GAUGE_STEERING:
            kind = BAR_STEER;
            snprintf(unit, sizeof(unit), "°");
            if (number) {
                steer = s->value;
                if (strcasecmp(s->unit, "rad") == 0) steer *= 57.29577951308232;
                int a = (int)rint(fabs(steer));
                if (a == 0) snprintf(val, sizeof(val), "0");
                else snprintf(val, sizeof(val), "%s %d", steer > 0 ? "L" : "R", a);
            }
            break;
        case TUI_GAUGE_RPM:
            snprintf(unit, sizeof(unit), "rpm");
            stops = TUI_GRAD_RPM;
            nstops = 11;
            if (number) {
                if (wide) tui_format_count(val, sizeof(val), llround(s->value));
                else tui_format_fixed(val, sizeof(val), s->value, 0);
                frac = s->value / gs->g[id].scale;
            }
            break;
        case TUI_GAUGE_SPEED:
            snprintf(unit, sizeof(unit), "km/h");
            if (number) {
                double kmh = tui_gauge_convert(id, s->unit, s->value);
                tui_format_fixed(val, sizeof(val), kmh, 0);
                frac = kmh / gs->g[id].scale;
            }
            break;
        case TUI_GAUGE_GEAR:
            kind = BAR_GEAR;
            if (shown) tui_gear_label(s, gear, sizeof(gear));
            break;
        case TUI_GAUGE_COOLANT:
        case TUI_GAUGE_OIL:
        case TUI_GAUGE_GEARBOX: {
            tui_temp_kind_t tk = id == TUI_GAUGE_COOLANT ? TUI_TEMP_COOLANT
                               : (id == TUI_GAUGE_OIL ? TUI_TEMP_OIL : TUI_TEMP_GEARBOX);
            const tui_temp_zones_t *z = tui_temp_zones(tk);
            snprintf(unit, sizeof(unit), "°C");
            if (number) {
                double c = tui_gauge_convert(id, s->unit, s->value);
                tui_zone_t zone = tui_temp_zone(tk, c);
                tui_format_fixed(val, sizeof(val), c, 0);
                frac = (c - z->lo) / (z->hi - z->lo);
                stops = zone == TUI_ZONE_COLD ? TUI_GRAD_COLD : (zone == TUI_ZONE_HOT ? TUI_GRAD_HOT : TUI_GRAD_THR);
                nstops = zone == TUI_ZONE_OK ? 5 : 6;
                if (!stale) vfg = tui_zone_text_fg(zone);
            }
            break;
        }
        default:
            kind = BAR_NONE;
            break;
    }

    tui_bar_cell_t cells[64];
    if (kind == BAR_GEAR) {
        int max_gear = tui_gear_strip_max(s ? s->max_gear : 0, gs->g[id].max_gear_seen);
        const char *cur = gear[0] ? gear : NULL;
        if (wide) {
            if (cur) tui_draw_right(g, val_x, y, val_w, cur, vrole, ascii);
            else tui_draw_right(g, val_x, y, val_w, dash, TUI_ROLE_DIM, ascii);
            int sw = limit - unit_x - 1;
            if (sw > 20) sw = 20;
            int n = tui_gear_strip(max_gear, stale ? NULL : cur, sw, cells, 64);
            put_cells(g, unit_x, y, cells, n, limit);
        } else {
            int sx = val_x, sw = 18;
            bool in_strip = false;
            if (cur) {
                in_strip = strcmp(cur, "R") == 0 || strcmp(cur, "N") == 0;
                int gn = atoi(cur);
                char back[8];
                snprintf(back, sizeof(back), "%d", gn);
                if (gn >= 1 && gn <= max_gear && strcmp(back, cur) == 0) in_strip = true;
            }
            if (cur && !in_strip) {
                int used = put(g, sx, y, cur, vrole, sw);
                sx += used + 1;
                sw -= used + 1;
            }
            if (sw > 0) {
                int n = tui_gear_strip(max_gear, stale ? NULL : cur, sw, cells, 64);
                put_cells(g, sx, y, cells, n, limit);
            }
        }
        return;
    }

    /* value and unit */
    if (wide) {
        if (val[0]) {
            if (vfg) {
                char fitted[64];
                tui_text_fit(fitted, sizeof(fitted), val, val_w, true, ascii);
                int vx = val_x + val_w - tui_utf8_width(fitted);
                tui_grid_put_fg(g, vx, y, fitted, vrole, vfg, val_w);
            } else {
                tui_draw_right(g, val_x, y, val_w, val, vrole, ascii);
            }
        } else {
            tui_draw_right(g, val_x, y, val_w, dash, TUI_ROLE_DIM, ascii);
        }
        if (bound) put(g, unit_x, y, unit, TUI_ROLE_DIM, 4);
    } else {
        char joined[64];
        if (val[0]) snprintf(joined, sizeof(joined), "%s%s", val, unit);
        else snprintf(joined, sizeof(joined), "%s", dash);
        char fitted[64];
        tui_text_fit(fitted, sizeof(fitted), joined, val_w, true, ascii);
        int vx = val_x + val_w - tui_utf8_width(fitted);
        tui_grid_put_fg(g, vx, y, fitted, val[0] ? vrole : TUI_ROLE_DIM, val[0] ? vfg : 0, val_w);
    }

    /* bar */
    if (kind == BAR_STEER) {
        int half = wide ? (bar_w - 1) / 2 : 4;
        if (half > 15) half = 15;
        tui_bar_steer(number ? steer : 0, half, stale, &st, cells);
        if (!number) {
            for (int i = 0; i < 2 * half + 1; i++) {
                if (i != half) cells[i].fg = TUI_FG_TRACK;
            }
        }
        put_cells(g, bar_x, y, cells, 2 * half + 1, limit);
    } else {
        tui_bar_meter(number ? frac : 0, bar_w, stops, nstops, stale, &st, cells);
        put_cells(g, bar_x, y, cells, bar_w, limit);
    }
}

static void draw_gauges(tui_grid_t *g, const tui_model_t *m, tui_view_t *v, const tui_layout_t *L) {
    tui_rect_t r = L->panel[TUI_PANEL_GAUGES];
    bool ascii = v->ascii;
    bool tabs = L->narrow;
    bool wide = tabs || v->cols >= 100;
    char right[160] = "";
    if (wide) {
        int max_w = r.w - 4 - (int)strlen(" Gauges ") - 3;
        gauge_scale_label(right, sizeof(right), v->gauges, m, max_w, ascii);
    }
    box(g, r, "Gauges", right[0] ? right : NULL, TUI_ROLE_DEFAULT, ascii);
    /* the reference script's column: (panel width - 2) / 3 */
    int colw = tabs ? r.w - 2 : (r.w - 2) / 3;
    for (int id = 0; id < TUI_GAUGE_COUNT; id++) {
        int row = tabs ? id : id / 3;
        int col = tabs ? 0 : id % 3;
        int x = r.x + 1 + col * colw;
        int y = r.y + 1 + row;
        if (y >= r.y + r.h - 1) break;
        draw_one_gauge(g, m, v, (tui_gauge_id_t)id, x, y, colw, wide);
    }
}

/*******************************************************************************
 * Dashboard
 ******************************************************************************/

void tui_render_dashboard(tui_grid_t *g, const tui_model_t *m, tui_view_t *v) {
    tui_grid_clear(g);
    if (v->cols < 30 || v->rows < 10) {
        tui_grid_put(g, 0, 0, "terminal too small", TUI_ROLE_ALERT, v->cols);
        return;
    }
    bool has_signals = m->signal_count > 0;
    tui_layout_t L;
    tui_layout_compute(&L, v->cols, v->rows, has_signals, m->signal_count, v->gauges != NULL, v->tab);

    draw_frame(g, v);
    draw_header(g, m, v);
    if (L.narrow) draw_tabs(g, &L, v, has_signals);
    if (L.visible[TUI_PANEL_GAUGES]) draw_gauges(g, m, v, &L);
    if (L.visible[TUI_PANEL_SIGNALS]) draw_signals(g, m, v, &L);
    if (L.visible[TUI_PANEL_RAW]) draw_raw(g, m, v, &L);
    if (L.visible[TUI_PANEL_LOG]) draw_log(g, m, v, &L);
    draw_footer(g, v, L.narrow);
}
