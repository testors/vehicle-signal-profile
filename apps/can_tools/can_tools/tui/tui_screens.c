/**
 * @file tui_screens.c
 * @brief Modal overlays and the connect screen (Pure C, draws into a grid)
 */

#include "tui_screens.h"

#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Modal frame
 ******************************************************************************/

tui_rect_t tui_modal_rect(int cols, int rows, int want_w, int want_h) {
    int max_w = cols >= 60 ? cols - 4 : cols;
    int max_h = rows >= 20 ? rows - 2 : rows;
    int w = want_w < max_w ? want_w : max_w;
    int h = want_h < max_h ? want_h : max_h;
    if (w < 10) w = cols;
    if (h < 5) h = rows;
    return (tui_rect_t){ (cols - w) / 2, (rows - h) / 2, w, h };
}

void tui_dim_grid(tui_grid_t *g) {
    for (int y = 0; y < g->rows; y++) {
        tui_grid_set_role(g, 0, y, g->cols, TUI_ROLE_DIM);
        tui_grid_set_fg(g, 0, y, g->cols, 0);
    }
}

static void clear_rect(tui_grid_t *g, tui_rect_t r) {
    for (int y = r.y; y < r.y + r.h; y++) tui_grid_fill(g, r.x, y, r.w, " ", TUI_ROLE_DEFAULT);
}

tui_rect_t tui_draw_modal(tui_grid_t *g, tui_rect_t r, const char *title, const char *right,
                          const tui_hint_t *hints, int nhints, bool ascii) {
    clear_rect(g, r);
    tui_draw_box(g, r, title, right, TUI_ROLE_BRIGHT, ascii);
    if (nhints <= 0 || r.h < 5) return (tui_rect_t){ r.x + 1, r.y + 1, r.w - 2, r.h - 2 };
    int sep = r.y + r.h - 3;
    tui_draw_text(g, r.x, sep, tui_glyph(TUI_G_T_RIGHT, ascii), TUI_ROLE_DEFAULT, 1);
    tui_draw_hline(g, r.x + 1, sep, r.w - 2, ascii, TUI_ROLE_DEFAULT);
    tui_draw_text(g, r.x + r.w - 1, sep, tui_glyph(TUI_G_T_LEFT, ascii), TUI_ROLE_DEFAULT, 1);
    tui_draw_hints(g, r.x + 2, r.y + r.h - 2, r.x + r.w - 2, hints, nhints, ascii);
    return (tui_rect_t){ r.x + 1, r.y + 1, r.w - 2, r.h - 4 };
}

int tui_wrap_text(const char *text, int cols, char lines[][256], int max_lines) {
    int n = 0;
    if (!text || cols < 4 || max_lines <= 0) return 0;
    char line[256] = "";
    int line_w = 0;
    const char *p = text;
    while (*p && n < max_lines) {
        while (*p == ' ') p++;
        if (!*p) break;
        const char *end = p;
        while (*end && *end != ' ') end++;
        char word[256];
        size_t wl = (size_t)(end - p) < sizeof(word) - 1 ? (size_t)(end - p) : sizeof(word) - 1;
        memcpy(word, p, wl);
        word[wl] = '\0';
        int ww = tui_utf8_width(word);
        if (line_w > 0 && line_w + 1 + ww > cols) {
            snprintf(lines[n++], 256, "%s", line);
            line[0] = '\0';
            line_w = 0;
            if (n >= max_lines) break;
        }
        if (ww > cols) {
            char cut[256];
            tui_text_fit(cut, sizeof(cut), word, cols, true, false);
            snprintf(word, sizeof(word), "%s", cut);
            ww = tui_utf8_width(word);
        }
        size_t ll = strlen(line);
        snprintf(line + ll, sizeof(line) - ll, "%s%s", line_w > 0 ? " " : "", word);
        line_w += (line_w > 0 ? 1 : 0) + ww;
        p = end;
    }
    if (line_w > 0 && n < max_lines) snprintf(lines[n++], 256, "%s", line);
    return n;
}

/*******************************************************************************
 * Discover overlay
 ******************************************************************************/

static void format_count(char *out, size_t len, long long v) {
    tui_format_count(out, len, v);
}

void tui_draw_discover(tui_grid_t *g, int cols, int rows, const tui_discover_view_t *v, bool ascii) {
    tui_rect_t r = tui_modal_rect(cols, rows, 82, 16);
    char title[160];
    const char *h = tui_glyph(TUI_G_H, ascii);
    if (v->current >= 0 && v->current < v->count) {
        snprintf(title, sizeof(title), "Discover %s phase %d/%d %s", h, v->current + 1, v->count,
                 v->rows[v->current].name);
    } else {
        snprintf(title, sizeof(title), "Discover");
    }

    tui_hint_t hints[4];
    int nh = 0;
    const char *enter = tui_glyph(TUI_G_ENTER, ascii);
    if (v->status) {
        /* analyzing: no keys */
    } else if (v->capturing) {
        hints[nh++] = (tui_hint_t){ "a", "abort (keeps captured phases)", 0 };
    } else if (v->session_active) {
        bool skippable = v->current >= 0 && v->current < v->count && v->rows[v->current].skippable;
        hints[nh++] = (tui_hint_t){ enter, "start phase", 0 };
        if (skippable) hints[nh++] = (tui_hint_t){ "s", "skip", 0 };
        hints[nh++] = (tui_hint_t){ "a", "abort (keeps captured phases)", 0 };
    } else {
        hints[nh++] = (tui_hint_t){ enter, "start baseline", 0 };
        hints[nh++] = (tui_hint_t){ "esc", "close", 0 };
    }
    tui_rect_t c = tui_draw_modal(g, r, title, NULL, hints, nh, ascii);
    int x0 = c.x + 2;
    int limit = c.x + c.w - 1;
    int live_y = c.y + c.h - 1;

    for (int i = 0; i < v->count; i++) {
        int y = c.y + i;
        if (y >= live_y) break;
        const tui_discover_row_t *row = &v->rows[i];
        bool hot = row->state == TUI_PHASE_ACTIVE || (row->next && row->state == TUI_PHASE_PENDING);
        const char *mark = " ";
        char mark_role = TUI_ROLE_DEFAULT;
        char name_role = TUI_ROLE_DIM;
        if (row->state == TUI_PHASE_DONE) {
            mark = tui_glyph(TUI_G_CHECK, ascii);
            mark_role = TUI_ROLE_BRIGHT;
            name_role = TUI_ROLE_DEFAULT;
        } else if (row->state == TUI_PHASE_SKIPPED) {
            mark = tui_glyph(TUI_G_SKIP, ascii);
            mark_role = TUI_ROLE_DIM;
        }
        if (hot) {
            mark = tui_glyph(TUI_G_ARROW, ascii);
            mark_role = TUI_ROLE_BRIGHT;
            name_role = TUI_ROLE_BRIGHT;
        }
        tui_draw_text(g, x0, y, mark, mark_role, limit - x0);
        tui_draw_fit(g, x0 + 2, y, row->name, name_role, 12, ascii);
        char dur[16];
        snprintf(dur, sizeof(dur), "%2d s", row->duration_s);
        tui_draw_text(g, x0 + 15, y, dur, name_role == TUI_ROLE_BRIGHT ? TUI_ROLE_DEFAULT : name_role,
                      limit - x0 - 15);
        int x = x0 + 22;
        if (x >= limit) continue;
        char buf[160], count[32];
        switch (row->state) {
            case TUI_PHASE_DONE:
                format_count(count, sizeof(count), row->frames);
                snprintf(buf, sizeof(buf), "%6s frames   %3d IDs", count, row->ids);
                x += tui_draw_text(g, x, y, buf, TUI_ROLE_DEFAULT, limit - x);
                if (row->note && x + 8 < limit) tui_draw_text(g, x + 8, y, row->note, TUI_ROLE_BRIGHT, limit - x - 8);
                break;
            case TUI_PHASE_ACTIVE: {
                int cells = 20;
                double frac = row->duration_s > 0 ? row->elapsed_s / (double)row->duration_s : 0.0;
                if (frac < 0) frac = 0;
                if (frac > 1) frac = 1;
                int full = (int)(frac * cells + 0.5);
                for (int k = 0; k < cells && x < limit; k++, x++) {
                    tui_draw_text(g, x, y, tui_glyph(k < full ? TUI_G_BAR_FULL : TUI_G_BAR_EMPTY, ascii),
                                  k < full ? TUI_ROLE_BRIGHT : TUI_ROLE_DIM, 1);
                }
                format_count(count, sizeof(count), row->frames);
                snprintf(buf, sizeof(buf), "  %4.1f s   %s frames", row->elapsed_s, count);
                tui_draw_text(g, x, y, buf, TUI_ROLE_DEFAULT, limit - x);
                break;
            }
            case TUI_PHASE_SKIPPED:
                tui_draw_text(g, x, y, "skipped", TUI_ROLE_DIM, limit - x);
                break;
            case TUI_PHASE_PENDING:
                if (row->skippable) tui_draw_text(g, x, y, "(skippable)", TUI_ROLE_DIM, limit - x);
                break;
        }
    }

    int y = c.y + v->count + 1;
    char lines[3][256];
    int w = limit - x0;
    int n = tui_wrap_text(v->instruction, w, lines, 2);
    for (int i = 0; i < n && y < live_y; i++, y++) tui_draw_text(g, x0, y, lines[i], TUI_ROLE_DEFAULT, w);
    if (v->warning && y < live_y) {
        char note[320];
        snprintf(note, sizeof(note), "NOTE: %s", v->warning);
        tui_draw_fit(g, x0, y, note, TUI_ROLE_WARN, w, ascii);
    }

    if (live_y > c.y + v->count) {
        if (v->status) {
            tui_draw_text(g, x0, live_y, v->status, TUI_ROLE_BRIGHT, w);
        } else if (v->live_text) {
            int x = x0 + tui_draw_text(g, x0, live_y, "live  ", TUI_ROLE_DIM, w);
            x += tui_draw_text(g, x, live_y, v->live_text, TUI_ROLE_DEFAULT, limit - x);
            if (v->live_spark && x + 2 < limit) tui_draw_text(g, x + 1, live_y, v->live_spark, TUI_ROLE_BRIGHT, limit - x - 1);
        }
    }
}

/*******************************************************************************
 * Discover result
 ******************************************************************************/

void tui_draw_result(tui_grid_t *g, int cols, int rows, const tui_result_view_t *v, bool ascii) {
    tui_rect_t r = tui_modal_rect(cols, rows, 81, v->count + 7);
    char title[600];
    snprintf(title, sizeof(title), "Discover result %s %s", tui_glyph(TUI_G_H, ascii), v->dir ? v->dir : "");
    tui_hint_t hints[3];
    int nh = 0;
    if (v->can_load) hints[nh++] = (tui_hint_t){ tui_glyph(TUI_G_ENTER, ascii), "load draft VSP into dashboard", 0 };
    hints[nh++] = (tui_hint_t){ "v", "view JSON", 0 };
    hints[nh++] = (tui_hint_t){ "esc", "close", 0 };
    tui_rect_t c = tui_draw_modal(g, r, title, v->bus[0] ? v->bus : NULL, hints, nh, ascii);

    int x0 = c.x;
    int limit = c.x + c.w;
    bool show_prior = c.w >= 79;
    bool show_range = c.w >= 67;
    int y = c.y;
    tui_draw_text(g, x0 + 2, y, "signal", TUI_ROLE_DIM, 11);
    tui_draw_text(g, x0 + 13, y, "id", TUI_ROLE_DIM, 8);
    tui_draw_text(g, x0 + 21, y, "layout", TUI_ROLE_DIM, 12);
    tui_draw_right(g, x0 + 33, y, 5, "hz", TUI_ROLE_DIM, ascii);
    tui_draw_text(g, x0 + 42, y, "conf", TUI_ROLE_DIM, limit - x0 - 42);
    if (show_range) tui_draw_text(g, x0 + 51, y, "raw range", TUI_ROLE_DIM, 16);
    if (show_prior) tui_draw_text(g, x0 + 67, y, "prior", TUI_ROLE_DIM, limit - x0 - 67);
    y++;
    for (int i = 0; i < v->count && y < c.y + c.h; i++, y++) {
        const tui_result_row_t *row = &v->rows[i];
        tui_draw_fit(g, x0 + 2, y, row->signal, TUI_ROLE_DEFAULT, 10, ascii);
        if (!row->found) {
            tui_draw_text(g, x0 + 13, y, tui_glyph(TUI_G_DASH, ascii), TUI_ROLE_DIM, 8);
            tui_draw_text(g, x0 + 21, y, row->missing, TUI_ROLE_DIM, limit - x0 - 21);
            continue;
        }
        tui_draw_text(g, x0 + 13, y, row->id, TUI_ROLE_DEFAULT, 8);
        tui_draw_fit(g, x0 + 21, y, row->layout, TUI_ROLE_DEFAULT, 12, ascii);
        tui_draw_right(g, x0 + 33, y, 5, row->hz, TUI_ROLE_DEFAULT, ascii);
        tui_draw_text(g, x0 + 42, y, row->conf, strcmp(row->conf, "HIGH") == 0 ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT,
                      limit - x0 - 42);
        if (show_range) tui_draw_fit(g, x0 + 51, y, row->range, TUI_ROLE_DEFAULT, 15, ascii);
        if (show_prior) tui_draw_fit(g, x0 + 67, y, row->prior, TUI_ROLE_DIM, limit - x0 - 67, ascii);
    }
    y++;
    if (y < c.y + c.h && v->written[0]) tui_draw_fit(g, x0 + 2, y, v->written, TUI_ROLE_DEFAULT, c.w - 3, ascii);
}

/*******************************************************************************
 * Text view and confirm modal
 ******************************************************************************/

void tui_draw_text_view(tui_grid_t *g, int cols, int rows, const char *title, const char *right,
                        const char *const *lines, int nlines, int *scroll, int want_w,
                        const tui_hint_t *hints, int nhints, bool ascii) {
    tui_rect_t r = tui_modal_rect(cols, rows, want_w, nlines + 4 + (nhints > 0 ? 2 : 0));
    tui_rect_t c = tui_draw_modal(g, r, title, right, hints, nhints, ascii);
    int visible = c.h - 2;
    if (visible < 1) visible = c.h;
    int top = visible == c.h ? c.y : c.y + 1;
    int max_scroll = nlines - visible > 0 ? nlines - visible : 0;
    if (*scroll > max_scroll) *scroll = max_scroll;
    if (*scroll < 0) *scroll = 0;
    for (int i = 0; i < visible && *scroll + i < nlines; i++) {
        tui_draw_fit(g, c.x + 2, top + i, lines[*scroll + i], TUI_ROLE_DEFAULT, c.w - 3, ascii);
    }
    int shown = nlines - *scroll < visible ? nlines - *scroll : visible;
    tui_draw_clip_marks(g, r, *scroll, nlines - *scroll - shown, ascii);
}

void tui_draw_confirm(tui_grid_t *g, int cols, int rows, const char *title,
                      const char *const *lines, int nlines, char role,
                      const tui_hint_t *hints, int nhints, bool ascii) {
    int w = 40;
    for (int i = 0; i < nlines; i++) {
        int lw = tui_utf8_width(lines[i]) + 6;
        if (lw > w) w = lw;
    }
    tui_rect_t r = tui_modal_rect(cols, rows, w, nlines + 4 + (nhints > 0 ? 2 : 0));
    tui_rect_t c = tui_draw_modal(g, r, title, NULL, hints, nhints, ascii);
    for (int i = 0; i < nlines && 1 + i < c.h; i++) {
        tui_draw_fit(g, c.x + 2, c.y + 1 + i, lines[i], role, c.w - 3, ascii);
    }
}

/*******************************************************************************
 * Profile picker
 ******************************************************************************/

static int picker_draft_block(int ndrafts) {
    return ndrafts > 0 ? ndrafts + 2 : 0;
}

/* Box height for n profile rows (prompt, blank, header, list, drafts, footer). */
static int picker_want_h(int nrows, int ndrafts) {
    int list = nrows > 0 ? nrows : 1;
    return 2 + 3 + list + picker_draft_block(ndrafts) + 2;
}

int tui_picker_list_rows(int cols, int rows, int nrows, int ndrafts) {
    int want_w = cols - 4 < 100 ? cols - 4 : 100;
    tui_rect_t r = tui_modal_rect(cols, rows, want_w, picker_want_h(nrows, ndrafts));
    int n = r.h - 4 - 3 - picker_draft_block(ndrafts);
    return n > 1 ? n : 1;
}

static void picker_row(tui_grid_t *g, int x, int y, int limit, const tui_picker_row_t *row, bool selected,
                       bool obd_dim, bool ascii) {
    char base = selected ? TUI_ROLE_SELECT : TUI_ROLE_DEFAULT;
    if (selected) tui_grid_fill(g, x, y, limit - x, " ", TUI_ROLE_SELECT);
    const char *mark = selected ? tui_glyph(TUI_G_ARROW, ascii) : (row->current ? tui_glyph(TUI_G_REC_ON, ascii) : " ");
    tui_draw_text(g, x, y, mark, base, 1);
    char num[16];
    tui_draw_fit(g, x + 2, y, row->make ? row->make : "", base, 9, ascii);
    tui_draw_fit(g, x + 12, y, row->model ? row->model : "", base, 12, ascii);
    tui_draw_fit(g, x + 25, y, row->gen ? row->gen : "", base, 5, ascii);
    tui_draw_fit(g, x + 31, y, row->years ? row->years : "", base, 10, ascii);
    snprintf(num, sizeof(num), "%d", row->signals);
    tui_draw_right(g, x + 42, y, 7, num, base, ascii);
    snprintf(num, sizeof(num), "%d", row->can);
    tui_draw_right(g, x + 50, y, 5, num, base, ascii);
    snprintf(num, sizeof(num), "%d", row->obd);
    tui_draw_right(g, x + 56, y, 5, num, selected ? base : (obd_dim ? TUI_ROLE_DIM : base), ascii);
    if (limit - (x + 63) >= 10 && row->file) {
        tui_draw_fit(g, x + 63, y, row->file, selected ? base : TUI_ROLE_DIM, limit - (x + 63), ascii);
    }
}

void tui_draw_picker(tui_grid_t *g, int cols, int rows, const tui_picker_view_t *v, bool ascii) {
    int want_w = cols - 4 < 100 ? cols - 4 : 100;
    tui_rect_t r = tui_modal_rect(cols, rows, want_w, picker_want_h(v->nrows, v->ndrafts));
    char right[64];
    if (v->indexing) snprintf(right, sizeof(right), "indexing %d/%d", v->indexed, v->total);
    else snprintf(right, sizeof(right), "%d profile%s", v->total, v->total == 1 ? "" : "s");
    const char *updown = tui_glyph(TUI_G_UPDOWN, ascii);
    tui_hint_t hints[5] = {
        { "type", "to filter", 0 },
        { updown, "select", 0 },
        { tui_glyph(TUI_G_ENTER, ascii), "load", 0 },
        { "^N", "none (raw only)", 0 },
        { "esc", "back", 0 },
    };
    tui_rect_t c = tui_draw_modal(g, r, "Profile", right, hints, 5, ascii);
    int x = c.x + 1;
    int limit = c.x + c.w - 1;

    char prompt[160];
    snprintf(prompt, sizeof(prompt), "%s ", tui_glyph(TUI_G_PROMPT, ascii));
    int px = x + tui_draw_text(g, x, c.y, prompt, TUI_ROLE_BRIGHT, limit - x);
    px += tui_draw_text(g, px, c.y, v->filter ? v->filter : "", TUI_ROLE_BRIGHT, limit - px);
    tui_draw_text(g, px, c.y, "_", TUI_ROLE_BRIGHT, limit - px);

    int y = c.y + 2;
    tui_draw_text(g, x + 2, y, "make", TUI_ROLE_DIM, 9);
    tui_draw_text(g, x + 12, y, "model", TUI_ROLE_DIM, 12);
    tui_draw_text(g, x + 25, y, "gen", TUI_ROLE_DIM, 5);
    tui_draw_text(g, x + 31, y, "years", TUI_ROLE_DIM, 10);
    tui_draw_right(g, x + 42, y, 7, "signals", TUI_ROLE_DIM, ascii);
    tui_draw_right(g, x + 50, y, 5, "can", TUI_ROLE_DIM, ascii);
    tui_draw_right(g, x + 56, y, 5, "obd", TUI_ROLE_DIM, ascii);
    if (limit - (x + 63) >= 10) tui_draw_text(g, x + 63, y, "file", TUI_ROLE_DIM, limit - (x + 63));
    y++;

    int list_rows = r.h - 4 - 3 - picker_draft_block(v->ndrafts);
    if (list_rows < 1) list_rows = 1;
    int sel = v->selected;
    if (sel < v->nrows) {
        if (sel < *v->scroll) *v->scroll = sel;
        if (sel >= *v->scroll + list_rows) *v->scroll = sel - list_rows + 1;
    }
    int max_scroll = v->nrows - list_rows > 0 ? v->nrows - list_rows : 0;
    if (*v->scroll > max_scroll) *v->scroll = max_scroll;
    if (*v->scroll < 0) *v->scroll = 0;

    if (v->nrows == 0) {
        const char *msg = !v->dir ? "no profile folder found (--profiles DIR or VSP_PROFILES_DIR)"
                        : v->indexing ? "indexing profiles..." : "no profile matches the filter";
        tui_draw_fit(g, x + 2, y, msg, TUI_ROLE_DIM, limit - x - 2, ascii);
    }
    for (int i = 0; i < list_rows && *v->scroll + i < v->nrows; i++) {
        int idx = *v->scroll + i;
        picker_row(g, x, y + i, limit, &v->rows[idx], idx == sel, v->obd_dim, ascii);
    }
    int shown = v->nrows - *v->scroll < list_rows ? v->nrows - *v->scroll : list_rows;
    if (shown < 0) shown = 0;
    tui_draw_clip_marks(g, r, *v->scroll, v->nrows - *v->scroll - shown, ascii);

    if (v->ndrafts > 0) {
        y = c.y + 3 + list_rows + 1;
        char head[64];
        snprintf(head, sizeof(head), "%s drafts from discover ", tui_glyph(TUI_G_H, ascii));
        int hx = x + 2 + tui_draw_text(g, x + 2, y, head, TUI_ROLE_DIM, limit - x - 2);
        tui_draw_hline(g, hx, y, limit - hx - 1, ascii, TUI_ROLE_DIM);
        y++;
        for (int i = 0; i < v->ndrafts && y < c.y + c.h; i++, y++) {
            bool selected = v->nrows + i == sel;
            char role = selected ? TUI_ROLE_SELECT : TUI_ROLE_DEFAULT;
            if (selected) tui_grid_fill(g, x, y, limit - x, " ", TUI_ROLE_SELECT);
            tui_draw_text(g, x, y, selected ? tui_glyph(TUI_G_ARROW, ascii) : " ", role, 1);
            char count[32];
            snprintf(count, sizeof(count), "%d signal%s", v->drafts[i].signals, v->drafts[i].signals == 1 ? "" : "s");
            int cw = tui_utf8_width(count);
            int pw = limit - (x + 2) - cw - 3;
            if (pw > 8) {
                tui_draw_fit(g, x + 2, y, v->drafts[i].file, role, pw, ascii);
                tui_draw_text(g, x + 2 + pw + 3, y, count, selected ? role : TUI_ROLE_DIM, cw);
            } else {
                tui_draw_fit(g, x + 2, y, v->drafts[i].file, role, limit - x - 2, ascii);
            }
        }
    }
}

/*******************************************************************************
 * Replay list
 ******************************************************************************/

static int replay_want_h(int nrows) {
    int list = nrows > 0 ? nrows : 1;
    return 4 + 1 + list;
}

int tui_replay_list_rows(int cols, int rows) {
    int want_w = cols - 4 < 100 ? cols - 4 : 100;
    tui_rect_t r = tui_modal_rect(cols, rows, want_w, rows);
    int n = r.h - 4 - 1;
    return n > 1 ? n : 1;
}

static void replay_row(tui_grid_t *g, int x, int y, int limit, const tui_replay_entry_t *e, bool selected,
                       bool ascii) {
    char base = selected ? TUI_ROLE_SELECT : TUI_ROLE_DEFAULT;
    if (selected) tui_grid_fill(g, x, y, limit - x, " ", TUI_ROLE_SELECT);
    tui_draw_text(g, x, y, selected ? tui_glyph(TUI_G_ARROW, ascii) : " ", base, 1);

    int avail = limit - (x + 2);
    int name_w = avail - 36;
    bool full = name_w >= 12;
    if (!full) name_w = avail;
    tui_draw_fit(g, x + 2, y, e->name, base, name_w, ascii);
    if (!full) return;

    int when_x = x + 2 + name_w + 2;
    int size_x = when_x + 16 + 2;
    int files_x = size_x + 9 + 2;
    tui_draw_text(g, when_x, y, e->when, selected ? base : TUI_ROLE_DIM, 16);
    tui_draw_right(g, size_x, y, 9, e->size, selected ? base : TUI_ROLE_DIM, ascii);
    char files[16];
    snprintf(files, sizeof(files), "%d", e->files);
    tui_draw_right(g, files_x, y, 5, files, selected ? base : TUI_ROLE_DIM, ascii);
}

void tui_draw_replay_list(tui_grid_t *g, int cols, int rows, const tui_replay_view_t *v, bool ascii) {
    int want_w = cols - 4 < 100 ? cols - 4 : 100;
    tui_rect_t r = tui_modal_rect(cols, rows, want_w, replay_want_h(v->nrows));
    char right[64];
    snprintf(right, sizeof(right), "%d capture%s", v->nrows, v->nrows == 1 ? "" : "s");
    const char *updown = tui_glyph(TUI_G_UPDOWN, ascii);
    tui_hint_t hints[3] = {
        { updown, "select", 0 },
        { tui_glyph(TUI_G_ENTER, ascii), "replay", 0 },
        { "esc", "back", 0 },
    };
    tui_rect_t c = tui_draw_modal(g, r, "Replay", right, hints, 3, ascii);
    int x = c.x + 1;
    int limit = c.x + c.w - 1;
    int y = c.y;

    int avail = limit - (x + 2);
    int name_w = avail - 36;
    bool full = name_w >= 12;
    if (!full) name_w = avail;
    tui_draw_text(g, x + 2, y, "capture", TUI_ROLE_DIM, name_w);
    if (full) {
        int when_x = x + 2 + name_w + 2;
        int size_x = when_x + 16 + 2;
        int files_x = size_x + 9 + 2;
        tui_draw_text(g, when_x, y, "when", TUI_ROLE_DIM, 16);
        tui_draw_right(g, size_x, y, 9, "size", TUI_ROLE_DIM, ascii);
        tui_draw_right(g, files_x, y, 5, "files", TUI_ROLE_DIM, ascii);
    }
    y++;

    int list_rows = r.h - 4 - 1;
    if (list_rows < 1) list_rows = 1;
    int sel = v->selected;
    if (sel < *v->scroll) *v->scroll = sel;
    if (sel >= *v->scroll + list_rows) *v->scroll = sel - list_rows + 1;
    int max_scroll = v->nrows - list_rows > 0 ? v->nrows - list_rows : 0;
    if (*v->scroll > max_scroll) *v->scroll = max_scroll;
    if (*v->scroll < 0) *v->scroll = 0;

    if (v->nrows == 0) {
        char msg[600];
        snprintf(msg, sizeof(msg), "no captures in %s", v->root ? v->root : "captures");
        tui_draw_fit(g, x + 2, y, msg, TUI_ROLE_DIM, limit - x - 2, ascii);
    }
    for (int i = 0; i < list_rows && *v->scroll + i < v->nrows; i++) {
        int idx = *v->scroll + i;
        replay_row(g, x, y + i, limit, &v->rows[idx], idx == sel, ascii);
    }
    int shown = v->nrows - *v->scroll < list_rows ? v->nrows - *v->scroll : list_rows;
    if (shown < 0) shown = 0;
    tui_draw_clip_marks(g, r, *v->scroll, v->nrows - *v->scroll - shown, ascii);
}

/*******************************************************************************
 * Connect screen
 ******************************************************************************/

void tui_draw_connect(tui_grid_t *g, int cols, int rows, const tui_connect_view_t *v, bool ascii) {
    tui_grid_clear(g);
    if (cols < 30 || rows < 10) {
        tui_grid_put(g, 0, 0, "terminal too small", TUI_ROLE_ALERT, cols);
        return;
    }
    tui_hint_t hints[6];
    int nh = 0;
    const char *enter = tui_glyph(TUI_G_ENTER, ascii);
    if (v->prompt_label) {
        hints[nh++] = (tui_hint_t){ enter, "open", 0 };
        hints[nh++] = (tui_hint_t){ "esc", "cancel", 0 };
    } else if (v->busy) {
        hints[nh++] = (tui_hint_t){ "esc", "cancel", 0 };
    } else {
        hints[nh++] = (tui_hint_t){ tui_glyph(TUI_G_UPDOWN, ascii), "select", 0 };
        hints[nh++] = (tui_hint_t){ enter, "connect", 0 };
        hints[nh++] = (tui_hint_t){ "r", "replay list", 0 };
        hints[nh++] = (tui_hint_t){ "R", "rescan", 0 };
        hints[nh++] = (tui_hint_t){ "o", "capture path", 0 };
        hints[nh++] = (tui_hint_t){ "q", "quit", 0 };
    }
    tui_rect_t full = { 0, 0, cols, rows };
    tui_rect_t c = tui_draw_modal(g, full, "can_tools", v->bt_state, hints, nh, ascii);
    int x = c.x + 2;
    int limit = c.x + c.w - 1;
    int y = c.y + 1;
    tui_draw_text(g, x, y++, "Adapters", TUI_ROLE_BRIGHT, limit - x);
    if (v->bt_note) tui_draw_fit(g, x, y++, v->bt_note, TUI_ROLE_WARN, limit - x, ascii);

    int bottom = c.y + c.h;                    /* exclusive */
    int tail = 2 + (v->last_text ? 2 : 0);     /* status + last line */
    int list_rows = bottom - y - tail - 1;
    if (list_rows < 1) list_rows = 1;
    int first = 0;
    if (v->selected < v->nrows && v->selected >= list_rows) first = v->selected - list_rows + 1;
    if (v->nrows == 0) {
        tui_draw_text(g, x + 2, y, "no adapters found yet", TUI_ROLE_DIM, limit - x - 2);
    }
    for (int i = 0; i < list_rows && first + i < v->nrows; i++) {
        const tui_connect_row_t *row = &v->rows[first + i];
        bool selected = first + i == v->selected;
        int yy = y + i;
        tui_draw_text(g, x, yy, selected ? tui_glyph(TUI_G_ARROW, ascii) : " ", TUI_ROLE_BRIGHT, 1);
        tui_draw_fit(g, x + 2, yy, row->name, selected ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT, 18, ascii);
        tui_draw_text(g, x + 21, yy, row->transport, TUI_ROLE_DEFAULT, 9);
        tui_draw_text(g, x + 31, yy, row->detail, TUI_ROLE_DIM, limit - x - 31);
    }
    y += (v->nrows > list_rows ? list_rows : (v->nrows > 0 ? v->nrows : 1)) + 1;

    if (v->prompt_label) {
        int px = x + tui_draw_text(g, x, y, v->prompt_label, TUI_ROLE_DEFAULT, limit - x);
        px += tui_draw_text(g, px, y, v->prompt_text ? v->prompt_text : "", TUI_ROLE_BRIGHT, limit - px);
        tui_draw_text(g, px, y, "_", TUI_ROLE_BRIGHT, limit - px);
    } else if (v->status) {
        tui_draw_fit(g, x, y, v->status, v->status_role ? v->status_role : TUI_ROLE_DIM, limit - x, ascii);
    }
    y += 2;
    if (v->last_text && y < bottom) {
        bool selected = v->selected == v->nrows;
        tui_draw_text(g, x, y, selected ? tui_glyph(TUI_G_ARROW, ascii) : " ", TUI_ROLE_BRIGHT, 1);
        tui_draw_fit(g, x + 2, y, v->last_text, selected ? TUI_ROLE_BRIGHT : TUI_ROLE_DEFAULT, limit - x - 2, ascii);
    }
}
