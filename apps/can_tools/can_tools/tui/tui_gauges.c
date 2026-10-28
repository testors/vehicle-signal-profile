/**
 * @file tui_gauges.c
 * @brief Gauges panel model (Pure C)
 *
 * "round" in the step 4 brief follows its reference script (Python round:
 * half to even); rint() in the default rounding mode does the same.
 */

#include "tui_gauges.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "tui_grid.h"

const unsigned char TUI_GRAD_THR[5] = { 34, 40, 46, 83, 120 };
const unsigned char TUI_GRAD_BRK[5] = { 124, 160, 196, 203, 210 };
const unsigned char TUI_GRAD_RPM[11] = { 41, 77, 113, 149, 185, 221, 215, 209, 203, 197, 196 };
const unsigned char TUI_GRAD_CYAN[7] = { 24, 31, 38, 45, 51, 87, 123 };
const unsigned char TUI_GRAD_COLD[6] = { 25, 26, 27, 33, 39, 45 };
const unsigned char TUI_GRAD_HOT[6] = { 124, 160, 196, 197, 203, 210 };

static const char *const kThrottle[] = { "accelerator.pedal.position", "throttle.position", NULL };
static const char *const kBrake[] = { "brake.pressure", "brake.pressure.front", "brake.pedal.position",
                                      "brake.switch", NULL };
static const char *const kSteering[] = { "steering.wheel.angle", NULL };
static const char *const kRpm[] = { "engine.speed", NULL };
static const char *const kSpeed[] = { "vehicle.speed", "wheel.speed", "wheel.speed.front_left", NULL };
static const char *const kGear[] = { "transmission.gear.selected", "transmission.gear.requested", NULL };
static const char *const kCoolant[] = { "engine.coolant.temperature", "thermal.coolant.temperature", NULL };
static const char *const kOil[] = { "engine.oil.temperature", NULL };
static const char *const kGearbox[] = { "transmission.oil.temperature", NULL };

const char *const *tui_gauge_canonicals(tui_gauge_id_t id) {
    switch (id) {
        case TUI_GAUGE_THROTTLE: return kThrottle;
        case TUI_GAUGE_BRAKE:    return kBrake;
        case TUI_GAUGE_STEERING: return kSteering;
        case TUI_GAUGE_RPM:      return kRpm;
        case TUI_GAUGE_SPEED:    return kSpeed;
        case TUI_GAUGE_GEAR:     return kGear;
        case TUI_GAUGE_COOLANT:  return kCoolant;
        case TUI_GAUGE_OIL:      return kOil;
        case TUI_GAUGE_GEARBOX:  return kGearbox;
        default: break;
    }
    return NULL;
}

const char *tui_gauge_label(tui_gauge_id_t id, bool narrow) {
    static const char *const kWide[TUI_GAUGE_COUNT] = { "Throttle", "Brake", "Steering", "RPM", "Speed",
                                                        "Gear", "Coolant", "Oil", "Gearbox" };
    static const char *const kNarrow[TUI_GAUGE_COUNT] = { "Thr", "Brk", "Str", "RPM", "Spd",
                                                          "Gear", "Cool", "Oil", "Gbox" };
    if ((int)id < 0 || id >= TUI_GAUGE_COUNT) return "";
    return narrow ? kNarrow[id] : kWide[id];
}

/*******************************************************************************
 * Binding and selection
 ******************************************************************************/

void tui_gauges_init(tui_gauges_t *gs) {
    memset(gs, 0, sizeof(*gs));
    for (int i = 0; i < TUI_GAUGE_COUNT; i++) gs->g[i].current = -1;
    tui_gauges_reset_scales(gs);
}

void tui_gauges_free(tui_gauges_t *gs) {
    for (int i = 0; i < TUI_GAUGE_COUNT; i++) free(gs->g[i].cand);
    memset(gs, 0, sizeof(*gs));
}

void tui_gauges_reset_scales(tui_gauges_t *gs) {
    for (int i = 0; i < TUI_GAUGE_COUNT; i++) {
        gs->g[i].scale = 0;
        gs->g[i].max_gear_seen = 0;
    }
    gs->g[TUI_GAUGE_RPM].scale = TUI_SCALE_RPM_START;
    gs->g[TUI_GAUGE_SPEED].scale = TUI_SCALE_SPEED_START;
    gs->g[TUI_GAUGE_BRAKE].scale = TUI_SCALE_BRAKE_START;
}

static int cmp_cand(const void *pa, const void *pb) {
    const tui_gauge_cand_t *a = (const tui_gauge_cand_t *)pa, *b = (const tui_gauge_cand_t *)pb;
    if (a->rank != b->rank) return a->rank - b->rank;
    if (a->diag != b->diag) return a->diag ? 1 : -1;     /* CAN frame before diagnostic */
    return a->order - b->order;                           /* then profile order */
}

void tui_gauges_bind(tui_gauges_t *gs, const tui_model_t *m) {
    tui_gauges_free(gs);
    tui_gauges_init(gs);
    for (int id = 0; id < TUI_GAUGE_COUNT; id++) {
        tui_gauge_t *g = &gs->g[id];
        const char *const *ids = tui_gauge_canonicals((tui_gauge_id_t)id);
        g->cand = (tui_gauge_cand_t *)calloc((size_t)m->signal_count + 1, sizeof(*g->cand));
        if (!g->cand) continue;
        for (int i = 0; i < m->signal_count; i++) {
            const tui_signal_t *s = &m->signals[i];
            if (!s->canonical[0]) continue;
            for (int r = 0; ids && ids[r]; r++) {
                if (strcmp(s->canonical, ids[r]) != 0) continue;
                g->cand[g->ncand++] = (tui_gauge_cand_t){ i, r, s->diagnostic, s->decoder_index };
                break;
            }
        }
        if (g->ncand > 1) qsort(g->cand, (size_t)g->ncand, sizeof(*g->cand), cmp_cand);
        g->current = g->ncand > 0 ? 0 : -1;
        if (g->ncand > 0) gs->bound++;
    }
}

static void select_candidate(tui_gauge_t *g, const tui_model_t *m, int64_t now) {
    if (g->ncand == 0) {
        g->current = -1;
        return;
    }
    int cur = g->current;
    const tui_signal_t *cs = cur >= 0 ? &m->signals[g->cand[cur].sig] : NULL;
    bool cur_live = cs && tui_signal_state(cs, now) == TUI_SIG_LIVE;

    int best = -1;
    for (int i = 0; i < g->ncand; i++) {
        const tui_signal_t *s = &m->signals[g->cand[i].sig];
        if (tui_signal_state(s, now) != TUI_SIG_LIVE) continue;
        if (best < 0 || g->cand[i].rank < g->cand[best].rank) {
            best = i;
            continue;
        }
        if (g->cand[i].rank > g->cand[best].rank || best == cur) continue;
        /* same priority: the current one, else the freshest */
        if (i == cur || s->last_ms > m->signals[g->cand[best].sig].last_ms) best = i;
    }
    if (best >= 0) {
        if (!cur_live || g->cand[best].rank < g->cand[cur].rank) g->current = best;
        return;
    }
    /* nothing live: keep the current one, unless it never had a sample and
       another candidate has data (show that, stale, instead of "—") */
    if (cs && cs->sampled) return;
    int newest = -1;
    for (int i = 0; i < g->ncand; i++) {
        const tui_signal_t *s = &m->signals[g->cand[i].sig];
        if (!s->sampled) continue;
        if (newest < 0 || s->last_ms > m->signals[g->cand[newest].sig].last_ms) newest = i;
    }
    g->current = newest >= 0 ? newest : 0;
}

double tui_scale_grow(double scale, double v, double step, double ignore) {
    if (v > ignore || v <= scale) return scale;
    return ceil(v / step) * step;
}

void tui_gauges_update(tui_gauges_t *gs, const tui_model_t *m, int64_t now_ms) {
    for (int id = 0; id < TUI_GAUGE_COUNT; id++) {
        tui_gauge_t *g = &gs->g[id];
        select_candidate(g, m, now_ms);
        const tui_signal_t *s = tui_gauge_signal(gs, m, (tui_gauge_id_t)id);
        if (!s || !s->sampled || !s->has_value || s->value_null) continue;
        double v = tui_gauge_convert((tui_gauge_id_t)id, s->unit, s->value);
        switch (id) {
            case TUI_GAUGE_RPM:
                g->scale = tui_scale_grow(g->scale, v, TUI_SCALE_RPM_STEP, TUI_SCALE_RPM_IGNORE);
                break;
            case TUI_GAUGE_SPEED:
                g->scale = tui_scale_grow(g->scale, v, TUI_SCALE_SPEED_STEP, TUI_SCALE_SPEED_IGNORE);
                break;
            case TUI_GAUGE_BRAKE:
                if (tui_brake_mode(s->canonical) == TUI_BRAKE_PRESSURE) {
                    g->scale = tui_scale_grow(g->scale, v, TUI_SCALE_BRAKE_STEP, TUI_SCALE_BRAKE_IGNORE);
                }
                break;
            case TUI_GAUGE_GEAR: {
                double r = rint(s->value);
                if (fabs(s->value - r) < 1e-6 && r >= 1 && r <= 10 && (int)r > g->max_gear_seen) {
                    g->max_gear_seen = (int)r;
                }
                break;
            }
            default:
                break;
        }
    }
}

const tui_signal_t *tui_gauge_signal(const tui_gauges_t *gs, const tui_model_t *m, tui_gauge_id_t id) {
    if ((int)id < 0 || id >= TUI_GAUGE_COUNT) return NULL;
    const tui_gauge_t *g = &gs->g[id];
    if (g->current < 0 || g->current >= g->ncand) return NULL;
    int sig = g->cand[g->current].sig;
    if (sig < 0 || sig >= m->signal_count) return NULL;
    return &m->signals[sig];
}

/*******************************************************************************
 * Units
 ******************************************************************************/

tui_brake_mode_t tui_brake_mode(const char *canonical) {
    if (canonical && strcmp(canonical, "brake.pedal.position") == 0) return TUI_BRAKE_PEDAL;
    if (canonical && strcmp(canonical, "brake.switch") == 0) return TUI_BRAKE_SWITCH;
    return TUI_BRAKE_PRESSURE;
}

bool tui_unit_is_degrees(const char *unit) {
    return unit && (strcasecmp(unit, "deg") == 0 || strcmp(unit, "°") == 0 || strcasecmp(unit, "degree") == 0 ||
                    strcasecmp(unit, "degrees") == 0);
}

double tui_gauge_convert(tui_gauge_id_t id, const char *unit, double v) {
    const char *u = unit ? unit : "";
    switch (id) {
        case TUI_GAUGE_COOLANT:
        case TUI_GAUGE_OIL:
        case TUI_GAUGE_GEARBOX:
            return tui_to_celsius(v, u);
        case TUI_GAUGE_BRAKE:
            if (strcasecmp(u, "kPa") == 0) return v / 100.0;
            if (strncasecmp(u, "psi", 3) == 0) return v * 0.0689476;
            if (strcasecmp(u, "mbar") == 0) return v / 1000.0;
            if (strcasecmp(u, "MPa") == 0) return v * 10.0;
            return v;
        case TUI_GAUGE_SPEED:
            if (strcasecmp(u, "mph") == 0) return v * 1.609344;
            if (strcasecmp(u, "m/s") == 0) return v * 3.6;
            return v;
        default:
            break;
    }
    return v;
}

int tui_gear_strip_max(int table_max, int observed_max) {
    int g = table_max > 0 ? table_max : observed_max;
    if (g > 10) g = 10;
    if (g < 6) g = 6;
    return g;
}

/*******************************************************************************
 * Bar cells
 ******************************************************************************/

int tui_grad_pick(const unsigned char *stops, int nstops, int i, int w) {
    if (nstops <= 0) return 0;
    if (w <= 1) return stops[nstops - 1];
    int k = (int)rint((double)i / (double)(w - 1) * (double)(nstops - 1));
    if (k < 0) k = 0;
    if (k >= nstops) k = nstops - 1;
    return stops[k];
}

static void cell_set(tui_bar_cell_t *c, const char *ch, int fg, char role) {
    snprintf(c->ch, sizeof(c->ch), "%s", ch);
    c->fg = (unsigned char)fg;
    c->role = role;
}

/* --ascii changes the glyphs only; colors stay. */
static void empty_cell(tui_bar_cell_t *c, const tui_bar_style_t *st) {
    cell_set(c, st->ascii ? "." : (st->colors > 0 ? "■" : "·"), TUI_FG_TRACK, TUI_ROLE_DIM);
}

static void filled_cell(tui_bar_cell_t *c, int fg, bool stale, const tui_bar_style_t *st) {
    cell_set(c, st->ascii ? "#" : "■", stale ? TUI_FG_STALE : fg, stale ? TUI_ROLE_STALE : TUI_ROLE_DEFAULT);
}

int tui_bar_meter(double frac, int w, const unsigned char *stops, int nstops, bool stale,
                  const tui_bar_style_t *st, tui_bar_cell_t *out) {
    if (w <= 0) return 0;
    if (!(frac >= 0)) frac = 0;   /* NaN too */
    if (frac > 1) frac = 1;
    int n = (int)rint(frac * w);
    if (frac > 0.02 && n == 0) n = 1;
    for (int i = 0; i < w; i++) {
        if (i < n) filled_cell(&out[i], tui_grad_pick(stops, nstops, i, w), stale, st);
        else empty_cell(&out[i], st);
    }
    return n;
}

int tui_steer_fill(double deg, int half) {
    double a = fabs(deg);
    int n = (int)rint(fmin(a, TUI_STEER_FULL_DEG) / TUI_STEER_FULL_DEG * half);
    if (a >= 5.0 && n == 0) n = 1;
    if (n > half) n = half;
    return n;
}

void tui_bar_steer(double deg, int half, bool stale, const tui_bar_style_t *st, tui_bar_cell_t *out) {
    if (half <= 0) return;
    tui_bar_cell_t side[32];
    if (half > 32) half = 32;
    int n = tui_steer_fill(deg, half);
    double a = fabs(deg);
    /* full lock (>= 450°): the whole filled bar in the brake reds, no triangle */
    bool lock = a >= TUI_STEER_LOCK_DEG;
    for (int k = 0; k < half; k++) {
        if (k >= n) {
            empty_cell(&side[k], st);
        } else if (lock) {
            cell_set(&side[k], st->ascii ? "X" : "■", stale ? TUI_FG_STALE : tui_grad_pick(TUI_GRAD_BRK, 5, k, half),
                     stale ? TUI_ROLE_STALE : TUI_ROLE_ALERT);
        } else {
            filled_cell(&side[k], tui_grad_pick(TUI_GRAD_CYAN, 7, k, half), stale, st);
        }
    }
    if (a > TUI_STEER_FULL_DEG && !lock) {
        const char *mark = deg > 0 ? (st->ascii ? "<" : "◀") : (st->ascii ? ">" : "▶");
        cell_set(&side[half - 1], mark, stale ? TUI_FG_STALE : TUI_FG_OVER, stale ? TUI_ROLE_STALE : TUI_ROLE_BRIGHT);
    }
    for (int k = 0; k < half; k++) {
        empty_cell(&out[k], st);
        empty_cell(&out[half + 1 + k], st);
    }
    if (deg > 0) {
        for (int k = 0; k < half; k++) out[half - 1 - k] = side[k];    /* left: outward from the center */
    } else if (deg < 0) {
        for (int k = 0; k < half; k++) out[half + 1 + k] = side[k];
    }
    cell_set(&out[half], st->ascii ? "|" : "┃", TUI_FG_CENTER, TUI_ROLE_DEFAULT);
}

int tui_gear_strip(int max_gear, const char *current, int width, tui_bar_cell_t *out, int out_max) {
    char labels[12][4];
    int nl = 0;
    snprintf(labels[nl++], 4, "R");
    snprintf(labels[nl++], 4, "N");
    for (int g = 1; g <= max_gear && nl < 12; g++) snprintf(labels[nl++], 4, "%d", g);
    int spaced = nl - 1;
    for (int i = 0; i < nl; i++) spaced += (int)strlen(labels[i]);
    bool sep = spaced <= width;
    int n = 0;
    for (int i = 0; i < nl && n < width && n < out_max; i++) {
        if (i > 0 && sep) {
            cell_set(&out[n++], " ", 0, TUI_ROLE_DEFAULT);
            if (n >= width || n >= out_max) break;
        }
        bool on = current && strcmp(current, labels[i]) == 0;
        for (const char *p = labels[i]; *p && n < width && n < out_max; p++) {
            char ch[2] = { *p, '\0' };
            cell_set(&out[n++], ch, on ? TUI_FG_GEAR_ON : TUI_FG_GEAR_OFF, on ? TUI_ROLE_BRIGHT : TUI_ROLE_DIM);
        }
    }
    return n;
}

void tui_gear_label(const tui_signal_t *s, char *out, size_t len) {
    if (s->label && s->label[0]) {
        snprintf(out, len, "%s", s->label);
        return;
    }
    if (!s->has_value || s->value_null) {
        snprintf(out, len, "—");
        return;
    }
    double r = rint(s->value);
    if (r == -1) snprintf(out, len, "R");
    else if (r == 0) snprintf(out, len, "N");
    else snprintf(out, len, "%d", (int)r);
}
