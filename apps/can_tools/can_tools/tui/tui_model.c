/**
 * @file tui_model.c
 * @brief TUI data model (Pure C)
 */

#include "tui_model.h"
#include "tui_grid.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static int64_t floor_div(int64_t a, int64_t b) {
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
    return q;
}

static int ring_slot(int64_t bucket, int ring) {
    int64_t m = bucket % ring;
    if (m < 0) m += ring;
    return (int)m;
}

/*******************************************************************************
 * Rate counter
 ******************************************************************************/

void tui_rate_init(tui_rate_t *r) {
    memset(r->count, 0, sizeof(r->count));
    for (int i = 0; i < TUI_RATE_RING; i++) r->bucket[i] = -1;
    r->first_ms = -1;
}

void tui_rate_add(tui_rate_t *r, int64_t ms) {
    int64_t b = floor_div(ms, TUI_RATE_BUCKET_MS);
    int slot = ring_slot(b, TUI_RATE_RING);
    if (r->bucket[slot] != b) {
        r->bucket[slot] = b;
        r->count[slot] = 0;
    }
    if (r->count[slot] < UINT16_MAX) r->count[slot]++;
    if (r->first_ms < 0) r->first_ms = ms;
}

double tui_rate_hz(const tui_rate_t *r, int64_t now_ms, int window_ms) {
    if (r->first_ms < 0) return -1.0;
    int64_t nb = window_ms / TUI_RATE_BUCKET_MS;
    if (nb > TUI_RATE_RING - 1) nb = TUI_RATE_RING - 1;
    int64_t now_b = floor_div(now_ms, TUI_RATE_BUCKET_MS);
    int64_t start_b = now_b - nb;
    int64_t first_b = floor_div(r->first_ms, TUI_RATE_BUCKET_MS);
    if (first_b > start_b) start_b = first_b;
    if (start_b >= now_b) return -1.0;
    long sum = 0;
    for (int64_t b = start_b; b < now_b; b++) {
        int slot = ring_slot(b, TUI_RATE_RING);
        if (r->bucket[slot] == b) sum += r->count[slot];
    }
    double span_s = (double)(now_b - start_b) * TUI_RATE_BUCKET_MS / 1000.0;
    return (double)sum / span_s;
}

/*******************************************************************************
 * Raw CAN table
 ******************************************************************************/

static uint32_t raw_hash(uint32_t id, bool ext) {
    uint32_t h = id * 2654435761u;
    return h ^ (ext ? 0x9E3779B9u : 0u);
}

void tui_raw_init(tui_raw_table_t *t) {
    memset(t, 0, sizeof(*t));
}

void tui_raw_free(tui_raw_table_t *t) {
    free(t->slots);
    memset(t, 0, sizeof(*t));
}

void tui_raw_clear(tui_raw_table_t *t) {
    if (t->slots) memset(t->slots, 0, (size_t)t->capacity * sizeof(*t->slots));
    t->count = 0;
}

static tui_raw_entry_t *raw_find_slot(tui_raw_entry_t *slots, int capacity, uint32_t id, bool ext) {
    uint32_t mask = (uint32_t)capacity - 1;
    uint32_t i = raw_hash(id, ext) & mask;
    for (;;) {
        tui_raw_entry_t *e = &slots[i];
        if (!e->used || (e->id == id && e->ext == ext)) return e;
        i = (i + 1) & mask;
    }
}

static bool raw_grow(tui_raw_table_t *t) {
    int capacity = t->capacity ? t->capacity * 2 : 256;
    tui_raw_entry_t *slots = (tui_raw_entry_t *)calloc((size_t)capacity, sizeof(*slots));
    if (!slots) return false;
    for (int i = 0; i < t->capacity; i++) {
        if (!t->slots[i].used) continue;
        *raw_find_slot(slots, capacity, t->slots[i].id, t->slots[i].ext) = t->slots[i];
    }
    free(t->slots);
    t->slots = slots;
    t->capacity = capacity;
    return true;
}

tui_raw_entry_t *tui_raw_update(tui_raw_table_t *t, const can_frame_t *frame, int64_t ts) {
    bool ext = frame->is_extended || frame->can_id > 0x7FF;
    if (t->capacity == 0 || (t->count + 1) * 2 > t->capacity) {
        if (t->count >= TUI_RAW_MAX_IDS && t->capacity > 0) {
            tui_raw_entry_t *e = raw_find_slot(t->slots, t->capacity, frame->can_id, ext);
            if (!e->used) return NULL;
        } else if (!raw_grow(t)) {
            return NULL;
        }
    }
    tui_raw_entry_t *e = raw_find_slot(t->slots, t->capacity, frame->can_id, ext);
    uint8_t dlc = frame->dlc > 8 ? 8 : frame->dlc;
    if (!e->used) {
        memset(e, 0, sizeof(*e));
        e->used = true;
        e->id = frame->can_id;
        e->ext = ext;
        e->first_ms = ts;
        tui_rate_init(&e->rate);
        for (int b = 0; b < 8; b++) e->byte_change_ms[b] = ts;
        e->last_change_ms = ts;
        e->dlc = dlc;
        memcpy(e->data, frame->data, dlc);
        t->count++;
    } else {
        int n = dlc > e->dlc ? dlc : e->dlc;
        for (int b = 0; b < n; b++) {
            if (b >= dlc || b >= e->dlc || e->data[b] != frame->data[b]) {
                e->byte_change_ms[b] = ts;
                e->last_change_ms = ts;
            }
        }
        e->dlc = dlc;
        memset(e->data, 0, sizeof(e->data));
        memcpy(e->data, frame->data, dlc);
    }
    e->last_ms = ts;
    e->count++;
    tui_rate_add(&e->rate, ts);
    return e;
}

double tui_raw_hz(const tui_raw_entry_t *e, int64_t now_ms) {
    double hz = tui_rate_hz(&e->rate, now_ms, TUI_RAW_HZ_WINDOW_MS);
    return hz < 0 ? 0.0 : hz;
}

bool tui_raw_matches(const tui_raw_entry_t *e, const char *filter) {
    if (!filter || !filter[0]) return true;
    const char *f = filter;
    if (f[0] == '0' && (f[1] == 'x' || f[1] == 'X')) f += 2;
    char hex[16];
    snprintf(hex, sizeof(hex), e->ext ? "%08X" : "%03X", e->id);
    size_t n = strlen(f);
    if (n == 0) return true;
    if (n > strlen(hex)) return false;
    for (size_t i = 0; i < n; i++) {
        if (!isxdigit((unsigned char)f[i])) return false;
        if (toupper((unsigned char)f[i]) != hex[i]) return false;
    }
    return true;
}

typedef struct {
    const tui_raw_entry_t *e;
    double hz;
} raw_sort_item_t;

static tui_sort_t g_sort_mode;

static int raw_id_cmp(const tui_raw_entry_t *a, const tui_raw_entry_t *b) {
    if (a->ext != b->ext) return a->ext ? 1 : -1;
    return (a->id > b->id) - (a->id < b->id);
}

static int raw_sort_cmp(const void *pa, const void *pb) {
    const raw_sort_item_t *a = (const raw_sort_item_t *)pa;
    const raw_sort_item_t *b = (const raw_sort_item_t *)pb;
    if (g_sort_mode == TUI_SORT_HZ) {
        double da = floor(a->hz * 10.0 + 0.5), db = floor(b->hz * 10.0 + 0.5);
        if (da != db) return da < db ? 1 : -1;
    } else if (g_sort_mode == TUI_SORT_CHANGED) {
        if (a->e->last_change_ms != b->e->last_change_ms)
            return a->e->last_change_ms < b->e->last_change_ms ? 1 : -1;
    }
    return raw_id_cmp(a->e, b->e);
}

int tui_raw_sorted(const tui_raw_table_t *t, tui_sort_t sort, int64_t now_ms,
                   const char *filter, const tui_raw_entry_t **out, int max) {
    if (t->count == 0 || max <= 0) return 0;
    raw_sort_item_t *items = (raw_sort_item_t *)calloc((size_t)t->count, sizeof(*items));
    if (!items) return 0;
    int n = 0;
    for (int i = 0; i < t->capacity && n < t->count; i++) {
        const tui_raw_entry_t *e = &t->slots[i];
        if (!e->used || !tui_raw_matches(e, filter)) continue;
        items[n].e = e;
        items[n].hz = tui_raw_hz(e, now_ms);
        n++;
    }
    g_sort_mode = sort;
    qsort(items, (size_t)n, sizeof(*items), raw_sort_cmp);
    int k = n < max ? n : max;
    for (int i = 0; i < k; i++) out[i] = items[i].e;
    free(items);
    return k;
}

/*******************************************************************************
 * Signals
 ******************************************************************************/

static tui_group_t group_from_name(const char *g) {
    if (!g) return TUI_GROUP_OTHER;
    if (strcmp(g, "rpm") == 0) return TUI_GROUP_RPM;
    if (strcmp(g, "gear") == 0) return TUI_GROUP_GEAR;
    if (strcmp(g, "steering") == 0) return TUI_GROUP_STEERING;
    if (strcmp(g, "throttle") == 0) return TUI_GROUP_THROTTLE;
    if (strcmp(g, "brake") == 0) return TUI_GROUP_BRAKE;
    return TUI_GROUP_OTHER;
}

/*******************************************************************************
 * Temperatures, palettes
 ******************************************************************************/

static bool ends_with(const char *s, const char *suffix) {
    size_t n = strlen(s), m = strlen(suffix);
    return n >= m && strcmp(s + n - m, suffix) == 0;
}

static bool unit_is_celsius(const char *unit) {
    return unit && (strcasecmp(unit, "C") == 0 || strcmp(unit, "°C") == 0 || strcasecmp(unit, "degC") == 0);
}

bool tui_unit_is_fahrenheit(const char *unit) {
    return unit && (strcasecmp(unit, "F") == 0 || strcmp(unit, "°F") == 0 || strcasecmp(unit, "degF") == 0);
}

double tui_to_celsius(double v, const char *unit) {
    return tui_unit_is_fahrenheit(unit) ? (v - 32.0) * 5.0 / 9.0 : v;
}

static tui_temp_kind_t temp_kind_for(const char *canonical, const char *unit) {
    if (strcmp(canonical, "engine.coolant.temperature") == 0 || strcmp(canonical, "thermal.coolant.temperature") == 0) {
        return TUI_TEMP_COOLANT;
    }
    if (strcmp(canonical, "engine.oil.temperature") == 0) return TUI_TEMP_OIL;
    if (strcmp(canonical, "transmission.oil.temperature") == 0) return TUI_TEMP_GEARBOX;
    if (ends_with(canonical, ".temperature") || unit_is_celsius(unit) || tui_unit_is_fahrenheit(unit)) {
        return TUI_TEMP_OTHER;
    }
    return TUI_TEMP_NONE;
}

static tui_palette_t palette_for(const tui_signal_t *s) {
    const char *c = s->canonical;
    if (s->temp == TUI_TEMP_COOLANT || s->temp == TUI_TEMP_OIL || s->temp == TUI_TEMP_GEARBOX) return TUI_PAL_TEMP;
    if (strcmp(c, "engine.speed") == 0 || s->group == TUI_GROUP_RPM) return TUI_PAL_RPM;
    if (strcmp(c, "accelerator.pedal.position") == 0 || strcmp(c, "throttle.position") == 0 ||
        s->group == TUI_GROUP_THROTTLE) {
        return TUI_PAL_THROTTLE;
    }
    if (strncmp(c, "brake.", 6) == 0 || s->group == TUI_GROUP_BRAKE) return TUI_PAL_BRAKE;
    return TUI_PAL_GENERIC;
}

const tui_temp_zones_t *tui_temp_zones(tui_temp_kind_t kind) {
    static const tui_temp_zones_t kCoolant = { 40, 130, 70, 110 };
    static const tui_temp_zones_t kOil = { 40, 150, 80, 130 };
    static const tui_temp_zones_t kGearbox = { 20, 140, 50, 120 };
    switch (kind) {
        case TUI_TEMP_COOLANT: return &kCoolant;
        case TUI_TEMP_OIL:     return &kOil;
        case TUI_TEMP_GEARBOX: return &kGearbox;
        default: break;
    }
    return NULL;
}

tui_zone_t tui_temp_zone(tui_temp_kind_t kind, double celsius) {
    const tui_temp_zones_t *z = tui_temp_zones(kind);
    if (!z) return TUI_ZONE_OK;
    if (celsius < z->cold_below) return TUI_ZONE_COLD;
    if (celsius >= z->hot_from) return TUI_ZONE_HOT;
    return TUI_ZONE_OK;
}

int tui_zone_text_fg(tui_zone_t z) {
    return z == TUI_ZONE_COLD ? 39 : (z == TUI_ZONE_HOT ? 196 : 46);
}

int tui_zone_cell_fg(tui_zone_t z) {
    return z == TUI_ZONE_COLD ? 33 : (z == TUI_ZONE_HOT ? 196 : 40);
}

int tui_palette_fg(tui_palette_t p, int level) {
    static const unsigned char kPal[4][4] = {
        { 31, 38, 45, 87 }, { 41, 149, 215, 196 }, { 34, 40, 46, 83 }, { 124, 160, 196, 203 },
    };
    if (level < 1) level = 1;
    if (level > 4) level = 4;
    if (p == TUI_PAL_TEMP) return 0;
    return kPal[p][level - 1];
}

void tui_signal_init(tui_signal_t *s, const tui_signal_desc_t *d) {
    memset(s, 0, sizeof(*s));
    snprintf(s->name, sizeof(s->name), "%s", d->name ? d->name : "");
    snprintf(s->unit, sizeof(s->unit), "%s", d->unit ? d->unit : "");
    s->group = group_from_name(d->group);
    s->diagnostic = d->diagnostic;
    s->discrete = d->has_value_table || s->group == TUI_GROUP_GEAR;
    s->decimals = tui_decimals_for_factor(d->factor);
    s->cycle_ms = d->cycle_ms > 0 ? d->cycle_ms : 0;
    s->decoder_index = d->decoder_index;
    s->max_gear = d->max_gear;
    {
        size_t n = 0;
        for (const char *p = d->canonical ? d->canonical : ""; *p && n + 1 < sizeof(s->canonical); p++) {
            s->canonical[n++] = (char)tolower((unsigned char)*p);
        }
        s->canonical[n] = '\0';
    }
    s->temp = temp_kind_for(s->canonical, s->unit);
    s->palette = palette_for(s);

    if (!d->diagnostic) {
        tui_format_can_id(s->src, sizeof(s->src), d->frame_id, d->frame_id > 0x7FF);
    } else {
        char bytes[40] = "";
        size_t n = 0;
        int digits = 0;
        for (const char *p = d->request_hex ? d->request_hex : ""; *p && n + 4 < sizeof(bytes); p++) {
            if (!isxdigit((unsigned char)*p)) continue;
            if (digits > 0 && digits % 2 == 0) bytes[n++] = ' ';
            bytes[n++] = (char)toupper((unsigned char)*p);
            digits++;
        }
        bytes[n] = '\0';
        bool obd = strncmp(bytes, "01", 2) == 0 && (bytes[2] == ' ' || bytes[2] == '\0');
        snprintf(s->src, sizeof(s->src), "%s", obd ? "obd" : "diag");
        snprintf(s->request, sizeof(s->request), "%s %s", obd ? "OBD" : "DIAG", bytes);
    }

    s->first_ms = -1;
    s->last_ms = -1;
    s->ts_head = 0;
    s->ts_count = 0;
    for (int i = 0; i < TUI_HIST_RING; i++) s->hist[i].bucket = -1;
}

void tui_signal_sample(tui_signal_t *s, int64_t ts, bool has_value, bool value_null,
                       double value, const char *label) {
    s->sampled = true;
    s->samples++;
    if (s->first_ms < 0) s->first_ms = ts;
    s->last_ms = ts;
    s->has_value = has_value;
    s->value_null = value_null;
    if (has_value) s->value = value;
    s->label = label;
    s->ts_ring[s->ts_head] = ts;
    s->ts_head = (s->ts_head + 1) % TUI_SIGNAL_TS_RING;
    if (s->ts_count < TUI_SIGNAL_TS_RING) s->ts_count++;

    int64_t b = floor_div(ts, TUI_HIST_BUCKET_MS);
    tui_hist_slot_t *slot = &s->hist[ring_slot(b, TUI_HIST_RING)];
    slot->bucket = b;
    slot->has_value = has_value;
    slot->value = value;
    slot->label = label;
}

double tui_signal_hz(const tui_signal_t *s, int64_t now_ms) {
    int64_t window_start = now_ms - TUI_SIGNAL_HZ_WINDOW_MS;
    int n = 0;
    int64_t t_last = 0, t_first = 0;
    for (int i = 0; i < s->ts_count; i++) {
        int idx = (s->ts_head - 1 - i + TUI_SIGNAL_TS_RING) % TUI_SIGNAL_TS_RING;
        int64_t ts = s->ts_ring[idx];
        if (ts < window_start || ts > now_ms) {
            if (ts < window_start) break;
            continue;
        }
        if (n == 0) t_last = ts;
        t_first = ts;
        n++;
    }
    if (n < 2 || t_last <= t_first) return -1.0;
    return (double)(n - 1) * 1000.0 / (double)(t_last - t_first);
}

/* The declared cycle, unless the signal measurably arrives slower than that
   (a gateway that rate-limits it): then its measured period. A stopped signal
   keeps its measured rate for the 5 s window, so it still goes stale. */
double tui_signal_period_ms(const tui_signal_t *s, int64_t now_ms) {
    double hz = tui_signal_hz(s, now_ms);
    double measured = hz > 0 ? 1000.0 / hz : 0.0;
    if (s->cycle_ms > 0) return measured > s->cycle_ms ? measured : (double)s->cycle_ms;
    if (measured > 0) return measured;
    return 1000.0;
}

tui_sig_state_t tui_signal_state(const tui_signal_t *s, int64_t now_ms) {
    if (!s->sampled) return TUI_SIG_NEVER;
    int64_t age = now_ms - s->last_ms;
    if (age > TUI_DEAD_MS) return TUI_SIG_DEAD;
    if ((double)age > 3.0 * tui_signal_period_ms(s, now_ms)) return TUI_SIG_STALE;
    return TUI_SIG_LIVE;
}

int tui_signal_compare(const void *pa, const void *pb) {
    const tui_signal_t *a = (const tui_signal_t *)pa;
    const tui_signal_t *b = (const tui_signal_t *)pb;
    if (a->diagnostic != b->diagnostic) return a->diagnostic ? 1 : -1;
    if (a->group != b->group) return (int)a->group - (int)b->group;
    int c = strcasecmp(a->name, b->name);
    if (c != 0) return c;
    c = strcmp(a->name, b->name);
    if (c != 0) return c;
    return a->decoder_index - b->decoder_index;
}

/* Latest slot with a bucket in [b_start, b_end] (and a number when need_value). */
static const tui_hist_slot_t *hist_latest(const tui_signal_t *s, int64_t b_start, int64_t b_end,
                                          bool need_value) {
    if (b_end - b_start >= TUI_HIST_RING) b_start = b_end - TUI_HIST_RING + 1;
    for (int64_t b = b_end; b >= b_start; b--) {
        const tui_hist_slot_t *slot = &s->hist[ring_slot(b, TUI_HIST_RING)];
        if (slot->bucket != b) continue;
        if (need_value && !slot->has_value) continue;
        return slot;
    }
    return NULL;
}

static void cell_range(int64_t now_ms, int cells, int i, int64_t *b_start, int64_t *b_end) {
    int64_t t0 = now_ms - TUI_HIST_WINDOW_MS;
    int64_t a = t0 + (int64_t)TUI_HIST_WINDOW_MS * i / cells;
    int64_t z = t0 + (int64_t)TUI_HIST_WINDOW_MS * (i + 1) / cells;   /* exclusive */
    *b_start = floor_div(a, TUI_HIST_BUCKET_MS);
    *b_end = floor_div(z - 1, TUI_HIST_BUCKET_MS);
    if (i > 0) {
        /* a bucket belongs to the cell its start falls in */
        int64_t prev_end = floor_div(a - 1, TUI_HIST_BUCKET_MS);
        if (*b_start == prev_end) (*b_start)++;
    }
}

/* Values of n equal slices of the last 10 s: the last sample in each slice.
   An empty slice holds the previous value while that sample is younger than
   3 periods (a slow signal draws a line; a stopped one goes blank). Returns
   whether any slice has a value, with the min/max of the real samples. */
static bool sample_series(const tui_signal_t *s, int64_t now_ms, int n, double *vals, bool *has,
                          double *mn_out, double *mx_out) {
    double mn = 0, mx = 0;
    bool any = false;
    for (int i = 0; i < n; i++) {
        int64_t bs, be;
        cell_range(now_ms, n, i, &bs, &be);
        if (be < bs) continue;
        const tui_hist_slot_t *slot = hist_latest(s, bs, be, true);
        if (!slot) continue;
        vals[i] = slot->value;
        has[i] = true;
        if (!any || slot->value < mn) mn = slot->value;
        if (!any || slot->value > mx) mx = slot->value;
        any = true;
    }
    double period = tui_signal_period_ms(s, now_ms);
    double last_v = 0;
    int64_t last_t = INT64_MIN;
    for (int i = 0; i < n; i++) {
        int64_t bs, be;
        cell_range(now_ms, n, i, &bs, &be);
        int64_t cell_end = (be + 1) * TUI_HIST_BUCKET_MS;
        if (has[i]) {
            const tui_hist_slot_t *slot = hist_latest(s, bs, be, true);
            last_v = vals[i];
            last_t = slot ? slot->bucket * TUI_HIST_BUCKET_MS : cell_end;
        } else if (last_t != INT64_MIN && (double)(cell_end - last_t) <= 3.0 * period) {
            vals[i] = last_v;
            has[i] = true;
        }
    }
    *mn_out = mn;
    *mx_out = mx;
    return any;
}

void tui_signal_sparkline(const tui_signal_t *s, int64_t now_ms, int cells, bool ascii,
                          char *out, size_t out_len) {
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (cells <= 0) return;
    double *vals = (double *)calloc((size_t)cells, sizeof(double));
    bool *has = (bool *)calloc((size_t)cells, sizeof(bool));
    if (!vals || !has) {
        free(vals);
        free(has);
        return;
    }
    double mn = 0, mx = 0;
    sample_series(s, now_ms, cells, vals, has, &mn, &mx);
    size_t n = 0;
    for (int i = 0; i < cells; i++) {
        const char *g;
        if (!has[i]) g = " ";
        else if (mx == mn) g = tui_glyph(TUI_G_FLAT, ascii);
        else {
            int level = (int)floor((vals[i] - mn) / (mx - mn) * 7.0 + 0.5);
            g = tui_spark(level, ascii);
        }
        size_t gl = strlen(g);
        if (n + gl + 1 > out_len) break;
        memcpy(out + n, g, gl);
        n += gl;
    }
    out[n] = '\0';
    free(vals);
    free(has);
}

unsigned int tui_braille_char(int left, int right) {
    static const unsigned char kLeft[5] = { 0, 0x40, 0x44, 0x46, 0x47 };
    static const unsigned char kRight[5] = { 0, 0x80, 0xA0, 0xB0, 0xB8 };
    if (left < 0) left = 0;
    if (left > 4) left = 4;
    if (right < 0) right = 0;
    if (right > 4) right = 4;
    if (left == 0 && right == 0) return 0;
    return 0x2800u + kLeft[left] + kRight[right];
}

int tui_braille_level(double v, double lo, double hi) {
    if (hi - lo < 1e-9) return 0;
    int l = 1 + (int)rint((v - lo) / (hi - lo) * 3.0);   /* round half to even, as the reference */
    return l < 1 ? 1 : (l > 4 ? 4 : l);
}

static size_t put_utf8(char *out, size_t n, size_t len, unsigned int cp) {
    if (cp < 0x80) {
        if (n + 2 > len) return n;
        out[n++] = (char)cp;
    } else if (cp < 0x800) {
        if (n + 3 > len) return n;
        out[n++] = (char)(0xC0 | (cp >> 6));
        out[n++] = (char)(0x80 | (cp & 0x3F));
    } else {
        if (n + 4 > len) return n;
        out[n++] = (char)(0xE0 | (cp >> 12));
        out[n++] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[n++] = (char)(0x80 | (cp & 0x3F));
    }
    out[n] = '\0';
    return n;
}

int tui_signal_braille(const tui_signal_t *s, int64_t now_ms, int cells, char *out, size_t out_len,
                       unsigned char *fg) {
    if (!out || out_len == 0) return 0;
    out[0] = '\0';
    if (cells <= 0) return 0;
    int ns = 2 * cells;
    double *vals = (double *)calloc((size_t)ns, sizeof(double));
    bool *has = (bool *)calloc((size_t)ns, sizeof(bool));
    if (!vals || !has) {
        free(vals);
        free(has);
        return 0;
    }
    double lo = 0, hi = 0;
    bool any = sample_series(s, now_ms, ns, vals, has, &lo, &hi);
    bool flat = !any || hi - lo < 1e-9;
    if (!flat && s->temp != TUI_TEMP_NONE) {
        double span = tui_unit_is_fahrenheit(s->unit) ? 18.0 : 10.0;
        if (hi - lo < span) {
            double mid = (hi + lo) / 2.0;
            lo = mid - span / 2.0;
            hi = mid + span / 2.0;
        }
    }
    size_t n = 0;
    int written = 0;
    for (int i = 0; i < cells; i++) {
        int a = 2 * i, b = 2 * i + 1;
        unsigned int cp = 0;
        int color = 0;
        if (has[a] || has[b]) {
            int level;
            if (flat) {
                cp = 0x2800u + (has[a] ? 0x04u : 0u) + (has[b] ? 0x20u : 0u);
                level = 2;
            } else {
                int la = has[a] ? tui_braille_level(vals[a], lo, hi) : 0;
                int lb = has[b] ? tui_braille_level(vals[b], lo, hi) : 0;
                cp = tui_braille_char(la, lb);
                level = la > lb ? la : lb;
            }
            if (s->palette == TUI_PAL_TEMP) {
                double v = has[b] ? vals[b] : vals[a];
                color = tui_zone_cell_fg(tui_temp_zone(s->temp, tui_to_celsius(v, s->unit)));
            } else {
                color = tui_palette_fg(s->palette, level);
            }
        }
        size_t before = n;
        n = cp ? put_utf8(out, n, out_len, cp) : put_utf8(out, n, out_len, ' ');
        if (n == before) break;
        if (fg) fg[i] = (unsigned char)color;
        written++;
    }
    free(vals);
    free(has);
    return written;
}

bool tui_signal_minmax(const tui_signal_t *s, int64_t now_ms, double *min, double *max) {
    int64_t b_end = floor_div(now_ms, TUI_HIST_BUCKET_MS);
    int64_t b_start = floor_div(now_ms - TUI_HIST_WINDOW_MS, TUI_HIST_BUCKET_MS) + 1;
    bool any = false;
    for (int64_t b = b_start; b <= b_end; b++) {
        const tui_hist_slot_t *slot = &s->hist[ring_slot(b, TUI_HIST_RING)];
        if (slot->bucket != b || !slot->has_value) continue;
        if (!any || slot->value < *min) *min = slot->value;
        if (!any || slot->value > *max) *max = slot->value;
        any = true;
    }
    return any;
}

void tui_signal_discrete_history(const tui_signal_t *s, int64_t now_ms, int cols,
                                 char *out, size_t out_len) {
    if (!out || out_len == 0) return;
    out[0] = '\0';
    char tokens[10][48];
    int nt = 0;
    for (int k = 0; k < 10; k++) {
        int64_t bs, be;
        cell_range(now_ms, 10, k, &bs, &be);
        if (be < bs) continue;
        const tui_hist_slot_t *slot = hist_latest(s, bs, be, false);
        if (!slot) continue;
        if (slot->label) snprintf(tokens[nt], sizeof(tokens[nt]), "%s", slot->label);
        else if (slot->has_value) tui_format_signal_value(tokens[nt], sizeof(tokens[nt]), slot->value, s->decimals);
        else continue;
        nt++;
    }
    /* Drop the oldest tokens until the rest fits. */
    for (int first = 0; first < nt; first++) {
        char buf[600] = "";
        size_t n = 0;
        for (int k = first; k < nt; k++) {
            n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s%s", k > first ? " " : "", tokens[k]);
            if (n >= sizeof(buf)) break;
        }
        if (tui_utf8_width(buf) <= cols) {
            snprintf(out, out_len, "%s", buf);
            return;
        }
    }
}

/*******************************************************************************
 * Bus classifier
 ******************************************************************************/

void tui_classifier_init(tui_classifier_t *c) {
    memset(c, 0, sizeof(*c));
    c->bus = TUI_BUS_DETECTING;
}

static void cls_drop_phase(tui_classifier_t *c) {
    if (c->phase_ready) disc_phase_free(&c->phase);
    c->phase_ready = false;
    c->accumulating = false;
}

void tui_classifier_free(tui_classifier_t *c) {
    cls_drop_phase(c);
}

void tui_classifier_start(tui_classifier_t *c, int64_t start_ms) {
    cls_drop_phase(c);
    c->started = true;
    c->paused = false;
    c->window_start_ms = start_ms;
    c->window_end_ms = start_ms + TUI_CLS_WINDOW_MS;
}

void tui_classifier_frame(tui_classifier_t *c, const can_frame_t *frame, int64_t ts) {
    if (!c->started || c->paused) return;
    if (ts < c->window_start_ms || ts >= c->window_end_ms) return;
    if (!c->accumulating) {
        disc_phase_init(&c->phase);
        c->phase_ready = true;
        c->accumulating = true;
    }
    disc_phase_add_frame(&c->phase, frame->can_id, frame->data, frame->dlc);
}

bool tui_classifier_tick(tui_classifier_t *c, int64_t now_ms) {
    if (!c->started || c->paused || now_ms < c->window_end_ms) return false;
    int frames = c->accumulating ? c->phase.total_frames : 0;
    tui_bus_t bus;
    if (frames == 0) {
        bus = TUI_BUS_SILENT;
    } else {
        disc_phase_finalize(&c->phase, TUI_CLS_WINDOW_MS / 1000.0);
        bus = disc_detect_bus_mode(&c->phase) == DISC_BUS_GATEWAY ? TUI_BUS_GATEWAY : TUI_BUS_DIRECT;
    }
    c->last_fast_ids = 0;
    for (int i = 0; frames > 0 && i < c->phase.id_count; i++) {
        if (c->phase.ids[i].frame_count / (TUI_CLS_WINDOW_MS / 1000.0) >= 20.0) c->last_fast_ids++;
    }
    cls_drop_phase(c);
    if (bus == TUI_BUS_DIRECT) c->direct_seen = true;
    c->windows++;
    c->last_window_frames = frames;
    c->window_start_ms = c->window_end_ms + (TUI_CLS_EVERY_MS - TUI_CLS_WINDOW_MS);
    c->window_end_ms = c->window_start_ms + TUI_CLS_WINDOW_MS;
    bool changed = bus != c->bus;
    c->bus = bus;
    return changed;
}

void tui_classifier_note_direct(tui_classifier_t *c) {
    c->direct_seen = true;
}

const char *tui_bus_text(tui_bus_t bus) {
    switch (bus) {
        case TUI_BUS_DIRECT:  return "PT-CAN DIRECT";
        case TUI_BUS_GATEWAY: return "GATEWAY";
        case TUI_BUS_SILENT:  return "SILENT (no frames)";
        case TUI_BUS_DETECTING: break;
    }
    return "detecting…";
}

/*******************************************************************************
 * Log
 ******************************************************************************/

static void log_add_line(tui_log_t *log, int64_t ms, const char *line, size_t len) {
    tui_log_level_t level = TUI_LOG_INFO;
    if (len >= 7 && strncmp(line, "[INFO] ", 7) == 0) {
        line += 7;
        len -= 7;
    } else if (len >= 6 && strncmp(line, "[WARN]", 6) == 0) {
        level = TUI_LOG_WARN;
    } else if (len >= 7 && strncmp(line, "[ERROR]", 7) == 0) {
        level = TUI_LOG_ERROR;
    }
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r')) len--;
    if (len == 0) return;
    tui_log_line_t *l = &log->lines[log->head];
    l->ms = ms;
    l->level = level;
    size_t n = len < sizeof(l->text) - 1 ? len : sizeof(l->text) - 1;
    memcpy(l->text, line, n);
    l->text[n] = '\0';
    log->head = (log->head + 1) % TUI_LOG_LINES;
    if (log->count < TUI_LOG_LINES) log->count++;
}

void tui_log_add(tui_log_t *log, int64_t ms, const char *text) {
    if (!text) return;
    const char *p = text;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        log_add_line(log, ms, p, len);
        if (!nl) break;
        p = nl + 1;
    }
}

const tui_log_line_t *tui_log_get(const tui_log_t *log, int i) {
    if (i < 0 || i >= log->count) return NULL;
    int idx = (log->head - 1 - i) % TUI_LOG_LINES;
    if (idx < 0) idx += TUI_LOG_LINES;
    return &log->lines[idx];
}

/*******************************************************************************
 * Model
 ******************************************************************************/

void tui_model_init(tui_model_t *m, int64_t origin_ms) {
    memset(m, 0, sizeof(*m));
    tui_raw_init(&m->raw);
    tui_classifier_init(&m->cls);
    tui_rate_init(&m->frames);
    m->origin_ms = origin_ms;
}

void tui_model_free(tui_model_t *m) {
    tui_raw_free(&m->raw);
    tui_classifier_free(&m->cls);
    free(m->signals);
    free(m->by_decoder);
    m->signals = NULL;
    m->by_decoder = NULL;
    m->signal_count = 0;
    m->decoder_count = 0;
}

bool tui_model_set_signals(tui_model_t *m, const tui_signal_desc_t *descs, int count) {
    free(m->signals);
    free(m->by_decoder);
    m->signals = NULL;
    m->by_decoder = NULL;
    m->signal_count = 0;
    m->decoder_count = 0;
    if (count <= 0) return true;
    m->signals = (tui_signal_t *)calloc((size_t)count, sizeof(*m->signals));
    if (!m->signals) return false;
    int max_dec = 0;
    for (int i = 0; i < count; i++) {
        tui_signal_init(&m->signals[i], &descs[i]);
        if (descs[i].decoder_index + 1 > max_dec) max_dec = descs[i].decoder_index + 1;
    }
    qsort(m->signals, (size_t)count, sizeof(*m->signals), tui_signal_compare);
    m->by_decoder = (int *)malloc((size_t)max_dec * sizeof(int));
    if (!m->by_decoder) {
        free(m->signals);
        m->signals = NULL;
        return false;
    }
    for (int i = 0; i < max_dec; i++) m->by_decoder[i] = -1;
    for (int i = 0; i < count; i++) m->by_decoder[m->signals[i].decoder_index] = i;
    m->signal_count = count;
    m->decoder_count = max_dec;
    return true;
}

void tui_model_frame(tui_model_t *m, const can_frame_t *frame, int64_t ts) {
    tui_raw_update(&m->raw, frame, ts);
    tui_rate_add(&m->frames, ts);
    m->frame_total++;
    tui_classifier_frame(&m->cls, frame, ts);
}

tui_signal_t *tui_model_signal_for_decoder(tui_model_t *m, int decoder_index) {
    if (decoder_index < 0 || decoder_index >= m->decoder_count) return NULL;
    int i = m->by_decoder[decoder_index];
    return i >= 0 ? &m->signals[i] : NULL;
}

double tui_model_frames_per_s(const tui_model_t *m, int64_t now_ms) {
    double hz = tui_rate_hz(&m->frames, now_ms, 1000);
    return hz < 0 ? 0.0 : hz;
}

/*******************************************************************************
 * Formatting
 ******************************************************************************/

int tui_decimals_for_factor(double factor) {
    double f = fabs(factor);
    if (f == 0.0 || fabs(f - floor(f + 0.5)) < 1e-9) return 0;
    int d = (int)ceil(-log10(f) - 1e-9);
    if (d < 0) d = 0;
    if (d > 3) d = 3;
    return d;
}

static void group_thousands(char *out, size_t len, const char *digits_with_sign) {
    /* digits_with_sign: optional '-', integer digits, optional '.' fraction */
    const char *p = digits_with_sign;
    bool neg = *p == '-';
    if (neg) p++;
    const char *dot = strchr(p, '.');
    size_t int_len = dot ? (size_t)(dot - p) : strlen(p);
    char buf[96];
    size_t n = 0;
    if (neg) buf[n++] = '-';
    for (size_t i = 0; i < int_len && n + 2 < sizeof(buf); i++) {
        if (i > 0 && (int_len - i) % 3 == 0) buf[n++] = ',';
        buf[n++] = p[i];
    }
    buf[n] = '\0';
    snprintf(out, len, "%s%s", buf, dot ? dot : "");
}

void tui_format_fixed(char *out, size_t len, double value, int decimals) {
    char raw[96];
    if (decimals < 0) decimals = 0;
    snprintf(raw, sizeof(raw), "%.*f", decimals, value);
    /* no "-0" / "-0.0" */
    bool all_zero = true;
    for (const char *p = raw; *p; p++) {
        if (*p >= '1' && *p <= '9') { all_zero = false; break; }
    }
    snprintf(out, len, "%s", (all_zero && raw[0] == '-') ? raw + 1 : raw);
}

void tui_format_value(char *out, size_t len, double value, int decimals) {
    char raw[96];
    tui_format_fixed(raw, sizeof(raw), value, decimals);
    if (fabs(value) >= 999.5) group_thousands(out, len, raw);
    else snprintf(out, len, "%s", raw);
}

void tui_format_signal_value(char *out, size_t len, double value, int decimals) {
    tui_format_value(out, len, value, decimals);
    /* 4+ integer digits (after rounding): drop the decimals */
    if (decimals > 0 && strchr(out, ',') != NULL) tui_format_value(out, len, value, 0);
}

void tui_format_age(char *out, size_t len, int64_t age_ms) {
    if (age_ms < 0) age_ms = 0;
    if (age_ms < 1000) snprintf(out, len, "%lldms", (long long)age_ms);
    else if (age_ms < 10000) snprintf(out, len, "%.1fs", (double)age_ms / 1000.0);
    else snprintf(out, len, "%.0fs", floor((double)age_ms / 1000.0));
}

void tui_format_hz(char *out, size_t len, double hz, bool ascii) {
    if (hz < 0) snprintf(out, len, "%s", tui_glyph(TUI_G_DASH, ascii));
    else if (hz >= 10.0) snprintf(out, len, "%.0f", hz);
    else snprintf(out, len, "%.1f", hz);
}

void tui_format_count(char *out, size_t len, long long n) {
    char raw[32];
    snprintf(raw, sizeof(raw), "%lld", n);
    group_thousands(out, len, raw);
}

void tui_format_mmss(char *out, size_t len, int64_t ms) {
    if (ms < 0) ms = 0;
    long long s = (long long)(ms / 1000);
    snprintf(out, len, "%02lld:%02lld", s / 60, s % 60);
}

void tui_format_hms(char *out, size_t len, int64_t ms) {
    if (ms < 0) ms = 0;
    long long s = (long long)(ms / 1000);
    snprintf(out, len, "%02lld:%02lld:%02lld", s / 3600, (s / 60) % 60, s % 60);
}

void tui_format_can_id(char *out, size_t len, uint32_t id, bool ext) {
    if (ext) snprintf(out, len, "0x%08X", id);
    else snprintf(out, len, "0x%03X", id);
}
