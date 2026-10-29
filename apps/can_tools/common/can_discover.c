/**
 * @file can_discover.c
 * @brief PT-CAN Signal Auto-Discovery — Analysis Logic (Pure C)
 *
 * disc_analyze_with_raw() pipeline:
 *   1. Group each phase's raw frames by CAN ID, and compare every CAN ID with
 *      the baseline (byte-level change scores, counter/constant filtering).
 *   2. Per signal, preselect the IDs that changed most in that signal's phase,
 *      split each into regions of adjacent changing bytes, and search raw
 *      bitfields inside each region.
 *   3. Score each candidate field:
 *        evidence     field change in the signal's phase vs baseline
 *        specificity  quiet in phases where this signal must not move
 *                     (e.g. wheel speed while stationary); phases where the
 *                     signal legitimately moves too (RPM while driving,
 *                     brake while shifting) are ignored
 *        shape        bit width, rate, sign and bit-significance fit
 *        prior        VSP corpus prior with the same ID *and* layout
 *      Candidates without a minimum amount of field-level change are dropped.
 *   4. Pick one field per signal jointly so that no two signals share bits;
 *      a signal whose best field belongs to another signal falls back to its
 *      next candidate.
 * Draft DBC and draft VSP are rendered from one shared export plan.
 */

#include "can_discover.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

/*******************************************************************************
 * Signal Names
 ******************************************************************************/

static const char *s_signal_names[DISC_SIG_COUNT] = {
    "steering", "rpm", "throttle", "brake", "gear", "wheel_speed"
};

#define DISC_PRIOR_NAME_LEN   64
#define DISC_PRIOR_UNIT_LEN   32
#define DISC_PRIOR_VALUE_LEN  32
#define DISC_PRIOR_CANON_LEN  48

/** One row of the embedded VSP corpus prior table (see can_discover_priors.h). */
typedef struct {
    disc_signal_t signal;
    uint32_t can_id;
    uint8_t  dlc;
    int      start_bit;
    int      bit_length;
    int      byte_order;
    char     sign_char;     /* '+', '-' or 'S' (sign-magnitude) */
    int      sign_bit;      /* sign-magnitude sign bit, -1 if none */
    char     name[DISC_PRIOR_NAME_LEN];
    char     factor[DISC_PRIOR_VALUE_LEN];
    char     offset[DISC_PRIOR_VALUE_LEN];
    char     min_val[DISC_PRIOR_VALUE_LEN];
    char     max_val[DISC_PRIOR_VALUE_LEN];
    char     unit[DISC_PRIOR_UNIT_LEN];
    char     canonical[DISC_PRIOR_CANON_LEN];
    int      support_count;
    const char *values_json;   /* VSP decode.values JSON array, NULL if none */
    const char *values_dbc;    /* DBC VAL_ pairs ("5 \"1\" 1 \"N\""), NULL if none */
} disc_discover_prior_t;

typedef struct {
    int      start_bit;
    int      bit_length;
    int      byte_order;
    char     sign_char;
    int      sign_bit;   /* sign-magnitude sign bit, -1 if none */
    int32_t  raw_min;
    int32_t  raw_max;
    int      active_unique;
    int      baseline_unique;
    double   score;      /* evidence + shape, used to rank guesses */
    double   shape;      /* shape part of score */
    uint64_t bitmask;
    int      seq;        /* evaluation order, for deterministic ties */
} disc_raw_guess_t;

typedef struct {
    char name[DISC_PRIOR_NAME_LEN];
    int  start_bit;
    int  bit_length;
    int  byte_order;
    char sign_char;
    int  sign_bit;           /* sign-magnitude sign bit, -1 if none */
    char factor[DISC_PRIOR_VALUE_LEN];
    char offset[DISC_PRIOR_VALUE_LEN];
    char min_val[DISC_PRIOR_VALUE_LEN];
    char max_val[DISC_PRIOR_VALUE_LEN];
    char unit[DISC_PRIOR_UNIT_LEN];
    char canonical[DISC_PRIOR_CANON_LEN];
    bool canonical_mapped;   /* canonical id came from a matched prior */
    const char *values_json; /* value table of the matched prior, NULL if none */
    const char *values_dbc;
    disc_signal_t signal;
    uint32_t can_id;
    uint64_t bitmask;
} disc_export_signal_t;

typedef struct {
    char   *data;
    size_t  len;
    size_t  cap;
    bool    failed;
} disc_string_builder_t;

static int signal_phase_index(disc_signal_t sig);

#include "can_discover_priors.h"

/*******************************************************************************
 * Small helpers
 ******************************************************************************/

static double clamp_score(double value, double lo, double hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static int dbc_big_endian_next_bit(int bit) {
    return (bit % 8 == 0) ? bit + 15 : bit - 1;
}

/** Next less-significant bit of a field in DBC numbering. */
static int less_significant_bit(int bit, int byte_order) {
    return (byte_order == 1) ? bit - 1 : dbc_big_endian_next_bit(bit);
}

/** Next more-significant bit of a field in DBC numbering. */
static int more_significant_bit(int bit, int byte_order) {
    if (byte_order == 1) return bit + 1;
    return (bit % 8 == 7) ? bit - 15 : bit + 1;
}

static uint64_t field_bitmask(int start_bit, int bit_length, int byte_order) {
    uint64_t mask = 0;
    int bit = start_bit;
    for (int i = 0; i < bit_length; i++) {
        if (bit >= 0 && bit < 64) mask |= (1ULL << bit);
        bit = (byte_order == 1) ? (bit + 1) : dbc_big_endian_next_bit(bit);
    }
    return mask;
}

static int span_bits(int64_t span) {
    int bits = 0;
    if (span <= 0) return 0;
    while (span > 0) {
        bits++;
        span >>= 1;
    }
    return bits;
}

static int32_t clamp_i32(int64_t v) {
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

static int cmp_int(const void *a, const void *b) {
    int ia = *(const int *)a;
    int ib = *(const int *)b;
    if (ia < ib) return -1;
    if (ia > ib) return 1;
    return 0;
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

/*******************************************************************************
 * Field geometry and fast extraction
 *
 * A frame payload is kept as two 64-bit words. Intel (little-endian) fields
 * are contiguous in the little-endian word, Motorola (big-endian) fields are
 * contiguous in the big-endian word, so extraction is one shift and mask.
 ******************************************************************************/

typedef struct {
    int      start;
    int      len;        /* magnitude bits for sign-magnitude fields */
    int      order;      /* 0 = Motorola/big, 1 = Intel/little */
    char     sign;       /* '+', '-' (two's complement) or 'S' (sign-magnitude) */
    int      sm_bit;     /* sign-magnitude sign bit (DBC numbering), -1 if none */
    int      shift;
    int      min_byte;
    int      max_byte;
    uint32_t mask;
    uint32_t sign_bit;   /* sign position in the raw value field_raw() returns */
    uint64_t bits;       /* DBC bit positions (bit b = byte b/8, bit b%8), incl. sm_bit */
    bool     valid;
} field_t;

/**
 * Build a field. sign 'S' needs sm_bit: the field is then an unsigned
 * magnitude and payload bit sm_bit its sign. field_raw() packs that sign above
 * the magnitude, so a raw value stays one integer per frame.
 */
static bool field_make_sm(int start, int len, int order, char sign, int sm_bit, field_t *f) {
    memset(f, 0, sizeof(*f));
    f->start = start;
    f->len = len;
    f->order = order;
    f->sm_bit = -1;
    f->sign = (sign == '-') ? '-' : '+';
    if (start < 0 || start >= 64 || len <= 0 || len > 32) return false;
    if (order != 0 && order != 1) return false;

    if (order == 1) {
        if (start + len > 64) return false;
        f->shift = start;
        f->min_byte = start / 8;
        f->max_byte = (start + len - 1) / 8;
    } else {
        int msb = (7 - start / 8) * 8 + (start % 8);
        int lsb = msb - len + 1;
        if (lsb < 0) return false;
        f->shift = lsb;
        f->min_byte = start / 8;
        f->max_byte = 7 - lsb / 8;
    }
    f->mask = (len == 32) ? 0xFFFFFFFFu : ((1u << len) - 1u);
    f->sign_bit = 1u << (len - 1);
    f->bits = field_bitmask(start, len, order);

    if (sign == 'S') {
        if (sm_bit < 0 || sm_bit >= 64 || len > 31) return false;
        if (f->bits & (1ULL << sm_bit)) return false;
        f->sign = 'S';
        f->sm_bit = sm_bit;
        f->sign_bit = 1u << len;
        f->bits |= 1ULL << sm_bit;
        if (sm_bit / 8 < f->min_byte) f->min_byte = sm_bit / 8;
        if (sm_bit / 8 > f->max_byte) f->max_byte = sm_bit / 8;
    }
    f->valid = true;
    return true;
}

static bool field_make(int start, int len, int order, char sign, field_t *f) {
    return field_make_sm(start, len, order, sign == 'S' ? '+' : sign, -1, f);
}

/** Signed reading of either kind (two's complement or sign-magnitude). */
static bool field_is_signed(const field_t *f) {
    return f->sign == '-' || f->sign == 'S';
}

static bool field_fits(const field_t *f, int dlc) {
    return f->valid && dlc > 0 && dlc <= DISC_MAX_DLC && f->max_byte < dlc;
}

static void frame_words(const can_frame_t *frame, uint64_t *le, uint64_t *be) {
    uint64_t l = 0, b = 0;
    int dlc = frame->dlc > DISC_MAX_DLC ? DISC_MAX_DLC : frame->dlc;
    for (int i = 0; i < dlc; i++) {
        l |= (uint64_t)frame->data[i] << (8 * i);
        b |= (uint64_t)frame->data[i] << (8 * (7 - i));
    }
    *le = l;
    *be = b;
}

static uint32_t field_raw(const field_t *f, uint64_t le, uint64_t be) {
    uint64_t word = (f->order == 1) ? le : be;
    uint32_t raw = (uint32_t)((word >> f->shift) & f->mask);
    if (f->sm_bit >= 0 && ((le >> f->sm_bit) & 1ULL)) raw |= f->sign_bit;
    return raw;
}

static int64_t field_value(const field_t *f, uint32_t raw) {
    if (f->sm_bit >= 0) {
        int64_t mag = (int64_t)(raw & f->mask);
        return (raw & f->sign_bit) ? -mag : mag;
    }
    if (f->sign == '-' && (raw & f->sign_bit)) {
        return (int64_t)raw - ((int64_t)1 << f->len);
    }
    return (int64_t)raw;
}

/*******************************************************************************
 * Public enum names
 ******************************************************************************/

const char *disc_signal_name(disc_signal_t sig) {
    if (sig < 0 || sig >= DISC_SIG_COUNT) return "unknown";
    return s_signal_names[sig];
}

static const char *s_conf_names[] = { "HIGH", "MEDIUM", "LOW" };
static const char *s_endian_names[] = { "unknown", "big", "little" };
static const char *s_sign_names[] = { "unknown", "unsigned", "signed", "sign_magnitude" };

const char *disc_confidence_name(disc_confidence_t conf) {
    if (conf > DISC_CONF_LOW) return "UNKNOWN";
    return s_conf_names[conf];
}

const char *disc_endian_name(disc_endian_t e) {
    if (e > DISC_ENDIAN_LITTLE) return "unknown";
    return s_endian_names[e];
}

const char *disc_sign_name(disc_sign_t s) {
    if (s > DISC_SIGN_SIGN_MAGNITUDE) return "unknown";
    return s_sign_names[s];
}

/*******************************************************************************
 * Bus Mode Detection
 ******************************************************************************/

static const char *s_bus_mode_names[] = { "direct", "gateway" };

const char *disc_bus_mode_name(disc_bus_mode_t mode) {
    if (mode > DISC_BUS_GATEWAY) return "unknown";
    return s_bus_mode_names[mode];
}

/** No rate-limited gateway relays a message at or above this rate. */
#define DISC_FAST_ID_HZ           20.0
/** Fast IDs needed for a direct tap (one chatty node is not enough). */
#define DISC_DIRECT_MIN_FAST_IDS  2
/** "Fast message" rate for RPM / wheel speed scoring. */
#define DISC_GATEWAY_FAST_HZ      1.0
#define DISC_DIRECT_FAST_HZ_MIN   5.0
#define DISC_DIRECT_FAST_HZ_MAX   30.0

/** Linear-interpolated percentile (q in 0..1) of an ascending array. */
static double sorted_percentile(const double *v, int n, double q) {
    if (n <= 0) return 0.0;
    double pos = q * (double)(n - 1);
    int lo = (int)pos;
    int hi = (lo + 1 < n) ? lo + 1 : lo;
    double w = pos - (double)lo;
    return v[lo] * (1.0 - w) + v[hi] * w;
}

/**
 * Detect bus mode from baseline capture statistics.
 *
 * Behind a central gateway (OBD port, e.g. BMW ZGW) only a few messages are
 * relayed, at low rates, so even the fastest IDs are slow. A direct tap always
 * carries some fast powertrain/chassis messages, however many slow body
 * messages share the bus, so the median rate does not separate the two: a
 * 992 GT3 chassis tap has a 4 Hz median but six IDs at 25-50 Hz.
 *
 *   >= DISC_DIRECT_MIN_FAST_IDS IDs at >= DISC_FAST_ID_HZ -> DIRECT, else GATEWAY.
 *
 * out_hz_threshold is the rate RPM / wheel speed scoring treats as fast:
 * 1 Hz behind a gateway; on a direct tap 0.8 x the 75th-percentile ID rate,
 * clamped to 5..30 Hz, so a bus whose engine messages run at 10 Hz still
 * credits them while a typical PT-CAN keeps 30 Hz.
 */
static disc_bus_mode_t detect_bus_mode(const disc_phase_t *baseline,
                                       double *out_hz_threshold) {
    *out_hz_threshold = DISC_DIRECT_FAST_HZ_MAX;
    if (!baseline || baseline->id_count == 0 || baseline->duration_s <= 0) {
        return DISC_BUS_DIRECT;
    }

    int n = baseline->id_count;
    double *hz_values = (double *)malloc((size_t)n * sizeof(*hz_values));
    if (!hz_values) return DISC_BUS_DIRECT;

    int fast_ids = 0;
    for (int i = 0; i < n; i++) {
        hz_values[i] = (double)baseline->ids[i].frame_count / baseline->duration_s;
        if (hz_values[i] >= DISC_FAST_ID_HZ) fast_ids++;
    }
    qsort(hz_values, (size_t)n, sizeof(double), cmp_double);
    double p75 = sorted_percentile(hz_values, n, 0.75);
    free(hz_values);

    if (fast_ids < DISC_DIRECT_MIN_FAST_IDS) {
        *out_hz_threshold = DISC_GATEWAY_FAST_HZ;
        return DISC_BUS_GATEWAY;
    }
    *out_hz_threshold = clamp_score(0.8 * p75, DISC_DIRECT_FAST_HZ_MIN, DISC_DIRECT_FAST_HZ_MAX);
    return DISC_BUS_DIRECT;
}

disc_bus_mode_t disc_detect_bus_mode(const disc_phase_t *baseline) {
    double threshold;
    return detect_bus_mode(baseline, &threshold);
}

/*******************************************************************************
 * Phase Capture
 ******************************************************************************/

void disc_result_init(disc_result_t *result) {
    if (!result) return;

    memset(result, 0, sizeof(*result));
    for (int i = 0; i < DISC_SIG_COUNT; i++) {
        result->signals[i].byte_idx = -1;
        result->signals[i].byte2_idx = -1;
        result->signals[i].start_bit = -1;
        result->signals[i].bit_length = 0;
        result->signals[i].byte_order = -1;
        result->signals[i].sign_char = '?';
        result->signals[i].sign_bit = -1;
    }
}

void disc_phase_init(disc_phase_t *phase) {
    memset(phase, 0, sizeof(*phase));
    phase->id_capacity = DISC_INITIAL_ID_CAPACITY;
    phase->ids = (disc_id_t *)calloc((size_t)phase->id_capacity, sizeof(*phase->ids));
    if (!phase->ids) phase->id_capacity = 0;
}

void disc_phase_free(disc_phase_t *phase) {
    if (!phase) return;
    free(phase->ids);
    phase->ids = NULL;
    phase->id_count = 0;
    phase->id_capacity = 0;
}

static void init_id_bytes(disc_id_t *id, int from, int to) {
    for (int b = from; b < to; b++) {
        id->bytes[b].min_val = 0xFF;
        id->bytes[b].max_val = 0x00;
        id->bytes[b].unique_count = 0;
        memset(id->bytes[b].seen, 0, sizeof(id->bytes[b].seen));
    }
}

/**
 * Find or create an ID slot in the phase.
 * Returns index, or -1 if storage is full or cannot grow.
 */
static int find_or_create_id(disc_phase_t *phase, uint32_t can_id, uint8_t dlc) {
    for (int i = 0; i < phase->id_count; i++) {
        if (phase->ids[i].can_id == can_id) {
            if (dlc > phase->ids[i].dlc) {
                init_id_bytes(&phase->ids[i], phase->ids[i].dlc, dlc);
                phase->ids[i].dlc = dlc;
            }
            return i;
        }
    }
    if (phase->id_count >= phase->id_capacity) {
        if (phase->id_capacity >= DISC_MAX_IDS) return -1;

        int new_capacity = phase->id_capacity > 0
                           ? phase->id_capacity * 2
                           : DISC_INITIAL_ID_CAPACITY;
        if (new_capacity > DISC_MAX_IDS) new_capacity = DISC_MAX_IDS;

        disc_id_t *new_ids = (disc_id_t *)realloc(phase->ids,
                                                  (size_t)new_capacity * sizeof(*phase->ids));
        if (!new_ids) return -1;

        memset(new_ids + phase->id_capacity, 0,
               (size_t)(new_capacity - phase->id_capacity) * sizeof(*phase->ids));
        phase->ids = new_ids;
        phase->id_capacity = new_capacity;
    }

    int idx = phase->id_count++;
    disc_id_t *id = &phase->ids[idx];
    id->can_id = can_id;
    id->dlc = dlc;
    id->frame_count = 0;
    init_id_bytes(id, 0, DISC_MAX_DLC);
    return idx;
}

void disc_phase_add_frame(disc_phase_t *phase, uint32_t can_id,
                           const uint8_t *data, uint8_t dlc) {
    if (dlc > DISC_MAX_DLC) dlc = DISC_MAX_DLC;

    int idx = find_or_create_id(phase, can_id, dlc);
    if (idx < 0) {
        phase->dropped_frames++;
        return;
    }

    disc_id_t *id = &phase->ids[idx];
    id->frame_count++;
    phase->total_frames++;

    for (int b = 0; b < dlc; b++) {
        disc_byte_t *bs = &id->bytes[b];
        uint8_t val = data[b];
        if (val < bs->min_val) bs->min_val = val;
        if (val > bs->max_val) bs->max_val = val;
        if (!bs->seen[val]) {
            bs->seen[val] = true;
            bs->unique_count++;
        }
    }
}

void disc_phase_finalize(disc_phase_t *phase, double duration_s) {
    phase->duration_s = duration_s;
}

void disc_raw_phase_init(disc_raw_phase_t *phase) {
    if (!phase) return;
    phase->frames = NULL;
    phase->frame_count = 0;
}

void disc_raw_phase_free(disc_raw_phase_t *phase) {
    if (!phase) return;
    free(phase->frames);
    phase->frames = NULL;
    phase->frame_count = 0;
}

bool disc_raw_phase_set(disc_raw_phase_t *phase,
                        const can_frame_t *frames,
                        int frame_count) {
    if (!phase || frame_count < 0) return false;

    can_frame_t *copy = NULL;
    if (frame_count > 0) {
        if (!frames) return false;
        copy = (can_frame_t *)malloc((size_t)frame_count * sizeof(*copy));
        if (!copy) return false;
        memcpy(copy, frames, (size_t)frame_count * sizeof(*copy));
    }

    free(phase->frames);
    phase->frames = copy;
    phase->frame_count = frame_count;
    return true;
}

static bool phase_has_data(const disc_phase_t *phase) {
    return phase && phase->id_count > 0 && phase->duration_s > 0;
}

/*******************************************************************************
 * Raw frames grouped by CAN ID
 ******************************************************************************/

/** All frames of one CAN ID in one phase, in capture order. */
typedef struct {
    uint32_t           can_id;
    int                count;
    const can_frame_t *frames;
    const uint64_t    *le;
    const uint64_t    *be;
    uint64_t           toggle;                    /* DBC bits that ever changed */
    uint64_t           alive;                     /* alive counter bits (view_alive_bits) */
    uint16_t           byte_unique[DISC_MAX_DLC];
} id_view_t;

typedef struct {
    can_frame_t *frames;
    uint64_t    *le;
    uint64_t    *be;
    id_view_t   *views;    /* sorted by can_id */
    int          view_count;
} raw_split_t;

typedef struct {
    uint32_t can_id;
    int      idx;
} frame_key_t;

static int cmp_frame_key(const void *a, const void *b) {
    const frame_key_t *x = (const frame_key_t *)a;
    const frame_key_t *y = (const frame_key_t *)b;
    if (x->can_id != y->can_id) return x->can_id < y->can_id ? -1 : 1;
    return (x->idx > y->idx) - (x->idx < y->idx);
}

static void raw_split_free(raw_split_t *s) {
    if (!s) return;
    free(s->frames);
    free(s->le);
    free(s->be);
    free(s->views);
    memset(s, 0, sizeof(*s));
}

/*
 * Alive counters in one ID's frames. A bit that flips in nearly every frame is
 * a counter's lowest bit; so is one whose 2-bit group with the bit above steps
 * by one odd value per frame, with a lost frame showing as a double or triple
 * step (a lost frame leaves the lowest bit unflipped: at 5 % loss only ~95 % of
 * pairs flip). The bits above belong to the counter while the group keeps
 * stepping as one value (forward by less than half its range, or by one steady
 * step). Checksum bits flip about half the time, and a signal's own jitter at
 * rest neither flips every frame nor steps steadily (it often repeats a value),
 * so neither is taken.
 */
#define DISC_ALIVE_FLIP_RATE   0.95
#define DISC_ALIVE_STEADY_RATE 0.80
#define DISC_ALIVE_MAX_BITS    8
#define DISC_ALIVE_STEP_RATE   0.60   /* dominant step of a lossy counter's low bits */
#define DISC_ALIVE_REPEAT_RATE 0.05   /* at most this many repeated values */

/** Bits [lo, lo+2) count by one odd step per frame: nearly no repeats, that
 *  step in most pairs (frames lost in between give the other steps). */
static bool view_low_bits_count(const id_view_t *v, int lo) {
    int hist[4] = {0}, total = v->count - 1;
    for (int i = 1; i < v->count; i++) {
        uint32_t a = (uint32_t)(v->le[i - 1] >> lo) & 3u;
        uint32_t b = (uint32_t)(v->le[i] >> lo) & 3u;
        hist[(b - a) & 3u]++;
    }
    if (total <= 0 || hist[0] > DISC_ALIVE_REPEAT_RATE * total) return false;
    int d = hist[1] >= hist[3] ? 1 : 3;   /* counting up or down; odd, so bit lo flips */
    return hist[d] >= DISC_ALIVE_STEP_RATE * total;
}

static bool view_group_steps_steadily(const id_view_t *v, int lo, int n) {
    uint32_t mod = 1u << n, mask = mod - 1u;
    int hist[1 << DISC_ALIVE_MAX_BITS] = {0};
    int forward = 0, total = v->count - 1;
    for (int i = 1; i < v->count; i++) {
        uint32_t a = (uint32_t)(v->le[i - 1] >> lo) & mask;
        uint32_t b = (uint32_t)(v->le[i] >> lo) & mask;
        uint32_t step = (b - a) & mask;
        hist[step]++;
        if (step >= 1 && step < (mod >> 1)) forward++;
    }
    int steady = 0;
    for (uint32_t st = 1; st < mod; st++) {
        if (hist[st] > steady) steady = hist[st];
    }
    return forward >= 0.9 * total || steady >= DISC_ALIVE_STEADY_RATE * total;
}

static uint64_t view_alive_bits(const id_view_t *v) {
    if (v->count < 8) return 0;
    uint64_t alive = 0;
    int total = v->count - 1;
    for (int bit = 0; bit < 64; bit++) {
        if (!((v->toggle >> bit) & 1ULL)) continue;
        int flips = 0;
        for (int i = 1; i < v->count; i++) {
            flips += (int)(((v->le[i] ^ v->le[i - 1]) >> bit) & 1ULL);
        }
        if (flips < DISC_ALIVE_FLIP_RATE * total &&
            !(bit + 1 < 64 && ((v->toggle >> (bit + 1)) & 1ULL) && view_low_bits_count(v, bit))) continue;
        int top = bit;
        while (top + 1 < 64 && top + 2 - bit <= DISC_ALIVE_MAX_BITS &&
               ((v->toggle >> (top + 1)) & 1ULL) &&
               view_group_steps_steadily(v, bit, top + 2 - bit)) {
            top++;
        }
        for (int b = bit; b <= top; b++) alive |= 1ULL << b;
        bit = top;
    }
    return alive;
}

static bool raw_split_build(const disc_raw_phase_t *raw, raw_split_t *s) {
    memset(s, 0, sizeof(*s));
    if (!raw || !raw->frames || raw->frame_count <= 0) return true;

    int n = raw->frame_count;
    frame_key_t *keys = (frame_key_t *)malloc((size_t)n * sizeof(*keys));
    s->frames = (can_frame_t *)malloc((size_t)n * sizeof(*s->frames));
    s->le = (uint64_t *)malloc((size_t)n * sizeof(*s->le));
    s->be = (uint64_t *)malloc((size_t)n * sizeof(*s->be));
    if (!keys || !s->frames || !s->le || !s->be) {
        free(keys);
        raw_split_free(s);
        return false;
    }

    for (int i = 0; i < n; i++) {
        keys[i].can_id = raw->frames[i].can_id;
        keys[i].idx = i;
    }
    qsort(keys, (size_t)n, sizeof(*keys), cmp_frame_key);

    int runs = 0;
    for (int i = 0; i < n; i++) {
        s->frames[i] = raw->frames[keys[i].idx];
        if (s->frames[i].dlc > DISC_MAX_DLC) s->frames[i].dlc = DISC_MAX_DLC;
        frame_words(&s->frames[i], &s->le[i], &s->be[i]);
        if (i == 0 || keys[i].can_id != keys[i - 1].can_id) runs++;
    }
    free(keys);

    s->views = (id_view_t *)calloc((size_t)runs, sizeof(*s->views));
    if (!s->views) {
        raw_split_free(s);
        return false;
    }

    int v = -1;
    for (int i = 0; i < n; i++) {
        if (v < 0 || s->frames[i].can_id != s->views[v].can_id) {
            v++;
            s->views[v].can_id = s->frames[i].can_id;
            s->views[v].frames = &s->frames[i];
            s->views[v].le = &s->le[i];
            s->views[v].be = &s->be[i];
        }
        s->views[v].count++;
    }
    s->view_count = runs;

    for (int vi = 0; vi < runs; vi++) {
        id_view_t *view = &s->views[vi];
        bool seen[DISC_MAX_DLC][256];
        memset(seen, 0, sizeof(seen));
        for (int i = 0; i < view->count; i++) {
            const can_frame_t *fr = &view->frames[i];
            for (int b = 0; b < fr->dlc; b++) {
                if (!seen[b][fr->data[b]]) {
                    seen[b][fr->data[b]] = true;
                    view->byte_unique[b]++;
                }
            }
            if (i > 0) view->toggle |= view->le[i] ^ view->le[i - 1];
        }
        view->alive = view_alive_bits(view);
    }
    return true;
}

static const id_view_t *raw_split_find(const raw_split_t *s, uint32_t can_id) {
    int lo = 0, hi = s ? s->view_count - 1 : -1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint32_t id = s->views[mid].can_id;
        if (id == can_id) return &s->views[mid];
        if (id < can_id) lo = mid + 1;
        else hi = mid - 1;
    }
    return NULL;
}

/*******************************************************************************
 * Field statistics on one view
 ******************************************************************************/

/** Unique-value counter: a generation-stamped table avoids clearing per call. */
typedef struct {
    uint16_t stamp[65536];
    uint16_t gen;
} uniq_set_t;

typedef struct {
    bool    have;
    int     n;
    int     unique;
    int     changes;
    int     plus_one;
    int64_t min;
    int64_t max;
    bool    lsb_toggles;
    bool    sign_bit_seen;
} fstats_t;

static void view_field_stats(const id_view_t *v, const field_t *f,
                             uniq_set_t *u, fstats_t *st) {
    memset(st, 0, sizeof(*st));
    if (!v || v->count == 0 || !f->valid || !u) return;

    if (++u->gen == 0) {
        memset(u->stamp, 0, sizeof(u->stamp));
        u->gen = 1;
    }

    uint32_t prev = 0;
    bool have_prev = false;
    for (int i = 0; i < v->count; i++) {
        if (v->frames[i].dlc <= f->max_byte) continue;
        uint32_t raw = field_raw(f, v->le[i], v->be[i]);
        int64_t val = field_value(f, raw);

        if (!st->have) {
            st->min = st->max = val;
            st->have = true;
        } else {
            if (val < st->min) st->min = val;
            if (val > st->max) st->max = val;
        }
        st->n++;

        int raw_bits = f->len + (f->sm_bit >= 0 ? 1 : 0);
        uint32_t key = (raw_bits <= 16) ? raw : ((raw ^ (raw >> 16)) & 0xFFFFu);
        if (u->stamp[key] != u->gen) {
            u->stamp[key] = u->gen;
            st->unique++;
        }
        if (raw & f->sign_bit) st->sign_bit_seen = true;

        if (have_prev && raw != prev) {
            st->changes++;
            if (((raw - prev) & f->mask) == 1u) st->plus_one++;
            if ((raw ^ prev) & 1u) st->lsb_toggles = true;
        }
        prev = raw;
        have_prev = true;
    }
}

/** True when the given DBC bits hold one constant value over both views. */
static bool views_bits_constant(const id_view_t *a, const id_view_t *b, uint64_t bits) {
    bool have = false;
    uint64_t value = 0;
    const id_view_t *views[2] = { a, b };
    for (int k = 0; k < 2; k++) {
        const id_view_t *v = views[k];
        if (!v) continue;
        for (int i = 0; i < v->count; i++) {
            uint64_t x = v->le[i] & bits;
            if (!have) {
                value = x;
                have = true;
            } else if (x != value) {
                return false;
            }
        }
    }
    return true;
}

/*******************************************************************************
 * Byte-level phase comparison
 ******************************************************************************/

/** Per-byte comparison result */
typedef struct {
    int     byte_idx;
    double  score;              /* change score for this byte */
    bool    is_counter;         /* rolling counter / checksum */
    bool    is_constant;        /* both phases have range == 0 */
    int     active_range;
    int     baseline_range;
    int     active_unique;
    int     baseline_unique;
} byte_cmp_t;

/** Per-CAN-ID comparison result */
typedef struct {
    uint32_t    can_id;
    uint8_t     dlc;
    double      hz;             /* frames / duration in active phase */
    double      total_score;    /* sum of non-ignored byte scores */
    int         changed_bytes;  /* count of bytes with score > 0 */
    byte_cmp_t  bytes[DISC_MAX_DLC];
    int         byte_count;
    int         best_byte;      /* byte index with highest score */
    int         second_byte;    /* byte index with second-highest score */
    int         pair_first;     /* first byte of strongest adjacent pair */
    int         pair_second;    /* second byte of strongest adjacent pair */
    double      pair_score;     /* combined score of strongest adjacent pair */
} id_cmp_t;

/** Mean jump of one byte between consecutive frames, the short way round
 *  (0..128): ~64 for a checksum, small for a sensor (noise or motion). -1
 *  without enough frames. */
static double view_byte_mean_jump(const id_view_t *v, int byte_idx) {
    if (!v || v->count < 8 || byte_idx < 0 || byte_idx >= DISC_MAX_DLC) return -1.0;
    bool have_prev = false;
    uint8_t prev = 0;
    long sum = 0;
    int pairs = 0;
    for (int i = 0; i < v->count; i++) {
        const can_frame_t *frame = &v->frames[i];
        if (frame->dlc <= byte_idx) continue;
        uint8_t value = frame->data[byte_idx];
        if (have_prev) {
            int d = (uint8_t)(value - prev);
            sum += d < 128 ? d : 256 - d;
            pairs++;
        }
        prev = value;
        have_prev = true;
    }
    return pairs >= 7 ? (double)sum / pairs : -1.0;
}

#define DISC_CRC_MEAN_JUMP 32.0   /* half a checksum's ~64 */

static double view_byte_increment_ratio(const id_view_t *v, int byte_idx) {
    if (!v || v->count < 4) return 0.0;
    if (byte_idx < 0 || byte_idx >= DISC_MAX_DLC) return 0.0;

    bool have_prev = false;
    uint8_t prev = 0;
    int changed = 0;
    int plus_one = 0;

    for (int i = 0; i < v->count; i++) {
        const can_frame_t *frame = &v->frames[i];
        if (frame->dlc <= byte_idx) continue;

        uint8_t value = frame->data[byte_idx];
        if (have_prev && value != prev) {
            changed++;
            if ((uint8_t)(value - prev) == 1) plus_one++;
        }
        prev = value;
        have_prev = true;
    }

    if (changed < 4) return 0.0;
    return (double)plus_one / (double)changed;
}

static const disc_id_t *find_id_in_phase(const disc_phase_t *phase, uint32_t can_id) {
    if (!phase) return NULL;
    for (int i = 0; i < phase->id_count; i++) {
        if (phase->ids[i].can_id == can_id) return &phase->ids[i];
    }
    return NULL;
}

/**
 * Compare one CAN ID between baseline and active phases.
 * Returns false if the ID is not present in the active phase.
 */
static bool compare_id(const disc_phase_t *baseline, const disc_phase_t *active,
                       const id_view_t *base_view, const id_view_t *act_view,
                       uint32_t can_id, id_cmp_t *out) {
    const disc_id_t *base_id = find_id_in_phase(baseline, can_id);
    const disc_id_t *act_id = find_id_in_phase(active, can_id);
    if (!act_id) return false;

    memset(out, 0, sizeof(*out));
    out->can_id = can_id;
    out->dlc = act_id->dlc;
    out->hz = (active->duration_s > 0) ?
              (double)act_id->frame_count / active->duration_s : 0;

    int n = act_id->dlc;
    if (n > DISC_MAX_DLC) n = DISC_MAX_DLC;
    out->byte_count = n;

    double best_score = -1;
    double second_score = -1;
    out->best_byte = -1;
    out->second_byte = -1;
    out->pair_first = -1;
    out->pair_second = -1;
    out->pair_score = 0;

    for (int b = 0; b < n; b++) {
        byte_cmp_t *bc = &out->bytes[b];
        bc->byte_idx = b;

        const disc_byte_t *a = &act_id->bytes[b];
        int a_range = a->max_val - a->min_val;
        int a_unique = a->unique_count;

        int b_range = 0;
        int b_unique = 0;
        if (base_id) {
            const disc_byte_t *bl = &base_id->bytes[b];
            if (bl->unique_count > 0) {
                b_range = bl->max_val - bl->min_val;
                b_unique = bl->unique_count;
            }
        }

        bc->active_range = a_range;
        bc->baseline_range = b_range;
        bc->active_unique = a_unique;
        bc->baseline_unique = b_unique;

        /* Counter/CRC: both phases have many unique values and, when the raw
         * frames are there, jump like a checksum from frame to frame. Sensor
         * noise with 16+ values at rest moves by small steps, so it stays. */
        if (a_unique > 15 && b_unique > 15) {
            double ja = view_byte_mean_jump(act_view, b), jb = view_byte_mean_jump(base_view, b);
            if (ja < 0.0 || jb < 0.0 || (ja >= DISC_CRC_MEAN_JUMP && jb >= DISC_CRC_MEAN_JUMP)) {
                bc->is_counter = true;
                continue;
            }
        }

        if (a_unique > 8 || b_unique > 8) {
            double active_inc_ratio = view_byte_increment_ratio(act_view, b);
            double base_inc_ratio = view_byte_increment_ratio(base_view, b);
            if (active_inc_ratio >= 0.78 || base_inc_ratio >= 0.78) {
                bc->is_counter = true;
                continue;
            }
        }

        if (a_range == 0 && b_range == 0) {
            bc->is_constant = true;
            continue;
        }

        double range_delta = (a_range > b_range) ? (a_range - b_range) : 0;
        double unique_delta = (a_unique > b_unique) ? (a_unique - b_unique) * 0.5 : 0;
        bc->score = range_delta + unique_delta;

        if (bc->score > 0) {
            out->changed_bytes++;
            out->total_score += bc->score;

            if (bc->score > best_score) {
                second_score = best_score;
                out->second_byte = out->best_byte;
                best_score = bc->score;
                out->best_byte = b;
            } else if (bc->score > second_score) {
                second_score = bc->score;
                out->second_byte = b;
            }
        }
    }

    for (int b = 0; b + 1 < n; b++) {
        double pair_score = out->bytes[b].score + out->bytes[b + 1].score;
        if (out->bytes[b].score <= 0 || out->bytes[b + 1].score <= 0) continue;
        if (pair_score > out->pair_score) {
            out->pair_score = pair_score;
            out->pair_first = b;
            out->pair_second = b + 1;
        }
    }

    return true;
}

static int cmp_id_cmp_score_desc(const void *a, const void *b) {
    const id_cmp_t *x = (const id_cmp_t *)a;
    const id_cmp_t *y = (const id_cmp_t *)b;
    if (x->total_score > y->total_score) return -1;
    if (x->total_score < y->total_score) return 1;
    return (x->can_id > y->can_id) - (x->can_id < y->can_id);
}

/**
 * Compare all CAN IDs between baseline and active phases.
 * Results are sorted by total_score descending. Returns the count.
 */
static int compare_phases(const disc_phase_t *baseline, const disc_phase_t *active,
                          const raw_split_t *base_split, const raw_split_t *act_split,
                          id_cmp_t *results, int max_results) {
    int count = 0;
    for (int i = 0; i < active->id_count && count < max_results; i++) {
        uint32_t can_id = active->ids[i].can_id;
        id_cmp_t cmp;
        if (compare_id(baseline, active,
                       raw_split_find(base_split, can_id),
                       raw_split_find(act_split, can_id),
                       can_id, &cmp) && cmp.total_score > 0) {
            results[count++] = cmp;
        }
    }
    qsort(results, (size_t)count, sizeof(*results), cmp_id_cmp_score_desc);
    return count;
}

/*******************************************************************************
 * Legacy per-phase classifiers (disc_classify / disc_classify_with_raw)
 ******************************************************************************/

static bool is_excluded(uint32_t can_id, const uint32_t *excl, int excl_count) {
    for (int i = 0; i < excl_count; i++) {
        if (excl[i] == can_id) return true;
    }
    return false;
}

static void fill_candidate(disc_candidate_t *cand, const id_cmp_t *cmp) {
    memset(cand, 0, sizeof(*cand));
    cand->can_id = cmp->can_id;
    cand->dlc = cmp->dlc;
    cand->hz = cmp->hz;
    cand->byte_idx = cmp->best_byte;
    cand->byte2_idx = -1;
    cand->score = cmp->total_score;
    cand->start_bit = -1;
    cand->bit_length = 0;
    cand->byte_order = -1;
    cand->sign_char = '?';
    cand->sign_bit = -1;
    cand->evidence_score = cmp->total_score;
}

static bool cmp_has_adjacent_pair(const id_cmp_t *cmp, double min_secondary_score) {
    if (!cmp || cmp->pair_first < 0 || cmp->pair_second < 0) return false;
    if (cmp->bytes[cmp->pair_first].score < min_secondary_score) return false;
    if (cmp->bytes[cmp->pair_second].score < min_secondary_score) return false;
    return true;
}

static void candidate_use_adjacent_pair(disc_candidate_t *cand, const id_cmp_t *cmp) {
    if (!cand || !cmp) return;
    if (cmp->pair_first < 0 || cmp->pair_second < 0) return;
    cand->byte_idx = cmp->pair_first;
    cand->byte2_idx = cmp->pair_second;
}

static int find_best(const id_cmp_t *results, int count,
                     const uint32_t *excl, int excl_count) {
    for (int i = 0; i < count; i++) {
        if (!is_excluded(results[i].can_id, excl, excl_count)) return i;
    }
    return -1;
}

static bool classify_steering(const id_cmp_t *results, int count,
                              const uint32_t *excl, int excl_count,
                              disc_candidate_t *out) {
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;
        const id_cmp_t *r = &results[i];
        if (cmp_has_adjacent_pair(r, 1.0)) {
            fill_candidate(out, r);
            candidate_use_adjacent_pair(out, r);
            out->confidence = DISC_CONF_HIGH;
            return true;
        }
    }

    int idx = find_best(results, count, excl, excl_count);
    if (idx >= 0) {
        fill_candidate(out, &results[idx]);
        out->confidence = (results[idx].total_score >= 20.0)
                          ? DISC_CONF_MEDIUM : DISC_CONF_LOW;
        return true;
    }
    return false;
}

static void classify_throttle_rpm(const id_cmp_t *results, int count,
                                  const uint32_t *excl, int excl_count,
                                  disc_candidate_t *throttle_out, bool *throttle_found,
                                  disc_candidate_t *rpm_out, bool *rpm_found,
                                  double hz_threshold) {
    *throttle_found = false;
    *rpm_found = false;

    int top_idx[8];
    int top_count = 0;
    for (int i = 0; i < count && top_count < 8; i++) {
        if (!is_excluded(results[i].can_id, excl, excl_count)) top_idx[top_count++] = i;
    }
    if (top_count == 0) return;

    if (top_count == 1) {
        fill_candidate(rpm_out, &results[top_idx[0]]);
        if (cmp_has_adjacent_pair(&results[top_idx[0]], 1.0))
            candidate_use_adjacent_pair(rpm_out, &results[top_idx[0]]);
        rpm_out->confidence = DISC_CONF_MEDIUM;
        *rpm_found = true;
        return;
    }

    int rpm_idx = -1;
    int throttle_idx = -1;
    double best_hz = -1;
    for (int i = 0; i < top_count; i++) {
        const id_cmp_t *r = &results[top_idx[i]];
        if (r->changed_bytes >= 2 && r->hz > best_hz) {
            best_hz = r->hz;
            rpm_idx = i;
        }
    }
    if (rpm_idx < 0) {
        best_hz = -1;
        for (int i = 0; i < top_count; i++) {
            if (results[top_idx[i]].hz > best_hz) {
                best_hz = results[top_idx[i]].hz;
                rpm_idx = i;
            }
        }
    }

    int best_simplicity = 9999;
    for (int i = 0; i < top_count; i++) {
        if (i == rpm_idx) continue;
        const id_cmp_t *r = &results[top_idx[i]];
        int simplicity = r->changed_bytes * 10 + r->dlc;
        if (simplicity < best_simplicity) {
            best_simplicity = simplicity;
            throttle_idx = i;
        }
    }

    if (rpm_idx >= 0) {
        const id_cmp_t *rr = &results[top_idx[rpm_idx]];
        fill_candidate(rpm_out, rr);
        if (cmp_has_adjacent_pair(rr, 1.0)) candidate_use_adjacent_pair(rpm_out, rr);
        rpm_out->confidence = (rr->changed_bytes >= 2 && rr->hz >= hz_threshold)
                              ? DISC_CONF_HIGH : DISC_CONF_MEDIUM;
        *rpm_found = true;
    }
    if (throttle_idx >= 0) {
        fill_candidate(throttle_out, &results[top_idx[throttle_idx]]);
        throttle_out->confidence = DISC_CONF_HIGH;
        *throttle_found = true;
    }
}

static bool classify_brake(const id_cmp_t *results, int count,
                           const uint32_t *excl, int excl_count,
                           disc_candidate_t *out) {
    int idx = find_best(results, count, excl, excl_count);
    if (idx < 0) return false;
    fill_candidate(out, &results[idx]);
    if (cmp_has_adjacent_pair(&results[idx], 2.0))
        candidate_use_adjacent_pair(out, &results[idx]);
    out->confidence = (results[idx].total_score >= 20.0) ? DISC_CONF_HIGH
                    : (results[idx].total_score >= 8.0) ? DISC_CONF_MEDIUM
                    : DISC_CONF_LOW;
    return true;
}

static bool byte_looks_like_enum(const byte_cmp_t *bc) {
    return !bc->is_counter && !bc->is_constant &&
           bc->active_unique >= 3 && bc->active_unique <= 8 &&
           bc->baseline_unique <= 2 && bc->score > 0;
}

static bool classify_gear(const id_cmp_t *results, int count,
                          const uint32_t *excl, int excl_count,
                          disc_candidate_t *out) {
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;
        const id_cmp_t *r = &results[i];
        for (int b = 0; b < r->byte_count; b++) {
            if (byte_looks_like_enum(&r->bytes[b])) {
                fill_candidate(out, r);
                out->byte_idx = b;
                out->byte2_idx = -1;
                out->confidence = DISC_CONF_HIGH;
                return true;
            }
        }
    }

    int idx = find_best(results, count, excl, excl_count);
    if (idx < 0) return false;
    fill_candidate(out, &results[idx]);
    out->confidence = DISC_CONF_LOW;
    return true;
}

static bool classify_wheel_speed(const id_cmp_t *results, int count,
                                 const uint32_t *excl, int excl_count,
                                 disc_candidate_t *out,
                                 double hz_threshold) {
    for (int pass = 0; pass < 2; pass++) {
        int min_bytes = (pass == 0) ? 4 : 2;
        for (int i = 0; i < count; i++) {
            if (is_excluded(results[i].can_id, excl, excl_count)) continue;
            const id_cmp_t *r = &results[i];
            if (r->changed_bytes >= min_bytes && r->hz >= hz_threshold) {
                fill_candidate(out, r);
                if (cmp_has_adjacent_pair(r, 1.0)) candidate_use_adjacent_pair(out, r);
                out->confidence = (pass == 0) ? DISC_CONF_HIGH : DISC_CONF_MEDIUM;
                return true;
            }
        }
    }

    int idx = find_best(results, count, excl, excl_count);
    if (idx < 0) return false;
    fill_candidate(out, &results[idx]);
    if (cmp_has_adjacent_pair(&results[idx], 1.0))
        candidate_use_adjacent_pair(out, &results[idx]);
    out->confidence = DISC_CONF_LOW;
    return true;
}

/*******************************************************************************
 * Signal Characterization
 ******************************************************************************/

static bool msb_looks_signed(const disc_byte_t *msb) {
    bool near_zero = false;     /* values in [0x00, 0x1F] */
    bool near_ff = false;       /* values in [0xE0, 0xFF] */
    bool has_middle = false;    /* values in [0x40, 0xBF] */

    for (int v = 0; v <= 0x1F; v++)
        if (msb->seen[v]) { near_zero = true; break; }
    for (int v = 0xE0; v <= 0xFF; v++)
        if (msb->seen[v]) { near_ff = true; break; }
    for (int v = 0x40; v <= 0xBF; v++)
        if (msb->seen[v]) { has_middle = true; break; }

    return near_zero && near_ff && !has_middle;
}

/** Byte-granular DBC layout from byte_idx/byte2_idx/endianness. */
static void candidate_byte_layout(const disc_candidate_t *cand,
                                  int *start_bit, int *bit_length,
                                  int *byte_order, char *sign_char) {
    bool is_word = cand->byte2_idx >= 0 && abs(cand->byte_idx - cand->byte2_idx) == 1;
    *sign_char = (cand->signedness == DISC_SIGN_SIGNED) ? '-' : '+';

    if (!is_word) {
        *start_bit = cand->byte_idx * 8;
        *bit_length = 8;
        *byte_order = 1;
        return;
    }

    *bit_length = 16;
    if (cand->endianness == DISC_ENDIAN_BIG) {
        *start_bit = cand->byte_idx * 8 + 7;
        *byte_order = 0;
    } else {
        *start_bit = cand->byte2_idx * 8;
        *byte_order = 1;
    }
}

static void candidate_fill_missing_layout(disc_candidate_t *cand) {
    if (cand->start_bit >= 0) return;
    int start_bit, bit_length, byte_order;
    char sign_char;
    candidate_byte_layout(cand, &start_bit, &bit_length, &byte_order, &sign_char);
    cand->start_bit = start_bit;
    cand->bit_length = bit_length;
    cand->byte_order = byte_order;
    cand->sign_char = sign_char;
}

void disc_characterize(const disc_phase_t *active, disc_candidate_t *cand) {
    cand->endianness = DISC_ENDIAN_UNKNOWN;
    cand->signedness = DISC_SIGN_UNKNOWN;
    cand->raw_min = 0;
    cand->raw_max = 0;

    const disc_id_t *id = find_id_in_phase(active, cand->can_id);
    if (!id || cand->byte_idx < 0) return;

    if (cand->byte2_idx >= 0 && abs(cand->byte_idx - cand->byte2_idx) != 1) {
        cand->byte2_idx = -1;
    }

    if (cand->byte2_idx < 0) {
        const disc_byte_t *b = &id->bytes[cand->byte_idx];
        cand->signedness = DISC_SIGN_UNSIGNED;
        cand->raw_min = b->min_val;
        cand->raw_max = b->max_val;
        candidate_fill_missing_layout(cand);
        return;
    }

    const disc_byte_t *b1 = &id->bytes[cand->byte_idx];
    const disc_byte_t *b2 = &id->bytes[cand->byte2_idx];
    int range1 = b1->max_val - b1->min_val;
    int range2 = b2->max_val - b2->min_val;

    /* MSB has smaller range (fewer distinct high-order values) */
    int msb_idx, lsb_idx;
    const disc_byte_t *msb, *lsb;
    if (range1 <= range2) {
        msb_idx = cand->byte_idx;  lsb_idx = cand->byte2_idx;
        msb = b1; lsb = b2;
    } else {
        msb_idx = cand->byte2_idx; lsb_idx = cand->byte_idx;
        msb = b2; lsb = b1;
    }

    cand->endianness = (msb_idx < lsb_idx) ? DISC_ENDIAN_BIG : DISC_ENDIAN_LITTLE;
    cand->byte_idx = msb_idx;
    cand->byte2_idx = lsb_idx;

    bool is_signed = msb_looks_signed(msb);
    cand->signedness = is_signed ? DISC_SIGN_SIGNED : DISC_SIGN_UNSIGNED;

    uint16_t u_min = ((uint16_t)msb->min_val << 8) | lsb->min_val;
    uint16_t u_max = ((uint16_t)msb->max_val << 8) | lsb->max_val;
    if (is_signed) {
        cand->raw_min = (int16_t)u_min;
        cand->raw_max = (int16_t)u_max;
        if (cand->raw_min > cand->raw_max) {
            int32_t tmp = cand->raw_min;
            cand->raw_min = cand->raw_max;
            cand->raw_max = tmp;
        }
    } else {
        cand->raw_min = u_min;
        cand->raw_max = u_max;
    }
    candidate_fill_missing_layout(cand);
}

/**
 * Set a candidate's bitfield. byte_idx is the most significant byte the field
 * touches and byte2_idx the next less significant one (JSON byte/byte2).
 */
static void candidate_set_layout(disc_candidate_t *cand, const field_t *f) {
    cand->start_bit = f->start;
    cand->bit_length = f->len;
    cand->byte_order = f->order;
    cand->sign_char = f->sign;
    cand->sign_bit = f->sm_bit;
    cand->signedness = (f->sign == 'S') ? DISC_SIGN_SIGN_MAGNITUDE :
                       (f->sign == '-') ? DISC_SIGN_SIGNED : DISC_SIGN_UNSIGNED;

    int lo = -1, hi = -1;
    for (int b = 0; b < DISC_MAX_DLC; b++) {
        if ((f->bits & (0xFFULL << (b * 8))) == 0) continue;
        if (lo < 0) lo = b;
        hi = b;
    }
    if (lo < 0 || lo == hi) {
        cand->byte_idx = lo;
        cand->byte2_idx = -1;
        cand->endianness = DISC_ENDIAN_UNKNOWN;
        return;
    }
    if (f->order == 1) {
        cand->byte_idx = hi;
        cand->byte2_idx = hi - 1;
        cand->endianness = DISC_ENDIAN_LITTLE;
    } else {
        cand->byte_idx = lo;
        cand->byte2_idx = lo + 1;
        cand->endianness = DISC_ENDIAN_BIG;
    }
}

/*******************************************************************************
 * Per-Signal Classification (Public API, legacy)
 ******************************************************************************/

bool disc_classify_with_raw(const disc_phase_t *baseline,
                            const disc_raw_phase_t *baseline_raw,
                            const disc_phase_t *active,
                            const disc_raw_phase_t *active_raw,
                            disc_signal_t signal,
                            const uint32_t *excl, int excl_count,
                            disc_candidate_t *out,
                            disc_candidate_t *rpm_out, bool *rpm_found) {
    if (rpm_found) *rpm_found = false;
    if (!baseline || !active || !out) return false;

    double hz_threshold;
    detect_bus_mode(baseline, &hz_threshold);

    memset(out, 0, sizeof(*out));
    out->byte_idx = -1;
    out->byte2_idx = -1;

    int max_results = active->id_count > 0 ? active->id_count : 1;
    id_cmp_t *cmp_buf = (id_cmp_t *)calloc((size_t)max_results, sizeof(*cmp_buf));
    raw_split_t base_split, act_split;
    bool split_ok = raw_split_build(baseline_raw, &base_split);
    split_ok = raw_split_build(active_raw, &act_split) && split_ok;
    if (!cmp_buf || !split_ok) {
        free(cmp_buf);
        raw_split_free(&base_split);
        raw_split_free(&act_split);
        return false;
    }

    int n = compare_phases(baseline, active, &base_split, &act_split, cmp_buf, max_results);
    bool found = false;

    switch (signal) {
    case DISC_SIG_STEERING:
        found = classify_steering(cmp_buf, n, excl, excl_count, out);
        break;
    case DISC_SIG_THROTTLE: {
        bool t_found = false, r_found = false;
        disc_candidate_t t_cand, r_cand;
        memset(&t_cand, 0, sizeof(t_cand));
        memset(&r_cand, 0, sizeof(r_cand));
        classify_throttle_rpm(cmp_buf, n, excl, excl_count,
                              &t_cand, &t_found, &r_cand, &r_found, hz_threshold);
        if (t_found) *out = t_cand;
        if (rpm_out && rpm_found) {
            *rpm_found = r_found;
            if (r_found) *rpm_out = r_cand;
        }
        found = t_found;
        break;
    }
    case DISC_SIG_BRAKE:
        found = classify_brake(cmp_buf, n, excl, excl_count, out);
        break;
    case DISC_SIG_GEAR:
        found = classify_gear(cmp_buf, n, excl, excl_count, out);
        break;
    case DISC_SIG_WHEEL_SPEED:
        found = classify_wheel_speed(cmp_buf, n, excl, excl_count, out, hz_threshold);
        break;
    case DISC_SIG_RPM:      /* detected via DISC_SIG_THROTTLE */
    default:
        found = false;
        break;
    }

    free(cmp_buf);
    raw_split_free(&base_split);
    raw_split_free(&act_split);
    return found;
}

bool disc_classify(const disc_phase_t *baseline,
                   const disc_phase_t *active,
                   disc_signal_t signal,
                   const uint32_t *excl, int excl_count,
                   disc_candidate_t *out,
                   disc_candidate_t *rpm_out, bool *rpm_found) {
    return disc_classify_with_raw(baseline, NULL, active, NULL,
                                  signal, excl, excl_count,
                                  out, rpm_out, rpm_found);
}

/*******************************************************************************
 * Analysis context
 ******************************************************************************/

typedef struct {
    id_cmp_t  *rows;     /* sorted by total_score desc */
    int        count;
    id_cmp_t **by_id;    /* sorted by can_id */
} cmp_table_t;

typedef struct {
    const disc_phase_t *phases;
    bool                has_raw;
    raw_split_t         split[DISC_NUM_PHASES];
    cmp_table_t         cmp[DISC_NUM_PHASES];
    uniq_set_t         *uniq;
    double              hz_threshold;
    disc_bus_mode_t     bus_mode;
} disc_ctx_t;

static int cmp_row_ptr_by_id(const void *a, const void *b) {
    const id_cmp_t *x = *(const id_cmp_t *const *)a;
    const id_cmp_t *y = *(const id_cmp_t *const *)b;
    return (x->can_id > y->can_id) - (x->can_id < y->can_id);
}

static const id_cmp_t *cmp_table_find(const cmp_table_t *t, uint32_t can_id) {
    int lo = 0, hi = t->count - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint32_t id = t->by_id[mid]->can_id;
        if (id == can_id) return t->by_id[mid];
        if (id < can_id) lo = mid + 1;
        else hi = mid - 1;
    }
    return NULL;
}

static void ctx_free(disc_ctx_t *ctx) {
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        raw_split_free(&ctx->split[p]);
        free(ctx->cmp[p].rows);
        free(ctx->cmp[p].by_id);
    }
    free(ctx->uniq);
    memset(ctx, 0, sizeof(*ctx));
}

static bool ctx_init(disc_ctx_t *ctx, const disc_phase_t phases[DISC_NUM_PHASES],
                     const disc_raw_phase_t *raw_phases, bool with_tables) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->phases = phases;
    ctx->bus_mode = detect_bus_mode(&phases[DISC_PHASE_BASELINE], &ctx->hz_threshold);

    ctx->uniq = (uniq_set_t *)calloc(1, sizeof(*ctx->uniq));
    if (!ctx->uniq) return false;

    if (raw_phases) {
        for (int p = 0; p < DISC_NUM_PHASES; p++) {
            if (!raw_split_build(&raw_phases[p], &ctx->split[p])) {
                ctx_free(ctx);
                return false;
            }
            if (ctx->split[p].view_count > 0) ctx->has_raw = true;
        }
    }

    if (!with_tables) return true;

    for (int p = 1; p < DISC_NUM_PHASES; p++) {
        const disc_phase_t *active = &phases[p];
        if (!phase_has_data(active)) continue;

        cmp_table_t *t = &ctx->cmp[p];
        t->rows = (id_cmp_t *)calloc((size_t)active->id_count, sizeof(*t->rows));
        t->by_id = (id_cmp_t **)calloc((size_t)active->id_count, sizeof(*t->by_id));
        if (!t->rows || !t->by_id) {
            ctx_free(ctx);
            return false;
        }
        t->count = compare_phases(&phases[DISC_PHASE_BASELINE], active,
                                  &ctx->split[DISC_PHASE_BASELINE], &ctx->split[p],
                                  t->rows, active->id_count);
        for (int i = 0; i < t->count; i++) t->by_id[i] = &t->rows[i];
        qsort(t->by_id, (size_t)t->count, sizeof(*t->by_id), cmp_row_ptr_by_id);
    }
    return true;
}

static const id_view_t *ctx_view(const disc_ctx_t *ctx, int phase, uint32_t can_id) {
    if (!ctx->has_raw) return NULL;
    return raw_split_find(&ctx->split[phase], can_id);
}

/*******************************************************************************
 * Scoring building blocks
 ******************************************************************************/

/**
 * Phases in which the given signal must stay still. A field that moves there
 * is probably something else. Phases where the signal legitimately moves too
 * (RPM/throttle/steering while driving, brake held while shifting) are not
 * listed, so they neither help nor hurt.
 */
static bool phase_is_contrast(disc_signal_t sig, int phase) {
    switch (sig) {
    case DISC_SIG_STEERING:
        return phase == DISC_PHASE_THROTTLE || phase == DISC_PHASE_BRAKE ||
               phase == DISC_PHASE_GEAR;
    case DISC_SIG_RPM:
        return phase == DISC_PHASE_STEERING || phase == DISC_PHASE_BRAKE;
    case DISC_SIG_THROTTLE:
        return phase == DISC_PHASE_STEERING || phase == DISC_PHASE_BRAKE ||
               phase == DISC_PHASE_GEAR;
    case DISC_SIG_BRAKE:
        return phase == DISC_PHASE_STEERING || phase == DISC_PHASE_THROTTLE;
    case DISC_SIG_GEAR:
        return phase == DISC_PHASE_STEERING || phase == DISC_PHASE_THROTTLE ||
               phase == DISC_PHASE_BRAKE;
    case DISC_SIG_WHEEL_SPEED:
        return phase == DISC_PHASE_STEERING || phase == DISC_PHASE_THROTTLE ||
               phase == DISC_PHASE_BRAKE || phase == DISC_PHASE_GEAR;
    default:
        return false;
    }
}

static double specificity_from_ratio(double ratio) {
    if (ratio <= 0.15) return 20.0;
    if (ratio <= 0.35) return 10.0;
    if (ratio <= 0.60) return 0.0;
    if (ratio <= 0.90) return -15.0;
    return -30.0;
}

/** How much a field moved in one phase relative to baseline (0..60). */
static double field_activity(disc_signal_t sig, const fstats_t *p, const fstats_t *base) {
    if (!p->have) return 0.0;
    int base_unique = base->have ? base->unique : 1;

    if (sig == DISC_SIG_GEAR) {
        if (p->unique <= base_unique) return 0.0;
        int u = p->unique > 6 ? 6 : p->unique;
        return 8.0 * u;
    }

    int gain = span_bits(p->max - p->min) - (base->have ? span_bits(base->max - base->min) : 0);
    double unique_gain = (double)(p->unique - base_unique);
    double score = (gain > 0 ? gain * 3.0 : 0.0) + clamp_score(unique_gain * 0.25, 0.0, 20.0);
    return score > 60.0 ? 60.0 : score;
}

/**
 * Minimum field-level evidence for accepting a candidate. Weak changes (a
 * byte ticking 50 -> 51) never pass, whatever bonuses they would collect.
 */
static bool evidence_gate(disc_signal_t sig, const fstats_t *a, const fstats_t *b,
                          bool *strong) {
    *strong = false;
    if (!a->have) return false;
    int base_unique = b->have ? b->unique : 1;

    if (sig == DISC_SIG_GEAR) {
        bool ok = a->unique >= 3 && a->unique <= 16 && base_unique <= 2;
        *strong = ok && a->unique <= 8 && base_unique <= 1;
        return ok;
    }

    int gain = span_bits(a->max - a->min) - (b->have ? span_bits(b->max - b->min) : 0);
    bool ok = a->unique >= 4 && gain >= 2;
    *strong = ok && a->unique >= 16 && gain >= 4;
    return ok;
}

/** Role fit for a candidate field: width, rate, sign, message shape. */
static double role_score(disc_signal_t sig, const id_cmp_t *row, const field_t *f,
                         const fstats_t *a, const fstats_t *b, double hz_threshold,
                         int wheel_parts) {
    double s = 0.0;
    int span = span_bits(a->max - a->min);
    int base_unique = b->have ? b->unique : 1;

    switch (sig) {
    case DISC_SIG_STEERING:
        if (field_is_signed(f) && a->min < 0 && a->max > 0) s += 10.0;
        if (f->len >= 12) s += 4.0;
        if (span >= 8) s += 4.0;
        break;
    case DISC_SIG_RPM:
        if (row->hz >= hz_threshold) s += 6.0;
        if (span >= 10) s += 8.0;
        else if (span < 8) s -= 6.0;
        if (f->len >= 12) s += 4.0;
        if (row->changed_bytes >= 2) s += 4.0;
        if (field_is_signed(f)) s -= 6.0;
        /* Idle speed jitters and sits well above zero; a released pedal does not. */
        if (b->have && b->unique > 1) s += 5.0;
        if (b->have && b->min > (a->max - a->min) / 20) s += 3.0;
        break;
    case DISC_SIG_THROTTLE:
        if (span >= 6 && span <= 12) s += 8.0;
        if (span >= 14) s -= 10.0;
        if (f->len <= 12) s += 4.0;
        if (row->changed_bytes <= 2) s += 4.0;
        if (field_is_signed(f)) s -= 6.0;
        if (b->have && b->unique <= 1) s += 4.0;   /* released pedal reads one value */
        break;
    case DISC_SIG_BRAKE:
        if (span >= 6) s += 4.0;
        if (base_unique <= 2) s += 4.0;
        if (row->changed_bytes <= 3) s += 2.0;
        if (field_is_signed(f)) s -= 6.0;
        break;
    case DISC_SIG_GEAR:
        if (a->unique >= 3 && a->unique <= 8) s += 10.0;
        if (f->len == 3 || f->len == 4 || f->len == 8) s += 4.0;
        if (row->hz >= hz_threshold * 4.0) s -= 4.0;
        break;
    case DISC_SIG_WHEEL_SPEED:
        if (wheel_parts >= 4) s += 16.0;
        else if (wheel_parts >= 2) s += 8.0;
        if (row->hz >= hz_threshold) s += 6.0;
        if (base_unique <= 2) s += 6.0;
        if (field_is_signed(f)) s -= 6.0;
        break;
    default:
        break;
    }
    return clamp_score(s, -20.0, 30.0);
}

/** Coarse role fit from byte-level statistics (preselection, no-raw mode). */
static double id_role_hint(disc_signal_t sig, const id_cmp_t *cmp, double hz_threshold) {
    double s = 0.0;
    switch (sig) {
    case DISC_SIG_STEERING:
        if (cmp_has_adjacent_pair(cmp, 1.0)) s += 16.0;
        if (cmp->changed_bytes >= 2) s += 6.0;
        break;
    case DISC_SIG_RPM:
        if (cmp_has_adjacent_pair(cmp, 1.0)) s += 10.0;
        if (cmp->changed_bytes >= 2) s += 12.0;
        if (cmp->hz >= hz_threshold) s += 8.0;
        break;
    case DISC_SIG_THROTTLE:
        if (cmp->changed_bytes <= 2) s += 10.0;
        break;
    case DISC_SIG_BRAKE:
        if (cmp_has_adjacent_pair(cmp, 2.0)) s += 8.0;
        if (cmp->changed_bytes <= 3) s += 4.0;
        break;
    case DISC_SIG_GEAR:
        for (int b = 0; b < cmp->byte_count; b++) {
            if (byte_looks_like_enum(&cmp->bytes[b])) {
                s += 24.0;
                break;
            }
        }
        break;
    case DISC_SIG_WHEEL_SPEED:
        if (cmp_has_adjacent_pair(cmp, 1.0)) s += 8.0;
        if (cmp->changed_bytes >= 4) s += 16.0;
        else if (cmp->changed_bytes >= 2) s += 8.0;
        if (cmp->hz >= hz_threshold) s += 8.0;
        break;
    default:
        break;
    }
    return s;
}

/** ID-level specificity: whole-message change in contrast phases. */
static double id_specificity(const disc_ctx_t *ctx, disc_signal_t sig, const id_cmp_t *row) {
    double max_other = 0.0;
    bool any = false;
    for (int p = 1; p < DISC_NUM_PHASES; p++) {
        if (!phase_is_contrast(sig, p) || !phase_has_data(&ctx->phases[p])) continue;
        any = true;
        const id_cmp_t *o = cmp_table_find(&ctx->cmp[p], row->can_id);
        if (o && o->total_score > max_other) max_other = o->total_score;
    }
    if (!any) return 0.0;
    double target = row->total_score > 1.0 ? row->total_score : 1.0;
    return specificity_from_ratio(max_other / target);
}

/**
 * Field-level specificity: how much the field moves in contrast phases,
 * relative to its move in the signal's own phase. Uses linear span (unique
 * count for gear) so idle jitter or a 25 rpm steering-load bump does not look
 * like a real move.
 */
static double field_specificity(const disc_ctx_t *ctx, disc_signal_t sig,
                                uint32_t can_id, const field_t *f,
                                const fstats_t *target) {
    const id_view_t *base_view = ctx_view(ctx, DISC_PHASE_BASELINE, can_id);
    fstats_t base;
    view_field_stats(base_view, f, ctx->uniq, &base);

    double t_move;
    if (sig == DISC_SIG_GEAR) {
        t_move = (double)(target->unique - (base.have ? base.unique : 1));
    } else {
        t_move = (double)(target->max - target->min) -
                 (base.have ? (double)(base.max - base.min) : 0.0);
    }
    if (t_move < 1.0) t_move = 1.0;

    double max_ratio = 0.0;
    bool any = false;
    for (int p = 1; p < DISC_NUM_PHASES; p++) {
        if (!phase_is_contrast(sig, p) || !phase_has_data(&ctx->phases[p])) continue;
        const id_view_t *v = ctx_view(ctx, p, can_id);
        if (!v) continue;
        fstats_t st;
        view_field_stats(v, f, ctx->uniq, &st);
        if (!st.have) continue;
        any = true;
        double move;
        if (sig == DISC_SIG_GEAR) {
            move = (double)(st.unique - (base.have ? base.unique : 1));
        } else {
            move = (double)(st.max - st.min) - (base.have ? (double)(base.max - base.min) : 0.0);
        }
        double ratio = move > 0.0 ? move / t_move : 0.0;
        if (ratio > max_ratio) max_ratio = ratio;
    }
    if (!any) return 0.0;
    if (max_ratio <= 0.05) return 20.0;
    if (max_ratio <= 0.15) return 10.0;
    if (max_ratio <= 0.35) return 0.0;
    if (max_ratio <= 0.60) return -15.0;
    return -30.0;
}

/*******************************************************************************
 * Raw bitfield search
 ******************************************************************************/

static void signal_search_lengths(disc_signal_t signal,
                                  const int **lengths_out,
                                  int *length_count_out) {
    /* Listed in tie-break preference order. */
    static const int k_steering[] = { 16, 12, 14, 10 };
    static const int k_rpm[] = { 16, 12, 14 };
    static const int k_throttle[] = { 8, 12, 10, 16 };
    static const int k_brake[] = { 12, 10, 8, 16, 1 };
    static const int k_gear[] = { 4, 8, 3, 16 };
    static const int k_wheel[] = { 16, 15, 14, 12 };

    switch (signal) {
    case DISC_SIG_STEERING:
        *lengths_out = k_steering;
        *length_count_out = (int)(sizeof(k_steering) / sizeof(k_steering[0]));
        break;
    case DISC_SIG_RPM:
        *lengths_out = k_rpm;
        *length_count_out = (int)(sizeof(k_rpm) / sizeof(k_rpm[0]));
        break;
    case DISC_SIG_THROTTLE:
        *lengths_out = k_throttle;
        *length_count_out = (int)(sizeof(k_throttle) / sizeof(k_throttle[0]));
        break;
    case DISC_SIG_BRAKE:
        *lengths_out = k_brake;
        *length_count_out = (int)(sizeof(k_brake) / sizeof(k_brake[0]));
        break;
    case DISC_SIG_GEAR:
        *lengths_out = k_gear;
        *length_count_out = (int)(sizeof(k_gear) / sizeof(k_gear[0]));
        break;
    case DISC_SIG_WHEEL_SPEED:
    default:
        *lengths_out = k_wheel;
        *length_count_out = (int)(sizeof(k_wheel) / sizeof(k_wheel[0]));
        break;
    }
}

static bool signal_allows_signed_guess(disc_signal_t signal) {
    return signal == DISC_SIG_STEERING || signal == DISC_SIG_BRAKE;
}

/** Signals whose search also tries sign-magnitude (VW-group steering angle). */
static bool signal_allows_sign_magnitude_guess(disc_signal_t signal) {
    return signal == DISC_SIG_STEERING;
}

static double raw_guess_family_bonus(disc_signal_t signal, int bit_length, char sign_char,
                                     int64_t raw_min, int64_t raw_max,
                                     int active_unique, int baseline_unique) {
    double score = 0.0;

    switch (signal) {
    case DISC_SIG_STEERING:
        if (bit_length >= 12) score += 4.0;
        if ((sign_char == '-' || sign_char == 'S') && raw_min < 0 && raw_max > 0) score += 8.0;
        if ((raw_max - raw_min) > 64) score += 3.0;
        break;
    case DISC_SIG_RPM:
        if (bit_length >= 12) score += 6.0;
        if (sign_char == '+') score += 2.0;
        if ((raw_max - raw_min) > 300) score += 3.0;
        break;
    case DISC_SIG_THROTTLE:
        if (bit_length == 8 || bit_length == 10 || bit_length == 12 || bit_length == 16) score += 4.0;
        if (sign_char == '+') score += 1.0;
        break;
    case DISC_SIG_BRAKE:
        if (bit_length == 1) score -= 30.0;
        if (bit_length == 10) score += 8.0;
        else if (bit_length == 12) score += 6.0;
        else if (bit_length == 8) score += 4.0;
        else if (bit_length == 16) score += 1.0;
        if (active_unique >= 4 && baseline_unique <= 2) score += 4.0;
        break;
    case DISC_SIG_GEAR:
        if (bit_length == 3 || bit_length == 4 || bit_length == 8) score += 5.0;
        if (active_unique >= 3 && active_unique <= 8) score += 10.0;
        if (baseline_unique <= 2) score += 4.0;
        break;
    case DISC_SIG_WHEEL_SPEED:
        if (bit_length == 16) score += 16.0;
        else if (bit_length == 15) score += 12.0;
        else if (bit_length == 14) score += 10.0;
        else if (bit_length == 12) score += 8.0;
        if (sign_char == '+') score += 2.0;
        if (baseline_unique <= 2) score += 6.0;
        break;
    default:
        break;
    }

    if (bit_length < 4 && signal != DISC_SIG_GEAR) score -= 20.0;
    return score;
}

static bool bit_toggles(const id_view_t *v, int bit) {
    return bit >= 0 && bit < 64 && ((v->toggle >> bit) & 1ULL) != 0;
}

/** Signals that read one value at rest: released pedal, car standing still. */
static bool signal_rests_in_baseline(disc_signal_t sig) {
    return sig == DISC_SIG_THROTTLE || sig == DISC_SIG_WHEEL_SPEED;
}

/** Baseline alive-counter bits for a signal that rests there, else 0. */
static uint64_t rest_alive_bits(disc_signal_t sig, const id_view_t *base) {
    return (signal_rests_in_baseline(sig) && base) ? base->alive : 0;
}

static bool mask_has_bit(uint64_t mask, int bit) {
    return bit >= 0 && bit < 64 && ((mask >> bit) & 1ULL) != 0;
}

/**
 * Toggle pattern of a field, LSB first: whether toggling bits have a gap of
 * constant bits between them (two signals glued together), and whether the
 * toggling run continues past either end of the field (field truncated).
 * Bits in `alive` (a counter next to the field) do not continue the run.
 */
static void field_toggle_shape(const id_view_t *v, const field_t *f, uint64_t alive,
                               bool *has_gap, bool *truncated) {
    *has_gap = false;
    *truncated = false;

    int lsb = f->start;
    if (f->order == 0) {
        for (int i = 1; i < f->len; i++) lsb = less_significant_bit(lsb, 0);
    }

    int first = -1, last = -1;
    int bit = lsb;
    int msb_bit = lsb;
    bool toggles[32];
    for (int i = 0; i < f->len; i++) {
        toggles[i] = bit_toggles(v, bit);
        if (toggles[i]) {
            if (first < 0) first = i;
            last = i;
        }
        msb_bit = bit;
        bit = more_significant_bit(bit, f->order);
    }
    for (int i = first + 1; first >= 0 && i < last; i++) {
        if (!toggles[i]) {
            *has_gap = true;
            break;
        }
    }
    int above = more_significant_bit(msb_bit, f->order);
    if (f->sm_bit >= 0 && above == f->sm_bit) above = more_significant_bit(above, f->order);
    if (last == f->len - 1 && bit_toggles(v, above) && !mask_has_bit(alive, above)) {
        *truncated = true;   /* run continues above the MSB (or the sign bit) */
    }
    int below = less_significant_bit(lsb, f->order);
    if (first == 0 && bit_toggles(v, below) && !mask_has_bit(alive, below)) {
        *truncated = true;   /* run continues below the LSB */
    }
}

/**
 * Byte significance for a field spanning two bytes: between consecutive
 * frames, the less significant part of a smooth signal changes more often
 * than the more significant part. A byte-swapped reading shows the opposite.
 * Change counts do not depend on how many bits each part holds or on how many
 * frames were captured.
 */
static double field_significance_score(const id_view_t *v, const field_t *f) {
    if (!v || f->max_byte != f->min_byte + 1) return 0.0;
    int lsb_byte = (f->order == 1) ? f->min_byte : f->max_byte;
    int msb_byte = (f->order == 1) ? f->max_byte : f->min_byte;
    uint8_t lsb_mask = (uint8_t)((f->bits >> (lsb_byte * 8)) & 0xFF);
    uint8_t msb_mask = (uint8_t)((f->bits >> (msb_byte * 8)) & 0xFF);
    if (!lsb_mask || !msb_mask) return 0.0;

    int cl = 0, cm = 0;
    bool have_prev = false;
    uint8_t pl = 0, pm = 0;
    for (int i = 0; i < v->count; i++) {
        if (v->frames[i].dlc <= f->max_byte) continue;
        uint8_t l = v->frames[i].data[lsb_byte] & lsb_mask;
        uint8_t m = v->frames[i].data[msb_byte] & msb_mask;
        if (have_prev) {
            if (l != pl) cl++;
            if (m != pm) cm++;
        }
        pl = l;
        pm = m;
        have_prev = true;
    }
    if (cl == 0 || cm == 0) return 0.0;      /* only meaningful if both vary */
    /* Asymmetric: a contradiction is strong evidence of a wrong byte order,
     * a confirmation only mildly helps (it depends on the frame rate). */
    if (cl > cm * 1.25) return 6.0;
    if (cl * 1.25 < cm) return -20.0;
    return 0.0;
}

/** Fraction of value changes that jump by more than a quarter of the span. */
static double view_field_jump_ratio(const id_view_t *v, const field_t *f, int64_t span) {
    if (!v || span <= 0) return 0.0;
    int changes = 0, jumps = 0;
    bool have_prev = false;
    int64_t prev = 0;
    for (int i = 0; i < v->count; i++) {
        if (v->frames[i].dlc <= f->max_byte) continue;
        int64_t val = field_value(f, field_raw(f, v->le[i], v->be[i]));
        if (have_prev && val != prev) {
            changes++;
            int64_t d = val > prev ? val - prev : prev - val;
            if (d * 4 > span) jumps++;
        }
        prev = val;
        have_prev = true;
    }
    return changes > 0 ? (double)jumps / (double)changes : 0.0;
}

/**
 * Fraction of sign changes that jump by more than a quarter of the span.
 * A signed signal passes through zero when its sign flips, so a reading that
 * jumps across the range at those moments decodes the wrong scheme: two's
 * complement data read as sign-magnitude (0 -> -1 becomes 0 -> -max), or
 * sign-magnitude data read as two's complement or unsigned (+1 -> -1 becomes
 * 1 -> -2^(n-1) or 1 -> 2^(n-1)+1). For '+' fields the MSB plays the sign.
 */
static double view_sign_flip_jump_ratio(const id_view_t *v, const field_t *f, int64_t span) {
    if (!v || span <= 0) return 0.0;
    int flips = 0, jumps = 0;
    bool have_prev = false;
    uint32_t prev_raw = 0;
    int64_t prev = 0;
    for (int i = 0; i < v->count; i++) {
        if (v->frames[i].dlc <= f->max_byte) continue;
        uint32_t raw = field_raw(f, v->le[i], v->be[i]);
        int64_t val = field_value(f, raw);
        if (have_prev && ((raw ^ prev_raw) & f->sign_bit)) {
            flips++;
            int64_t d = val > prev ? val - prev : prev - val;
            if (d * 4 > span) jumps++;
        }
        prev_raw = raw;
        prev = val;
        have_prev = true;
    }
    return flips > 0 ? (double)jumps / (double)flips : 0.0;
}

/** Score one bitfield hypothesis on the active vs baseline frames of one ID. */
static bool eval_guess(disc_ctx_t *ctx, disc_signal_t sig,
                       const id_view_t *act, const id_view_t *base,
                       const field_t *f, int seq, disc_raw_guess_t *g) {
    fstats_t a, b;
    view_field_stats(act, f, ctx->uniq, &a);
    if (!a.have || a.n < 2) return false;
    view_field_stats(base, f, ctx->uniq, &b);

    /* Evidence in bits: span gained and distinct values gained (log2), so a
     * noisy field with many distinct values does not outrank a smooth one. */
    int sa = span_bits(a.max - a.min);
    int sb = b.have ? span_bits(b.max - b.min) : 0;
    double ua = log2((double)a.unique);
    double ub = b.have && b.unique > 0 ? log2((double)b.unique) : 0.0;
    double evidence = (sa > sb ? (sa - sb) * 2.0 : 0.0) + (ua > ub ? (ua - ub) * 2.0 : 0.0);
    if (evidence <= 0.0) return false;          /* no more change than baseline */

    double shape = 0.0;
    if (a.changes >= 8 && a.unique >= 8 &&
        (double)a.plus_one / (double)a.changes >= 0.75) {
        shape -= 100.0;                         /* rolling counter */
    }
    shape += field_significance_score(act, f);
    if (a.changes > 0 && !a.lsb_toggles) shape -= 15.0;   /* shifted into constant LSBs */
    /* Tie-break between equivalent widths: prefer an end on a byte boundary. */
    bool aligned = (f->order == 1) ? (f->start % 8 == 0 || (f->start + f->len) % 8 == 0)
                                   : (f->start % 8 == 7 || f->shift % 8 == 0);
    if (aligned) shape += 1.0;

    if (sig != DISC_SIG_GEAR) {
        bool has_gap, truncated;
        /* A released pedal and a car standing still read one value, so an
         * alive counter ticking in baseline is not part of the signal. Glued
         * onto a pedal, counter ticks also hide its jumps between frames. */
        uint64_t alive = rest_alive_bits(sig, base);
        field_toggle_shape(act, f, alive, &has_gap, &truncated);
        if (f->bits & alive) shape -= 20.0;
        if (has_gap) shape -= 40.0;             /* unrelated bits glued on top */
        if (truncated) shape -= 15.0;           /* cuts through a toggling run */
        shape -= 30.0 * view_field_jump_ratio(act, f, a.max - a.min);
        if (signal_allows_signed_guess(sig)) {
            shape -= 30.0 * view_sign_flip_jump_ratio(act, f, a.max - a.min);
        }
    }
    shape += raw_guess_family_bonus(sig, f->len, f->sign, a.min, a.max, a.unique,
                                    b.have ? b.unique : 1);

    memset(g, 0, sizeof(*g));
    g->start_bit = f->start;
    g->bit_length = f->len;
    g->byte_order = f->order;
    g->sign_char = f->sign;
    g->sign_bit = f->sm_bit;
    g->raw_min = clamp_i32(a.min);
    g->raw_max = clamp_i32(a.max);
    g->active_unique = a.unique;
    g->baseline_unique = b.have ? b.unique : 0;
    g->score = evidence + shape;
    g->shape = shape;
    g->bitmask = f->bits;
    g->seq = seq;
    return true;
}

static int cmp_raw_guess_score_desc(const void *a, const void *b) {
    const disc_raw_guess_t *ga = (const disc_raw_guess_t *)a;
    const disc_raw_guess_t *gb = (const disc_raw_guess_t *)b;
    if (ga->score > gb->score) return -1;
    if (ga->score < gb->score) return 1;
    return (ga->seq > gb->seq) - (ga->seq < gb->seq);
}

static int cmp_raw_guess_start_asc(const void *a, const void *b) {
    const disc_raw_guess_t *ga = (const disc_raw_guess_t *)a;
    const disc_raw_guess_t *gb = (const disc_raw_guess_t *)b;
    return (ga->start_bit > gb->start_bit) - (ga->start_bit < gb->start_bit);
}

/** MSB position of a field of `len` bits whose LSB sits at DBC bit `lsb`. */
static int field_start_from_lsb(int lsb, int len, int order) {
    if (order == 1) return lsb;
    int bit = lsb;
    for (int i = 1; i < len; i++) {
        bit = more_significant_bit(bit, order);
        if (bit < 0 || bit >= 64) return -1;
    }
    return bit;
}

#define DISC_MAX_REGION_GUESSES 768

/**
 * Search bitfields inside bytes [byte_lo, byte_hi] of one ID. Field LSBs are
 * restricted to bits that actually toggle and start a toggling run (or sit on
 * a nibble boundary); a full scan is the fallback. Returns guesses sorted by
 * score.
 */
static int region_guesses(disc_ctx_t *ctx, disc_signal_t sig,
                          const id_view_t *act, const id_view_t *base,
                          int dlc, int byte_lo, int byte_hi,
                          disc_raw_guess_t *out, int max_out) {
    const int *lengths = NULL;
    int length_count = 0;
    signal_search_lengths(sig, &lengths, &length_count);
    int sign_variants = signal_allows_signed_guess(sig) ? 2 : 1;
    if (signal_allows_sign_magnitude_guess(sig)) sign_variants = 3;
    uint64_t rest_alive = rest_alive_bits(sig, base);
    int count = 0;
    int seq = 0;

    for (int pass = 0; pass < 2 && count == 0; pass++) {
        bool full_scan = (pass == 1);
        for (int order = 1; order >= 0; order--) {
            for (int lsb = byte_lo * 8; lsb <= byte_hi * 8 + 7; lsb++) {
                if (!full_scan) {
                    if (!bit_toggles(act, lsb)) continue;
                    int below = less_significant_bit(lsb, order);
                    /* A run also starts right above an alive counter. */
                    bool run_start = !bit_toggles(act, below) ||
                                     mask_has_bit(rest_alive, below);
                    if (!run_start && lsb % 4 != 0) continue;
                }

                /* Candidate widths: the signal's usual widths, plus (for signed
                 * signals) the width of the toggling run so the sign bit lands
                 * on the real MSB. */
                int widths[8];
                int width_count = 0;
                for (int li = 0; li < length_count; li++) widths[width_count++] = lengths[li];
                if (sign_variants >= 2) {
                    int run = 0;
                    for (int bit = lsb; run < 17 && bit_toggles(act, bit);
                         bit = more_significant_bit(bit, order)) {
                        run++;
                    }
                    bool listed = false;
                    for (int li = 0; li < width_count; li++) listed |= (widths[li] == run);
                    if (run >= 4 && run <= 16 && !listed) widths[width_count++] = run;
                }

                for (int wi = 0; wi < width_count; wi++) {
                    int len = widths[wi];
                    int start = field_start_from_lsb(lsb, len, order);
                    /* A Motorola field inside one byte has the same bits as the Intel
                       field: skip it. One that crosses a byte (19|8@0) is its own layout. */
                    if (order == 0 && start >= 0 && start / 8 == lsb / 8) continue;
                    for (int si = 0; si < sign_variants; si++) {
                        field_t f;
                        if (si == 2) {
                            /* Same bits, MSB as sign of a (len-1)-bit magnitude. */
                            if (len < 5) continue;
                            int msb = (order == 1) ? lsb + len - 1 : start;
                            int mag_start = field_start_from_lsb(lsb, len - 1, order);
                            if (mag_start < 0 ||
                                !field_make_sm(mag_start, len - 1, order, 'S', msb, &f)) continue;
                        } else if (!field_make(start, len, order, si ? '-' : '+', &f)) {
                            continue;
                        }
                        if (!field_fits(&f, dlc)) continue;
                        if (f.min_byte < byte_lo || f.max_byte > byte_hi) continue;

                        disc_raw_guess_t g;
                        if (!eval_guess(ctx, sig, act, base, &f, seq++, &g)) continue;
                        if (g.score <= 0.0) continue;
                        if (count < max_out) out[count++] = g;
                    }
                }
            }
        }
    }

    qsort(out, (size_t)count, sizeof(*out), cmp_raw_guess_score_desc);
    return count;
}

/**
 * Split a wheel-speed message into per-wheel fields (same width and byte
 * order, non-overlapping, 2-byte aligned). Deterministic for given frames.
 */
static int wheel_split(disc_ctx_t *ctx, const id_view_t *act, const id_view_t *base,
                       int dlc, disc_raw_guess_t parts[4]) {
    static const int k_lengths[] = { 16, 15, 14, 12 };
    disc_raw_guess_t guesses[64];
    int count = 0;
    int seq = 0;

    if (!act) return 0;
    for (int byte_idx = 0; byte_idx + 1 < dlc; byte_idx += 2) {
        for (int li = 0; li < 4; li++) {
            for (int order = 1; order >= 0; order--) {
                int start = (order == 1) ? byte_idx * 8 : byte_idx * 8 + 7;
                field_t f;
                if (!field_make(start, k_lengths[li], order, '+', &f)) continue;
                if (!field_fits(&f, dlc)) continue;
                disc_raw_guess_t g;
                if (!eval_guess(ctx, DISC_SIG_WHEEL_SPEED, act, base, &f, seq++, &g)) continue;
                if (g.score <= 0.0 || count >= 64) continue;
                guesses[count++] = g;
            }
        }
    }
    qsort(guesses, (size_t)count, sizeof(*guesses), cmp_raw_guess_score_desc);

    int best_len = 0, best_order = 1, best_n = -1;
    double best_sum = 0.0;
    for (int i = 0; i < count; i++) {
        int n = 0;
        double sum = 0.0;
        uint64_t used = 0;
        for (int j = 0; j < count; j++) {
            if (guesses[j].bit_length != guesses[i].bit_length) continue;
            if (guesses[j].byte_order != guesses[i].byte_order) continue;
            if (guesses[j].score < 12.0 || (guesses[j].bitmask & used)) continue;
            used |= guesses[j].bitmask;
            n++;
            sum += guesses[j].score;
        }
        if (n > best_n || (n == best_n && sum > best_sum)) {
            best_n = n;
            best_sum = sum;
            best_len = guesses[i].bit_length;
            best_order = guesses[i].byte_order;
        }
    }

    int chosen = 0;
    uint64_t used = 0;
    for (int i = 0; i < count && chosen < 4; i++) {
        if (guesses[i].bit_length != best_len || guesses[i].byte_order != best_order) continue;
        if (guesses[i].score < 12.0 || (guesses[i].bitmask & used)) continue;
        used |= guesses[i].bitmask;
        parts[chosen++] = guesses[i];
    }
    qsort(parts, (size_t)chosen, sizeof(*parts), cmp_raw_guess_start_asc);
    return chosen;
}

/*******************************************************************************
 * VSP corpus priors
 ******************************************************************************/

typedef enum {
    PRIOR_NONE = 0,
    PRIOR_TOUCH,      /* same signal + ID, overlapping bits, not adoptable */
    PRIOR_NARROWER,   /* ours contains the prior, same LSB; our extra top bits are constant */
    PRIOR_SUPERSET,   /* prior field contains ours; extra bits are constant */
    PRIOR_EXACT       /* same signal + ID + layout */
} prior_kind_t;

static bool prior_field(const disc_discover_prior_t *p, field_t *f) {
    return field_make_sm(p->start_bit, p->bit_length, p->byte_order, p->sign_char,
                         p->sign_bit, f);
}

/**
 * The prior's scale must agree with what was observed: the observed raw range,
 * converted with the prior's factor/offset, has to fall inside the prior's own
 * [min, max] (with a small tolerance). Priors without a usable range pass.
 */
static bool prior_scale_plausible(const disc_discover_prior_t *p,
                                  int64_t raw_min, int64_t raw_max) {
    char *end = NULL;
    double factor = strtod(p->factor, &end);
    if (end == p->factor) return true;
    double offset = strtod(p->offset, &end);
    if (end == p->offset) offset = 0.0;
    double lo = strtod(p->min_val, &end);
    if (end == p->min_val) return true;
    double hi = strtod(p->max_val, &end);
    if (end == p->max_val || !(hi > lo)) return true;

    double a = (double)raw_min * factor + offset;
    double b = (double)raw_max * factor + offset;
    double phys_lo = a < b ? a : b;
    double phys_hi = a < b ? b : a;
    double tol = 0.05 * (hi - lo) + fabs(factor);
    return phys_lo >= lo - tol && phys_hi <= hi + tol;
}

/**
 * Physical sanity of a prior's scale: at rest (brake/pedal released, idle,
 * standing still) these signals sit at the low end of their observed physical
 * range. A scale that maps the rest value to the top (e.g. a wrong-sign
 * factor) contradicts the capture. rest_raw < 0 sentinel: no baseline data.
 */
static bool prior_rest_plausible(disc_signal_t sig, const disc_discover_prior_t *p,
                                 int64_t raw_min, int64_t raw_max,
                                 bool have_rest, int64_t rest_raw) {
    if (!have_rest) return true;
    if (sig != DISC_SIG_BRAKE && sig != DISC_SIG_THROTTLE &&
        sig != DISC_SIG_RPM && sig != DISC_SIG_WHEEL_SPEED) return true;

    char *end = NULL;
    double factor = strtod(p->factor, &end);
    if (end == p->factor || factor == 0.0) return true;
    double offset = strtod(p->offset, &end);
    if (end == p->offset) offset = 0.0;

    int64_t lo_raw = raw_min < rest_raw ? raw_min : rest_raw;
    int64_t hi_raw = raw_max > rest_raw ? raw_max : rest_raw;
    double a = (double)lo_raw * factor + offset;
    double b = (double)hi_raw * factor + offset;
    double lo = a < b ? a : b, hi = a < b ? b : a;
    if (!(hi > lo)) return true;
    double rest = (double)rest_raw * factor + offset;
    /* Idle sits above zero: small blips, and the undershoot after them, can
     * leave it a third of the way up the throttle phase's range. */
    double frac = (sig == DISC_SIG_RPM) ? 0.5 : 0.25;
    return rest <= lo + frac * (hi - lo);
}

/** Observed values a prior is checked against. */
typedef struct {
    int64_t raw_min;     /* signal phase */
    int64_t raw_max;
    bool    have_rest;
    int64_t rest_raw;    /* baseline midpoint */
} prior_obs_t;

static void prior_obs_from_stats(prior_obs_t *o, const fstats_t *active, const fstats_t *base) {
    o->raw_min = active->min;
    o->raw_max = active->max;
    o->have_rest = base->have;
    o->rest_raw = base->have ? base->min + (base->max - base->min) / 2 : 0;
}

static bool prior_plausible(disc_signal_t sig, const disc_discover_prior_t *p,
                            const prior_obs_t *o) {
    return prior_scale_plausible(p, o->raw_min, o->raw_max) &&
           prior_rest_plausible(sig, p, o->raw_min, o->raw_max, o->have_rest, o->rest_raw);
}

/**
 * Exact-layout prior with the same sign and a plausible scale for the
 * observed data (highest support, first in table on ties). Used by both
 * discovery and export so they agree.
 */
static int find_exact_prior(disc_signal_t sig, uint32_t can_id, const field_t *f,
                            const prior_obs_t *obs) {
    int best = -1;
    for (int i = 0; i < DISC_CAN_DISCOVER_PRIOR_COUNT; i++) {
        const disc_discover_prior_t *p = &s_can_discover_priors[i];
        if (p->signal != sig || p->can_id != can_id) continue;
        if (p->start_bit != f->start || p->bit_length != f->len ||
            p->byte_order != f->order || p->sign_char != f->sign) continue;
        if (f->sign == 'S' && p->sign_bit != f->sm_bit) continue;
        if (!prior_plausible(sig, p, obs)) continue;
        if (best < 0 || p->support_count > s_can_discover_priors[best].support_count) best = i;
    }
    return best;
}

/** Observed data for field f on one ID (active phase + baseline views). */
static void observe_field(disc_ctx_t *ctx, const id_view_t *act, const id_view_t *base,
                          const field_t *f, prior_obs_t *o) {
    fstats_t a, b;
    view_field_stats(act, f, ctx->uniq, &a);
    view_field_stats(base, f, ctx->uniq, &b);
    prior_obs_from_stats(o, &a, &b);
}

static bool sign_bit_seen(disc_ctx_t *ctx, const id_view_t *act, const id_view_t *base,
                          const field_t *f) {
    field_t u = *f;
    u.sign = '+';
    fstats_t a, b;
    view_field_stats(act, &u, ctx->uniq, &a);
    view_field_stats(base, &u, ctx->uniq, &b);
    return a.sign_bit_seen || b.sign_bit_seen;
}

/**
 * Match a discovered field against corpus priors for the same signal and ID.
 * A prior is adopted only when it agrees with the observed data:
 *   - exact layout, and the sign either matches or the sign bit never set;
 *   - the same bits read with the other sign scheme (two's complement or
 *     unsigned vs sign-magnitude), when the prior's reading is at least as
 *     continuous across sign flips (with raw frames);
 *   - a superset layout whose extra bits never change (with raw frames);
 *   - or, last, a narrower layout with the same LSB, when our extra top bits
 *     never change: the capture cannot tell the widths apart, and the corpus
 *     knows more than our search widths and byte-boundary tie-break. Among
 *     several, the best supported, then the widest.
 * On adoption *f becomes the prior's layout/sign.
 */
static int field_lsb(const field_t *f) {
    int lsb = f->start;
    if (f->order == 0) {
        for (int i = 1; i < f->len; i++) lsb = less_significant_bit(lsb, 0);
    }
    return lsb;
}

static prior_kind_t resolve_prior(disc_ctx_t *ctx, disc_signal_t sig, uint32_t can_id,
                                  int dlc, const id_view_t *act, const id_view_t *base,
                                  field_t *f, int *support_out) {
    int best_exact = -1, best_super = -1, best_narrow = -1;
    bool touch = false;
    bool have_raw = act != NULL;

    for (int i = 0; i < DISC_CAN_DISCOVER_PRIOR_COUNT; i++) {
        const disc_discover_prior_t *p = &s_can_discover_priors[i];
        if (p->signal != sig || p->can_id != can_id) continue;

        field_t pf;
        if (!prior_field(p, &pf) || !field_fits(&pf, dlc)) continue;
        if (pf.bits & f->bits) touch = true;

        bool same_layout = pf.start == f->start && pf.len == f->len &&
                           pf.order == f->order && pf.sm_bit == f->sm_bit;
        bool sign_ok = pf.sign == f->sign ||
                       (have_raw && !sign_bit_seen(ctx, act, base, &pf));
        bool reread = !same_layout && have_raw && pf.order == f->order &&
                      pf.bits == f->bits && (pf.sm_bit >= 0 || f->sm_bit >= 0);
        prior_obs_t obs = {0};
        if (have_raw) {
            observe_field(ctx, act, base, &pf, &obs);
            if (!prior_plausible(sig, p, &obs)) continue;
        }

        if (reread) {
            prior_obs_t own;
            observe_field(ctx, act, base, f, &own);
            double prior_flip = view_sign_flip_jump_ratio(act, &pf, obs.raw_max - obs.raw_min);
            double own_flip = view_sign_flip_jump_ratio(act, f, own.raw_max - own.raw_min);
            if (prior_flip > own_flip) continue;
            same_layout = true;
            sign_ok = true;
        }

        if (same_layout) {
            if (!sign_ok) continue;
            if (best_exact < 0 ||
                p->support_count > s_can_discover_priors[best_exact].support_count) {
                best_exact = i;
            }
        } else if (have_raw && pf.order == f->order && (pf.bits & f->bits) == f->bits) {
            if (!sign_ok) continue;
            if (!views_bits_constant(act, base, pf.bits & ~f->bits)) continue;
            if (best_super < 0 ||
                p->support_count > s_can_discover_priors[best_super].support_count) {
                best_super = i;
            }
        } else if (have_raw && pf.order == f->order && pf.sm_bit == f->sm_bit &&
                   (pf.bits & f->bits) == pf.bits && field_lsb(&pf) == field_lsb(f)) {
            if (!sign_ok) continue;
            if (!views_bits_constant(act, base, f->bits & ~pf.bits)) continue;
            const disc_discover_prior_t *b =
                best_narrow >= 0 ? &s_can_discover_priors[best_narrow] : NULL;
            if (!b || p->support_count > b->support_count ||
                (p->support_count == b->support_count && p->bit_length > b->bit_length)) {
                best_narrow = i;
            }
        }
    }

    int chosen = (best_exact >= 0) ? best_exact : (best_super >= 0) ? best_super : best_narrow;
    if (chosen >= 0) {
        prior_field(&s_can_discover_priors[chosen], f);
        *support_out = s_can_discover_priors[chosen].support_count;
        return (best_exact >= 0) ? PRIOR_EXACT
             : (best_super >= 0) ? PRIOR_SUPERSET : PRIOR_NARROWER;
    }
    *support_out = 0;
    return touch ? PRIOR_TOUCH : PRIOR_NONE;
}

static double prior_points(prior_kind_t kind, int support) {
    switch (kind) {
    case PRIOR_EXACT:    return 10.0 + (support > 6 ? 6 : support);
    case PRIOR_SUPERSET:
    case PRIOR_NARROWER: return 6.0 + (support > 4 ? 4 : support);
    case PRIOR_TOUCH:    return 2.0;
    default:             return 0.0;
    }
}

/*******************************************************************************
 * Candidate options and joint selection
 ******************************************************************************/

#define DISC_PRESELECT_IDS        8
#define DISC_OPTIONS_PER_SIGNAL   8
#define DISC_GUESSES_PER_REGION   2
#define DISC_MIN_ACCEPT_SCORE     18.0

typedef struct {
    disc_candidate_t cand;
    double           score;
    uint64_t         mask;      /* bits this option occupies in its message */
    bool             strong;
    field_t          field;     /* the reported layout (after prior resolution) */
    double           layout_alt;/* best other layout over the same bits that reads other
                                   values (e.g. 8|12 over 12|8), -1 if none */
} disc_option_t;

/** a and b read the same quantity from these frames: one is the other times
 *  2^k (|k| <= 8) plus a constant (bits that never change, e.g. a fixed upper
 *  nibble). Only width, scale or offset differ, which the capture cannot tell
 *  apart (priors and ranges decide those). A counter or flag inside one of
 *  them makes the difference vary, so 8|12 over a counter is not 12|8. */
static bool fields_read_alike(const field_t *a, const field_t *b,
                              const id_view_t *v1, const id_view_t *v2) {
    uint32_t ks = (1u << 17) - 1;   /* bit k + 8: k still possible */
    int64_t offset[17];
    bool seen = false;
    const id_view_t *views[2] = { v1, v2 };
    for (int vi = 0; vi < 2 && ks; vi++) {
        const id_view_t *v = views[vi];
        for (int i = 0; v && i < v->count && ks; i++) {
            int64_t x = field_value(a, field_raw(a, v->le[i], v->be[i]));
            int64_t y = field_value(b, field_raw(b, v->le[i], v->be[i]));
            for (int k = -8; k <= 8; k++) {
                if (!(ks & (1u << (k + 8)))) continue;
                /* k >= 0: y = x * 2^k + c; k < 0: x = y * 2^-k + c */
                int64_t c = k >= 0 ? y - x * ((int64_t)1 << k) : x - y * ((int64_t)1 << -k);
                if (!seen) offset[k + 8] = c;
                else if (c != offset[k + 8]) ks &= ~(1u << (k + 8));
            }
            seen = true;
        }
    }
    return ks != 0;
}

typedef struct {
    int    idx;
    double pre;
} pre_rank_t;

static int cmp_pre_rank_desc(const void *a, const void *b) {
    const pre_rank_t *x = (const pre_rank_t *)a;
    const pre_rank_t *y = (const pre_rank_t *)b;
    if (x->pre > y->pre) return -1;
    if (x->pre < y->pre) return 1;
    return (x->idx > y->idx) - (x->idx < y->idx);
}

static int cmp_option_desc(const void *a, const void *b) {
    const disc_option_t *x = (const disc_option_t *)a;
    const disc_option_t *y = (const disc_option_t *)b;
    if (x->score > y->score) return -1;
    if (x->score < y->score) return 1;
    if (x->cand.can_id != y->cand.can_id) return x->cand.can_id < y->cand.can_id ? -1 : 1;
    return (x->cand.start_bit > y->cand.start_bit) - (x->cand.start_bit < y->cand.start_bit);
}

static bool option_add(disc_option_t *opts, int *count, int max, const disc_option_t *o) {
    for (int i = 0; i < *count; i++) {
        if (opts[i].cand.can_id == o->cand.can_id && opts[i].mask == o->mask &&
            opts[i].cand.start_bit == o->cand.start_bit) {
            if (o->score > opts[i].score) opts[i] = *o;
            return true;
        }
    }
    if (*count >= max) return false;
    opts[(*count)++] = *o;
    return true;
}

/** Build a scored option from one raw guess. Returns false if rejected. */
static bool make_raw_option(disc_ctx_t *ctx, disc_signal_t sig, const id_cmp_t *row,
                            const id_view_t *act, const id_view_t *base,
                            const disc_raw_guess_t *g, disc_option_t *opt) {
    field_t f;
    if (!field_make_sm(g->start_bit, g->bit_length, g->byte_order, g->sign_char,
                       g->sign_bit, &f)) return false;

    memset(opt, 0, sizeof(*opt));
    fill_candidate(&opt->cand, row);
    opt->mask = f.bits;

    int wheel_parts = 0;
    prior_kind_t kind = PRIOR_NONE;
    int support = 0;

    if (sig == DISC_SIG_WHEEL_SPEED) {
        disc_raw_guess_t parts[4];
        wheel_parts = wheel_split(ctx, act, base, row->dlc, parts);
        if (wheel_parts >= 2) {
            int pick = 0;
            uint64_t all = 0;
            for (int i = 0; i < wheel_parts; i++) {
                all |= parts[i].bitmask;
                if (parts[i].bitmask & f.bits) pick = i;
            }
            /* Report the part that overlaps this guess; the option owns all parts. */
            field_make(parts[pick].start_bit, parts[pick].bit_length,
                       parts[pick].byte_order, '+', &f);
            opt->mask = all;
            prior_obs_t obs;
            observe_field(ctx, act, base, &f, &obs);
            int idx = find_exact_prior(sig, row->can_id, &f, &obs);
            if (idx >= 0) {
                kind = PRIOR_EXACT;
                support = s_can_discover_priors[idx].support_count;
            }
        }
    }
    if (wheel_parts < 2) {
        kind = resolve_prior(ctx, sig, row->can_id, row->dlc, act, base, &f, &support);
        opt->mask = f.bits;
    }

    fstats_t a, b;
    view_field_stats(act, &f, ctx->uniq, &a);
    view_field_stats(base, &f, ctx->uniq, &b);
    if (!evidence_gate(sig, &a, &b, &opt->strong)) return false;

    double evidence = field_activity(sig, &a, &b);
    double spec = field_specificity(ctx, sig, row->can_id, &f, &a);
#ifdef DISC_DEBUG
    fprintf(stderr, "    raw %-11s 0x%03X %2d|%-2d@%d n=%d uniq=%d/%d span=%lld/%lld g.shape=%.1f\n",
            disc_signal_name(sig), row->can_id, f.start, f.len, f.order, a.n, a.unique,
            b.have ? b.unique : 0, (long long)(a.max - a.min),
            (long long)(b.have ? b.max - b.min : 0), g->shape);
#endif
    double role = role_score(sig, row, &f, &a, &b, ctx->hz_threshold, wheel_parts);
    double shape = clamp_score(g->shape, -40.0, 30.0) + role;
    double prior = prior_points(kind, support);
    double score = evidence + spec + shape + prior;
    if (score < DISC_MIN_ACCEPT_SCORE) return false;

    disc_candidate_t *c = &opt->cand;
    candidate_set_layout(c, &f);
    c->raw_min = clamp_i32(a.min);
    c->raw_max = clamp_i32(a.max);
    c->score = score;
    c->evidence_score = evidence;
    c->specificity_score = spec;
    c->shape_score = shape;
    c->prior_score = prior;
    c->prior_support = support;
    opt->score = score;
    opt->field = f;
    opt->layout_alt = -1.0;
    return true;
}

/** Option from byte-level statistics only (no raw frames available). */
static bool make_byte_option(disc_ctx_t *ctx, disc_signal_t sig, const id_cmp_t *row,
                             disc_option_t *opt) {
    int phase_idx = signal_phase_index(sig);
    memset(opt, 0, sizeof(*opt));
    disc_candidate_t *c = &opt->cand;
    fill_candidate(c, row);

    if (sig == DISC_SIG_GEAR) {
        for (int b = 0; b < row->byte_count; b++) {
            if (byte_looks_like_enum(&row->bytes[b])) {
                c->byte_idx = b;
                break;
            }
        }
    } else if (sig != DISC_SIG_THROTTLE && cmp_has_adjacent_pair(row, 1.0)) {
        candidate_use_adjacent_pair(c, row);
    }
    disc_characterize(&ctx->phases[phase_idx], c);
    if (c->byte_idx < 0 || c->start_bit < 0) return false;

    const byte_cmp_t *bc = &row->bytes[c->byte2_idx >= 0 ? c->byte2_idx : c->byte_idx];
    if (sig == DISC_SIG_GEAR) {
        if (!byte_looks_like_enum(&row->bytes[c->byte_idx])) return false;
        opt->strong = row->bytes[c->byte_idx].baseline_unique <= 1;
    } else {
        if (bc->active_unique < 4 || bc->active_range - bc->baseline_range < 3) return false;
        opt->strong = bc->active_unique >= 16;
    }

    field_t f;
    if (!field_make(c->start_bit, c->bit_length, c->byte_order, c->sign_char, &f)) return false;
    prior_obs_t obs = { c->raw_min, c->raw_max, false, 0 };
    int idx = find_exact_prior(sig, row->can_id, &f, &obs);
    int support = idx >= 0 ? s_can_discover_priors[idx].support_count : 0;

    double evidence = clamp_score(row->total_score, 0.0, 60.0);
    double spec = id_specificity(ctx, sig, row);
    double shape = clamp_score(id_role_hint(sig, row, ctx->hz_threshold), 0.0, 30.0);
    double prior = prior_points(idx >= 0 ? PRIOR_EXACT : PRIOR_NONE, support);
    double score = evidence + spec + shape + prior;
    if (score < DISC_MIN_ACCEPT_SCORE) return false;

    candidate_set_layout(c, &f);
    c->score = score;
    c->evidence_score = evidence;
    c->specificity_score = spec;
    c->shape_score = shape;
    c->prior_score = prior;
    c->prior_support = support;
    opt->mask = f.bits;
    opt->score = score;
    opt->field = f;
    opt->layout_alt = -1.0;
    return true;
}

#define DISC_LAYOUT_ALT_TRIES 6

/** Score of the best other layout over the bits of guess g that reads other
 *  values (after prior resolution), -1 if none: how sure the field's exact bits
 *  are, which the non-overlapping options never compare. */
static double layout_runner_up(disc_ctx_t *ctx, disc_signal_t sig, const id_cmp_t *row,
                               const id_view_t *act, const id_view_t *base,
                               const disc_raw_guess_t *guesses, int n, int g,
                               const disc_option_t *chosen) {
    double best = -1.0;
    int tries = 0;
    for (int h = 0; h < n && tries < DISC_LAYOUT_ALT_TRIES; h++) {
        if (h == g || !(guesses[h].bitmask & guesses[g].bitmask)) continue;
        tries++;
        disc_option_t alt;
        if (!make_raw_option(ctx, sig, row, act, base, &guesses[h], &alt)) continue;
        /* another wheel of the same message is the same answer */
        if (sig == DISC_SIG_WHEEL_SPEED && alt.mask == chosen->mask) continue;
        if (fields_read_alike(&alt.field, &chosen->field, act, base)) continue;
        if (alt.score > best) best = alt.score;
    }
    return best;
}

/** Collect the best scored field options for one signal. */
static int build_options(disc_ctx_t *ctx, disc_signal_t sig,
                         disc_option_t *opts, int max_opts) {
    int phase_idx = signal_phase_index(sig);
    if (phase_idx <= DISC_PHASE_BASELINE || !phase_has_data(&ctx->phases[phase_idx])) return 0;

    const cmp_table_t *table = &ctx->cmp[phase_idx];
    if (table->count == 0) return 0;

    pre_rank_t *ranks = (pre_rank_t *)malloc((size_t)table->count * sizeof(*ranks));
    if (!ranks) return 0;
    for (int i = 0; i < table->count; i++) {
        const id_cmp_t *row = &table->rows[i];
        ranks[i].idx = i;
        ranks[i].pre = clamp_score(row->total_score, 0.0, 80.0) +
                       id_role_hint(sig, row, ctx->hz_threshold) +
                       id_specificity(ctx, sig, row);
    }
    qsort(ranks, (size_t)table->count, sizeof(*ranks), cmp_pre_rank_desc);

    int preselect = table->count < DISC_PRESELECT_IDS ? table->count : DISC_PRESELECT_IDS;
    /* At most DISC_MAX_DLC/2 regions per ID (runs are separated by a quiet
     * byte), each yielding up to DISC_GUESSES_PER_REGION options. */
    const int pool_max = DISC_PRESELECT_IDS * (DISC_MAX_DLC / 2 + 1) * DISC_GUESSES_PER_REGION;
    disc_option_t *pool = (disc_option_t *)calloc((size_t)pool_max, sizeof(*pool));
    int pool_count = 0;
    disc_raw_guess_t *guesses =
        (disc_raw_guess_t *)malloc(DISC_MAX_REGION_GUESSES * sizeof(*guesses));
    if (!guesses || !pool) {
        free(guesses);
        free(pool);
        free(ranks);
        return 0;
    }

    for (int r = 0; r < preselect; r++) {
        const id_cmp_t *row = &table->rows[ranks[r].idx];
        const id_view_t *act = ctx_view(ctx, phase_idx, row->can_id);
        const id_view_t *base = ctx_view(ctx, DISC_PHASE_BASELINE, row->can_id);

        if (!act) {
            disc_option_t opt;
            if (make_byte_option(ctx, sig, row, &opt)) {
                option_add(pool, &pool_count, pool_max, &opt);
            }
            continue;
        }

        /* Regions = runs of adjacent changing bytes, widened by one byte for
         * MSB headroom (not for gear enums). Wheel speed uses the whole frame. */
        int b = 0;
        while (b < row->byte_count) {
            if (row->bytes[b].score <= 0) {
                b++;
                continue;
            }
            int run_lo = b;
            while (b + 1 < row->byte_count && row->bytes[b + 1].score > 0) b++;
            int run_hi = b++;

            int lo = run_lo, hi = run_hi;
            if (sig == DISC_SIG_WHEEL_SPEED) {
                lo = 0;
                hi = row->dlc - 1;
            } else if (sig != DISC_SIG_GEAR) {
                if (lo > 0) lo--;
                if (hi + 1 < row->dlc) hi++;
            }

            int n = region_guesses(ctx, sig, act, base, row->dlc, lo, hi,
                                   guesses, DISC_MAX_REGION_GUESSES);
            uint64_t taken = 0;
            int picked = 0;
            for (int g = 0; g < n && picked < DISC_GUESSES_PER_REGION; g++) {
                if (guesses[g].bitmask & taken) continue;
                /* a guess the option checks reject leaves its bits to the next one */
                disc_option_t opt;
                if (!make_raw_option(ctx, sig, row, act, base, &guesses[g], &opt)) continue;
                taken |= guesses[g].bitmask;
                picked++;
                opt.layout_alt = layout_runner_up(ctx, sig, row, act, base, guesses, n, g, &opt);
                option_add(pool, &pool_count, pool_max, &opt);
            }
            if (sig == DISC_SIG_WHEEL_SPEED) break;
        }
    }
    free(guesses);
    free(ranks);

    qsort(pool, (size_t)pool_count, sizeof(pool[0]), cmp_option_desc);
    int count = pool_count < max_opts ? pool_count : max_opts;
    memcpy(opts, pool, (size_t)count * sizeof(*opts));
    free(pool);
    return count;
}

static bool options_conflict(const disc_option_t *a, const disc_option_t *b) {
    return a->cand.can_id == b->cand.can_id && (a->mask & b->mask) != 0;
}

typedef struct {
    disc_option_t (*opts)[DISC_OPTIONS_PER_SIGNAL];
    int    *count;
    int     cur[DISC_SIG_COUNT];
    int     best[DISC_SIG_COUNT];
    double  best_sum;
    double  bound[DISC_SIG_COUNT + 1];   /* best possible sum of signals s.. */
    int     forbid_sig;                  /* -1 = none; else these options of it are off */
    bool    forbid[DISC_OPTIONS_PER_SIGNAL];
} assign_state_t;

static void assign_dfs(assign_state_t *st, int s, double sum) {
    if (s == DISC_SIG_COUNT) {
        if (sum > st->best_sum + 1e-9) {
            st->best_sum = sum;
            memcpy(st->best, st->cur, sizeof(st->best));
        }
        return;
    }
    if (sum + st->bound[s] <= st->best_sum + 1e-9) return;

    for (int j = 0; j < st->count[s]; j++) {
        if (s == st->forbid_sig && st->forbid[j]) continue;
        const disc_option_t *o = &st->opts[s][j];
        bool clash = false;
        for (int t = 0; t < s && !clash; t++) {
            if (st->cur[t] >= 0) clash = options_conflict(o, &st->opts[t][st->cur[t]]);
        }
        if (clash) continue;
        st->cur[s] = j;
        assign_dfs(st, s + 1, sum + o->score);
    }
    st->cur[s] = -1;
    assign_dfs(st, s + 1, sum);
}

static void disc_analyze_internal(const disc_phase_t phases[DISC_NUM_PHASES],
                                  const disc_raw_phase_t *raw_phases,
                                  disc_result_t *result) {
    if (!phases || !result) return;

    disc_result_init(result);

    disc_ctx_t ctx;
    if (!ctx_init(&ctx, phases, raw_phases, true)) {
        result->bus_mode = disc_detect_bus_mode(&phases[DISC_PHASE_BASELINE]);
        return;
    }
    result->bus_mode = ctx.bus_mode;

    disc_option_t (*opts)[DISC_OPTIONS_PER_SIGNAL] =
        calloc(DISC_SIG_COUNT, sizeof(*opts));
    int count[DISC_SIG_COUNT] = {0};
    if (!opts) {
        ctx_free(&ctx);
        return;
    }

    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        count[s] = build_options(&ctx, (disc_signal_t)s, opts[s], DISC_OPTIONS_PER_SIGNAL);
#ifdef DISC_DEBUG
        for (int j = 0; j < count[s]; j++) {
            const disc_candidate_t *c = &opts[s][j].cand;
            fprintf(stderr, "  opt %-11s 0x%03X %2d|%-2d@%d%c score=%6.1f ev=%5.1f sp=%5.1f sh=%5.1f pr=%4.1f%s\n",
                    disc_signal_name((disc_signal_t)s), c->can_id, c->start_bit, c->bit_length,
                    c->byte_order, c->sign_char, opts[s][j].score, c->evidence_score,
                    c->specificity_score, c->shape_score, c->prior_score,
                    opts[s][j].strong ? " strong" : "");
        }
#endif
    }

    assign_state_t st;
    memset(&st, 0, sizeof(st));
    st.opts = opts;
    st.count = count;
    st.best_sum = -1.0;
    st.forbid_sig = -1;
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        st.cur[s] = -1;
        st.best[s] = -1;
    }
    st.bound[DISC_SIG_COUNT] = 0.0;
    for (int s = DISC_SIG_COUNT - 1; s >= 0; s--) {
        st.bound[s] = st.bound[s + 1] + (count[s] > 0 ? opts[s][0].score : 0.0);
    }
    assign_dfs(&st, 0, 0.0);

    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        int j = st.best[s];
        if (j < 0) continue;
        const disc_option_t *chosen = &opts[s][j];

        /* Runner-up assignment: the best total with this signal kept off its
         * location (every option over the same bits; the exact bits are the
         * layout margin below). The others may move, so two signals that swap
         * at an equal total get no margin. */
        assign_state_t alt = st;
        alt.best_sum = -1.0;
        alt.forbid_sig = s;
        for (int k = 0; k < count[s]; k++) {
            alt.forbid[k] = options_conflict(&opts[s][k], chosen);
        }
        for (int t = 0; t < DISC_SIG_COUNT; t++) alt.cur[t] = -1;
        assign_dfs(&alt, 0, 0.0);
        double assign_margin = st.best_sum - (alt.best_sum > 0.0 ? alt.best_sum : 0.0);
        if (assign_margin < 0.0) assign_margin = 0.0;
        /* ... and the best other reading of the same bits (12|8 vs 8|12). Both
         * readings are scored on the same field data, so a smaller gap separates
         * them than separates locations (calibrated on discover_regression). */
        double layout_margin = chosen->layout_alt >= 0.0 ? chosen->score - chosen->layout_alt : chosen->score;
        if (layout_margin < 0.0) layout_margin = 0.0;
        double margin = assign_margin < layout_margin ? assign_margin : layout_margin;

        disc_candidate_t cand = chosen->cand;
        cand.ambiguity_margin = margin;
        bool clear = (assign_margin >= 15.0 || assign_margin >= 0.15 * chosen->score) &&
                     (layout_margin >= 10.0 || layout_margin >= 0.10 * chosen->score);
        bool some = margin >= 5.0 || margin >= 0.05 * chosen->score;
        if (chosen->strong && clear && cand.specificity_score >= 0.0) {
            cand.confidence = DISC_CONF_HIGH;
        } else if (some && cand.specificity_score >= -15.0) {
            cand.confidence = DISC_CONF_MEDIUM;
        } else {
            cand.confidence = DISC_CONF_LOW;
        }
        result->signals[s] = cand;
        result->found[s] = true;
    }

    free(opts);
    ctx_free(&ctx);
}

void disc_analyze(const disc_phase_t phases[DISC_NUM_PHASES],
                  disc_result_t *result) {
    disc_analyze_internal(phases, NULL, result);
}

void disc_analyze_with_raw(const disc_phase_t phases[DISC_NUM_PHASES],
                           const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                           disc_result_t *result) {
    disc_analyze_internal(phases, raw_phases, result);
}

/*******************************************************************************
 * String builder
 ******************************************************************************/

static bool disc_sb_reserve(disc_string_builder_t *sb, size_t extra) {
    if (!sb || sb->failed) return false;
    if (extra > SIZE_MAX - sb->len - 1) {
        sb->failed = true;
        return false;
    }

    size_t need = sb->len + extra + 1;
    if (need <= sb->cap) return true;

    size_t new_cap = sb->cap ? sb->cap : 1024;
    while (new_cap < need) {
        if (new_cap > SIZE_MAX / 2) {
            new_cap = need;
            break;
        }
        new_cap *= 2;
    }

    char *new_data = (char *)realloc(sb->data, new_cap);
    if (!new_data) {
        sb->failed = true;
        return false;
    }

    sb->data = new_data;
    sb->cap = new_cap;
    return true;
}

static bool disc_sb_appendf(disc_string_builder_t *sb, const char *fmt, ...) {
    if (!sb || !fmt) return false;

    va_list ap;
    va_start(ap, fmt);
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int needed = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (needed < 0) {
        va_end(ap_copy);
        sb->failed = true;
        return false;
    }

    if (!disc_sb_reserve(sb, (size_t)needed)) {
        va_end(ap_copy);
        return false;
    }

    vsnprintf(sb->data + sb->len, sb->cap - sb->len, fmt, ap_copy);
    va_end(ap_copy);
    sb->len += (size_t)needed;
    return true;
}

static bool disc_sb_append_json_string(disc_string_builder_t *sb, const char *s) {
    if (!disc_sb_appendf(sb, "\"")) return false;
    if (!s) s = "";

    for (; *s; s++) {
        unsigned char ch = (unsigned char)*s;
        switch (ch) {
        case '"':
            if (!disc_sb_appendf(sb, "\\\"")) return false;
            break;
        case '\\':
            if (!disc_sb_appendf(sb, "\\\\")) return false;
            break;
        case '\n':
            if (!disc_sb_appendf(sb, "\\n")) return false;
            break;
        case '\r':
            if (!disc_sb_appendf(sb, "\\r")) return false;
            break;
        case '\t':
            if (!disc_sb_appendf(sb, "\\t")) return false;
            break;
        default:
            if (ch < 0x20) {
                if (!disc_sb_appendf(sb, "\\u%04x", ch)) return false;
            } else {
                if (!disc_sb_reserve(sb, 1)) return false;
                sb->data[sb->len++] = (char)ch;
                sb->data[sb->len] = '\0';
            }
            break;
        }
    }

    return disc_sb_appendf(sb, "\"");
}

static char *disc_sb_finish(disc_string_builder_t *sb, size_t *length_out) {
    if (!disc_sb_reserve(sb, 0)) {
        free(sb->data);
        if (length_out) *length_out = 0;
        return NULL;
    }
    sb->data[sb->len] = '\0';
    if (length_out) *length_out = sb->len;
    return sb->data;
}

/*******************************************************************************
 * Signal metadata
 ******************************************************************************/

static int signal_phase_index(disc_signal_t sig) {
    switch (sig) {
    case DISC_SIG_STEERING:    return DISC_PHASE_STEERING;
    case DISC_SIG_RPM:         return DISC_PHASE_THROTTLE;
    case DISC_SIG_THROTTLE:    return DISC_PHASE_THROTTLE;
    case DISC_SIG_BRAKE:       return DISC_PHASE_BRAKE;
    case DISC_SIG_GEAR:        return DISC_PHASE_GEAR;
    case DISC_SIG_WHEEL_SPEED: return DISC_PHASE_WHEEL_SPEED;
    default:                   return DISC_PHASE_BASELINE;
    }
}

static const char *signal_dbc_name(disc_signal_t sig) {
    switch (sig) {
    case DISC_SIG_STEERING:    return "Steering";
    case DISC_SIG_RPM:         return "RPM";
    case DISC_SIG_THROTTLE:    return "Throttle";
    case DISC_SIG_BRAKE:       return "Brake";
    case DISC_SIG_GEAR:        return "Gear";
    case DISC_SIG_WHEEL_SPEED: return "WheelSpeed";
    default:                   return "Signal";
    }
}

/** Default canonical id when no prior names the quantity. */
static const char *signal_canonical_id(disc_signal_t sig) {
    switch (sig) {
    case DISC_SIG_STEERING:    return "steering.wheel.angle";
    case DISC_SIG_RPM:         return "engine.speed";
    case DISC_SIG_THROTTLE:    return "throttle.position";
    case DISC_SIG_BRAKE:       return "brake.pressure";
    case DISC_SIG_GEAR:        return "transmission.gear.selected";
    case DISC_SIG_WHEEL_SPEED: return "wheel.speed";
    default:                   return "unknown";
    }
}

static const char *canonical_label(const char *canonical, disc_signal_t sig) {
    static const struct { const char *id; const char *label; } k_labels[] = {
        { "steering.wheel.angle",        "Steering wheel angle" },
        { "engine.speed",                "Engine speed" },
        { "throttle.position",           "Throttle position" },
        { "accelerator.pedal.position",  "Accelerator pedal position" },
        { "brake.pedal.position",        "Brake pedal position" },
        { "brake.pressure",              "Brake pressure" },
        { "brake.pressure.front",        "Front brake pressure" },
        { "brake.pressure.rear",         "Rear brake pressure" },
        { "transmission.gear.selected",  "Selected gear" },
        { "transmission.gear.requested", "Requested gear" },
        { "vehicle.speed",               "Vehicle speed" },
        { "wheel.speed",                 "Wheel speed" },
        { "wheel.speed.front_left",      "Front left wheel speed" },
        { "wheel.speed.front_right",     "Front right wheel speed" },
        { "wheel.speed.rear_left",       "Rear left wheel speed" },
        { "wheel.speed.rear_right",      "Rear right wheel speed" },
    };
    for (size_t i = 0; i < sizeof(k_labels) / sizeof(k_labels[0]); i++) {
        if (strcmp(k_labels[i].id, canonical) == 0) return k_labels[i].label;
    }
    return signal_dbc_name(sig);
}

/*******************************************************************************
 * Export plan (shared by draft DBC and draft VSP)
 ******************************************************************************/

#define DISC_PLAN_MAX_FIELDS (DISC_SIG_COUNT * 4)

typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    int      cycle_ms;
    int      first;     /* index into fields */
    int      count;
} plan_msg_t;

typedef struct {
    disc_export_signal_t fields[DISC_PLAN_MAX_FIELDS];
    int                  field_count;
    plan_msg_t           msgs[DISC_PLAN_MAX_FIELDS];
    int                  msg_count;
} export_plan_t;

static int cmp_export_field(const void *a, const void *b) {
    const disc_export_signal_t *x = (const disc_export_signal_t *)a;
    const disc_export_signal_t *y = (const disc_export_signal_t *)b;
    if (x->can_id != y->can_id) return x->can_id < y->can_id ? -1 : 1;
    return (x->start_bit > y->start_bit) - (x->start_bit < y->start_bit);
}

static void export_from_field(disc_ctx_t *ctx, disc_export_signal_t *out,
                              disc_signal_t sig, uint32_t can_id, const field_t *f,
                              const char *fallback_name,
                              int32_t raw_min, int32_t raw_max) {
    memset(out, 0, sizeof(*out));
    out->signal = sig;
    out->can_id = can_id;
    out->start_bit = f->start;
    out->bit_length = f->len;
    out->byte_order = f->order;
    out->sign_char = f->sign;
    out->sign_bit = f->sm_bit;
    out->bitmask = f->bits;

    prior_obs_t obs = { raw_min, raw_max, false, 0 };
    if (ctx->has_raw) {
        observe_field(ctx, ctx_view(ctx, signal_phase_index(sig), can_id),
                      ctx_view(ctx, DISC_PHASE_BASELINE, can_id), f, &obs);
    }
    int idx = find_exact_prior(sig, can_id, f, &obs);
    if (idx >= 0) {
        const disc_discover_prior_t *p = &s_can_discover_priors[idx];
        snprintf(out->name, sizeof(out->name), "%s", p->name);
        snprintf(out->factor, sizeof(out->factor), "%s", p->factor);
        snprintf(out->offset, sizeof(out->offset), "%s", p->offset);
        snprintf(out->min_val, sizeof(out->min_val), "%s", p->min_val);
        snprintf(out->max_val, sizeof(out->max_val), "%s", p->max_val);
        snprintf(out->unit, sizeof(out->unit), "%s", p->unit);
        snprintf(out->canonical, sizeof(out->canonical), "%s",
                 p->canonical[0] ? p->canonical : signal_canonical_id(sig));
        out->canonical_mapped = p->canonical[0] != '\0';
        out->values_json = p->values_json;
        out->values_dbc = p->values_dbc;
        return;
    }

    snprintf(out->name, sizeof(out->name), "%s", fallback_name);
    snprintf(out->factor, sizeof(out->factor), "1");
    snprintf(out->offset, sizeof(out->offset), "0");
    snprintf(out->min_val, sizeof(out->min_val), "%d", raw_min);
    snprintf(out->max_val, sizeof(out->max_val), "%d", raw_max);
    snprintf(out->canonical, sizeof(out->canonical), "%s", signal_canonical_id(sig));
}

static void plan_unique_name(export_plan_t *plan, disc_export_signal_t *f) {
    char base[DISC_PRIOR_NAME_LEN];
    snprintf(base, sizeof(base), "%s", f->name);
    for (int n = 2; n < 100; n++) {
        bool clash = false;
        for (int i = 0; i < plan->field_count && !clash; i++) {
            clash = plan->fields[i].can_id == f->can_id && strcmp(plan->fields[i].name, f->name) == 0;
        }
        if (!clash) return;
        snprintf(f->name, sizeof(f->name), "%.56s_%d", base, n);
    }
}

/** DBC message id: a 29-bit id carries bit 31 (the VSP writer's id_format
 *  rule, id > 0x7FF, decides which ids are 29-bit). */
static uint32_t dbc_message_id(uint32_t can_id) {
    return can_id > 0x7FFu ? (can_id | 0x80000000u) : can_id;
}

static bool plan_add(export_plan_t *plan, const disc_export_signal_t *f) {
    if (plan->field_count >= DISC_PLAN_MAX_FIELDS) return false;
    for (int i = 0; i < plan->field_count; i++) {
        if (plan->fields[i].can_id == f->can_id && (plan->fields[i].bitmask & f->bitmask)) {
            return false;   /* never emit overlapping fields */
        }
    }
    disc_export_signal_t copy = *f;
    plan_unique_name(plan, &copy);
    plan->fields[plan->field_count++] = copy;
    return true;
}

static int estimate_cycle_ms_from_phase(const disc_phase_t *phase, uint32_t can_id) {
    const disc_id_t *id = find_id_in_phase(phase, can_id);
    if (!id || phase->duration_s <= 0 || id->frame_count <= 0) return -1;
    double hz = (double)id->frame_count / phase->duration_s;
    if (hz <= 0) return -1;
    return (int)(1000.0 / hz + 0.5);
}

/**
 * Mean frame interval over the phase. Adapters often deliver frames in
 * batches that share one timestamp, so per-frame intervals are unreliable;
 * the span divided by the frame count is not.
 */
static int compute_cycle_ms_from_view(const id_view_t *v) {
    if (!v || v->count < 3) return -1;
    int64_t first = -1, last = -1;
    int n = 0;
    for (int i = 0; i < v->count; i++) {
        int64_t ts = v->frames[i].timestamp_ms;
        if (ts <= 0) continue;
        if (first < 0) first = ts;
        last = ts;
        n++;
    }
    if (n < 3 || last <= first) return -1;
    double mean = (double)(last - first) / (double)(n - 1);
    return mean >= 0.5 ? (int)(mean + 0.5) : -1;
}

static int estimate_message_cycle_ms(const disc_ctx_t *ctx, uint32_t can_id,
                                     const disc_result_t *result) {
    int cycle_ms = compute_cycle_ms_from_view(ctx_view(ctx, DISC_PHASE_BASELINE, can_id));
    if (cycle_ms > 0) return cycle_ms;

    cycle_ms = estimate_cycle_ms_from_phase(&ctx->phases[DISC_PHASE_BASELINE], can_id);
    if (cycle_ms > 0) return cycle_ms;

    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        if (!result->found[s] || result->signals[s].can_id != can_id) continue;
        int phase_idx = signal_phase_index((disc_signal_t)s);
        cycle_ms = compute_cycle_ms_from_view(ctx_view(ctx, phase_idx, can_id));
        if (cycle_ms > 0) return cycle_ms;
        cycle_ms = estimate_cycle_ms_from_phase(&ctx->phases[phase_idx], can_id);
        if (cycle_ms > 0) return cycle_ms;
        if (result->signals[s].hz > 0) return (int)(1000.0 / result->signals[s].hz + 0.5);
    }
    return -1;
}

/**
 * Turn a discovery result into the exact list of messages and fields that
 * both exporters write. Each found signal contributes its reported layout
 * (wheel speed may contribute one field per wheel). Prior name/scale/unit is
 * applied only on an exact layout + sign match.
 */
static bool build_export_plan(disc_ctx_t *ctx, const disc_result_t *result,
                              export_plan_t *plan) {
    memset(plan, 0, sizeof(*plan));

    int order[DISC_SIG_COUNT];
    int n = 0;
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        if (result->found[s] && result->signals[s].start_bit >= 0) order[n++] = s;
    }
    for (int i = 0; i < n; i++) {           /* higher score claims bits first */
        for (int j = i + 1; j < n; j++) {
            if (result->signals[order[j]].score > result->signals[order[i]].score) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        disc_signal_t sig = (disc_signal_t)order[i];
        const disc_candidate_t *cand = &result->signals[sig];
        field_t main_field;
        if (!field_make_sm(cand->start_bit, cand->bit_length, cand->byte_order,
                           cand->sign_char, cand->sign_bit, &main_field)) {
            continue;
        }

        if (sig == DISC_SIG_WHEEL_SPEED && ctx->has_raw) {
            disc_raw_guess_t parts[4];
            int phase_idx = signal_phase_index(sig);
            int count = wheel_split(ctx, ctx_view(ctx, phase_idx, cand->can_id),
                                    ctx_view(ctx, DISC_PHASE_BASELINE, cand->can_id),
                                    cand->dlc, parts);
            bool contains_main = false;
            for (int k = 0; k < count; k++) {
                contains_main |= parts[k].start_bit == main_field.start &&
                                 parts[k].bit_length == main_field.len &&
                                 parts[k].byte_order == main_field.order;
            }
            if (count >= 2 && contains_main) {
                for (int k = 0; k < count; k++) {
                    field_t pf;
                    if (!field_make(parts[k].start_bit, parts[k].bit_length,
                                    parts[k].byte_order, '+', &pf)) continue;
                    char name[DISC_PRIOR_NAME_LEN];
                    snprintf(name, sizeof(name), "WheelSpeed_%d", k + 1);
                    disc_export_signal_t e;
                    export_from_field(ctx, &e, sig, cand->can_id, &pf, name,
                                      parts[k].raw_min, parts[k].raw_max);
                    /* Which wheel a part is stays unknown without a prior (the
                       profile corpus orders FL FR RL RR in 72 % of messages):
                       the reported part is "wheel.speed", the rest go without
                       a canonical id rather than share it. */
                    bool reported = parts[k].start_bit == main_field.start &&
                                    parts[k].bit_length == main_field.len &&
                                    parts[k].byte_order == main_field.order;
                    if (!e.canonical_mapped && !reported) e.canonical[0] = '\0';
                    plan_add(plan, &e);
                }
                continue;
            }
        }

        disc_export_signal_t e;
        export_from_field(ctx, &e, sig, cand->can_id, &main_field, signal_dbc_name(sig),
                          cand->raw_min, cand->raw_max);
        plan_add(plan, &e);
    }

    qsort(plan->fields, (size_t)plan->field_count, sizeof(plan->fields[0]), cmp_export_field);

    for (int i = 0; i < plan->field_count; i++) {
        const disc_export_signal_t *f = &plan->fields[i];
        if (plan->msg_count == 0 || plan->msgs[plan->msg_count - 1].can_id != f->can_id) {
            plan_msg_t *m = &plan->msgs[plan->msg_count++];
            m->can_id = f->can_id;
            m->first = i;
            m->count = 0;
            m->dlc = 0;
            for (int s = 0; s < DISC_SIG_COUNT; s++) {
                if (result->found[s] && result->signals[s].can_id == f->can_id &&
                    result->signals[s].dlc > m->dlc) {
                    m->dlc = result->signals[s].dlc;
                }
            }
            if (m->dlc > DISC_MAX_DLC) m->dlc = DISC_MAX_DLC;
            m->cycle_ms = estimate_message_cycle_ms(ctx, f->can_id, result);
        }
        plan->msgs[plan->msg_count - 1].count++;
    }
    return true;
}

/** Prepare context + plan for an exporter. Caller frees ctx with ctx_free. */
static export_plan_t *prepare_export(const disc_phase_t phases[DISC_NUM_PHASES],
                                     const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                                     const disc_result_t *result, disc_ctx_t *ctx) {
    if (!ctx_init(ctx, phases, raw_phases, false)) return NULL;
    export_plan_t *plan = (export_plan_t *)calloc(1, sizeof(*plan));
    if (!plan || !build_export_plan(ctx, result, plan)) {
        free(plan);
        ctx_free(ctx);
        return NULL;
    }
    return plan;
}

static int collect_gear_values(const id_view_t *v, const disc_export_signal_t *sig,
                               int values[16]) {
    field_t f;
    if (!v || !field_make_sm(sig->start_bit, sig->bit_length, sig->byte_order,
                             sig->sign_char, sig->sign_bit, &f)) {
        return 0;
    }

    int count = 0;
    for (int i = 0; i < v->count; i++) {
        if (v->frames[i].dlc <= f.max_byte) continue;
        int value = (int)field_value(&f, field_raw(&f, v->le[i], v->be[i]));
        bool seen = false;
        for (int j = 0; j < count && !seen; j++) seen = values[j] == value;
        if (seen) continue;
        if (count >= 16) return 0;
        values[count++] = value;
    }
    qsort(values, (size_t)count, sizeof(*values), cmp_int);
    return count;
}

/*******************************************************************************
 * Draft DBC Export
 ******************************************************************************/

char *disc_render_draft_dbc(const disc_phase_t phases[DISC_NUM_PHASES],
                            const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                            const disc_result_t *result,
                            size_t *length_out) {
    if (length_out) *length_out = 0;
    if (!phases || !raw_phases || !result) return NULL;

    disc_ctx_t ctx;
    export_plan_t *plan = prepare_export(phases, raw_phases, result, &ctx);
    if (!plan) return NULL;

    disc_string_builder_t out = {0};

#define OUT(...) do { if (!disc_sb_appendf(&out, __VA_ARGS__)) goto fail; } while (0)

    OUT("VERSION \"\"\n\n");
    OUT("NS_ :\n");
    OUT("\tNS_DESC_\n\tBA_DEF_\n\tBA_\n\tVAL_\n\tCAT_DEF_\n\tCAT_\n\tFILTER\n");
    OUT("\tBA_DEF_DEF_\n\tEV_DATA_\n\tENVVAR_DATA_\n\tSGTYPE_\n\tSGTYPE_VAL_\n");
    OUT("\tBA_DEF_SGTYPE_\n\tBA_SGTYPE_\n\tSIG_TYPE_REF_\n\tVAL_TABLE_\n");
    OUT("\tSIG_GROUP_\n\tSIGTYPE_VALTYPE_\n\tSG_MUL_VAL_\n\n");
    OUT("BS_: 500\n\n");
    OUT("BU_: DISCOVER\n\n");

    for (int m = 0; m < plan->msg_count; m++) {
        const plan_msg_t *msg = &plan->msgs[m];
        OUT("BO_ %u MSG_%03X: %u DISCOVER\n", dbc_message_id(msg->can_id), msg->can_id, msg->dlc);
        for (int i = msg->first; i < msg->first + msg->count; i++) {
            const disc_export_signal_t *sig = &plan->fields[i];
            bool sm = sig->sign_char == 'S';
            OUT(" SG_ %s : %d|%d@%d%c (%s,%s) [%s|%s] \"%s\" Vector__XXX\n",
                sig->name, sig->start_bit, sig->bit_length, sig->byte_order,
                sm ? '+' : sig->sign_char, sig->factor, sig->offset, sig->min_val,
                sig->max_val, sig->unit);
            if (sm) {
                /* DBC has no sign-magnitude type; vsp2dbc/dbc2vsp use this pair. */
                OUT(" SG_ %s_Sign : %d|1@1+ (1,0) [0|1] \"\" Vector__XXX\n",
                    sig->name, sig->sign_bit);
            }
        }
        OUT("\n");
    }

    for (int i = 0; i < plan->field_count; i++) {
        const disc_export_signal_t *sig = &plan->fields[i];
        if (sig->values_dbc && sig->values_dbc[0]) {
            /* The matched prior's value table (raw code -> label). */
            OUT("\nVAL_ %u %s %s ;\n", dbc_message_id(sig->can_id), sig->name, sig->values_dbc);
            continue;
        }
        if (sig->signal != DISC_SIG_GEAR) continue;
        int values[16];
        int value_count = collect_gear_values(ctx_view(&ctx, DISC_PHASE_GEAR, sig->can_id),
                                              sig, values);
        if (value_count > 0) {
            OUT("\nVAL_ %u %s", dbc_message_id(sig->can_id), sig->name);
            for (int v = 0; v < value_count; v++) {
                OUT(" %d \"raw_0x%02X\"", values[v], (unsigned)values[v]);
            }
            OUT(" ;\n");
        }
    }

    OUT("\nBA_DEF_ BO_ \"GenMsgCycleTime\" INT 0 10000;\n");
    OUT("BA_DEF_DEF_ \"GenMsgCycleTime\" 0;\n\n");
    for (int m = 0; m < plan->msg_count; m++) {
        if (plan->msgs[m].cycle_ms > 0) {
            OUT("BA_ \"GenMsgCycleTime\" BO_ %u %d;\n",
                dbc_message_id(plan->msgs[m].can_id), plan->msgs[m].cycle_ms);
        }
    }

#undef OUT

    free(plan);
    ctx_free(&ctx);
    return disc_sb_finish(&out, length_out);

fail:
    free(plan);
    ctx_free(&ctx);
    free(out.data);
    return NULL;
}

/*******************************************************************************
 * Draft VSP Export
 ******************************************************************************/

static bool disc_sb_append_decode(disc_string_builder_t *sb, const char *indent,
                                  const disc_export_signal_t *sig) {
    return disc_sb_appendf(sb, "%s\"start_bit\": %d,\n", indent, sig->start_bit) &&
           disc_sb_appendf(sb, "%s\"bit_length\": %d,\n", indent, sig->bit_length) &&
           disc_sb_appendf(sb, "%s\"byte_order\": \"%s\",\n", indent,
                           sig->byte_order == 1 ? "little_endian" : "big_endian") &&
           disc_sb_appendf(sb, "%s\"signed\": %s,\n", indent,
                           sig->sign_char == '-' ? "true" : "false") &&
           (sig->sign_char != 'S' ||
            disc_sb_appendf(sb, "%s\"sign_bit\": %d,\n", indent, sig->sign_bit)) &&
           disc_sb_appendf(sb, "%s\"factor\": %s,\n", indent, sig->factor[0] ? sig->factor : "1") &&
           disc_sb_appendf(sb, "%s\"offset\": %s,\n", indent, sig->offset[0] ? sig->offset : "0") &&
           disc_sb_appendf(sb, "%s\"minimum\": %s,\n", indent, sig->min_val[0] ? sig->min_val : "0") &&
           disc_sb_appendf(sb, "%s\"maximum\": %s,\n", indent, sig->max_val[0] ? sig->max_val : "0") &&
           disc_sb_appendf(sb, "%s\"unit\": ", indent) &&
           disc_sb_append_json_string(sb, sig->unit) &&
           (!sig->values_json || !sig->values_json[0] ||
            disc_sb_appendf(sb, ",\n%s\"values\": %s", indent, sig->values_json)) &&
           disc_sb_appendf(sb, ",\n%s\"mux\": null\n", indent);
}

char *disc_render_draft_vsp(const disc_phase_t phases[DISC_NUM_PHASES],
                            const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                            const disc_result_t *result,
                            size_t *length_out) {
    if (length_out) *length_out = 0;
    if (!phases || !raw_phases || !result) return NULL;

    disc_ctx_t ctx;
    export_plan_t *plan = prepare_export(phases, raw_phases, result, &ctx);
    if (!plan) return NULL;

    disc_string_builder_t out = {0};

#define OUT(...) do { if (!disc_sb_appendf(&out, __VA_ARGS__)) goto fail; } while (0)
#define OUTS(s) do { if (!disc_sb_append_json_string(&out, (s))) goto fail; } while (0)

    OUT("{\n");
    OUT("  \"schema\": \"vehicle_data_profile.v1\",\n");
    OUT("  \"vehicle\": {\n");
    OUT("    \"manufacturer\": \"unknown\",\n");
    OUT("    \"model\": \"unknown\"\n");
    OUT("  },\n");
    OUT("  \"applicability\": {\n");
    OUT("    \"generation\": null,\n");
    OUT("    \"platform\": null,\n");
    OUT("    \"year_start\": null,\n");
    OUT("    \"year_end\": null,\n");
    OUT("    \"market\": null,\n");
    OUT("    \"trim\": null,\n");
    OUT("    \"powertrain\": null\n");
    OUT("  },\n");
    OUT("  \"buses\": [\n");
    OUT("    {\"id\": \"can0\", \"type\": \"can\", \"bitrate\": 500000}\n");
    OUT("  ],\n");
    OUT("  \"can_frames\": [\n");

    for (int m = 0; m < plan->msg_count; m++) {
        const plan_msg_t *msg = &plan->msgs[m];
        OUT("%s    {\n", m == 0 ? "" : ",\n");
        OUT("      \"bus\": \"can0\",\n");
        OUT("      \"frame_id\": %u,\n", msg->can_id);
        OUT("      \"frame_id_hex\": \"0x%X\",\n", msg->can_id);
        OUT("      \"id_format\": \"%s\",\n", msg->can_id > 0x7FFu ? "extended29" : "standard11");
        OUT("      \"name\": \"MSG_%03X\",\n", msg->can_id);
        OUT("      \"dlc\": %u,\n", msg->dlc);
        if (msg->cycle_ms > 0) OUT("      \"cycle_time_ms\": %d,\n", msg->cycle_ms);
        OUT("      \"signals\": [\n");
        for (int i = msg->first; i < msg->first + msg->count; i++) {
            const disc_export_signal_t *sig = &plan->fields[i];
            OUT("%s        {\n", i == msg->first ? "" : ",\n");
            OUT("          \"name\": ");
            OUTS(sig->name);
            OUT(",\n          \"decode\": {\n");
            if (!disc_sb_append_decode(&out, "            ", sig)) goto fail;
            OUT("          }\n");
            OUT("        }");
        }
        OUT("\n      ]\n");
        OUT("    }");
    }

    OUT("\n  ],\n");
    OUT("  \"diagnostic_queries\": [],\n");
    OUT("  \"signals\": [\n");

    for (int i = 0; i < plan->field_count; i++) {
        const disc_export_signal_t *sig = &plan->fields[i];
        const disc_candidate_t *cand = &result->signals[sig->signal];
        OUT("%s    {\n", i == 0 ? "" : ",\n");
        OUT("      \"name\": ");
        OUTS(sig->name);
        OUT(",\n");
        OUT("      \"acquisition\": {\n");
        OUT("        \"type\": \"can_frame\",\n");
        OUT("        \"bus\": \"can0\",\n");
        OUT("        \"frame_id\": %u,\n", sig->can_id);
        OUT("        \"frame_id_hex\": \"0x%X\",\n", sig->can_id);
        OUT("        \"frame_name\": \"MSG_%03X\"\n", sig->can_id);
        OUT("      },\n");
        OUT("      \"decode\": {\n");
        if (!disc_sb_append_decode(&out, "        ", sig)) goto fail;
        OUT("      },\n");
        if (sig->canonical[0]) {   /* a wheel part of unknown position has none */
            OUT("      \"canonical\": {\n");
            OUT("        \"id\": ");
            OUTS(sig->canonical);
            OUT(",\n        \"label\": ");
            OUTS(canonical_label(sig->canonical, sig->signal));
            OUT(",\n        \"quantity\": \"unknown\",\n");
            OUT("        \"preferred_unit\": ");
            OUTS(sig->unit);
            OUT(",\n        \"confidence\": \"%s\",\n", sig->canonical_mapped ? "mapped" : "heuristic");
            OUT("        \"aliases\": [");
            OUTS(sig->name);
            OUT("]\n");
            OUT("      },\n");
        }
        OUT("      \"discovery\": {\n");
        OUT("        \"tool\": \"can_discover\",\n");
        OUT("        \"signal\": ");
        OUTS(disc_signal_name(sig->signal));
        OUT(",\n        \"confidence\": ");
        OUTS(disc_confidence_name(cand->confidence));
        OUT(",\n");
        OUT("        \"observed_hz\": %.3f,\n", cand->hz);
        OUT("        \"final_score\": %.3f,\n", cand->score);
        OUT("        \"evidence_score\": %.3f,\n", cand->evidence_score);
        OUT("        \"specificity_score\": %.3f,\n", cand->specificity_score);
        OUT("        \"shape_score\": %.3f,\n", cand->shape_score);
        OUT("        \"prior_score\": %.3f,\n", cand->prior_score);
        OUT("        \"prior_support\": %d,\n", cand->prior_support);
        OUT("        \"ambiguity_margin\": %.3f,\n", cand->ambiguity_margin);
        OUT("        \"raw_min\": %d,\n", cand->raw_min);
        OUT("        \"raw_max\": %d\n", cand->raw_max);
        OUT("      }\n");
        OUT("    }");
    }

    OUT("\n  ]\n");
    OUT("}\n");

#undef OUT
#undef OUTS

    free(plan);
    ctx_free(&ctx);
    return disc_sb_finish(&out, length_out);

fail:
    free(plan);
    ctx_free(&ctx);
    free(out.data);
    return NULL;
}

/*******************************************************************************
 * File writers
 ******************************************************************************/

static bool write_text_file(const char *path, char *text, size_t len) {
    if (!text) return false;
    FILE *f = fopen(path, "w");
    if (!f) {
        free(text);
        return false;
    }
    bool ok = fwrite(text, 1, len, f) == len;
    free(text);
    if (fclose(f) != 0) ok = false;
    return ok;
}

bool disc_write_draft_dbc(const char *path,
                          const disc_phase_t phases[DISC_NUM_PHASES],
                          const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                          const disc_result_t *result) {
    if (!path || !phases || !raw_phases || !result) return false;
    size_t len = 0;
    char *text = disc_render_draft_dbc(phases, raw_phases, result, &len);
    return write_text_file(path, text, len);
}

bool disc_write_draft_vsp(const char *path,
                          const disc_phase_t phases[DISC_NUM_PHASES],
                          const disc_raw_phase_t raw_phases[DISC_NUM_PHASES],
                          const disc_result_t *result) {
    if (!path || !phases || !raw_phases || !result) return false;
    size_t len = 0;
    char *text = disc_render_draft_vsp(phases, raw_phases, result, &len);
    return write_text_file(path, text, len);
}
