/**
 * @file discover_regression.c
 * @brief Synthetic regression tests for the can_discover engine.
 *
 * Each scenario synthesizes the six capture phases the live tool records,
 * runs disc_analyze_with_raw(), and checks:
 *   - each expected signal is found on an acceptable CAN ID / bitfield
 *   - absent signals are not reported
 *   - draft DBC and draft VSP describe the same fields, with DLC <= 8
 *   - analysis is safe on a 512 KB thread stack
 *
 * Synthetic captures prove regressions, not real-vehicle accuracy. Use
 * discover_replay on recorded sessions for that.
 */

#include "can_discover.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*******************************************************************************
 * Frame sink
 ******************************************************************************/

typedef struct {
    can_frame_t *frames;
    int count;
    int cap;
} sink_t;

static void sink_push(sink_t *s, uint32_t id, const uint8_t *d, uint8_t dlc, int64_t ts) {
    if (s->count == s->cap) {
        s->cap = s->cap ? s->cap * 2 : 4096;
        s->frames = (can_frame_t *)realloc(s->frames, (size_t)s->cap * sizeof(*s->frames));
        if (!s->frames) { fprintf(stderr, "out of memory\n"); exit(2); }
    }
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.can_id = id;
    f.dlc = dlc;
    memcpy(f.data, d, dlc);
    f.timestamp_ms = ts;
    f.is_extended = id > 0x7FF;
    s->frames[s->count++] = f;
}

/*******************************************************************************
 * DBC bit helpers (independent of the engine)
 ******************************************************************************/

static int next_bit(int bit, int order) {
    if (order == 1) return bit + 1;
    return (bit % 8 == 0) ? bit + 15 : bit - 1;
}

static uint64_t field_mask(int start, int len, int order) {
    uint64_t m = 0;
    int bit = start;
    for (int i = 0; i < len; i++) {
        if (bit >= 0 && bit < 64) m |= 1ULL << bit;
        bit = next_bit(bit, order);
    }
    return m;
}

/** Pack raw into data using DBC numbering (Intel: start=LSB, Motorola: start=MSB). */
static void put_field(uint8_t d[8], int start, int len, int order, uint32_t raw) {
    int bit = start;
    for (int i = 0; i < len; i++) {
        int value_bit = (order == 1) ? i : (len - 1 - i);
        uint8_t b = (uint8_t)((raw >> value_bit) & 1u);
        d[bit / 8] = (uint8_t)((d[bit / 8] & ~(1u << (bit % 8))) | (b << (bit % 8)));
        bit = next_bit(bit, order);
    }
}

/*******************************************************************************
 * Deterministic noise
 ******************************************************************************/

static uint32_t g_rng = 1;
static uint32_t rng_next(void) {
    g_rng = g_rng * 1103515245u + 12345u;
    return (g_rng >> 16) & 0x7FFF;
}
static double jitter(double amp) {
    return ((double)rng_next() / 32767.0 * 2.0 - 1.0) * amp;
}

/*******************************************************************************
 * Scenario definition
 ******************************************************************************/

#define MAX_ALT 4

typedef struct {
    uint32_t id;
    int start, len, order;
} field_ref_t;

typedef struct {
    bool present;              /* signal exists in this capture */
    int alt_count;
    field_ref_t alt[MAX_ALT];  /* acceptable locations */
    bool not_high;             /* the capture cannot tell the alternatives apart: never HIGH */
} expect_t;

typedef void (*gen_fn)(int phase, int k, double t, sink_t *sink);

typedef struct {
    const char *name;
    gen_fn gen;
    int tick_hz;                       /* generator tick rate */
    double durations[DISC_NUM_PHASES]; /* seconds; 0 = phase skipped */
    expect_t expect[DISC_SIG_COUNT];   /* a "_loss<P>_s<S>" name suffix drops P % of the
                                          frames at random (seed S), as a lossy adapter does */
} scenario_t;

#define FIELD(i, s, l, o) { (i), (s), (l), (o) }

/*******************************************************************************
 * Reviewer repro scenarios (ported verbatim from the review harness)
 ******************************************************************************/

typedef struct {
    int rpm_scale;
    uint32_t rpm_id;
    bool rpm_big_endian;   /* bytes 0-1 BE, else bytes 2-3 LE + counter */
    uint32_t noise_base;
    bool no_brake;         /* sim5: brake absent, 0x500 byte3 50 -> 51 */
} review_cfg_t;

static review_cfg_t g_review;

static void gen_review(int p, int k, double t, sink_t *sink) {
    const review_cfg_t *c = &g_review;
    int64_t ts = (int64_t)k * 10 + 1;
    int rpm = 800 * c->rpm_scale;
    double thr = 0, steer = 0;
    int brake = 0, gear = 0, ws = 0;
    if (p == 2) { thr = 50 + 50 * sin(t * 2); rpm = (int)((800 + 3000 * thr / 100.0) * c->rpm_scale); }
    if (p == 1) { steer = 3000 * sin(t); }
    if (p == 3 && !c->no_brake) { brake = (int)(500 + 400 * sin(t * 3)); }
    if (p == 4) { gear = (k / 250) % 4; brake = 600; }
    if (p == 5) { ws = (int)(3000 + 1000 * sin(t * 0.5)); rpm = (int)((1500 + 500 * sin(t * 0.5)) * c->rpm_scale); thr = 20 + 10 * sin(t); }

    uint8_t d[8] = {0};
    if (c->rpm_big_endian) {
        d[0] = (uint8_t)(rpm >> 8); d[1] = (uint8_t)(rpm & 0xFF);
    } else {
        d[0] = (uint8_t)(k & 0xF); d[2] = (uint8_t)(rpm & 0xFF); d[3] = (uint8_t)(rpm >> 8);
    }
    sink_push(sink, c->rpm_id, d, 8, ts);
    memset(d, 0, 8); d[5] = (uint8_t)(thr * 2.55); sink_push(sink, 0x1A0, d, 8, ts);
    memset(d, 0, 8); int16_t s = (int16_t)steer; d[1] = (uint8_t)((uint16_t)s >> 8); d[2] = (uint8_t)(s & 0xFF); sink_push(sink, 0x0A5, d, 8, ts);
    memset(d, 0, 8); d[4] = (uint8_t)((brake >> 8) & 0x3); d[5] = (uint8_t)(brake & 0xFF); sink_push(sink, 0x0B0, d, 8, ts);
    memset(d, 0, 8); d[1] = (uint8_t)gear; sink_push(sink, 0x200, d, 8, ts);
    memset(d, 0, 8); for (int w = 0; w < 4; w++) { d[w * 2] = (uint8_t)(ws & 0xFF); d[w * 2 + 1] = (uint8_t)(ws >> 8); } sink_push(sink, 0x1F0, d, 8, ts);
    if (c->no_brake) {
        memset(d, 0, 8); d[3] = (p == 3 && k > 500) ? 51 : 50; sink_push(sink, 0x500, d, 8, ts);
    }
    for (int x = 0; x < 20; x++) { memset(d, 0, 8); d[0] = (uint8_t)(rng_next() % 2); sink_push(sink, c->noise_base + (uint32_t)x, d, 8, ts); }
}

/*******************************************************************************
 * Physics model for the realistic scenarios
 ******************************************************************************/

typedef struct {
    double steer_deg;
    double rpm;
    double thr_pct;
    double brake_bar;
    int    gear;          /* 0=P 1=R 2=N 3=D */
    double wheel_kph[4];
    double veh_kph;
    double coolant_c;
    double battery_v;
} phys_t;

static double g_coolant = 80.0;
static double g_battery = 13.8;

static void physics(int p, double t, phys_t *ph) {
    memset(ph, 0, sizeof(*ph));
    ph->rpm = 750 + jitter(3);
    switch (p) {
    case 1:
        ph->steer_deg = 450 * sin(2 * M_PI * t / 4.13);
        ph->rpm += 25 * fabs(sin(2 * M_PI * t / 4.13));
        break;
    case 2: {
        double lag = t - 0.2;
        double thr_now = 100 * fmax(0.0, sin(2 * M_PI * t / 2.63));
        double thr_lag = 100 * fmax(0.0, sin(2 * M_PI * lag / 2.63));
        ph->thr_pct = thr_now;
        ph->rpm = 750 + 5200 * thr_lag / 100.0 + jitter(5);
        break;
    }
    case 3:
        /* irregular presses: period not a multiple of any frame rate, varying force */
        ph->brake_bar = 80 * fmax(0.0, sin(2 * M_PI * t / 2.3)) * (0.85 + 0.15 * sin(0.7 * t));
        break;
    case 4:
        ph->gear = (t < 2.5) ? 0 : (t < 5.0) ? 1 : (t < 7.5) ? 2 : 3;
        ph->brake_bar = (t < 0.5 ? t / 0.5 : 1.0) * (25 + 3 * sin(t));
        if (ph->gear == 1 || ph->gear == 3) ph->rpm -= 40;
        break;
    case 5:
        ph->gear = 3;
        ph->veh_kph = 30 + 10 * sin(2 * M_PI * t / 10.0);
        for (int w = 0; w < 4; w++) ph->wheel_kph[w] = ph->veh_kph * (1.0 + 0.01 * (w - 1.5)) + jitter(0.05);
        ph->rpm = 1200 + ph->veh_kph * 40 + jitter(10);
        ph->thr_pct = 15 + 10 * sin(2 * M_PI * t / 3.0);
        ph->steer_deg = 30 * sin(2 * M_PI * t / 5.0);
        ph->brake_bar = (t >= 10 && t < 12) ? 20 : 0;
        break;
    default:
        break;
    }
    ph->coolant_c = g_coolant;
    ph->battery_v = g_battery;
}

static uint8_t crc8(const uint8_t *d, int n) {
    uint8_t c = 0xFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x1D : (c << 1));
    }
    return c;
}

/* --- Coupled, BMW-like layout ------------------------------------------- */

/* When set, gen_coupled sends VW-style sign-magnitude steering on this ID
 * (every g_sm_steer_div ticks) instead of the signed 0x0A5 message. */
static uint32_t g_sm_steer_id = 0;
static int g_sm_steer_div = 1;
static bool g_bmw_gear = false;   /* gear on 0x3F9 48|4 with BMW codes instead of 0x3FD */

/* 0x0D9 pedal variants: the throttle field next to bits that also move at rest. */
typedef enum {
    PEDAL_PLAIN = 0,      /* 12-bit LE @16, 0 at rest */
    PEDAL_REST_JITTER,    /* same field, throttle-plate style: 2 % at rest, +0..3 LSB noise */
    PEDAL_BE_OVER_COUNTER /* 10-bit Motorola @23, +1 alive counter in the 4 bits below
                             its LSB (same byte), checksum in byte 0 */
} pedal_mode_t;
static pedal_mode_t g_pedal_mode = PEDAL_PLAIN;

static void gen_coupled(int p, int k, double t, sink_t *sink) {
    phys_t ph;
    physics(p, t, &ph);
    int64_t ts = 1000000 + (int64_t)p * 100000 + (int64_t)k * 10;
    uint8_t d[8];

    /* Slow environmental drift: coolant +1 LSB every 9 s, battery random walk. */
    if (k % 900 == 899) g_coolant += 0.75;
    if (k % 100 == 0) g_battery += jitter(0.02);

    if (g_sm_steer_id) {
        /* Sign-magnitude steering (Porsche/Audi 0x0A0 style): crc b0, counter
         * nibble b1, magnitude 12 bits @32 (0.12 deg), sign bit 44, status b3. */
        if (k % g_sm_steer_div == 0) {
            memset(d, 0, 8);
            d[1] = (uint8_t)((k / g_sm_steer_div) & 0xF);
            d[3] = 0x28;
            put_field(d, 32, 12, 1, (uint32_t)lround(fabs(ph.steer_deg) / 0.12));
            if (ph.steer_deg < 0) d[5] |= 0x10;
            d[0] = crc8(d + 1, 7);
            sink_push(sink, g_sm_steer_id, d, 8, ts);
        }
    } else {
        /* 0x0A5 steering, 100 Hz: counter nibble b0, crc b1, signed LE16 @16, factor 0.1 */
        memset(d, 0, 8);
        d[0] = (uint8_t)(k & 0xF);
        put_field(d, 16, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(ph.steer_deg * 10));
        d[1] = crc8(d + 2, 6);
        sink_push(sink, 0x0A5, d, 8, ts);
    }

    /* 0x0F3 engine, 50 Hz: counter b0, torque b2 (follows throttle), RPM BE16 @ byte4 (start 39), factor 0.25 */
    if (k % 2 == 0) {
        memset(d, 0, 8);
        d[0] = (uint8_t)((k / 2) & 0xF);
        d[2] = (uint8_t)lround(fmin(255, ph.thr_pct * 2.0 + 20 + jitter(1)));
        put_field(d, 39, 16, 0, (uint32_t)lround(ph.rpm * 4));
        d[7] = crc8(d, 7);
        sink_push(sink, 0x0F3, d, 8, ts);
    }

    /* 0x0D9 pedal, 20 Hz: 12-bit LE @16 factor 0.025 %, kickdown bit @31 */
    if (k % 5 == 0 && g_pedal_mode == PEDAL_BE_OVER_COUNTER) {
        memset(d, 0, 8);
        put_field(d, 23, 10, 0, (uint32_t)lround(ph.thr_pct / 0.1));
        put_field(d, 29, 4, 0, (uint32_t)((k / 5) & 0xF));
        d[0] = crc8(d + 1, 5);
        sink_push(sink, 0x0D9, d, 6, ts);
    } else if (k % 5 == 0) {
        memset(d, 0, 8);
        uint32_t raw = (g_pedal_mode == PEDAL_REST_JITTER)
                           ? (uint32_t)lround((ph.thr_pct + 2.0) / 0.025) + (rng_next() & 3)
                           : (uint32_t)lround(ph.thr_pct / 0.025);
        put_field(d, 16, 12, 1, raw);
        if (ph.thr_pct > 95) d[3] |= 0x80;
        sink_push(sink, 0x0D9, d, 6, ts);
    }

    /* 0x173 brake, 50 Hz: 12-bit LE @0 pressure (0.1 bar), light switch bit @16 */
    if (k % 2 == 0) {
        memset(d, 0, 8);
        put_field(d, 0, 12, 1, (uint32_t)lround(ph.brake_bar * 10));
        if (ph.brake_bar > 2) d[2] |= 0x01;
        sink_push(sink, 0x173, d, 8, ts);
    }

    /* BMW F-series 0x3F9, 5 Hz: gear nibble at bit 48 with codes P=3 R=2 N=1 1st=5,
     * coolant/oil temperatures in bytes 4-5 (VSP prior with a value table). */
    if (g_bmw_gear && k % 20 == 0) {
        static const uint8_t bmw_codes[4] = { 3, 2, 1, 5 };
        memset(d, 0, 8);
        d[4] = (uint8_t)(g_coolant + 48);
        d[5] = 0x67;
        put_field(d, 48, 4, 1, bmw_codes[ph.gear]);
        d[7] = 0x93;
        sink_push(sink, 0x3F9, d, 8, ts);
    }

    /* 0x3FD gear, 10 Hz: counter b0 nibble, gear 4-bit @8 with codes P=1 R=2 N=3 D=4 */
    if (!g_bmw_gear && k % 10 == 0) {
        static const uint8_t codes[4] = { 1, 2, 3, 4 };
        memset(d, 0, 8);
        d[0] = (uint8_t)((k / 10) & 0xF);
        put_field(d, 8, 4, 1, codes[ph.gear]);
        sink_push(sink, 0x3FD, d, 4, ts);
    }

    /* 0x254 wheel speeds, 50 Hz: 4 x LE16 factor 0.01 km/h */
    if (k % 2 == 0) {
        memset(d, 0, 8);
        for (int w = 0; w < 4; w++) put_field(d, w * 16, 16, 1, (uint32_t)lround(ph.wheel_kph[w] * 100));
        sink_push(sink, 0x254, d, 8, ts);
    }

    /* 0x3B4 battery voltage, 1 Hz: 8-bit 0.1 V offset 0 */
    if (k % 100 == 0) {
        memset(d, 0, 8);
        d[0] = (uint8_t)lround(g_battery * 10);
        sink_push(sink, 0x3B4, d, 8, ts);
    }

    /* 0x2C4 coolant temp, 2 Hz: byte 3, C + 48 */
    if (k % 50 == 0) {
        memset(d, 0, 8);
        d[3] = (uint8_t)lround(g_coolant + 48);
        sink_push(sink, 0x2C4, d, 8, ts);
    }

    /* 0x130 terminal status, 10 Hz, constant; 0x380 counter + crc only */
    if (k % 10 == 0) {
        memset(d, 0, 8); d[0] = 0x45; d[1] = 0x40; sink_push(sink, 0x130, d, 5, ts);
        memset(d, 0, 8); d[0] = (uint8_t)(k / 10); d[1] = crc8(d, 1); sink_push(sink, 0x380, d, 8, ts);
    }
}

/* 0x0A0 at 10 Hz matches the VSP corpus prior; 0x0B5 has none (data only). */
static void gen_sm_prior(int p, int k, double t, sink_t *sink) {
    g_sm_steer_id = 0x0A0;
    g_sm_steer_div = 10;
    gen_coupled(p, k, t, sink);
    g_sm_steer_id = 0;
}

/* BMW 0x3F9 gear: the matched prior's value table must reach the draft VSP/DBC. */
static void gen_bmw_gear(int p, int k, double t, sink_t *sink) {
    g_bmw_gear = true;
    gen_coupled(p, k, t, sink);
    g_bmw_gear = false;
}

static void gen_tps_jitter(int p, int k, double t, sink_t *sink) {
    g_pedal_mode = PEDAL_REST_JITTER;
    gen_coupled(p, k, t, sink);
    g_pedal_mode = PEDAL_PLAIN;
}

static void gen_be_pedal_counter(int p, int k, double t, sink_t *sink) {
    g_pedal_mode = PEDAL_BE_OVER_COUNTER;
    gen_coupled(p, k, t, sink);
    g_pedal_mode = PEDAL_PLAIN;
}

static void gen_sm_no_prior(int p, int k, double t, sink_t *sink) {
    g_sm_steer_id = 0x0B5;
    g_sm_steer_div = 2;
    gen_coupled(p, k, t, sink);
    g_sm_steer_id = 0;
}

/* --- Porsche 992 GT3 layout, as recorded standing still --------------------
 * Patterns from real PT-CAN captures (apps/can_tools/captures/20261028_*):
 *   0x121 pedal: crc b0, alive counter in bits 8-11 that advances ~4 per
 *         received frame (bits 10-11 cycle every frame), pedal 8 bits @12
 *         (0.4 %). Stabs to ~28 %: the pedal jumps between 25 Hz frames,
 *         while the counter ticks make a field glued onto it look smooth.
 *   0x0A8 engine: RPM 16 bits @48 (0.25 rpm, only the 0.5 rpm bit moves).
 *         Small blips over a jittery idle, with an undershoot after each one,
 *         put idle in the lower third of the throttle phase's range.
 *   0x0A0 sign-magnitude steering, 0x106 brake pressure 10 bits @16. */

static double gt3_blip(double t, double at) {
    double x = (t - at) / 0.6;
    return (x >= 0 && x < 1) ? sin(M_PI * x) : 0.0;
}

/** Pedal stab: in over 0.12 s, a slight sag, out over 0.1 s. */
static double gt3_stab(double t, double at) {
    double x = t - at;
    if (x < 0 || x >= 0.65) return 0.0;
    if (x < 0.12) return x / 0.12;
    if (x < 0.55) return 1.0 - 0.15 * (x - 0.12) / 0.43;
    return 0.85 * (0.65 - x) / 0.10;
}

static void gen_gt3_standing(int p, int k, double t, sink_t *sink) {
    int64_t ts = 3000000 + (int64_t)p * 100000 + (int64_t)k * 10;
    uint8_t d[8];

    double steer = (p == 1) ? 450 * sin(2 * M_PI * t / 4.13) : -4.0;
    double thr = 0.0, rpm = 775 + jitter(25), brake = 0.0;
    if (p == 2) {
        thr = 28.0 * (gt3_stab(t, 1.0) + gt3_stab(t, 4.5));
        double lag = 28.0 * (gt3_blip(t - 0.15, 1.0) + gt3_blip(t - 0.15, 4.5));
        double dip = gt3_blip(t, 1.75) + gt3_blip(t, 5.25);
        rpm += 13.0 * lag - 160.0 * dip;
    }
    if (p == 3) brake = 40 * fmax(0.0, sin(2 * M_PI * t / 2.3));

    if (k % 10 == 0) {          /* 0x0A0 steering, 10 Hz */
        memset(d, 0, 8);
        d[1] = (uint8_t)((k / 10) & 0xF);
        d[3] = 0x28;
        put_field(d, 32, 12, 1, (uint32_t)lround(fabs(steer) / 0.12));
        if (steer < 0) d[5] |= 0x10;
        d[0] = crc8(d + 1, 7);
        sink_push(sink, 0x0A0, d, 8, ts);
    }
    if (k % 10 == 5) {          /* 0x0A8 engine, 10 Hz */
        memset(d, 0, 8);
        d[1] = (uint8_t)((k / 10) & 0xF);
        d[2] = 0x81;
        put_field(d, 48, 16, 1, (uint32_t)lround(rpm * 2) * 2);
        d[0] = crc8(d + 1, 7);
        sink_push(sink, 0x0A8, d, 8, ts);
    }
    if (k % 4 == 0) {           /* 0x121 pedal, 25 Hz */
        memset(d, 0, 8);
        d[1] = (uint8_t)((((k / 4) & 3) << 2) | (1 + (rng_next() & 1)));
        put_field(d, 12, 8, 1, (uint32_t)lround(thr / 0.4));
        d[4] = 0x20; d[5] = 0x4E; d[6] = 0x7E; d[7] = 0xFE;
        d[0] = crc8(d + 1, 7);
        sink_push(sink, 0x121, d, 8, ts);
    }
    if (k % 2 == 0) {           /* 0x106 brake, 50 Hz */
        memset(d, 0, 8);
        d[1] = (uint8_t)((k / 2) & 0xF);
        put_field(d, 16, 10, 1, (uint32_t)lround((brake + 30.6) / 0.3));
        d[5] = 0x66; d[7] = 0xFE;
        d[0] = crc8(d + 1, 7);
        sink_push(sink, 0x106, d, 8, ts);
    }
    if (k % 2 == 1) {           /* 0x0FD counter + crc only, 50 Hz */
        memset(d, 0, 8);
        d[1] = (uint8_t)((k / 2) & 0xF);
        d[2] = 0x3F; d[3] = 0x80;
        d[0] = crc8(d + 1, 7);
        sink_push(sink, 0x0FD, d, 8, ts);
    }
}

/* --- RPM and throttle in one message ------------------------------------- */

static void gen_shared_msg(int p, int k, double t, sink_t *sink) {
    phys_t ph;
    physics(p, t, &ph);
    int64_t ts = (int64_t)k * 10 + 5;
    uint8_t d[8];

    memset(d, 0, 8);
    put_field(d, 7, 16, 0, (uint32_t)lround(ph.rpm * 4));          /* RPM BE16 bytes 0-1 */
    d[2] = (uint8_t)(k & 0x0F);                                    /* alive counter */
    d[4] = (uint8_t)lround(ph.thr_pct * 2.54);                     /* pedal 8-bit */
    sink_push(sink, 0x1A0, d, 8, ts);

    memset(d, 0, 8);
    put_field(d, 0, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(ph.steer_deg * 23.0)); /* LE16 signed */
    sink_push(sink, 0x0C4, d, 7, ts);

    if (k % 2 == 0) {
        memset(d, 0, 8);
        put_field(d, 7, 16, 0, (uint32_t)lround(ph.brake_bar * 100)); /* BE16 */
        sink_push(sink, 0x0A8, d, 8, ts);
    }
    if (k % 5 == 0) {
        memset(d, 0, 8);
        d[0] = (uint8_t)(0x50 + ph.gear);                          /* enum byte */
        sink_push(sink, 0x1D2, d, 6, ts);
    }
    if (k % 2 == 0) {
        memset(d, 0, 8);
        for (int w = 0; w < 4; w++) put_field(d, w * 16, 16, 1, (uint32_t)lround(ph.wheel_kph[w] * 64));
        sink_push(sink, 0x0CE, d, 8, ts);
    }
}

/* --- Low-rate gateway (D-CAN via gateway, ~5 Hz) ------------------------- */

static void gen_gateway(int p, int k, double t, sink_t *sink) {
    if (k % 20 != 0) return;                      /* 5 Hz everything */
    phys_t ph;
    physics(p, t, &ph);
    int64_t ts = (int64_t)k * 10 + 3;
    uint8_t d[8];

    memset(d, 0, 8);
    put_field(d, 0, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(ph.steer_deg * 10));
    sink_push(sink, 0x0C8, d, 8, ts);
    memset(d, 0, 8);
    put_field(d, 16, 16, 1, (uint32_t)lround(ph.rpm * 4));
    sink_push(sink, 0x0AA, d, 8, ts);
    memset(d, 0, 8);
    d[6] = (uint8_t)lround(ph.thr_pct * 2.54);
    sink_push(sink, 0x0AB, d, 8, ts);
    memset(d, 0, 8);
    put_field(d, 7, 12, 0, (uint32_t)lround(ph.brake_bar * 10));
    sink_push(sink, 0x0EF, d, 8, ts);
    memset(d, 0, 8);
    d[5] = (uint8_t)(ph.gear + 1);
    sink_push(sink, 0x0BA, d, 8, ts);
    memset(d, 0, 8);
    for (int w = 0; w < 4; w++) put_field(d, w * 16, 16, 1, (uint32_t)lround(ph.wheel_kph[w] * 50));
    sink_push(sink, 0x0CE, d, 8, ts);
    for (int x = 0; x < 10; x++) {
        memset(d, 0, 8);
        d[0] = (uint8_t)x;
        sink_push(sink, 0x500 + (uint32_t)x, d, 8, ts);
    }
}

/* --- Nobody touched anything: every phase looks like baseline ----------- */

static void gen_quiet(int p, int k, double t, sink_t *sink) {
    (void)p;
    gen_coupled(0, k, t, sink);
}

/* --- Busy bus: coupled layout plus many distracting fields ---------------- */

static void gen_busy(int p, int k, double t, sink_t *sink) {
    gen_coupled(p, k, t, sink);

    phys_t ph;
    physics(p, t, &ph);
    int64_t ts = 2000000 + (int64_t)p * 100000 + (int64_t)k * 10;
    uint8_t d[8];
    bool driving = (p == DISC_PHASE_WHEEL_SPEED);
    static uint32_t odo = 123456;
    static uint8_t drift[20];
    if (k == 0 && p == 0) {
        odo = 123456;
        for (int i = 0; i < 20; i++) drift[i] = (uint8_t)(40 + i * 7);
    }

    if (k % 2 == 0) {
        /* 0x1B0 lateral accel / 0x1B1 yaw rate: only move while driving */
        double lat = driving ? 2.0 * sin(2 * M_PI * t / 5.0) : jitter(0.01);
        double yaw = driving ? 10.0 * sin(2 * M_PI * t / 5.0 + 0.3) : jitter(0.05);
        memset(d, 0, 8);
        put_field(d, 0, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(lat * 100));
        put_field(d, 16, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(yaw * 100));
        d[6] = (uint8_t)(k & 0x0F);
        sink_push(sink, 0x1B0, d, 8, ts);

        /* 0x1C0 manifold pressure (kPa) and 0x1C1 torque request track throttle */
        memset(d, 0, 8);
        d[3] = (uint8_t)lround(30 + ph.thr_pct * 0.7 + jitter(0.6));
        sink_push(sink, 0x1C0, d, 8, ts);
        memset(d, 0, 8);
        put_field(d, 4, 12, 1, (uint32_t)lround(ph.thr_pct * 30 + 200 + jitter(4)));
        sink_push(sink, 0x1C1, d, 8, ts);
    }

    /* 0x1B2 odometer (1 m), ticks up while driving */
    if (driving && k % 10 == 0) odo += 1;
    if (k % 10 == 0) {
        memset(d, 0, 8);
        put_field(d, 0, 24, 1, odo);
        sink_push(sink, 0x1B2, d, 8, ts);
    }

    /* 0x600.. slow drift sensors, 0x1D0 noisy light sensor in every phase */
    if (k % 50 == 0) {
        for (int i = 0; i < 20; i++) {
            if (k % 200 == 0 && (rng_next() & 3) == 0) drift[i] = (uint8_t)(drift[i] + ((rng_next() & 1) ? 1 : -1));
            memset(d, 0, 8);
            d[i % 8] = drift[i];
            sink_push(sink, 0x600 + (uint32_t)i, d, 8, ts);
        }
    }
    if (k % 5 == 0) {
        memset(d, 0, 8);
        put_field(d, 8, 10, 1, (uint32_t)(500 + (int)jitter(60)));
        sink_push(sink, 0x1D0, d, 8, ts);
    }
}

/* --- Adapter batching: frames arrive in 20 ms groups sharing a timestamp,
 *     two wheel speeds in half a frame next to a counter, and IDs where the
 *     corpus has priors with implausible scales (0x0EF brake). ----------- */

static void gen_batched(int p, int k, double t, sink_t *sink) {
    phys_t ph;
    physics(p, t, &ph);
    int64_t ts = 5000000 + (int64_t)p * 100000 + ((int64_t)k * 10 / 20) * 20;
    uint8_t d[8];
    uint8_t counter = (uint8_t)(k & 0x0F);

    memset(d, 0, 8);
    put_field(d, 0, 16, 1, (uint32_t)(uint16_t)(int16_t)lround(ph.steer_deg * 11));
    d[7] = counter;
    sink_push(sink, 0x0C4, d, 8, ts);

    memset(d, 0, 8);
    d[0] = 0x10;
    put_field(d, 16, 16, 1, (uint32_t)lround(ph.rpm * 4));
    d[7] = counter;
    sink_push(sink, 0x0A5, d, 8, ts);

    memset(d, 0, 8);
    d[1] = (uint8_t)lround(ph.thr_pct * 2.5);
    d[7] = counter;
    sink_push(sink, 0x0D9, d, 8, ts);

    if (k % 2 == 0) {
        memset(d, 0, 8);
        d[2] = (uint8_t)lround(ph.brake_bar * 2.5);
        d[7] = counter;
        sink_push(sink, 0x0EF, d, 8, ts);
    } else {
        memset(d, 0, 8);
        put_field(d, 0, 16, 1, (uint32_t)lround(ph.wheel_kph[0] * 100));
        put_field(d, 16, 16, 1, (uint32_t)lround(ph.wheel_kph[1] * 100));
        d[7] = counter;
        sink_push(sink, 0x1A0, d, 8, ts);
    }
}

/* --- Engine review edge cases (one test message 0x6FA) ---------------------
 * RPM elsewhere on 0x6FB (LE16 @48) unless the case is about RPM itself. */

typedef enum {
    EDGE_BE8_ACROSS = 0,   /* 8-bit Motorola 19|8 across bytes 2-3: counter below, flag above */
    EDGE_NOISY_BYTE,       /* one-byte sensor with 16 values at rest (the old CRC cut) */
    EDGE_TWINS,            /* the same 12-bit value at 0 and 24: RPM / throttle undecidable */
    EDGE_RPM_FLAT_HIGH,    /* LE16 RPM whose high byte never moves; idle has 16 low-byte values */
} edge_mode_t;
static edge_mode_t g_edge;

static void gen_edge(int p, int k, double t, sink_t *sink) {
    int64_t ts = (int64_t)p * 100000 + (int64_t)k * 10 + 1;
    uint8_t d[8] = {0};
    int thr = (p == 2) ? (int)lround(125 + 120 * sin(t * 3)) : 0;
    switch (g_edge) {
        case EDGE_BE8_ACROSS:
            put_field(d, 19, 8, 0, (uint32_t)thr);
            d[3] |= (uint8_t)(k & 15);            /* alive counter right below the LSB */
            d[2] |= (uint8_t)((k % 2) << 4);      /* toggling flag right above the MSB */
            break;
        case EDGE_NOISY_BYTE:
            if (p != 2) thr = (int)(rng_next() % 16);
            d[2] = (uint8_t)thr;
            break;
        case EDGE_TWINS: {
            int v = (p == 2) ? (int)(1000 + 900 * sin(t * 3)) : 0;
            put_field(d, 0, 12, 1, (uint32_t)v);
            put_field(d, 24, 12, 1, (uint32_t)v);
            break;
        }
        case EDGE_RPM_FLAT_HIGH: {
            int rpm = (p == 2) ? (int)(1142 + 118 * sin(t * 3)) : 1024 + (int)(rng_next() % 16);
            put_field(d, 16, 16, 1, (uint32_t)rpm);
            break;
        }
    }
    sink_push(sink, 0x6FA, d, 8, ts);
    if (g_edge == EDGE_BE8_ACROSS || g_edge == EDGE_NOISY_BYTE) {
        memset(d, 0, 8);
        int rpm = (p == 2) ? (int)(8000 + 4000 * sin(t * 3)) : 3200 + (int)(rng_next() % 4);
        put_field(d, 48, 16, 1, (uint32_t)rpm);
        sink_push(sink, 0x6FB, d, 8, ts);
    }
}

static void gen_edge_be8(int p, int k, double t, sink_t *sink) { g_edge = EDGE_BE8_ACROSS; gen_edge(p, k, t, sink); }
static void gen_edge_noisy(int p, int k, double t, sink_t *sink) { g_edge = EDGE_NOISY_BYTE; gen_edge(p, k, t, sink); }
static void gen_edge_twins(int p, int k, double t, sink_t *sink) { g_edge = EDGE_TWINS; gen_edge(p, k, t, sink); }
static void gen_edge_rpm_flat(int p, int k, double t, sink_t *sink) { g_edge = EDGE_RPM_FLAT_HIGH; gen_edge(p, k, t, sink); }

/*******************************************************************************
 * Scenario table
 ******************************************************************************/

#define DUR_DEFAULT { 5, 8, 8, 8, 10, 15 }
#define REVIEW_DUR  { 10, 10, 10, 10, 10, 10 }

#define EXPECT_NONE { false, 0, {{0}}, false }
#define EXPECT1(...) { true, 1, { __VA_ARGS__ }, false }
#define EXPECT2(...) { true, 2, { __VA_ARGS__ }, false }
#define EXPECT4(...) { true, 4, { __VA_ARGS__ }, false }
#define EXPECT2_NOT_HIGH(...) { true, 2, { __VA_ARGS__ }, true }
#define EDGE_DUR    { 5, 8, 8, 8, 8, 8 }
#define GT3_EXPECT  { EXPECT1(FIELD(0x0A0, 32, 13, 1)), EXPECT1(FIELD(0x0A8, 48, 16, 1)), \
                      EXPECT1(FIELD(0x121, 12, 8, 1)), EXPECT1(FIELD(0x106, 16, 10, 1)), \
                      EXPECT_NONE, EXPECT_NONE }
#define MOTOROLA_PEDAL_EXPECT { EXPECT1(FIELD(0x0A5, 16, 16, 1)), EXPECT1(FIELD(0x0F3, 39, 16, 0)), \
                      EXPECT1(FIELD(0x0D9, 23, 10, 0)), EXPECT1(FIELD(0x173, 0, 12, 1)), \
                      EXPECT1(FIELD(0x3FD, 8, 4, 1)), \
                      EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1), \
                              FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) }

static const review_cfg_t k_review_cfg[] = {
    { 4,  0x316, false, 0x300, false },  /* sim  */
    { 1,  0x316, false, 0x300, false },  /* sim2 */
    { 4,  0x300, true,  0x400, false },  /* sim3 */
    { 16, 0x0E8, true,  0x400, false },  /* sim4 */
    { 4,  0x316, false, 0x300, true  },  /* sim5 */
};

#define R_STEER EXPECT1(FIELD(0x0A5, 15, 16, 0))
#define R_THR   EXPECT1(FIELD(0x1A0, 40, 8, 1))
#define R_BRAKE EXPECT1(FIELD(0x0B0, 33, 10, 0))
#define R_GEAR  EXPECT1(FIELD(0x200, 8, 8, 1))
#define R_WHEEL EXPECT4(FIELD(0x1F0, 0, 16, 1), FIELD(0x1F0, 16, 16, 1), \
                        FIELD(0x1F0, 32, 16, 1), FIELD(0x1F0, 48, 16, 1))

static scenario_t g_scenarios[] = {
    { "review_sim",  gen_review, 100, REVIEW_DUR,
      { R_STEER, EXPECT1(FIELD(0x316, 16, 16, 1)), R_THR, R_BRAKE, R_GEAR, R_WHEEL } },
    { "review_sim2", gen_review, 100, REVIEW_DUR,
      { R_STEER, EXPECT1(FIELD(0x316, 16, 16, 1)), R_THR, R_BRAKE, R_GEAR, R_WHEEL } },
    { "review_sim3", gen_review, 100, REVIEW_DUR,
      { R_STEER, EXPECT1(FIELD(0x300, 7, 16, 0)), R_THR, R_BRAKE, R_GEAR, R_WHEEL } },
    { "review_sim4", gen_review, 100, REVIEW_DUR,
      { R_STEER, EXPECT1(FIELD(0x0E8, 7, 16, 0)), R_THR, R_BRAKE, R_GEAR, R_WHEEL } },
    { "review_sim5_no_brake", gen_review, 100, REVIEW_DUR,
      { R_STEER, EXPECT1(FIELD(0x316, 16, 16, 1)), R_THR, EXPECT_NONE, R_GEAR, R_WHEEL } },
    { "coupled", gen_coupled, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    { "shared_msg", gen_shared_msg, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0C4, 0, 16, 1)),
        EXPECT1(FIELD(0x1A0, 7, 16, 0)),
        EXPECT1(FIELD(0x1A0, 32, 8, 1)),
        EXPECT1(FIELD(0x0A8, 7, 16, 0)),
        EXPECT1(FIELD(0x1D2, 0, 8, 1)),
        EXPECT4(FIELD(0x0CE, 0, 16, 1), FIELD(0x0CE, 16, 16, 1),
                FIELD(0x0CE, 32, 16, 1), FIELD(0x0CE, 48, 16, 1)) } },
    { "gateway_5hz", gen_gateway, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0C8, 0, 16, 1)),
        EXPECT1(FIELD(0x0AA, 16, 16, 1)),
        EXPECT1(FIELD(0x0AB, 48, 8, 1)),
        EXPECT1(FIELD(0x0EF, 7, 12, 0)),
        EXPECT1(FIELD(0x0BA, 40, 8, 1)),
        EXPECT4(FIELD(0x0CE, 0, 16, 1), FIELD(0x0CE, 16, 16, 1),
                FIELD(0x0CE, 32, 16, 1), FIELD(0x0CE, 48, 16, 1)) } },
    { "quiet_no_actions", gen_quiet, 100, DUR_DEFAULT, {
        EXPECT_NONE, EXPECT_NONE, EXPECT_NONE, EXPECT_NONE, EXPECT_NONE, EXPECT_NONE } },
    { "busy_bus", gen_busy, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    { "batched_two_wheels", gen_batched, 100, { 5, 8, 8, 8, 0, 15 }, {
        EXPECT1(FIELD(0x0C4, 0, 16, 1)),
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0D9, 8, 8, 1)),
        EXPECT1(FIELD(0x0EF, 16, 8, 1)),
        EXPECT_NONE,
        EXPECT2(FIELD(0x1A0, 0, 16, 1), FIELD(0x1A0, 16, 16, 1)) } },
    { "coupled_skip_gear_wheel", gen_coupled, 100, { 5, 8, 8, 8, 0, 0 }, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT_NONE,
        EXPECT_NONE } },
    /* Steering truth is the 13-bit raw (magnitude + sign as MSB). */
    { "sign_magnitude_steering_prior", gen_sm_prior, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A0, 32, 13, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    { "bmw_gear_value_table", gen_bmw_gear, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3F9, 48, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    { "sign_magnitude_steering_no_prior", gen_sm_no_prior, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0B5, 32, 13, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    /* Rest noise in the pedal's own LSBs is signal, not a counter. */
    { "coupled_tps_jitter_at_rest", gen_tps_jitter, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 16, 12, 1)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    /* Not the VW layout: Motorola pedal with a +1 counter below it in one byte. */
    { "coupled_motorola_pedal_over_counter", gen_be_pedal_counter, 100, DUR_DEFAULT, {
        EXPECT1(FIELD(0x0A5, 16, 16, 1)),
        EXPECT1(FIELD(0x0F3, 39, 16, 0)),
        EXPECT1(FIELD(0x0D9, 23, 10, 0)),
        EXPECT1(FIELD(0x173, 0, 12, 1)),
        EXPECT1(FIELD(0x3FD, 8, 4, 1)),
        EXPECT4(FIELD(0x254, 0, 16, 1), FIELD(0x254, 16, 16, 1),
                FIELD(0x254, 32, 16, 1), FIELD(0x254, 48, 16, 1)) } },
    /* Pedal next to an alive counter; RPM prior with idle mid-range. */
    { "gt3_counter_pedal_small_blips", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    /* Engine review: frames lost at random (the counter LSB then misses flips). */
    { "gt3_counter_pedal_small_blips_loss3_s3", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    { "gt3_counter_pedal_small_blips_loss5_s1", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    { "gt3_counter_pedal_small_blips_loss5_s2", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    { "gt3_counter_pedal_small_blips_loss5_s3", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    { "gt3_counter_pedal_small_blips_loss10_s1", gen_gt3_standing, 100, { 5, 8, 8, 8, 0, 0 }, GT3_EXPECT },
    { "coupled_motorola_pedal_over_counter_loss10_s1", gen_be_pedal_counter, 100, DUR_DEFAULT,
      MOTOROLA_PEDAL_EXPECT },
    /* Engine review: 8-bit Motorola field across a byte boundary. */
    { "motorola_8bit_across_bytes", gen_edge_be8, 100, EDGE_DUR, {
        EXPECT_NONE, EXPECT1(FIELD(0x6FB, 48, 16, 1)), EXPECT1(FIELD(0x6FA, 19, 8, 0)),
        EXPECT_NONE, EXPECT_NONE, EXPECT_NONE } },
    /* Engine review: 16 byte values at rest is sensor noise, not a CRC. */
    { "noisy_byte_sensor_16_values", gen_edge_noisy, 100, EDGE_DUR, {
        EXPECT_NONE, EXPECT1(FIELD(0x6FB, 48, 16, 1)), EXPECT1(FIELD(0x6FA, 16, 8, 1)),
        EXPECT_NONE, EXPECT_NONE, EXPECT_NONE } },
    { "rpm_idle_16_values_flat_high_byte", gen_edge_rpm_flat, 100, EDGE_DUR, {
        EXPECT_NONE, EXPECT1(FIELD(0x6FA, 16, 16, 1)), EXPECT_NONE,
        EXPECT_NONE, EXPECT_NONE, EXPECT_NONE } },
    /* Engine review: RPM and throttle swap at equal total: neither may be HIGH. */
    { "twin_fields_rpm_throttle_undecidable", gen_edge_twins, 100, EDGE_DUR, {
        EXPECT_NONE,
        EXPECT2_NOT_HIGH(FIELD(0x6FA, 0, 12, 1), FIELD(0x6FA, 24, 12, 1)),
        EXPECT2_NOT_HIGH(FIELD(0x6FA, 0, 12, 1), FIELD(0x6FA, 24, 12, 1)),
        EXPECT_NONE, EXPECT_NONE, EXPECT_NONE } },
};

#define SCENARIO_COUNT ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

/*******************************************************************************
 * Capture synthesis
 ******************************************************************************/

typedef struct {
    disc_phase_t phases[DISC_NUM_PHASES];
    disc_raw_phase_t raw[DISC_NUM_PHASES];
} capture_t;

static void capture_build(const scenario_t *sc, int index, capture_t *cap) {
    g_rng = 1;
    g_coolant = 80.0;
    g_battery = 13.8;
    g_sm_steer_id = 0;
    if (sc->gen == gen_review) g_review = k_review_cfg[index];

    sink_t sink = {0};
    int loss_pct = 0;
    uint32_t loss = 1;   /* xorshift32 state */
    const char *suffix = strstr(sc->name, "_loss");
    if (suffix && sscanf(suffix, "_loss%d_s%u", &loss_pct, &loss) != 2) loss_pct = 0;
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        disc_phase_init(&cap->phases[p]);
        disc_raw_phase_init(&cap->raw[p]);
        if (sc->durations[p] <= 0) continue;   /* skipped phase */

        sink.count = 0;
        int ticks = (int)lround(sc->durations[p] * sc->tick_hz);
        for (int k = 0; k < ticks; k++) {
            sc->gen(p, k, (double)k / sc->tick_hz, &sink);
        }
        if (loss_pct > 0) {
            /* adapter loss: drop frames at random, the generator's values untouched */
            int kept = 0;
            for (int i = 0; i < sink.count; i++) {
                loss ^= loss << 13;
                loss ^= loss >> 17;
                loss ^= loss << 5;
                if (loss % 100 >= (uint32_t)loss_pct) sink.frames[kept++] = sink.frames[i];
            }
            sink.count = kept;
        }
        for (int i = 0; i < sink.count; i++) {
            disc_phase_add_frame(&cap->phases[p], sink.frames[i].can_id,
                                 sink.frames[i].data, sink.frames[i].dlc);
        }
        disc_phase_finalize(&cap->phases[p], sc->durations[p]);
        disc_raw_phase_set(&cap->raw[p], sink.frames, sink.count);
    }
    free(sink.frames);
}

static void capture_free(capture_t *cap) {
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        disc_phase_free(&cap->phases[p]);
        disc_raw_phase_free(&cap->raw[p]);
    }
}

/*******************************************************************************
 * Output parsing (DBC SG_ lines, VSP decode blocks)
 ******************************************************************************/

typedef struct {
    uint32_t id;
    int start, len, order;
    char sign;          /* '+', '-' or 'S' (sign-magnitude) */
    int sign_bit;       /* valid when sign == 'S' */
    char name[64];
} out_field_t;

#define MAX_OUT 64

static int parse_dbc(const char *dbc, out_field_t *out, int max, int *bad_dlc) {
    int n = 0;
    uint32_t cur = 0;
    *bad_dlc = 0;
    for (const char *line = dbc; line && *line; ) {
        const char *eol = strchr(line, '\n');
        unsigned id, dlc;
        int s, l, o;
        char sign;
        char name[64];
        if (sscanf(line, "BO_ %u %*s %u", &id, &dlc) == 2) {
            cur = id;
            if (dlc > 8) (*bad_dlc)++;
        } else if (sscanf(line, " SG_ %63s : %d|%d@%d%c", name, &s, &l, &o, &sign) == 5) {
            /* <Name>_Sign (1 bit) is the sign of sign-magnitude <Name>. */
            size_t nl = strlen(name);
            if (l == 1 && nl > 5 && strcmp(name + nl - 5, "_Sign") == 0 && n > 0 &&
                out[n - 1].id == cur && strncmp(out[n - 1].name, name, nl - 5) == 0 &&
                out[n - 1].name[nl - 5] == '\0') {
                out[n - 1].sign = 'S';
                out[n - 1].sign_bit = s;
            } else if (n < max) {
                out[n].id = cur; out[n].start = s; out[n].len = l; out[n].order = o;
                out[n].sign = sign; out[n].sign_bit = -1;
                snprintf(out[n].name, sizeof(out[n].name), "%s", name);
                n++;
            }
        }
        line = eol ? eol + 1 : NULL;
    }
    return n;
}

/** Parse the top-level "signals" array of a draft VSP (acquisition + decode). */
static int parse_vsp(const char *vsp, out_field_t *out, int max, int *bad_dlc) {
    int n = 0;
    *bad_dlc = 0;
    const char *frames = strstr(vsp, "\"can_frames\"");
    const char *sigs = strstr(vsp, "\n  \"signals\"");
    if (!sigs) return 0;
    for (const char *p = frames; p && p < sigs; ) {
        p = strstr(p, "\"dlc\": ");
        if (!p || p > sigs) break;
        if (atoi(p + 7) > 8) (*bad_dlc)++;
        p += 7;
    }
    const char *p = sigs;
    while (n < max) {
        const char *fid = strstr(p, "\"frame_id\": ");
        if (!fid) break;
        const char *sb = strstr(fid, "\"start_bit\": ");
        const char *bl = sb ? strstr(sb, "\"bit_length\": ") : NULL;
        const char *bo = bl ? strstr(bl, "\"byte_order\": \"") : NULL;
        const char *sg = bo ? strstr(bo, "\"signed\": ") : NULL;
        if (!sg) break;
        out[n].id = (uint32_t)strtoul(fid + 12, NULL, 10);
        out[n].start = atoi(sb + 13);
        out[n].len = atoi(bl + 14);
        out[n].order = (strncmp(bo + 15, "little", 6) == 0) ? 1 : 0;
        out[n].sign = (strncmp(sg + 10, "true", 4) == 0) ? '-' : '+';
        out[n].sign_bit = -1;
        out[n].name[0] = '\0';
        const char *fac = strstr(sg, "\"factor\": ");
        const char *smb = strstr(sg, "\"sign_bit\": ");
        if (smb && (!fac || smb < fac)) {
            out[n].sign = 'S';
            out[n].sign_bit = atoi(smb + 12);
        }
        n++;
        p = sg;
    }
    return n;
}

static bool field_in(const out_field_t *f, const out_field_t *set, int n) {
    for (int i = 0; i < n; i++) {
        if (set[i].id == f->id && set[i].start == f->start &&
            set[i].len == f->len && set[i].order == f->order && set[i].sign == f->sign &&
            (f->sign != 'S' || set[i].sign_bit == f->sign_bit)) {
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * Checks
 ******************************************************************************/

static int g_failures = 0;

#define CHECK(cond, ...) do {                         \
    if (!(cond)) {                                    \
        g_failures++;                                 \
        printf("    FAIL: " __VA_ARGS__);             \
        printf("\n");                                 \
        ok = false;                                   \
    }                                                 \
} while (0)

/** Decode an unsigned DBC field from a frame (independent of the engine). */
static bool decode_field(const can_frame_t *fr, int start, int len, int order, uint32_t *out) {
    uint32_t v = 0;
    int bit = start;
    for (int i = 0; i < len; i++) {
        if (bit < 0 || bit / 8 >= fr->dlc) return false;
        uint32_t b = (fr->data[bit / 8] >> (bit % 8)) & 1u;
        if (order == 1) v |= b << i;
        else v = (v << 1) | b;
        bit = next_bit(bit, order);
    }
    *out = v;
    return true;
}

typedef struct { uint32_t t, f; } pair_t;

static int cmp_pair(const void *a, const void *b) {
    const pair_t *x = (const pair_t *)a, *y = (const pair_t *)b;
    if (x->t != y->t) return x->t < y->t ? -1 : 1;
    if (x->f != y->f) return x->f < y->f ? -1 : 1;
    return 0;
}

/**
 * True when the found field decodes the expected field without losing or
 * adding information on the observed frames: sorted by the expected value,
 * the found value must be a strictly increasing function of it. Extra constant
 * bits or a different-but-equivalent width pass; truncation, byte swaps and
 * neighbouring fields fail.
 */
static bool field_equivalent(const disc_raw_phase_t *raw, const field_ref_t *want,
                             const disc_candidate_t *c) {
    if (!raw || c->can_id != want->id || c->start_bit < 0 || c->bit_length <= 0) return false;
    pair_t *pairs = (pair_t *)malloc((size_t)(raw->frame_count + 1) * sizeof(*pairs));
    int n = 0;
    for (int i = 0; i < raw->frame_count; i++) {
        const can_frame_t *fr = &raw->frames[i];
        if (fr->can_id != want->id) continue;
        uint32_t t, f;
        if (!decode_field(fr, want->start, want->len, want->order, &t)) continue;
        if (!decode_field(fr, c->start_bit, c->bit_length, c->byte_order, &f)) continue;
        if (c->sign_char == 'S') {
            uint32_t sign;
            if (!decode_field(fr, c->sign_bit, 1, 1, &sign)) continue;
            f |= sign << c->bit_length;
        }
        pairs[n].t = t;
        pairs[n].f = f;
        n++;
    }
    bool ok = n > 0;
    int distinct = 0;
    if (ok) {
        qsort(pairs, (size_t)n, sizeof(*pairs), cmp_pair);
        distinct = 1;
        for (int i = 1; i < n && ok; i++) {
            if (pairs[i].t == pairs[i - 1].t) ok = pairs[i].f == pairs[i - 1].f;
            else { ok = pairs[i].f > pairs[i - 1].f; distinct++; }
        }
    }
    free(pairs);
    return ok && distinct >= 2;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/** Sign bit of sign-magnitude steering scenarios; 0 = two's complement steering. */
static int expected_steering_sign_bit(const scenario_t *sc) {
    return (sc->gen == gen_sm_prior || sc->gen == gen_sm_no_prior ||
            sc->gen == gen_gt3_standing) ? 44 : 0;
}

/** Only gen_gateway relays every ID slowly; the rest are direct taps, including
 *  busy_bus where 20 slow sensor IDs pull the median rate below 10 Hz. */
static disc_bus_mode_t expected_bus_mode(const scenario_t *sc) {
    /* one fast ID (the single-message edge cases) does not make a direct bus */
    return (sc->gen == gen_gateway || sc->gen == gen_edge_twins || sc->gen == gen_edge_rpm_flat)
               ? DISC_BUS_GATEWAY : DISC_BUS_DIRECT;
}

static bool run_scenario(int index, double *elapsed_ms) {
    const scenario_t *sc = &g_scenarios[index];
    bool ok = true;
    capture_t cap;
    capture_build(sc, index, &cap);

    disc_result_t r;
    double t0 = now_ms();
    disc_analyze_with_raw(cap.phases, cap.raw, &r);
    *elapsed_ms = now_ms() - t0;

    printf("[%s] (%.1f ms, bus=%s)\n", sc->name, *elapsed_ms, disc_bus_mode_name(r.bus_mode));
    CHECK(r.bus_mode == expected_bus_mode(sc), "bus mode %s, expected %s",
          disc_bus_mode_name(r.bus_mode), disc_bus_mode_name(expected_bus_mode(sc)));
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        const disc_candidate_t *c = &r.signals[s];
        const expect_t *e = &sc->expect[s];
        if (r.found[s]) {
            printf("    %-12s 0x%03X %2d|%-2d@%d%c  score=%6.1f %-6s ev=%5.1f sp=%5.1f sh=%5.1f pr=%5.1f m=%5.1f byte=%d,%d\n",
                   disc_signal_name((disc_signal_t)s), c->can_id, c->start_bit, c->bit_length,
                   c->byte_order, c->sign_char, c->score, disc_confidence_name(c->confidence),
                   c->evidence_score, c->specificity_score, c->shape_score, c->prior_score,
                   c->ambiguity_margin, c->byte_idx, c->byte2_idx);
        } else {
            printf("    %-12s not found\n", disc_signal_name((disc_signal_t)s));
        }

        if (!e->present) {
            CHECK(!r.found[s], "%s reported but absent (0x%03X, %s)",
                  disc_signal_name((disc_signal_t)s), c->can_id,
                  disc_confidence_name(c->confidence));
            continue;
        }
        CHECK(r.found[s], "%s not found", disc_signal_name((disc_signal_t)s));
        if (!r.found[s]) continue;
        if (e->not_high) {
            CHECK(c->confidence != DISC_CONF_HIGH, "%s is HIGH but the capture cannot tell its "
                  "alternatives apart (margin %.1f)", disc_signal_name((disc_signal_t)s), c->ambiguity_margin);
        }

        bool match = false;
        int phase_idx = (s == DISC_SIG_STEERING) ? DISC_PHASE_STEERING :
                        (s == DISC_SIG_RPM || s == DISC_SIG_THROTTLE) ? DISC_PHASE_THROTTLE :
                        (s == DISC_SIG_BRAKE) ? DISC_PHASE_BRAKE :
                        (s == DISC_SIG_GEAR) ? DISC_PHASE_GEAR : DISC_PHASE_WHEEL_SPEED;
        for (int a = 0; a < e->alt_count && !match; a++) {
            match = field_equivalent(&cap.raw[phase_idx], &e->alt[a], c);
        }
        CHECK(match, "%s at 0x%03X %d|%d@%d does not decode the expected field",
              disc_signal_name((disc_signal_t)s), c->can_id, c->start_bit,
              c->bit_length, c->byte_order);
        if (s == DISC_SIG_STEERING && expected_steering_sign_bit(sc) > 0) {
            CHECK(c->sign_char == 'S' && c->sign_bit == expected_steering_sign_bit(sc),
                  "steering should be sign-magnitude with sign bit %d (got %c, %d)",
                  expected_steering_sign_bit(sc), c->sign_char, c->sign_bit);
        } else if (s == DISC_SIG_STEERING) {
            CHECK(c->sign_char == '-', "steering should be signed");
        }

        /* byte/byte2 must follow MSB/LSB convention for 2-byte fields */
        if (c->byte2_idx >= 0 && c->bit_length > 8) {
            int msb = c->byte_idx, lsb = c->byte2_idx;
            bool le = c->byte_order == 1;
            CHECK(le ? msb > lsb : msb < lsb,
                  "%s byte/byte2 order %d,%d inconsistent with byte_order %d",
                  disc_signal_name((disc_signal_t)s), msb, lsb, c->byte_order);
        }
    }

    /* DBC / VSP consistency */
    char *dbc = disc_render_draft_dbc(cap.phases, cap.raw, &r, NULL);
    char *vsp = disc_render_draft_vsp(cap.phases, cap.raw, &r, NULL);
    CHECK(dbc != NULL, "DBC render failed");
    CHECK(vsp != NULL, "VSP render failed");
    if (dbc && vsp) {
        out_field_t df[MAX_OUT], vf[MAX_OUT];
        int bad_dbc_dlc = 0, bad_vsp_dlc = 0;
        int dn = parse_dbc(dbc, df, MAX_OUT, &bad_dbc_dlc);
        int vn = parse_vsp(vsp, vf, MAX_OUT, &bad_vsp_dlc);
        CHECK(bad_dbc_dlc == 0, "DBC has %d messages with DLC > 8", bad_dbc_dlc);
        CHECK(bad_vsp_dlc == 0, "VSP has %d frames with DLC > 8", bad_vsp_dlc);
        CHECK(dn == vn, "DBC has %d fields but VSP has %d", dn, vn);
        for (int i = 0; i < dn; i++) {
            CHECK(field_in(&df[i], vf, vn), "DBC field 0x%03X %d|%d@%d%c missing from VSP",
                  df[i].id, df[i].start, df[i].len, df[i].order, df[i].sign);
        }
        for (int i = 0; i < dn; i++) {
            for (int j = i + 1; j < dn; j++) {
                if (df[i].id != df[j].id) continue;
                uint64_t a = field_mask(df[i].start, df[i].len, df[i].order);
                uint64_t b = field_mask(df[j].start, df[j].len, df[j].order);
                CHECK((a & b) == 0, "DBC fields overlap in 0x%03X", df[i].id);
            }
        }
        /* every found signal's layout (as reported in the result) must be exported */
        for (int s = 0; s < DISC_SIG_COUNT; s++) {
            if (!r.found[s]) continue;
            const disc_candidate_t *c = &r.signals[s];
            out_field_t f = { c->can_id, c->start_bit, c->bit_length, c->byte_order,
                              c->sign_char == 'S' ? 'S' : c->sign_char == '-' ? '-' : '+',
                              c->sign_bit, "" };
            CHECK(field_in(&f, df, dn), "%s layout 0x%03X %d|%d@%d%c not in DBC",
                  disc_signal_name((disc_signal_t)s), f.id, f.start, f.len, f.order, f.sign);
        }
        /* GenMsgCycleTime must match the measured rate of the message */
        for (int s = 0; s < DISC_SIG_COUNT; s++) {
            if (!r.found[s] || r.signals[s].hz <= 0) continue;
            char key[64];
            snprintf(key, sizeof(key), "BO_ %u ", r.signals[s].can_id);
            const char *line = strstr(dbc, "BA_ \"GenMsgCycleTime\" ");
            int cycle = -1;
            while (line) {
                const char *k = strstr(line, key);
                const char *eol = strchr(line, '\n');
                if (k && (!eol || k < eol)) { cycle = atoi(k + strlen(key)); break; }
                line = eol ? strstr(eol, "BA_ \"GenMsgCycleTime\" ") : NULL;
            }
            double want = 1000.0 / r.signals[s].hz;
            CHECK(cycle > 0 && cycle > want * 0.65 && cycle < want * 1.35,
                  "0x%03X cycle time %d ms, expected ~%.0f ms", r.signals[s].can_id, cycle, want);
        }
        /* A released brake / pedal must not decode to the top of its scale.
         * Steering may use a negative factor (left/right convention). */
        for (const char *sg = strstr(dbc, " SG_ "); sg; sg = strstr(sg + 1, " SG_ ")) {
            if (strncmp(sg, " SG_ Steering", 13) == 0) continue;
            double factor = 0;
            const char *paren = strchr(sg, '(');
            if (paren && sscanf(paren, "(%lf,", &factor) == 1) {
                CHECK(factor > 0, "DBC field with non-positive factor: %.40s", sg + 1);
            }
        }
        if (strcmp(sc->name, "bmw_gear_value_table") == 0) {
            /* raw 5 = 1st (value 1), raw 2 = R (-1), raw 3 = P (no value) */
            CHECK(strstr(vsp, "{\"raw\": 5, \"label\": \"1\", \"value\": 1}") != NULL,
                  "draft VSP lacks the 0x3F9 gear value table (raw 5 = 1st)");
            CHECK(strstr(vsp, "{\"raw\": 2, \"label\": \"R\", \"value\": -1}") != NULL,
                  "draft VSP lacks raw 2 = R (-1)");
            CHECK(strstr(vsp, "{\"raw\": 3, \"label\": \"P\", \"value\": null}") != NULL,
                  "draft VSP lacks raw 3 = P (null)");
            CHECK(strstr(dbc, "VAL_ 1017 ") != NULL && strstr(dbc, " 5 \"1\"") != NULL,
                  "draft DBC lacks the 0x3F9 VAL_ table");
        }
        if (strncmp(sc->name, "gt3_counter_pedal_small_blips", 29) == 0) {
            /* 49|12 also decodes these frames monotonically, but reads twice the
             * rpm and wraps above 2048: the 0.25 rpm prior has to be adopted. */
            const disc_candidate_t *c = &r.signals[DISC_SIG_RPM];
            CHECK(r.found[DISC_SIG_RPM] && c->start_bit == 48 && c->bit_length == 16 &&
                  c->prior_score > 0,
                  "RPM should adopt the 0x0A8 48|16 prior (got %d|%d, prior %.1f)",
                  c->start_bit, c->bit_length, c->prior_score);
            CHECK(strstr(dbc, " : 48|16@1+ (0.25,0) ") != NULL,
                  "draft DBC lacks 0x0A8 RPM 48|16 with factor 0.25");
            /* Top pedal bits never move at 28 %: the 8-bit prior wins over a
             * 12-bit field that ends on a byte boundary. */
            CHECK(strstr(dbc, " : 12|8@1+ (0.4,0) ") != NULL,
                  "draft DBC lacks 0x121 pedal 12|8 with factor 0.4");
        }
        if (getenv("SHOW_DBC")) puts(dbc);
        if (getenv("SHOW_VSP")) puts(vsp);
    }
    free(dbc);
    free(vsp);

    capture_free(&cap);
    return ok;
}

/*******************************************************************************
 * Small-stack test
 ******************************************************************************/

typedef struct {
    bool done;
} stack_job_t;

static void *stack_worker(void *arg) {
    stack_job_t *job = (stack_job_t *)arg;
    capture_t cap;
    capture_build(&g_scenarios[0], 0, &cap);

    disc_candidate_t out, rpm;
    bool rpm_found = false;
    uint32_t excl[1] = {0};
    disc_classify(&cap.phases[DISC_PHASE_BASELINE], &cap.phases[DISC_PHASE_STEERING],
                  DISC_SIG_STEERING, excl, 0, &out, &rpm, &rpm_found);
    disc_classify_with_raw(&cap.phases[DISC_PHASE_BASELINE], &cap.raw[DISC_PHASE_BASELINE],
                           &cap.phases[DISC_PHASE_THROTTLE], &cap.raw[DISC_PHASE_THROTTLE],
                           DISC_SIG_THROTTLE, excl, 0, &out, &rpm, &rpm_found);

    disc_result_t r;
    disc_analyze_with_raw(cap.phases, cap.raw, &r);
    char *dbc = disc_render_draft_dbc(cap.phases, cap.raw, &r, NULL);
    char *vsp = disc_render_draft_vsp(cap.phases, cap.raw, &r, NULL);
    free(dbc);
    free(vsp);
    capture_free(&cap);
    job->done = true;
    return NULL;
}

static bool run_small_stack(void) {
    bool ok = true;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 512 * 1024);
    stack_job_t job = { false };
    pthread_t th;
    printf("[small_stack_512k]\n");
    if (pthread_create(&th, &attr, stack_worker, &job) != 0) {
        CHECK(false, "pthread_create failed");
        return ok;
    }
    pthread_join(th, NULL);
    pthread_attr_destroy(&attr);
    CHECK(job.done, "worker did not finish");
    if (ok) printf("    ok\n");
    return ok;
}

/*******************************************************************************
 * Session dump (same layout as a live can_discover capture session)
 ******************************************************************************/

static const char *k_slugs[DISC_NUM_PHASES] = {
    "baseline", "steering", "throttle", "brake", "gear", "wheel_speed"
};

/** Write a scenario as a capture session folder for discover_replay. */
static int dump_session(const char *name, const char *dir) {
    int index = -1;
    for (int i = 0; i < SCENARIO_COUNT; i++) {
        if (strcmp(g_scenarios[i].name, name) == 0) index = i;
    }
    if (index < 0) {
        fprintf(stderr, "unknown scenario: %s\n", name);
        return 2;
    }
    const scenario_t *sc = &g_scenarios[index];
    capture_t cap;
    capture_build(sc, index, &cap);

    char path[1024];
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        if (sc->durations[p] <= 0) continue;
        snprintf(path, sizeof(path), "%s/%02d_%s.frames.tsv", dir, p, k_slugs[p]);
        FILE *f = fopen(path, "w");
        if (!f) { perror(path); capture_free(&cap); return 2; }
        fprintf(f, "timestamp_ms\tcan_id\tis_extended\tdlc\tdata_hex\n");
        for (int i = 0; i < cap.raw[p].frame_count; i++) {
            const can_frame_t *fr = &cap.raw[p].frames[i];
            fprintf(f, "%lld\t0x%X\t%d\t%u\t", (long long)fr->timestamp_ms, fr->can_id,
                    fr->is_extended ? 1 : 0, fr->dlc);
            for (int b = 0; b < fr->dlc; b++) fprintf(f, "%02X", fr->data[b]);
            fputc('\n', f);
        }
        fclose(f);
    }

    snprintf(path, sizeof(path), "%s/manifest.json", dir);
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); capture_free(&cap); return 2; }
    fprintf(f, "{\n  \"format\":\"can_discover_dump/v1\",\n  \"status\":\"completed\",\n  \"phases\":[\n");
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        bool skipped = sc->durations[p] <= 0;
        fprintf(f, "%s    {\"index\":%d,\"slug\":\"%s\",\"skipped\":%s,\"captured\":%s,"
                   "\"actual_duration_s\":%.3f,\"analyzed_duration_s\":%.3f}",
                p ? ",\n" : "", p, k_slugs[p], skipped ? "true" : "false",
                skipped ? "false" : "true", sc->durations[p], sc->durations[p]);
    }
    fprintf(f, "\n  ]\n}\n");
    fclose(f);

    /* In-memory result, for comparison with the replay output. */
    disc_result_t r;
    disc_analyze_with_raw(cap.phases, cap.raw, &r);
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        const disc_candidate_t *c = &r.signals[s];
        if (r.found[s]) {
            printf("%s 0x%03X %d|%d@%d%c %.1f %s\n", disc_signal_name((disc_signal_t)s), c->can_id,
                   c->start_bit, c->bit_length, c->byte_order, c->sign_char, c->score,
                   disc_confidence_name(c->confidence));
        } else {
            printf("%s not_found\n", disc_signal_name((disc_signal_t)s));
        }
    }
    capture_free(&cap);
    return 0;
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(int argc, char **argv) {
    if (argc == 4 && strcmp(argv[1], "--dump") == 0) return dump_session(argv[2], argv[3]);
    const char *only = argc > 1 ? argv[1] : NULL;
    int passed = 0, ran = 0;
    double total_ms = 0.0;

    for (int i = 0; i < SCENARIO_COUNT; i++) {
        if (only && strcmp(only, g_scenarios[i].name) != 0) continue;
        double ms = 0.0;
        ran++;
        if (run_scenario(i, &ms)) passed++;
        total_ms += ms;
    }
    if (!only || strcmp(only, "small_stack") == 0) {
        ran++;
        if (run_small_stack()) passed++;
    }

    printf("\n%d/%d passed, %d failed checks, analysis total %.1f ms\n",
           passed, ran, g_failures, total_ms);
    return g_failures == 0 ? 0 : 1;
}
