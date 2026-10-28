/**
 * @file tui_gauges.h
 * @brief Gauges panel model (Pure C): 9 gauges bound to signals by
 *        canonical id, candidate selection, unit conversion, sticky scales,
 *        and the btop-style bar cells (meter, steering, gear strip)
 *
 * Indices refer to tui_model_t.signals; rebind after the signal set changes.
 * All times are source-clock milliseconds.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tui_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TUI_GAUGE_THROTTLE = 0, TUI_GAUGE_BRAKE, TUI_GAUGE_STEERING,
    TUI_GAUGE_RPM, TUI_GAUGE_SPEED, TUI_GAUGE_GEAR,
    TUI_GAUGE_COOLANT, TUI_GAUGE_OIL, TUI_GAUGE_GEARBOX,
    TUI_GAUGE_COUNT
} tui_gauge_id_t;

/* Colors (xterm-256) */
#define TUI_FG_TRACK      237
#define TUI_FG_CENTER     252
#define TUI_FG_OVER       51
#define TUI_FG_GEAR_OFF   239
#define TUI_FG_GEAR_ON    231
#define TUI_FG_STALE      240

/* Steering (ISO 8855: positive = left) */
#define TUI_STEER_FULL_DEG 180.0   /* bar end, linear */
#define TUI_STEER_LOCK_DEG 450.0   /* typical road car full lock (2.5-3 turns lock to lock):
                                       the whole filled bar turns to the brake reds */

/* Sticky scales: start, growth step, ignore above */
#define TUI_SCALE_RPM_START    7000.0
#define TUI_SCALE_RPM_STEP     1000.0
#define TUI_SCALE_RPM_IGNORE   20000.0
#define TUI_SCALE_SPEED_START  200.0
#define TUI_SCALE_SPEED_STEP   50.0
#define TUI_SCALE_SPEED_IGNORE 500.0
#define TUI_SCALE_BRAKE_START  100.0
#define TUI_SCALE_BRAKE_STEP   20.0
#define TUI_SCALE_BRAKE_IGNORE 300.0

typedef enum { TUI_BRAKE_PRESSURE = 0, TUI_BRAKE_PEDAL, TUI_BRAKE_SWITCH } tui_brake_mode_t;

typedef struct {
    int  sig;      /* index into the model's signals */
    int  rank;     /* canonical priority, 0 = best */
    bool diag;
    int  order;    /* profile order (decoder index) */
} tui_gauge_cand_t;

typedef struct {
    tui_gauge_cand_t *cand;
    int               ncand;
    int               current;      /* index into cand, -1 = none */
    double            scale;        /* sticky full scale (rpm, speed, brake pressure) */
    int               max_gear_seen;
} tui_gauge_t;

typedef struct {
    tui_gauge_t g[TUI_GAUGE_COUNT];
    int         bound;              /* gauges with at least one candidate */
} tui_gauges_t;

void tui_gauges_init(tui_gauges_t *gs);
void tui_gauges_free(tui_gauges_t *gs);
/** Collect candidates from the model's signals (after tui_model_set_signals)
 *  and reset the scales. */
void tui_gauges_bind(tui_gauges_t *gs, const tui_model_t *m);
/** Back to the start scales (connect, replay start, profile change). */
void tui_gauges_reset_scales(tui_gauges_t *gs);
/** Candidate selection and scale growth from the current samples. */
void tui_gauges_update(tui_gauges_t *gs, const tui_model_t *m, int64_t now_ms);
/** The signal a gauge shows, NULL without candidates. */
const tui_signal_t *tui_gauge_signal(const tui_gauges_t *gs, const tui_model_t *m, tui_gauge_id_t id);

/** Canonical ids of a gauge in priority order (NULL-terminated). */
const char *const *tui_gauge_canonicals(tui_gauge_id_t id);
const char *tui_gauge_label(tui_gauge_id_t id, bool narrow);

/** Brake mode from the shown signal's canonical id. */
tui_brake_mode_t tui_brake_mode(const char *canonical);
/** Display value in the gauge's unit: °C, bar, km/h; throttle stays % or deg. */
double tui_gauge_convert(tui_gauge_id_t id, const char *unit, double v);
/** Throttle in degrees (unit "deg" / "°"): scale 0..90 instead of 0..100. */
bool   tui_unit_is_degrees(const char *unit);
/** Scale growth: the new sticky scale for v (start, step, ignore limit). */
double tui_scale_grow(double scale, double v, double step, double ignore);
/** Highest forward gear for the strip: table max (<= 10), else observed, >= 6. */
int    tui_gear_strip_max(int table_max, int observed_max);

/* Bar cells */
typedef struct {
    char          ch[8];
    unsigned char fg;     /* 0 = role color */
    char          role;
} tui_bar_cell_t;

/** colors: 256 / 8 / 0 (no colors: empty cells are "·"). */
typedef struct {
    int  colors;
    bool ascii;
} tui_bar_style_t;

/** btop meter: w cells, filled by frac with the gradient picked by cell
 *  position. stale: filled cells gray. Returns the filled count. */
int  tui_bar_meter(double frac, int w, const unsigned char *stops, int nstops, bool stale,
                   const tui_bar_style_t *st, tui_bar_cell_t *out);
/** Steering: half cells left, center, half cells right (2 * half + 1).
 *  Over 180°: the end cell is ◀/▶ (51). From 450° (full lock): the filled
 *  cells use the BRK gradient by distance from the center, no triangle. */
void tui_bar_steer(double deg, int half, bool stale, const tui_bar_style_t *st, tui_bar_cell_t *out);
/** Filled cells of the steering bar on the turn side (0..half). */
int  tui_steer_fill(double deg, int half);
/** Gear strip "R N 1 .. max" fitted to width (spaced when it fits, else
 *  compact), current label highlighted. Returns the cells written. */
int  tui_gear_strip(int max_gear, const char *current, int width, tui_bar_cell_t *out, int out_max);
/** Gear label for a sample: the value-table label, else -1 R, 0 N, n. */
void tui_gear_label(const tui_signal_t *s, char *out, size_t len);

/* Gradients */
extern const unsigned char TUI_GRAD_THR[5];
extern const unsigned char TUI_GRAD_BRK[5];
extern const unsigned char TUI_GRAD_RPM[11];
extern const unsigned char TUI_GRAD_CYAN[7];
extern const unsigned char TUI_GRAD_COLD[6];
extern const unsigned char TUI_GRAD_HOT[6];
/** Gradient stop for cell i of w. */
int tui_grad_pick(const unsigned char *stops, int nstops, int i, int w);

#ifdef __cplusplus
}
#endif
