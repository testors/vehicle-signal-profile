/**
 * @file tui_model.h
 * @brief TUI data model (Pure C): raw CAN table, signal stats and history,
 *        bus classifier, log, formatting
 *
 * All times are source-clock milliseconds (frame_source_now_ms / frame
 * timestamps): wall clock live, replay clock in replay, simulated clock in
 * snapshot runs. Nothing here reads the wall clock.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_discover.h"
#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Rate counter: 100 ms buckets on the source clock
 ******************************************************************************/

#define TUI_RATE_BUCKET_MS 100
#define TUI_RATE_RING      64          /* covers 6.4 s */

typedef struct {
    uint16_t count[TUI_RATE_RING];
    int64_t  bucket[TUI_RATE_RING];   /* bucket number held by the slot, -1 = empty */
    int64_t  first_ms;                /* first event, -1 = none */
} tui_rate_t;

void tui_rate_init(tui_rate_t *r);
void tui_rate_add(tui_rate_t *r, int64_t ms);
/** Events per second over the complete buckets of the last window_ms before
 *  now (the current, partial bucket is excluded; a younger counter uses the
 *  time since its first event). Returns -1 when there is no complete bucket. */
double tui_rate_hz(const tui_rate_t *r, int64_t now_ms, int window_ms);

/*******************************************************************************
 * Raw CAN table
 ******************************************************************************/

typedef struct {
    bool       used;
    uint32_t   id;
    bool       ext;
    uint8_t    dlc;
    uint8_t    data[8];
    int64_t    byte_change_ms[8];
    int64_t    last_change_ms;     /* most recent byte change */
    int64_t    first_ms;
    int64_t    last_ms;
    uint64_t   count;
    tui_rate_t rate;
} tui_raw_entry_t;

typedef enum { TUI_SORT_HZ = 0, TUI_SORT_ID, TUI_SORT_CHANGED, TUI_SORT_COUNT } tui_sort_t;

typedef struct {
    tui_raw_entry_t *slots;     /* open addressing by (id, ext) */
    int              capacity;  /* power of two */
    int              count;
} tui_raw_table_t;

#define TUI_RAW_MAX_IDS 8192
#define TUI_RAW_HZ_WINDOW_MS 2000

void tui_raw_init(tui_raw_table_t *t);
void tui_raw_free(tui_raw_table_t *t);
void tui_raw_clear(tui_raw_table_t *t);
/** Record a frame at ts; NULL when the table is full. */
tui_raw_entry_t *tui_raw_update(tui_raw_table_t *t, const can_frame_t *frame, int64_t ts);
double tui_raw_hz(const tui_raw_entry_t *e, int64_t now_ms);
/** Entries matching the filter (hex prefix of the displayed id, optional
 *  "0x"; NULL/"" = all) in sort order. Writes up to max pointers. */
int tui_raw_sorted(const tui_raw_table_t *t, tui_sort_t sort, int64_t now_ms,
                   const char *filter, const tui_raw_entry_t **out, int max);
bool tui_raw_matches(const tui_raw_entry_t *e, const char *filter);

/*******************************************************************************
 * Signals
 ******************************************************************************/

typedef enum {
    TUI_GROUP_RPM = 0, TUI_GROUP_GEAR, TUI_GROUP_STEERING, TUI_GROUP_THROTTLE,
    TUI_GROUP_BRAKE, TUI_GROUP_OTHER,
} tui_group_t;

/** Temperature signals: canonical id ending ".temperature" or unit C/°C/F/°F. */
typedef enum {
    TUI_TEMP_NONE = 0,
    TUI_TEMP_COOLANT,      /* engine.coolant.temperature, thermal.coolant.temperature */
    TUI_TEMP_OIL,          /* engine.oil.temperature */
    TUI_TEMP_GEARBOX,      /* transmission.oil.temperature */
    TUI_TEMP_OTHER,
} tui_temp_kind_t;

typedef enum { TUI_ZONE_COLD = 0, TUI_ZONE_OK, TUI_ZONE_HOT } tui_zone_t;

/** Timeline colors by height (section 5 of the step 4 brief). */
typedef enum {
    TUI_PAL_GENERIC = 0,   /* cyan 31 38 45 87 */
    TUI_PAL_RPM,           /* 41 149 215 196 */
    TUI_PAL_THROTTLE,      /* 34 40 46 83 */
    TUI_PAL_BRAKE,         /* 124 160 196 203 */
    TUI_PAL_TEMP,          /* coolant/oil/gearbox: zone color of the newer sample */
} tui_palette_t;

/* Temperature zones (variant A): bar range and zone limits in °C. */
typedef struct {
    double lo, hi;         /* bar range */
    double cold_below;     /* < cold_below: cold */
    double hot_from;       /* >= hot_from: hot */
} tui_temp_zones_t;

const tui_temp_zones_t *tui_temp_zones(tui_temp_kind_t kind);   /* NULL for NONE / OTHER */
tui_zone_t tui_temp_zone(tui_temp_kind_t kind, double celsius);
/** "F" / "°F" (case-insensitive). */
bool   tui_unit_is_fahrenheit(const char *unit);
double tui_to_celsius(double v, const char *unit);
/** Zone colors: text (cold 39, ok 46, hot 196) and timeline cell (33, 40, 196). */
int    tui_zone_text_fg(tui_zone_t z);
int    tui_zone_cell_fg(tui_zone_t z);

#define TUI_HIST_BUCKET_MS 50
#define TUI_HIST_RING      256         /* > 10 s of 50 ms buckets (no wrap inside the window) */
#define TUI_HIST_WINDOW_MS 10000
#define TUI_SIGNAL_HZ_WINDOW_MS 5000
#define TUI_DEAD_MS 10000
#define TUI_SIGNAL_TS_RING 512

typedef struct {
    int64_t     bucket;     /* -1 = empty */
    bool        has_value;
    double      value;
    const char *label;      /* owned by the decoder */
} tui_hist_slot_t;

/** What the app tells the model about a signal (copied). */
typedef struct {
    const char *name;
    const char *unit;          /* may be NULL */
    const char *group;         /* vsp group name: rpm gear steering throttle brake custom */
    bool        diagnostic;
    bool        has_value_table;
    double      factor;
    double      cycle_ms;      /* 0 = unknown */
    uint32_t    frame_id;      /* CAN */
    const char *request_hex;   /* diagnostic, e.g. "01 2F" */
    int         decoder_index;
    const char *canonical;     /* canonical.id, may be NULL / "" */
    int         max_gear;      /* highest forward gear in the value table, 0 = none */
} tui_signal_desc_t;

typedef struct {
    char        name[64];
    char        unit[16];
    char        src[16];       /* "0x0F3", "0x18DAF110", "obd", "diag" */
    char        request[40];   /* "OBD 01 2F", "DIAG 22 2B 21" */
    tui_group_t group;
    bool        diagnostic;
    bool        discrete;      /* value table or gear group: text history */
    int         decimals;
    double      cycle_ms;
    int         decoder_index;
    char        canonical[64]; /* lower case */
    tui_temp_kind_t temp;
    tui_palette_t   palette;
    int         max_gear;

    bool        sampled;
    uint64_t    samples;
    int64_t     first_ms;
    int64_t     last_ms;
    bool        has_value;
    bool        value_null;
    double      value;
    const char *label;
    int64_t     ts_ring[TUI_SIGNAL_TS_RING];   /* recent sample times (hz) */
    int         ts_head;
    int         ts_count;
    tui_hist_slot_t hist[TUI_HIST_RING];
} tui_signal_t;

typedef enum {
    TUI_SIG_NEVER = 0,   /* no sample yet */
    TUI_SIG_LIVE,
    TUI_SIG_STALE,       /* age > 3 periods: gray */
    TUI_SIG_DEAD,        /* age > 10 s: blanked */
} tui_sig_state_t;

/** Fill a signal from its description (static part) and reset its state. */
void tui_signal_init(tui_signal_t *s, const tui_signal_desc_t *d);
void tui_signal_sample(tui_signal_t *s, int64_t ts, bool has_value, bool value_null,
                       double value, const char *label);
/** (n-1)/(t_last - t_first) over the samples of the last 5 s; -1 (unknown)
 *  with fewer than 2. A stopped signal keeps its true rate until its samples
 *  leave the window (the display shows 0 once it is stale). */
double tui_signal_hz(const tui_signal_t *s, int64_t now_ms);
/** Stale period: cycle_ms, else 1/measured hz, else 1 s. */
double tui_signal_period_ms(const tui_signal_t *s, int64_t now_ms);
tui_sig_state_t tui_signal_state(const tui_signal_t *s, int64_t now_ms);
/** Display order: rpm, gear, steering, throttle, brake, then by name;
 *  diagnostic signals last. qsort comparator over tui_signal_t. */
int tui_signal_compare(const void *a, const void *b);

/** Sparkline of the last 10 s in `cells` columns (UTF-8 into out). One bucket
 *  per cell, last value in the bucket, scaled to the window min..max; constant
 *  -> flat line; empty bucket -> space. */
void tui_signal_sparkline(const tui_signal_t *s, int64_t now_ms, int cells, bool ascii,
                          char *out, size_t out_len);
/** btop-style braille timeline of the last 10 s in `cells` columns: two
 *  samples per cell (each the last value in its half of the cell's time
 *  range, held like the sparkline), height 1..4 over the window min..max
 *  (temperatures: at least a 10 °C / 18 °F span), flat window -> "⠤".
 *  Writes UTF-8 to out and a color index per cell to fg (0 for blank
 *  cells). Returns the number of cells written. */
int  tui_signal_braille(const tui_signal_t *s, int64_t now_ms, int cells, char *out, size_t out_len,
                        unsigned char *fg);
/** Braille code point for left / right heights 0..4 (0, 0 -> 0: blank). */
unsigned int tui_braille_char(int left, int right);
/** Height 1..4 of v in lo..hi (lo == hi: 0). */
int  tui_braille_level(double v, double lo, double hi);
/** Timeline color for a height 1..4 in a palette (TUI_PAL_TEMP: 0). */
int  tui_palette_fg(tui_palette_t p, int level);

/** Recent values as text, one per second, oldest left ("2 2 2 3 3"), fitted
 *  to cols. */
void tui_signal_discrete_history(const tui_signal_t *s, int64_t now_ms, int cols,
                                 char *out, size_t out_len);
/** Min and max of the history window; false when empty. */
bool tui_signal_minmax(const tui_signal_t *s, int64_t now_ms, double *min, double *max);

/*******************************************************************************
 * Bus classifier (same statistics as discover's baseline)
 ******************************************************************************/

typedef enum {
    TUI_BUS_DETECTING = 0,
    TUI_BUS_DIRECT,
    TUI_BUS_GATEWAY,
    TUI_BUS_SILENT,
} tui_bus_t;

#define TUI_CLS_WINDOW_MS 3000
#define TUI_CLS_EVERY_MS  5000

typedef struct {
    bool         started;
    bool         paused;        /* TX mode: the monitor is not running */
    tui_bus_t    bus;
    bool         direct_seen;   /* sticky until disconnect */
    int64_t      window_start_ms;
    int64_t      window_end_ms;
    bool         accumulating;
    disc_phase_t phase;
    bool         phase_ready;
    int          windows;
    int          last_window_frames;
    int          last_fast_ids;  /* IDs >= 20 Hz in the last window */
} tui_classifier_t;

void tui_classifier_init(tui_classifier_t *c);
void tui_classifier_free(tui_classifier_t *c);
/** First window [start, start + 3 s); then every 5 s over the last 3 s. */
void tui_classifier_start(tui_classifier_t *c, int64_t start_ms);
void tui_classifier_frame(tui_classifier_t *c, const can_frame_t *frame, int64_t ts);
/** Evaluate a finished window; true when the classification changed. */
bool tui_classifier_tick(tui_classifier_t *c, int64_t now_ms);
/** Record DIRECT from elsewhere (discover baseline): locks TX. */
void tui_classifier_note_direct(tui_classifier_t *c);
const char *tui_bus_text(tui_bus_t bus);   /* "detecting…", "PT-CAN DIRECT", ... */

/*******************************************************************************
 * Log
 ******************************************************************************/

#define TUI_LOG_LINES 512

typedef enum { TUI_LOG_INFO = 0, TUI_LOG_WARN, TUI_LOG_ERROR } tui_log_level_t;

typedef struct {
    int64_t         ms;
    tui_log_level_t level;
    char            text[200];
} tui_log_line_t;

typedef struct {
    tui_log_line_t lines[TUI_LOG_LINES];
    int            head;      /* next write */
    int            count;
} tui_log_t;

/** Add one or more lines ("[INFO] " stripped; [WARN]/[ERROR] set the level). */
void tui_log_add(tui_log_t *log, int64_t ms, const char *text);
/** i = 0 is the newest line. */
const tui_log_line_t *tui_log_get(const tui_log_t *log, int i);

/*******************************************************************************
 * Model
 ******************************************************************************/

typedef struct {
    tui_raw_table_t  raw;
    tui_signal_t    *signals;        /* display order */
    int              signal_count;
    int             *by_decoder;     /* decoder index -> signals[] index */
    int              decoder_count;
    tui_classifier_t cls;
    tui_log_t        log;
    tui_rate_t       frames;         /* every frame */
    uint64_t         frame_total;
    int64_t          origin_ms;      /* start of monitoring / first replay frame */
} tui_model_t;

void tui_model_init(tui_model_t *m, int64_t origin_ms);
void tui_model_free(tui_model_t *m);
/** Replace the signal set (sorted into display order). */
bool tui_model_set_signals(tui_model_t *m, const tui_signal_desc_t *descs, int count);
/** Raw table, frame rate and classifier for one frame at ts. */
void tui_model_frame(tui_model_t *m, const can_frame_t *frame, int64_t ts);
tui_signal_t *tui_model_signal_for_decoder(tui_model_t *m, int decoder_index);
double tui_model_frames_per_s(const tui_model_t *m, int64_t now_ms);

/*******************************************************************************
 * Formatting
 ******************************************************************************/

/** Integer factor -> 0, else clamp(ceil(-log10(factor)), 0, 3). */
int  tui_decimals_for_factor(double factor);
/** "%.*f" without "-0" / "-0.0" (no thousands separators). */
void tui_format_fixed(char *out, size_t len, double value, int decimals);
/** Fixed decimals with thousands separators when |integer part| >= 1000. */
void tui_format_value(char *out, size_t len, double value, int decimals);
/** Signal value: like tui_format_value, but 0 decimals when |value| >= 1000. */
void tui_format_signal_value(char *out, size_t len, double value, int decimals);
/** "10ms", "0.8s", "12s". */
void tui_format_age(char *out, size_t len, int64_t age_ms);
/** "%.0f" when >= 10, else "%.1f"; "—" when unknown (< 0). */
void tui_format_hz(char *out, size_t len, double hz, bool ascii);
/** "1,842" */
void tui_format_count(char *out, size_t len, long long n);
/** "mm:ss" (minutes may exceed 59). */
void tui_format_mmss(char *out, size_t len, int64_t ms);
/** "HH:MM:SS" */
void tui_format_hms(char *out, size_t len, int64_t ms);
/** CAN id as displayed: "0x0F3", or 8 hex digits for extended ids. */
void tui_format_can_id(char *out, size_t len, uint32_t id, bool ext);

#ifdef __cplusplus
}
#endif
