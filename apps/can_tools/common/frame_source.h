/**
 * @file frame_source.h
 * @brief CAN frame source abstraction + capture buffer (Pure C)
 *
 * A frame source delivers CAN frames to a sink callback while the caller
 * pumps it. The discover session (and later the TUI) reads frames the same
 * way from a live adapter (frame_source_live.h) or a saved capture (replay).
 *
 * Threading: single-threaded. Frames are delivered only from inside
 * frame_source_pump() (live: CoreBluetooth/IOBluetooth callbacks on the main
 * run loop; replay: directly).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Capture buffer
 ******************************************************************************/

/** The capture buffer starts small and doubles on demand up to the cap.
 *  Cap: 1,000,000 frames x sizeof(can_frame_t) (32 bytes) = ~32 MB worst case,
 *  about 8 minutes of a saturated 500 kbit/s bus (~2000 frames/s). Frames past
 *  the cap (or after a failed allocation) are counted as dropped. */
#define FRAME_BUFFER_INITIAL_CAPACITY 4096
#define FRAME_BUFFER_MAX              1000000

typedef struct {
    can_frame_t *frames;
    int          count;
    int          capacity;
    int          dropped;          /**< Frames not stored since last reset */
    int64_t      last_stored_ms;   /**< Timestamp of newest stored frame */
} frame_buffer_t;

/** Empty the buffer and its drop counter (keeps the allocation). */
void frame_buffer_reset(frame_buffer_t *buf);

/** Store a frame, growing the buffer if needed; counts it as dropped if the
 *  buffer is at its cap or cannot grow. */
void frame_buffer_append(frame_buffer_t *buf, const can_frame_t *frame);

/** Release the allocation and reset. */
void frame_buffer_free(frame_buffer_t *buf);

/** frame_sink_fn adapter: ctx is a frame_buffer_t *. */
void frame_buffer_sink(const can_frame_t *frame, void *ctx);

/*******************************************************************************
 * Frame source
 ******************************************************************************/

/** Receives each frame. The frame pointer is only valid during the call. */
typedef void (*frame_sink_fn)(const can_frame_t *frame, void *ctx);

/** Implementation vtable. `impl` is the pointer given to frame_source_create(). */
typedef struct {
    /** Begin delivering frames to sink (live: enter monitor mode). */
    bool    (*start)(void *impl, frame_sink_fn sink, void *sink_ctx);
    /** Block for up to `seconds`, delivering frames that arrive. */
    void    (*pump)(void *impl, double seconds);
    /** Source can still be used (live: adapter connected). */
    bool    (*is_open)(void *impl);
    /** Finite source has no frames left (optional; NULL = never, e.g. live). */
    bool    (*exhausted)(void *impl);
    /** Clock of the source in ms, same base as frame timestamps
     *  (optional; NULL = wall clock, which is what the live adapter stamps). */
    int64_t (*now_ms)(void *impl);
    /** Stop delivering frames (live: leave monitor mode). */
    void    (*stop)(void *impl);
    /** Free impl (optional). */
    void    (*destroy)(void *impl);
    /** Frames carry controller timestamps (hw_timestamp, µs) (optional; NULL = no). */
    bool    (*hw_timestamps)(void *impl);
} frame_source_ops_t;

typedef struct frame_source frame_source_t;

/** Wrap an implementation. Returns NULL on allocation failure (impl is then
 *  not destroyed). */
frame_source_t *frame_source_create(const frame_source_ops_t *ops, void *impl);

bool    frame_source_start(frame_source_t *src, frame_sink_fn sink, void *sink_ctx);
void    frame_source_pump(frame_source_t *src, double seconds);
bool    frame_source_is_open(const frame_source_t *src);
bool    frame_source_exhausted(const frame_source_t *src);
int64_t frame_source_now_ms(const frame_source_t *src);
void    frame_source_stop(frame_source_t *src);
/** True between a successful start and stop. */
bool    frame_source_is_running(const frame_source_t *src);
/** The source stamps frames in hardware (PCAN): writers add the timestamp_us column. */
bool    frame_source_has_hw_timestamps(const frame_source_t *src);
/** impl of src when it was created with ops, else NULL (typed accessors). */
void   *frame_source_impl_of(const frame_source_t *src, const frame_source_ops_t *ops);
/** Stops the source if running, then destroys it. NULL is ignored. */
void    frame_source_free(frame_source_t *src);

/** Wall clock in ms since the epoch (gettimeofday). */
int64_t frame_source_wall_ms(void);

/*******************************************************************************
 * Replay source
 ******************************************************************************/

/**
 * Replay frames in order.
 *
 * pace = false: every pump delivers all remaining frames at once and returns
 *               immediately; the clock is the timestamp of the last delivered
 *               frame (the first frame's before any is delivered).
 * pace = true:  frames are delivered in real time by timestamp_ms; the clock
 *               starts at the first frame's timestamp on the first start(),
 *               runs while started and is frozen while stopped.
 *
 * A replay source stays open until freed; frame_source_exhausted() reports
 * the end. start() after stop() resumes where it stopped.
 */
frame_source_t *frame_source_replay_create(const can_frame_t *frames, int count, bool pace);

/** Load a *.frames.tsv (frame_tsv_load) into a replay source. NULL if the
 *  file is missing or unreadable (errors are reported on stderr). */
frame_source_t *frame_source_replay_open(const char *tsv_path, bool pace);

/**
 * Manual clock: never reads the wall clock and never sleeps. The clock starts
 * at the first frame's timestamp and moves only through
 * frame_source_replay_advance(), which (once started) delivers every frame with
 * timestamp_ms <= the new clock. pump() does nothing. Used by snapshot runs.
 */
frame_source_t *frame_source_replay_create_manual(const can_frame_t *frames, int count);
bool frame_source_replay_advance(frame_source_t *src, int64_t clock_ms);

#define FRAME_SOURCE_REPLAY_MIN_SPEED 0.25
#define FRAME_SOURCE_REPLAY_MAX_SPEED 16.0

/** True for sources made by frame_source_replay_*(). The calls below return
 *  false (or a neutral value) for any other source. */
bool   frame_source_is_replay(const frame_source_t *src);
/** Paced replay speed, clamped to 0.25x..16x (clock ms per wall ms). */
bool   frame_source_replay_set_speed(frame_source_t *src, double speed);
double frame_source_replay_speed(const frame_source_t *src);
/** Paced replay pause: the clock freezes and nothing is delivered. */
bool   frame_source_replay_set_paused(frame_source_t *src, bool paused);
bool   frame_source_replay_paused(const frame_source_t *src);
/** First/last frame timestamps and how many frames were delivered so far. */
bool   frame_source_replay_range(const frame_source_t *src, int64_t *first_ms, int64_t *last_ms,
                                 int *delivered, int *total);

#ifdef __cplusplus
}
#endif
