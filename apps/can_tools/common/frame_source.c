/**
 * @file frame_source.c
 * @brief CAN frame source abstraction, replay source and capture buffer (Pure C)
 */

#include "frame_source.h"
#include "frame_tsv.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

/*******************************************************************************
 * Capture buffer
 ******************************************************************************/

void frame_buffer_reset(frame_buffer_t *buf) {
    buf->count = 0;
    buf->dropped = 0;
    buf->last_stored_ms = 0;
}

void frame_buffer_append(frame_buffer_t *buf, const can_frame_t *frame) {
    if (buf->count >= buf->capacity) {
        int new_capacity = buf->capacity > 0
                           ? buf->capacity * 2
                           : FRAME_BUFFER_INITIAL_CAPACITY;
        if (new_capacity > FRAME_BUFFER_MAX) new_capacity = FRAME_BUFFER_MAX;
        can_frame_t *grown = NULL;
        if (new_capacity > buf->capacity) {
            grown = (can_frame_t *)realloc(buf->frames,
                                           (size_t)new_capacity * sizeof(*grown));
        }
        if (!grown) {
            buf->dropped++;
            return;
        }
        buf->frames = grown;
        buf->capacity = new_capacity;
    }
    buf->frames[buf->count++] = *frame;
    buf->last_stored_ms = frame->timestamp_ms;
}

void frame_buffer_free(frame_buffer_t *buf) {
    free(buf->frames);
    buf->frames = NULL;
    buf->capacity = 0;
    frame_buffer_reset(buf);
}

void frame_buffer_sink(const can_frame_t *frame, void *ctx) {
    if (ctx && frame) frame_buffer_append((frame_buffer_t *)ctx, frame);
}

/*******************************************************************************
 * Frame source
 ******************************************************************************/

struct frame_source {
    const frame_source_ops_t *ops;
    void *impl;
    bool running;
};

int64_t frame_source_wall_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

frame_source_t *frame_source_create(const frame_source_ops_t *ops, void *impl) {
    if (!ops || !ops->start || !ops->pump || !ops->is_open || !ops->stop) return NULL;
    frame_source_t *src = (frame_source_t *)calloc(1, sizeof(*src));
    if (!src) return NULL;
    src->ops = ops;
    src->impl = impl;
    return src;
}

bool frame_source_start(frame_source_t *src, frame_sink_fn sink, void *sink_ctx) {
    if (!src || src->running) return false;
    src->running = src->ops->start(src->impl, sink, sink_ctx);
    return src->running;
}

void frame_source_pump(frame_source_t *src, double seconds) {
    if (src) src->ops->pump(src->impl, seconds);
}

bool frame_source_is_open(const frame_source_t *src) {
    return src && src->ops->is_open(src->impl);
}

bool frame_source_exhausted(const frame_source_t *src) {
    if (!src) return true;
    return src->ops->exhausted ? src->ops->exhausted(src->impl) : false;
}

int64_t frame_source_now_ms(const frame_source_t *src) {
    if (src && src->ops->now_ms) return src->ops->now_ms(src->impl);
    return frame_source_wall_ms();
}

void frame_source_stop(frame_source_t *src) {
    if (!src || !src->running) return;
    src->ops->stop(src->impl);
    src->running = false;
}

bool frame_source_is_running(const frame_source_t *src) {
    return src && src->running;
}

void frame_source_free(frame_source_t *src) {
    if (!src) return;
    frame_source_stop(src);
    if (src->ops->destroy) src->ops->destroy(src->impl);
    free(src);
}

/*******************************************************************************
 * Replay source
 ******************************************************************************/

typedef struct {
    can_frame_t  *frames;
    int           count;
    int           cursor;          /**< Next frame to deliver */
    bool          pace;
    bool          manual;          /**< clock moves only via frame_source_replay_advance */
    bool          paused;          /**< pace: clock frozen, nothing delivered */
    double        speed;           /**< pace: source ms per wall ms */
    bool          started;
    frame_sink_fn sink;
    void         *sink_ctx;
    int64_t       clock_ms;        /**< Source clock (frozen value while stopped) */
    int64_t       anchor_wall_ms;  /**< Wall time at the last start (pace) */
    int64_t       anchor_clock_ms; /**< Source clock at the last start (pace) */
} replay_impl_t;

static int64_t replay_clock(const replay_impl_t *r) {
    if (r->pace && !r->manual && r->started && !r->paused) {
        double elapsed = (double)(frame_source_wall_ms() - r->anchor_wall_ms) * r->speed;
        return r->anchor_clock_ms + (int64_t)elapsed;
    }
    return r->clock_ms;
}

/* Freeze the running clock into clock_ms and restart the anchor from now. */
static void replay_reanchor(replay_impl_t *r) {
    r->clock_ms = replay_clock(r);
    r->anchor_clock_ms = r->clock_ms;
    r->anchor_wall_ms = frame_source_wall_ms();
}

static void replay_deliver(replay_impl_t *r, const can_frame_t *frame) {
    if (r->sink) r->sink(frame, r->sink_ctx);
}

static bool replay_start(void *impl, frame_sink_fn sink, void *sink_ctx) {
    replay_impl_t *r = (replay_impl_t *)impl;
    r->sink = sink;
    r->sink_ctx = sink_ctx;
    r->started = true;
    r->anchor_wall_ms = frame_source_wall_ms();
    r->anchor_clock_ms = r->clock_ms;
    return true;
}

/** Sleep; false if a signal (e.g. SIGINT) ended the wait early. */
static bool sleep_ms(int64_t ms) {
    if (ms <= 0) return true;
    struct timespec ts = { .tv_sec = (time_t)(ms / 1000),
                           .tv_nsec = (long)(ms % 1000) * 1000000L };
    return nanosleep(&ts, NULL) == 0;
}

static void replay_pump(void *impl, double seconds) {
    replay_impl_t *r = (replay_impl_t *)impl;

    if (r->manual) return;   /* frame_source_replay_advance() drives it */

    if (!r->pace) {
        if (!r->started) return;
        while (r->cursor < r->count) {
            const can_frame_t *frame = &r->frames[r->cursor++];
            r->clock_ms = frame->timestamp_ms;
            replay_deliver(r, frame);
        }
        return;
    }

    int64_t deadline = frame_source_wall_ms() + (int64_t)(seconds * 1000.0);
    if (r->paused) {
        int64_t wait = deadline - frame_source_wall_ms();
        if (wait > 0) sleep_ms(wait);
        return;
    }
    for (;;) {
        if (r->started) {
            int64_t now_clock = replay_clock(r);
            while (r->cursor < r->count &&
                   r->frames[r->cursor].timestamp_ms <= now_clock) {
                replay_deliver(r, &r->frames[r->cursor++]);
            }
        }
        int64_t wall = frame_source_wall_ms();
        if (wall >= deadline) break;

        int64_t wait = deadline - wall;
        if (r->started && r->cursor < r->count) {
            int64_t due = (int64_t)((double)(r->frames[r->cursor].timestamp_ms - replay_clock(r)) /
                                    r->speed);
            if (due < wait) wait = due;
        }
        if (wait < 1) wait = 1;
        if (!sleep_ms(wait)) break;   /* interrupted: let the caller react */
    }
}

static bool replay_is_open(void *impl) {
    (void)impl;
    return true;
}

static bool replay_exhausted(void *impl) {
    const replay_impl_t *r = (const replay_impl_t *)impl;
    return r->cursor >= r->count;
}

static int64_t replay_now_ms(void *impl) {
    return replay_clock((const replay_impl_t *)impl);
}

static void replay_stop(void *impl) {
    replay_impl_t *r = (replay_impl_t *)impl;
    r->clock_ms = replay_clock(r);
    r->started = false;
    r->sink = NULL;
    r->sink_ctx = NULL;
}

static void replay_destroy(void *impl) {
    replay_impl_t *r = (replay_impl_t *)impl;
    if (!r) return;
    free(r->frames);
    free(r);
}

static const frame_source_ops_t kReplayOps = {
    .start = replay_start,
    .pump = replay_pump,
    .is_open = replay_is_open,
    .exhausted = replay_exhausted,
    .now_ms = replay_now_ms,
    .stop = replay_stop,
    .destroy = replay_destroy,
};

static replay_impl_t *replay_of(const frame_source_t *src) {
    return (src && src->ops == &kReplayOps) ? (replay_impl_t *)src->impl : NULL;
}

bool frame_source_is_replay(const frame_source_t *src) {
    return replay_of(src) != NULL;
}

bool frame_source_replay_set_speed(frame_source_t *src, double speed) {
    replay_impl_t *r = replay_of(src);
    if (!r) return false;
    if (speed < FRAME_SOURCE_REPLAY_MIN_SPEED) speed = FRAME_SOURCE_REPLAY_MIN_SPEED;
    if (speed > FRAME_SOURCE_REPLAY_MAX_SPEED) speed = FRAME_SOURCE_REPLAY_MAX_SPEED;
    if (r->pace && !r->manual && r->started && !r->paused) replay_reanchor(r);
    r->speed = speed;
    return true;
}

double frame_source_replay_speed(const frame_source_t *src) {
    replay_impl_t *r = replay_of(src);
    return r ? r->speed : 1.0;
}

bool frame_source_replay_set_paused(frame_source_t *src, bool paused) {
    replay_impl_t *r = replay_of(src);
    if (!r) return false;
    if (paused == r->paused) return true;
    if (paused) {
        r->clock_ms = replay_clock(r);
        r->paused = true;
    } else {
        r->paused = false;
        r->anchor_clock_ms = r->clock_ms;
        r->anchor_wall_ms = frame_source_wall_ms();
    }
    return true;
}

bool frame_source_replay_paused(const frame_source_t *src) {
    replay_impl_t *r = replay_of(src);
    return r && r->paused;
}

bool frame_source_replay_advance(frame_source_t *src, int64_t clock_ms) {
    replay_impl_t *r = replay_of(src);
    if (!r || !r->manual) return false;
    if (clock_ms > r->clock_ms) r->clock_ms = clock_ms;
    if (!r->started) return true;
    while (r->cursor < r->count && r->frames[r->cursor].timestamp_ms <= r->clock_ms) {
        replay_deliver(r, &r->frames[r->cursor++]);
    }
    return true;
}

bool frame_source_replay_range(const frame_source_t *src, int64_t *first_ms, int64_t *last_ms,
                               int *delivered, int *total) {
    replay_impl_t *r = replay_of(src);
    if (!r) return false;
    if (first_ms) *first_ms = r->count > 0 ? r->frames[0].timestamp_ms : 0;
    if (last_ms) *last_ms = r->count > 0 ? r->frames[r->count - 1].timestamp_ms : 0;
    if (delivered) *delivered = r->cursor;
    if (total) *total = r->count;
    return true;
}

/* Takes ownership of frames (malloc'd, may be NULL when count == 0). */
static frame_source_t *replay_wrap(can_frame_t *frames, int count, bool pace) {
    replay_impl_t *r = (replay_impl_t *)calloc(1, sizeof(*r));
    if (!r) {
        free(frames);
        return NULL;
    }
    r->frames = frames;
    r->count = count;
    r->pace = pace;
    r->speed = 1.0;
    r->clock_ms = count > 0 ? frames[0].timestamp_ms : 0;

    frame_source_t *src = frame_source_create(&kReplayOps, r);
    if (!src) replay_destroy(r);
    return src;
}

frame_source_t *frame_source_replay_create(const can_frame_t *frames, int count, bool pace) {
    if (count < 0 || (count > 0 && !frames)) return NULL;
    can_frame_t *copy = NULL;
    if (count > 0) {
        copy = (can_frame_t *)malloc((size_t)count * sizeof(*copy));
        if (!copy) return NULL;
        memcpy(copy, frames, (size_t)count * sizeof(*copy));
    }
    return replay_wrap(copy, count, pace);
}

frame_source_t *frame_source_replay_create_manual(const can_frame_t *frames, int count) {
    frame_source_t *src = frame_source_replay_create(frames, count, false);
    replay_impl_t *r = replay_of(src);
    if (r) r->manual = true;
    return src;
}

frame_source_t *frame_source_replay_open(const char *tsv_path, bool pace) {
    if (!tsv_path) return NULL;
    frame_list_t list;
    int rc = frame_tsv_load(tsv_path, tsv_path, &list);
    if (rc <= 0) {
        if (rc == 0) fprintf(stderr, "[ERROR] Cannot open %s: %s\n", tsv_path, strerror(ENOENT));
        return NULL;
    }
    return replay_wrap(list.frames, list.count, pace);
}
