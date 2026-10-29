/**
 * @file frame_source_pcan.c
 * @brief Frame source over an open, listen-only PCAN channel (Pure C)
 */

#include "frame_source_pcan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BATCH_MAX        256
#define LOG_EVERY_US     5000000   /* overrun / read error / error frame lines */
#define BUS_LOG_EVERY_US 2000000
#define STATUS_POLL_US   1000000

typedef struct {
    pcan_msg_t       m;
    pcan_timestamp_t ts;
    int64_t          host_us;
} pcan_rx_t;

typedef struct {
    pcan_channel_t   ch;
    frame_sink_fn    sink;
    void            *sink_ctx;
    bool             running;
    bool             dead;
    int              bad_reads;          /* read errors in a row */
    pcan_clock_t     clock;
    frame_source_pcan_stats_t st;
    int64_t          overrun_logged_us;
    uint64_t         overruns_logged;
    int64_t          error_logged_us;
    int64_t          errframes_logged_us;
    uint64_t         errframes_logged;
    uint32_t         bus_logged;         /* bus bits of the last bus-state line */
    int64_t          bus_logged_us;
    int64_t          status_polled_us;
    pcan_rx_t        batch[BATCH_MAX];
} pcan_impl_t;

static bool sleep_s(double seconds) {
    if (seconds <= 0) return true;
    struct timespec ts = { .tv_sec = (time_t)seconds,
                           .tv_nsec = (long)((seconds - (double)(time_t)seconds) * 1e9) };
    return nanosleep(&ts, NULL) == 0;
}

static void note_bus(pcan_impl_t *p, uint32_t bits) {
    p->st.bus_status = bits;
    if (!p->running || bits == p->bus_logged) return;
    int64_t now = pcan_mono_us();
    if (p->bus_logged_us && now - p->bus_logged_us < BUS_LOG_EVERY_US) return;
    fprintf(stderr, "[%s] PCAN %s: bus state %s\n", bits ? "WARN" : "INFO", p->ch.label, pcan_bus_state_text(bits));
    p->bus_logged = bits;
    p->bus_logged_us = now;
}

static void note_overrun(pcan_impl_t *p) {
    p->st.overruns++;
    int64_t now = pcan_mono_us();
    if (p->overrun_logged_us && now - p->overrun_logged_us < LOG_EVERY_US) return;
    fprintf(stderr, "[WARN] PCAN %s: receive queue overrun, frames were lost (%llu so far)\n", p->ch.label,
            (unsigned long long)p->st.overruns);
    p->overrun_logged_us = now;
    p->overruns_logged = p->st.overruns;
}

static void note_read_error(pcan_impl_t *p, pcan_status_t sts) {
    p->st.read_errors++;
    p->bad_reads++;
    char text[300];
    pcan_status_text(p->ch.api, sts, text, sizeof(text));
    if (p->bad_reads >= FRAME_SOURCE_PCAN_DEAD_READS && !p->dead) {
        p->dead = true;
        fprintf(stderr, "[ERROR] PCAN %s: the device stopped answering (unplugged?): %s\n", p->ch.label, text);
        return;
    }
    int64_t now = pcan_mono_us();
    if (p->error_logged_us && now - p->error_logged_us < LOG_EVERY_US) return;
    fprintf(stderr, "[WARN] PCAN %s: CAN_Read failed: %s\n", p->ch.label, text);
    p->error_logged_us = now;
}

static void note_error_frames(pcan_impl_t *p) {
    uint64_t n = p->st.counts.error;
    if (n == p->errframes_logged) return;
    int64_t now = pcan_mono_us();
    if (p->errframes_logged_us && now - p->errframes_logged_us < LOG_EVERY_US) return;
    fprintf(stderr, "[WARN] PCAN %s: %llu error frame%s seen (bus errors; wrong bitrate?)\n", p->ch.label,
            (unsigned long long)n, n == 1 ? "" : "s");
    p->errframes_logged = n;
    p->errframes_logged_us = now;
}

/* Read until the queue is empty, batch by batch. deliver = false drops the
 * messages (they queued up before a start). Returns messages read. */
static int drain(pcan_impl_t *p, bool deliver) {
    const pcan_api_t *api = p->ch.api;
    int total = 0;
    bool empty = false;
    while (!empty && !p->dead && p->ch.open) {
        int n = 0;
        while (n < BATCH_MAX) {
            pcan_rx_t *rx = &p->batch[n];
            pcan_status_t sts = api->read(p->ch.handle, &rx->m, &rx->ts);
            rx->host_us = pcan_wall_us();
            pcan_read_t res = pcan_read_result(sts);
            if (res == PCAN_READ_ERROR) {
                note_read_error(p, sts);
                empty = true;
                break;
            }
            p->bad_reads = 0;
            if (res == PCAN_READ_EMPTY) {
                empty = true;
                break;
            }
            if (deliver && (sts & (PCAN_ERROR_OVERRUN | PCAN_ERROR_QOVERRUN))) note_overrun(p);
            n++;
        }

        /* anchor / drift from the frames the controller stamped */
        for (int i = 0; i < n; i++) {
            pcan_kind_t kind = pcan_msg_kind(&p->batch[i].m);
            if (kind == PCAN_KIND_DATA || kind == PCAN_KIND_RTR) {
                pcan_clock_observe(&p->clock, p->batch[i].host_us,
                                   (int64_t)pcan_timestamp_us(&p->batch[i].ts));
            }
        }
        pcan_clock_end_batch(&p->clock);

        for (int i = 0; i < n; i++) {
            const pcan_rx_t *rx = &p->batch[i];
            if (!deliver) {
                p->st.dropped_at_start++;
                continue;
            }
            pcan_kind_t kind = pcan_msg_kind(&rx->m);
            pcan_counts_add(&p->st.counts, kind);
            if (kind == PCAN_KIND_STATUS) p->status_polled_us = 0;   /* read the bus state now */
            if (kind != PCAN_KIND_DATA || !p->running || !p->sink) continue;
            can_frame_t f;
            pcan_msg_to_frame(&rx->m, &f);
            can_frame_set_hw_time_us(&f, pcan_clock_map(&p->clock, (int64_t)pcan_timestamp_us(&rx->ts)));
            p->st.delivered++;
            p->sink(&f, p->sink_ctx);
        }
        total += n;
    }
    if (deliver) note_error_frames(p);
    return total;
}

/* Bus state from CAN_GetStatus, once a second (and right after a status
 * message): one source of truth, so the log does not flap between sources. */
static void poll_status(pcan_impl_t *p) {
    int64_t now = pcan_mono_us();
    if (p->status_polled_us && now - p->status_polled_us < STATUS_POLL_US) return;
    p->status_polled_us = now;
    pcan_status_t sts = p->ch.api->get_status(p->ch.handle);
    if (pcan_read_result(sts) != PCAN_READ_ERROR) note_bus(p, pcan_bus_bits(sts));
}

static bool pcan_src_start(void *impl, frame_sink_fn sink, void *sink_ctx) {
    pcan_impl_t *p = (pcan_impl_t *)impl;
    if (p->dead || !p->ch.open) return false;
    drain(p, false);     /* what queued up while stopped is not part of this capture */
    if (p->dead) return false;
    p->sink = sink;
    p->sink_ctx = sink_ctx;
    p->running = true;
    return true;
}

static void pcan_src_pump(void *impl, double seconds) {
    pcan_impl_t *p = (pcan_impl_t *)impl;
    if (!p->running || p->dead || !p->ch.open) {
        sleep_s(seconds);
        return;
    }
    int64_t deadline = pcan_mono_us() + (int64_t)(seconds * 1e6);
    drain(p, true);
    poll_status(p);
    while (p->running && !p->dead) {
        int64_t now = pcan_mono_us();
        if (now >= deadline) break;
        if (!pcan_wait(&p->ch, deadline - now)) break;   /* a signal: let the caller react */
        if (drain(p, true) == 0 && p->ch.rx_fd >= 0) {
            /* woke without a message: do not spin on the descriptor */
            int64_t left = deadline - pcan_mono_us();
            sleep_s((double)(left < 500 ? left : 500) / 1e6);
        }
    }
}

static bool pcan_src_is_open(void *impl) {
    const pcan_impl_t *p = (const pcan_impl_t *)impl;
    return p->ch.open && !p->dead;
}

static void pcan_src_stop(void *impl) {
    pcan_impl_t *p = (pcan_impl_t *)impl;
    p->running = false;
    p->sink = NULL;
    p->sink_ctx = NULL;
}

static bool pcan_src_hw(void *impl) {
    (void)impl;
    return true;
}

static void fill_stats(const pcan_impl_t *p, frame_source_pcan_stats_t *out) {
    *out = p->st;
    const pcan_clock_t *c = &p->clock;
    out->anchored = c->anchored;
    out->reanchors = c->reanchors;
    out->have_drift = false;
    if (c->have_drift) {
        out->have_drift = true;
        out->drift_us = c->drift_us;
        out->drift_after_s = (double)(c->drift_host_us - c->anchored_host_us) / 1e6;
    } else if (c->win_valid && c->batch_host_us - c->anchored_host_us >= 1000000) {
        /* a partial window, but only once there is a second to compare */
        out->have_drift = true;
        out->drift_us = c->win_min;
        out->drift_after_s = (double)(c->batch_host_us - c->anchored_host_us) / 1e6;
    }
}

static void pcan_src_destroy(void *impl) {
    pcan_impl_t *p = (pcan_impl_t *)impl;
    if (!p) return;
    frame_source_pcan_stats_t st;
    fill_stats(p, &st);
    fprintf(stderr, "[INFO] PCAN %s closed: %llu frame%s", p->ch.label, (unsigned long long)st.delivered,
            st.delivered == 1 ? "" : "s");
    if (st.counts.rtr || st.counts.status || st.counts.error || st.counts.other) {
        fprintf(stderr, " (not delivered: %llu RTR, %llu status, %llu error, %llu other)",
                (unsigned long long)st.counts.rtr, (unsigned long long)st.counts.status,
                (unsigned long long)st.counts.error, (unsigned long long)st.counts.other);
    }
    if (st.overruns) fprintf(stderr, ", %llu overrun%s", (unsigned long long)st.overruns, st.overruns == 1 ? "" : "s");
    if (st.read_errors) fprintf(stderr, ", %llu read error%s", (unsigned long long)st.read_errors,
                                st.read_errors == 1 ? "" : "s");
    if (st.have_drift) {
        fprintf(stderr, "; clock drift (host - mapped) %+.3f ms after %.1f s", (double)st.drift_us / 1000.0,
                st.drift_after_s);
    }
    if (st.reanchors) fprintf(stderr, "; re-anchored %d time%s", st.reanchors, st.reanchors == 1 ? "" : "s");
    fprintf(stderr, "\n");
    pcan_close(&p->ch);
    free(p);
}

static const frame_source_ops_t kPcanOps = {
    .start = pcan_src_start,
    .pump = pcan_src_pump,
    .is_open = pcan_src_is_open,
    .exhausted = NULL,     /* a live bus has no end */
    .now_ms = NULL,        /* wall clock: frames are mapped onto it */
    .stop = pcan_src_stop,
    .destroy = pcan_src_destroy,
    .hw_timestamps = pcan_src_hw,
};

frame_source_t *frame_source_pcan_create(pcan_channel_t *ch) {
    if (!ch || !ch->open) return NULL;
    pcan_impl_t *p = (pcan_impl_t *)calloc(1, sizeof(*p));
    if (!p) return NULL;
    p->ch = *ch;
    pcan_clock_reset(&p->clock);
    frame_source_t *src = frame_source_create(&kPcanOps, p);
    if (!src) {
        free(p);
        return NULL;
    }
    ch->open = false;    /* the source owns the channel now */
    ch->rx_fd = -1;
    return src;
}

bool frame_source_is_pcan(const frame_source_t *src) {
    return frame_source_impl_of(src, &kPcanOps) != NULL;
}

const pcan_channel_t *frame_source_pcan_channel(const frame_source_t *src) {
    const pcan_impl_t *p = (const pcan_impl_t *)frame_source_impl_of(src, &kPcanOps);
    return p ? &p->ch : NULL;
}

bool frame_source_pcan_stats(const frame_source_t *src, frame_source_pcan_stats_t *out) {
    const pcan_impl_t *p = (const pcan_impl_t *)frame_source_impl_of(src, &kPcanOps);
    if (!p || !out) return false;
    fill_stats(p, out);
    return true;
}
