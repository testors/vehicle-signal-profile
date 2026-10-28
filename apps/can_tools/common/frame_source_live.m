/**
 * @file frame_source_live.m
 * @brief Live frame source over an OBDAdapter monitor (Objective-C, MRC)
 */

#import "frame_source_live.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    OBDAdapter   *adapter;          /* retained */
    char          monitor_cmd[16];
    frame_sink_fn sink;
    void         *sink_ctx;
} live_impl_t;

static void live_frame(const can_frame_t *frame, void *context) {
    live_impl_t *live = (live_impl_t *)context;
    if (live && live->sink) live->sink(frame, live->sink_ctx);
}

static bool live_start(void *impl, frame_sink_fn sink, void *sink_ctx) {
    live_impl_t *live = (live_impl_t *)impl;
    live->sink = sink;
    live->sink_ctx = sink_ctx;
    [live->adapter startMonitorWithCallback:live_frame context:live];

    char mon_str[24];
    snprintf(mon_str, sizeof(mon_str), "%s\r", live->monitor_cmd);
    [live->adapter sendRawString:mon_str];
    return true;
}

static void live_pump(void *impl, double seconds) {
    [((live_impl_t *)impl)->adapter runLoopFor:seconds];
}

static bool live_is_open(void *impl) {
    return ((live_impl_t *)impl)->adapter.isConnected;
}

static void live_stop(void *impl) {
    live_impl_t *live = (live_impl_t *)impl;
    [live->adapter stopMonitor];
    [live->adapter sendRawString:"\r"];
    [live->adapter runLoopFor:0.2];
    live->sink = NULL;
    live->sink_ctx = NULL;
}

static void live_destroy(void *impl) {
    live_impl_t *live = (live_impl_t *)impl;
    if (!live) return;
    [live->adapter release];
    free(live);
}

static const frame_source_ops_t kLiveOps = {
    .start = live_start,
    .pump = live_pump,
    .is_open = live_is_open,
    .exhausted = NULL,      /* never: a live bus has no end */
    .now_ms = NULL,         /* wall clock: OBDAdapter stamps frames with gettimeofday */
    .stop = live_stop,
    .destroy = live_destroy,
};

frame_source_t *frame_source_live_create(OBDAdapter *adapter, const char *monitor_cmd) {
    if (!adapter || !monitor_cmd) return NULL;
    live_impl_t *live = (live_impl_t *)calloc(1, sizeof(*live));
    if (!live) return NULL;
    live->adapter = [adapter retain];
    snprintf(live->monitor_cmd, sizeof(live->monitor_cmd), "%s", monitor_cmd);

    frame_source_t *src = frame_source_create(&kLiveOps, live);
    if (!src) live_destroy(live);
    return src;
}
