/**
 * @file cmd_probe.m
 * @brief VSP signal acquisition-rate validator.
 *
 * Validates whether selected vehicle-signal-profile signals can be observed
 * from passive CAN frames or diagnostic queries at a target rate.
 *
 * Profile loading, signal selection and frame decoding live in
 * common/vsp_engine and the adapter connection in common/obd_connect; this
 * command adds the OBD schedulers, rate statistics, the --plan output and the
 * result JSON. Moved from vsp_probe/main.m (can_tools probe / vsp_probe alias).
 *
 * --device pcan / pcan1..pcan8 (PEAK PCAN-USB, listen-only) runs only the CAN
 * phase from the PCAN frame source: --mode obd and --mode auto (which may
 * fall back to queries) are refused before the profile is even loaded.
 */

#import <Foundation/Foundation.h>
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include "elm327.h"
#include "stn2255.h"
#include "vsp_engine.h"
#import "obd_adapter.h"
#import "obd_connect.h"
#import "obd_poll.h"
#include "commands.h"
#include "cli_pcan.h"
#include "frame_source_pcan.h"

typedef NS_ENUM(NSInteger, VSPProbeMode) {
    VSPProbeModeAuto = 0,
    VSPProbeModeCAN,
    VSPProbeModeOBD,
};

typedef NS_ENUM(NSInteger, VSPProbeOBDStrategy) {
    VSPProbeOBDStrategyAuto = 0,
    VSPProbeOBDStrategyPoll,
    VSPProbeOBDStrategyPeriodic,
};

typedef struct {
    const char *profile_path;
    const char *device;
    const char *signals;
    const char *out_path;
    const char *bitrate;     /* --bitrate (PCAN only) */
    VSPProbeMode mode;
    VSPProbeOBDStrategy obd_strategy;
    bool plan_only;
    double target_hz;
    double duration_s;
    int timeout_ms;
} vsp_probe_options_t;

/** Probe bookkeeping per selected signal (decoder index); samples and
 *  values are kept by the decoder. */
typedef struct {
    int    attempts;
    int    errors;
    double test_duration_s;
} vsp_probe_stats_t;

typedef struct {
    vsp_decoder_t *decoder;  /* NULL: only count frames */
    bool diagnostic;
    int frames;            /* every frame received, matched or not */
} vsp_monitor_context_t;

static volatile sig_atomic_t g_interrupted = 0;

static void signal_handler(int sig) {
    (void)sig;
    g_interrupted = 1;
}

static int64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void usage(void) {
    fprintf(stderr,
            "Usage: vsp_probe <profile.vsp> [--device NAME] [--hz N] [--seconds N]\n"
            "                 [--mode auto|can|obd] [--obd poll|periodic|auto]\n"
            "                 [--signals LIST] [--timeout-ms N] [--plan] [--out FILE]\n"
            "                 [--bitrate RATE|auto]\n\n"
            "Default signal groups: rpm, gear, steering, throttle, brake.\n"
            "--signals accepts group names, canonical ids, or VSP signal names.\n"
            "--device pcan / pcan1..pcan8: PEAK PCAN-USB, listen-only: needs --mode can;\n"
            "--bitrate (PCAN only) is auto or one of 1M 800k 500k ... 5k (default 500k).\n");
}

static bool parse_double_arg(const char *s, double *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    double v = strtod(s, &end);
    if (!end || *end != '\0') return false;
    *out = v;
    return true;
}

static bool parse_int_arg(const char *s, int *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_options(int argc, const char *argv[], vsp_probe_options_t *opts) {
    memset(opts, 0, sizeof(*opts));
    opts->mode = VSPProbeModeAuto;
    opts->obd_strategy = VSPProbeOBDStrategyAuto;
    opts->target_hz = 5.0;
    opts->duration_s = 10.0;
    opts->timeout_ms = 800;

    if (argc < 2) return false;
    opts->profile_path = argv[1];

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            opts->device = argv[++i];
        } else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
            if (!parse_double_arg(argv[++i], &opts->target_hz)) return false;
        } else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            if (!parse_double_arg(argv[++i], &opts->duration_s)) return false;
        } else if (strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
            if (!parse_int_arg(argv[++i], &opts->timeout_ms)) return false;
        } else if (strcmp(argv[i], "--signals") == 0 && i + 1 < argc) {
            opts->signals = argv[++i];
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            opts->out_path = argv[++i];
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            opts->bitrate = argv[++i];
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            const char *mode = argv[++i];
            if (strcasecmp(mode, "auto") == 0) opts->mode = VSPProbeModeAuto;
            else if (strcasecmp(mode, "can") == 0) opts->mode = VSPProbeModeCAN;
            else if (strcasecmp(mode, "obd") == 0) opts->mode = VSPProbeModeOBD;
            else return false;
        } else if (strcmp(argv[i], "--obd") == 0 && i + 1 < argc) {
            const char *strategy = argv[++i];
            if (strcasecmp(strategy, "auto") == 0) opts->obd_strategy = VSPProbeOBDStrategyAuto;
            else if (strcasecmp(strategy, "poll") == 0) opts->obd_strategy = VSPProbeOBDStrategyPoll;
            else if (strcasecmp(strategy, "periodic") == 0) opts->obd_strategy = VSPProbeOBDStrategyPeriodic;
            else return false;
        } else if (strcmp(argv[i], "--plan") == 0) {
            opts->plan_only = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            return false;
        } else {
            fprintf(stderr, "[ERROR] Unexpected argument: %s\n", argv[i]);
            return false;
        }
    }

    if (opts->target_hz <= 0.0 || opts->duration_s <= 0.0 || opts->timeout_ms <= 0) {
        return false;
    }
    if (!opts->plan_only && (!opts->device || !*opts->device)) {
        fprintf(stderr, "[ERROR] --device is required unless --plan is used\n");
        return false;
    }
    return true;
}

static vsp_acq_filter_t acquisition_filter(VSPProbeMode mode) {
    if (mode == VSPProbeModeCAN) return VSP_ACQ_CAN;
    if (mode == VSPProbeModeOBD) return VSP_ACQ_DIAGNOSTIC;
    return VSP_ACQ_ANY;
}

static NSString *ns_string(const char *s) {
    return [NSString stringWithUTF8String:s ? s : ""] ?: @"";
}

static int count_kind(const vsp_decoder_t *dec, vsp_signal_kind_t kind) {
    int n = 0;
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (vsp_decoder_signal_info(dec, i)->kind == kind) n++;
    }
    return n;
}

static void monitor_callback(const can_frame_t *frame, void *context) {
    vsp_monitor_context_t *ctx = (vsp_monitor_context_t *)context;
    if (!ctx || !frame) return;
    ctx->frames += 1;

    double ts = frame->hw_timestamp ? (double)can_frame_time_us(frame) / 1000.0
              : frame->timestamp_ms > 0 ? (double)frame->timestamp_ms : (double)now_ms();

    if (!ctx->diagnostic) {
        vsp_decoder_feed_can(ctx->decoder, frame, ts);
        return;
    }

    vsp_decoder_feed_diagnostic(ctx->decoder, frame, ts, NULL);
}

static void send_best_effort(OBDAdapter *adapter, const char *cmd) {
    char resp[512] = "";
    [adapter sendCommand:cmd response:resp maxLen:sizeof(resp)];
}

static bool send_required(OBDAdapter *adapter, const char *cmd) {
    char resp[512] = "";
    if (![adapter sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
        elm327_is_error_response(resp)) {
        fprintf(stderr, "[ERROR] Command failed: %s", cmd);
        if (resp[0]) fprintf(stderr, " => %s", resp);
        fprintf(stderr, "\n");
        return false;
    }
    return true;
}

/* Reset + STN detection before an OBD phase sets up its own protocol
 * (common/obd_connect BASIC profile). */
static bool init_basic_adapter(OBDAdapter *adapter, stn2255_info_t *stnInfo, bool *stnDetected) {
    obd_init_config_t cfg = { .profile = OBD_INIT_BASIC };
    obd_link_t link;
    obd_link_init(&link);
    bool ok = obd_init_adapter(adapter, &cfg, &link);
    *stnDetected = link.stn_detected;
    if (stnInfo) *stnInfo = link.stn_info;
    return ok;
}

static NSMutableArray<NSNumber *> *unique_can_ids_for_items(const vsp_decoder_t *dec,
                                                            bool diagnostic) {
    NSMutableSet<NSNumber *> *seen = [NSMutableSet set];
    NSMutableArray<NSNumber *> *ids = [NSMutableArray array];
    vsp_signal_kind_t kind = diagnostic ? VSP_SIGNAL_DIAGNOSTIC_QUERY : VSP_SIGNAL_CAN_FRAME;
    for (int idx = 0; idx < vsp_decoder_signal_count(dec); idx++) {
        const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, idx);
        if (item->kind != kind) continue;
        uint32_t vals[2] = { diagnostic ? item->tx_id : item->frame_id,
                             diagnostic ? item->rx_id : 0 };
        for (int i = 0; i < 2; i++) {
            if (vals[i] == 0) continue;
            NSNumber *n = @(vals[i]);
            if (![seen containsObject:n]) {
                [seen addObject:n];
                [ids addObject:n];
            }
        }
    }
    return ids;
}

static void cleanup_periodic_obd(OBDAdapter *adapter, NSString *strategyName);

/* Receive-only setup (common/obd_connect PASSIVE profile). On STN the
 * profile's CAN IDs become STFAP pass filters narrowed by frame_id_mask;
 * dec = NULL clears the filters (unfiltered listen). A plain ELM327 gets no
 * filters: it holds only one. */
static bool setup_passive_can(OBDAdapter *adapter,
                              const vsp_decoder_t *dec,
                              bool *stnDetectedOut) {
    NSMutableArray<NSNumber *> *ids = unique_can_ids_for_items(dec, false);
    obd_can_filter_t *filters = (obd_can_filter_t *)calloc(ids.count + 1, sizeof(*filters));
    if (!filters) return false;
    int filterCount = 0;
    for (NSNumber *n in ids) {
        uint32_t mask = 0x7FF;
        for (int idx = 0; idx < vsp_decoder_signal_count(dec); idx++) {
            const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, idx);
            if (item->kind != VSP_SIGNAL_CAN_FRAME) continue;
            if (item->frame_id == n.unsignedIntValue && item->frame_id_mask) mask = item->frame_id_mask & 0x7FF;
        }
        filters[filterCount].id = n.unsignedIntValue & mask;
        filters[filterCount].mask = mask;
        filterCount++;
    }

    obd_init_config_t cfg = {
        .profile = OBD_INIT_PASSIVE,
        .filters = filters,
        .filter_count = filterCount,
        .apply_elm_filters = false,
    };
    obd_link_t link;
    obd_link_init(&link);
    bool ok = obd_init_adapter(adapter, &cfg, &link);
    if (stnDetectedOut) *stnDetectedOut = link.stn_detected;
    free(filters);
    return ok;
}

static int run_monitor_phase(OBDAdapter *adapter,
                             const char *monitorCmd,
                             vsp_decoder_t *dec,
                             bool diagnostic,
                             double seconds) {
    vsp_monitor_context_t ctx = { .decoder = dec, .diagnostic = diagnostic, .frames = 0 };
    [adapter startMonitorWithCallback:monitor_callback context:&ctx];
    [adapter sendRawString:[[NSString stringWithFormat:@"%s\r", monitorCmd] UTF8String]];

    NSDate *end = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while (!g_interrupted && adapter.isConnected &&
           [[NSDate date] compare:end] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }

    [adapter stopMonitor];
    [adapter sendRawString:"\r"];
    [adapter runLoopFor:0.2];
    return ctx.frames;
}

/* Frames heard in a short receive-only listen (silent monitor, no filters). */
static int listen_for_broadcast(OBDAdapter *adapter, double seconds) {
    bool stnDetected = false;
    if (!setup_passive_can(adapter, NULL, &stnDetected)) return -1;
    return run_monitor_phase(adapter, stnDetected ? "STMA" : "ATMA", NULL, false, seconds);
}

static bool run_can_phase(OBDAdapter *adapter,
                          vsp_decoder_t *dec,
                          vsp_probe_stats_t *stats,
                          double seconds,
                          NSMutableDictionary *phaseOut) {
    if (count_kind(dec, VSP_SIGNAL_CAN_FRAME) == 0) return true;

    bool stnDetected = false;
    if (!setup_passive_can(adapter, dec, &stnDetected)) return false;

    NSString *monitorCmd = stnDetected ? @"STMA" : @"ATMA";
    phaseOut[@"name"] = @"can";
    phaseOut[@"strategy"] = @"passive_can_monitor";
    phaseOut[@"monitor_command"] = monitorCmd;
    phaseOut[@"stn_detected"] = @(stnDetected);

    int64_t startMs = now_ms();
    int frames = run_monitor_phase(adapter, monitorCmd.UTF8String, dec, false, seconds);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"frames_received"] = @(frames);
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (vsp_decoder_signal_info(dec, i)->kind == VSP_SIGNAL_CAN_FRAME) {
            stats[i].test_duration_s = phaseDuration;
        }
    }
    return true;
}

/* First diagnostic signal of each request (query_key), in signal order.
 * Returns a malloc'd index array (count in *countOut). */
static int *unique_query_items(const vsp_decoder_t *dec, int *countOut) {
    int total = vsp_decoder_signal_count(dec);
    int *unique = (int *)calloc((size_t)total + 1, sizeof(int));
    int n = 0;
    for (int i = 0; unique && i < total; i++) {
        const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, i);
        if (item->kind != VSP_SIGNAL_DIAGNOSTIC_QUERY) continue;
        bool seen = false;
        for (int j = 0; j < n && !seen; j++) {
            seen = strcmp(vsp_decoder_signal_info(dec, unique[j])->query_key, item->query_key) == 0;
        }
        if (!seen) unique[n++] = i;
    }
    *countOut = n;
    return unique;
}

/* diagnostic_queries[] with role flow_control for the tx/rx of a scheduled
 * request, in profile order. Returns how many were written to out. */
static int flow_control_queries_for_items(const vsp_profile_t *profile,
                                          const vsp_decoder_t *dec,
                                          const int *queryItems,
                                          int queryCount,
                                          vsp_query_info_t *out) {
    vsp_profile_info_t pinfo;
    vsp_profile_get_info(profile, &pinfo);

    int n = 0;
    for (int qi = 0; qi < pinfo.query_count; qi++) {
        vsp_query_info_t q;
        if (!vsp_profile_query(profile, qi, &q)) continue;
        if (!q.role || strcmp(q.role, "flow_control") != 0) {
            continue;
        }

        uint32_t tx = q.tx_id;
        uint32_t rx = q.rx_id;
        for (int i = 0; i < queryCount; i++) {
            const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, queryItems[i]);
            if (item->tx_id == tx && (rx == 0 || item->rx_id == rx)) {
                out[n++] = q;
                break;
            }
        }
    }
    return n;
}

typedef struct {
    uint32_t tx;
    char payload[256];
} vsp_raw_schedule_t;

/* Schedule payload: the query's payload_hex, else an ISO-TP single frame. */
static bool schedule_payload(const char *payloadHex, const char *requestHex, uint8_t filler,
                             char *out, size_t outLen) {
    char single[64];
    if (!payloadHex || !payloadHex[0]) {
        if (!vsp_single_frame_payload_hex(requestHex, filler, single, sizeof(single))) return false;
        payloadHex = single;
    }
    vsp_hex_strip(payloadHex, out, outLen);
    return true;
}

static bool setup_periodic_obd(OBDAdapter *adapter,
                               const vsp_profile_t *profile,
                               const vsp_decoder_t *dec,
                               double targetHz,
                               VSPProbeOBDStrategy strategy,
                               NSString **strategyNameOut) {
    stn2255_info_t stn = {0};
    bool stnDetected = false;
    init_basic_adapter(adapter, &stn, &stnDetected);

    NSString *deviceName = [adapter.connectedDeviceName lowercaseString];
    bool useMic = [deviceName containsString:@"vlinker"];
    /* vLinker monitoring mode is receive-only; the manufacturer confirmed
       ATMA/STM/STMA keep the bus silent and do not transmit PM messages. */
    if (useMic) {
        if (strategy == VSPProbeOBDStrategyPeriodic) {
            fprintf(stderr, "[ERROR] vLinker periodic OBD is unsupported: "
                    "monitoring mode disables PM transmissions\n");
        }
        return false;
    }
    if (strategy == VSPProbeOBDStrategyPeriodic && !stnDetected) {
        fprintf(stderr, "[ERROR] Periodic OBD requires STN/OBDLink periodic-message support\n");
        return false;
    }
    if (strategy == VSPProbeOBDStrategyAuto && !stnDetected) {
        return false;
    }

    int periodMs = (int)llround(1000.0 / targetHz);
    if (periodMs < 20) periodMs = 20;

    int queryCount = 0;
    int *queries = unique_query_items(dec, &queryCount);
    vsp_profile_info_t pinfo;
    vsp_profile_get_info(profile, &pinfo);
    vsp_query_info_t *flowControls =
        (vsp_query_info_t *)calloc((size_t)pinfo.query_count + 1, sizeof(*flowControls));
    vsp_raw_schedule_t *rawSchedules =
        (vsp_raw_schedule_t *)calloc((size_t)(queryCount + pinfo.query_count) + 1, sizeof(*rawSchedules));
    if (!queries || !flowControls || !rawSchedules) {
        free(queries);
        free(flowControls);
        free(rawSchedules);
        return false;
    }
    int flowCount = flow_control_queries_for_items(profile, dec, queries, queryCount, flowControls);
    int scheduleCount = 0;
    bool payloadsOk = true;
    for (int i = 0; i < queryCount; i++) {
        const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, queries[i]);
        vsp_raw_schedule_t *sched = &rawSchedules[scheduleCount];
        if (!schedule_payload(item->query_payload_hex, item->request_hex, 0x55,
                              sched->payload, sizeof(sched->payload))) {
            payloadsOk = false;
            break;
        }
        sched->tx = item->tx_id;
        scheduleCount++;
    }
    for (int i = 0; payloadsOk && i < flowCount; i++) {
        const vsp_query_info_t *q = &flowControls[i];
        vsp_raw_schedule_t *sched = &rawSchedules[scheduleCount];
        if (q->has_tx_id &&
            schedule_payload(q->payload_hex, q->request_hex, 0xAA,
                             sched->payload, sizeof(sched->payload))) {
            sched->tx = q->tx_id;
            scheduleCount++;
        }
    }
    free(queries);
    free(flowControls);
    if (!payloadsOk) {
        free(rawSchedules);
        return false;
    }
    send_best_effort(adapter, "ATH1");
    send_best_effort(adapter, "ATS1");

    if (!useMic && stnDetected) {
        *strategyNameOut = @"periodic_stn_raw";
        if (!send_required(adapter, "STPPMC") ||
            !send_required(adapter, "STCFCPC") ||
            !send_required(adapter, "STP 31") ||
            !send_required(adapter, "STPO") ||
            !send_required(adapter, "ATCSM0") ||
            !send_required(adapter, "ATH1") ||
            !send_required(adapter, "ATCAF0") ||
            !send_required(adapter, "STCMM 1") ||
            !send_required(adapter, "STCSEGR 0") ||
            !send_required(adapter, "STCSEGT 0") ||
            !send_required(adapter, "STFAC")) {
            cleanup_periodic_obd(adapter, *strategyNameOut);
            free(rawSchedules);
            return false;
        } else {
            for (NSNumber *n in unique_can_ids_for_items(dec, true)) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "STFAP %03X,7FF", n.unsignedIntValue);
                send_best_effort(adapter, cmd);
            }

            bool scheduled = true;
            for (int i = 0; i < scheduleCount; i++) {
                char cmd[128];
                snprintf(cmd, sizeof(cmd), "STPPMA %d, %03X, %s",
                         periodMs,
                         rawSchedules[i].tx,
                         rawSchedules[i].payload);
                if (!send_required(adapter, cmd)) {
                    scheduled = false;
                    break;
                }
                [adapter runLoopFor:(double)periodMs / 1000.0 / MAX(scheduleCount, 1)];
            }
            free(rawSchedules);
            if (scheduled) return true;
            cleanup_periodic_obd(adapter, *strategyNameOut);
            return false;
        }
    }

    free(rawSchedules);
    return false;
}

static void cleanup_periodic_obd(OBDAdapter *adapter, NSString *strategyName) {
    if ([strategyName isEqualToString:@"periodic_stn_raw"]) {
        send_best_effort(adapter, "STPPMC");
    }
    send_best_effort(adapter, "STCMM 0");
}

static bool poll_query_once(OBDAdapter *adapter,
                            vsp_decoder_t *dec,
                            vsp_probe_stats_t *stats,
                            int queryIdx,
                            double timeoutSeconds) {
    const vsp_signal_info_t *queryItem = vsp_decoder_signal_info(dec, queryIdx);
    int total = vsp_decoder_signal_count(dec);
    for (int i = 0; i < total; i++) {
        const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, i);
        if (item->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY &&
            strcmp(item->query_key, queryItem->query_key) == 0) stats[i].attempts += 1;
    }

    /* common/obd_poll: ATSH, ATFCSH, ATCRA, then the request */
    static can_frame_t frames[1024];
    int frameCount = obd_poll_query(adapter, queryItem->tx_id, queryItem->rx_id,
                                    queryItem->request_bare_hex, timeoutSeconds,
                                    frames, (int)(sizeof(frames) / sizeof(frames[0])));
    if (frameCount < 0) {
        for (int i = 0; i < total; i++) {
            const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, i);
            if (item->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY &&
                strcmp(item->query_key, queryItem->query_key) == 0) stats[i].errors += 1;
        }
        return false;
    }

    /* With ATH1 every response line is one CAN frame "ID PCI DATA..."; decode each
     * frame like the monitor path does instead of the reassembled payload. */
    double ts = (double)now_ms();
    for (int i = 0; i < frameCount; i++) {
        vsp_decoder_feed_diagnostic(dec, &frames[i], ts, queryItem->query_key);
    }
    return frameCount > 0;
}

static void set_diagnostic_duration(const vsp_decoder_t *dec, vsp_probe_stats_t *stats,
                                    double phaseDuration) {
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (vsp_decoder_signal_info(dec, i)->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY) {
            stats[i].test_duration_s = phaseDuration;
        }
    }
}

static bool run_poll_obd_phase(OBDAdapter *adapter,
                               vsp_decoder_t *dec,
                               vsp_probe_stats_t *stats,
                               double targetHz,
                               double seconds,
                               int timeoutMs,
                               NSMutableDictionary *phaseOut) {
    if (!obd_poll_setup(adapter)) return false;

    int queryCount = 0;
    int *queries = unique_query_items(dec, &queryCount);
    if (!queries) return false;
    phaseOut[@"name"] = @"obd";
    phaseOut[@"strategy"] = @"poll";
    phaseOut[@"query_count"] = @((NSUInteger)queryCount);

    double interval = 1.0 / targetHz;
    double timeout = (double)timeoutMs / 1000.0;
    int64_t startMs = now_ms();
    NSDate *end = [NSDate dateWithTimeIntervalSinceNow:seconds];

    while (!g_interrupted && adapter.isConnected &&
           [[NSDate date] compare:end] == NSOrderedAscending) {
        int64_t cycleStart = now_ms();
        for (int q = 0; q < queryCount; q++) {
            if (g_interrupted) break;
            poll_query_once(adapter, dec, stats, queries[q], timeout);
        }
        double elapsed = (double)(now_ms() - cycleStart) / 1000.0;
        double sleepS = interval - elapsed;
        if (sleepS > 0.0) [adapter runLoopFor:sleepS];
    }
    free(queries);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    set_diagnostic_duration(dec, stats, phaseDuration);
    return true;
}

static bool run_periodic_obd_phase(OBDAdapter *adapter,
                                   const vsp_profile_t *profile,
                                   vsp_decoder_t *dec,
                                   vsp_probe_stats_t *stats,
                                   double targetHz,
                                   double seconds,
                                   VSPProbeOBDStrategy strategy,
                                   NSMutableDictionary *phaseOut) {
    NSString *strategyName = nil;
    if (!setup_periodic_obd(adapter, profile, dec, targetHz, strategy, &strategyName)) {
        return false;
    }

    int queryCount = 0;
    free(unique_query_items(dec, &queryCount));
    phaseOut[@"name"] = @"obd";
    phaseOut[@"strategy"] = strategyName ?: @"periodic";
    phaseOut[@"query_count"] = @((NSUInteger)queryCount);
    phaseOut[@"monitor_command"] = @"STM";

    int64_t startMs = now_ms();
    run_monitor_phase(adapter, "STM", dec, true, seconds);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    int scheduledAttempts = (int)floor(targetHz * phaseDuration);
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (vsp_decoder_signal_info(dec, i)->kind != VSP_SIGNAL_DIAGNOSTIC_QUERY) continue;
        stats[i].test_duration_s = phaseDuration;
        stats[i].attempts = scheduledAttempts;
    }
    cleanup_periodic_obd(adapter, strategyName);
    return true;
}

static bool run_obd_phase(OBDAdapter *adapter,
                          const vsp_profile_t *profile,
                          vsp_decoder_t *dec,
                          vsp_probe_stats_t *stats,
                          const vsp_probe_options_t *opts,
                          NSMutableDictionary *phaseOut) {
    if (count_kind(dec, VSP_SIGNAL_DIAGNOSTIC_QUERY) == 0) return true;

    if (opts->obd_strategy != VSPProbeOBDStrategyPoll) {
        if (run_periodic_obd_phase(adapter, profile, dec, stats,
                                   opts->target_hz, opts->duration_s,
                                   opts->obd_strategy, phaseOut)) {
            return true;
        }
        if (opts->obd_strategy == VSPProbeOBDStrategyPeriodic) return false;
        fprintf(stderr, "[WARN] Periodic OBD unavailable; falling back to poll mode\n");
    }
    return run_poll_obd_phase(adapter, dec, stats, opts->target_hz, opts->duration_s,
                              opts->timeout_ms, phaseOut);
}

/* CAN phase from a listen-only PCAN channel: the whole bus, decoded in software. */
static bool run_can_phase_pcan(frame_source_t *src,
                               vsp_decoder_t *dec,
                               vsp_probe_stats_t *stats,
                               double seconds,
                               NSMutableDictionary *phaseOut) {
    const pcan_channel_t *ch = frame_source_pcan_channel(src);
    phaseOut[@"name"] = @"can";
    phaseOut[@"strategy"] = @"pcan_listen_only";
    phaseOut[@"interface"] = @"pcan";
    phaseOut[@"library_version"] = ns_string(pcan_api_version());
    phaseOut[@"bitrate"] = @(ch ? ch->bitrate : 0);
    phaseOut[@"timestamps"] = @"hardware";

    vsp_monitor_context_t ctx = { .decoder = dec, .diagnostic = false, .frames = 0 };
    int64_t startMs = now_ms();
    frame_source_start(src, monitor_callback, &ctx);
    int64_t endUs = pcan_mono_us() + (int64_t)(seconds * 1e6);
    while (!g_interrupted && frame_source_is_open(src)) {
        int64_t left = endUs - pcan_mono_us();
        if (left <= 0) break;
        frame_source_pump(src, left < 50000 ? (double)left / 1e6 : 0.05);
    }
    frame_source_stop(src);
    bool open = frame_source_is_open(src);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"frames_received"] = @(ctx.frames);
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    if (!open) phaseOut[@"device_lost"] = @YES;
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (vsp_decoder_signal_info(dec, i)->kind == VSP_SIGNAL_CAN_FRAME) {
            stats[i].test_duration_s = phaseDuration;
        }
    }
    return open;
}

static bool probe_cancelled(void) {
    return g_interrupted != 0;
}

static NSDictionary *plan_for_item(const vsp_decoder_t *dec, int idx) {
    const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, idx);
    NSMutableDictionary *d = [NSMutableDictionary dictionary];
    d[@"name"] = ns_string(item->name);
    d[@"canonical_id"] = ns_string(item->canonical_id);
    d[@"group"] = ns_string(item->group);
    d[@"acquisition_type"] = ns_string(item->acquisition_type);

    /* JSON values as written in the profile (unit and query id may be null). */
    NSDictionary *decode = vsp_decoder_signal_json(dec, idx)[@"decode"];
    if ([decode isKindOfClass:[NSDictionary class]]) {
        d[@"unit"] = decode[@"unit"] ?: @"";
    }

    if (item->kind == VSP_SIGNAL_CAN_FRAME) {
        d[@"frame_id"] = [NSString stringWithFormat:@"0x%03X", item->frame_id];
        if (item->cycle_ms > 0.0) {
            d[@"frame_cycle_ms"] = @(item->cycle_ms);
            d[@"frame_hz"] = @(((double)llround(1000.0 / item->cycle_ms * 1000.0)) / 1000.0);
        }
    } else {
        d[@"query_id"] = vsp_decoder_query_json(dec, idx)[@"id"] ?: @"";
        d[@"tx_id"] = [NSString stringWithFormat:@"0x%03X", item->tx_id];
        if (item->rx_id) d[@"rx_id"] = [NSString stringWithFormat:@"0x%03X", item->rx_id];
        d[@"request_hex"] = ns_string(item->request_hex);
        if (item->positive_marker_hex) d[@"positive_marker_hex"] = ns_string(item->positive_marker_hex);
    }
    return d;
}

static NSDictionary *result_for_item(const vsp_decoder_t *dec,
                                     const vsp_probe_stats_t *stats,
                                     int idx,
                                     double targetHz,
                                     double durationS) {
    const vsp_signal_info_t *item = vsp_decoder_signal_info(dec, idx);
    const vsp_signal_state_t *state = vsp_decoder_signal_state(dec, idx);
    const vsp_probe_stats_t *stat = &stats[idx];
    NSMutableDictionary *d = [[plan_for_item(dec, idx) mutableCopy] autorelease];
    double itemDuration = stat->test_duration_s > 0.0 ? stat->test_duration_s : durationS;
    double observed = itemDuration > 0.0 ? (double)state->samples / itemDuration : 0.0;
    bool met = observed + (targetHz * 0.05) >= targetHz;

    d[@"target_hz"] = @(targetHz);
    d[@"observed_hz"] = @(((double)llround(observed * 1000.0)) / 1000.0);
    d[@"measured_duration_s"] = @(((double)llround(itemDuration * 1000.0)) / 1000.0);
    d[@"samples"] = @(state->samples);
    d[@"attempts"] = @(stat->attempts);
    d[@"errors"] = @(stat->errors);
    d[@"target_met"] = @(met);
    d[@"status"] = state->samples > 0 ? (met ? @"pass" : @"under_target") : @"no_samples";
    /* The profile's frame period (cycle_ms) tells a slow frame from lost frames. A muxed
     * signal sees only the frames of its mux value, so it is not compared. */
    if (item->cycle_ms > 0.0 && state->samples > 0 && !item->has_mux) {
        double frameHz = 1000.0 / item->cycle_ms;
        double ratio = observed / frameHz;
        d[@"frame_rate_ratio"] = @(((double)llround(ratio * 1000.0)) / 1000.0);
        if (ratio < 0.5) d[@"frame_rate_low"] = @YES;   /* frames lost, filtered or sent slower here */
        if (!met && ratio >= 0.9 && targetHz > frameHz * 1.05) d[@"status"] = @"frame_limited";
    }
    if (state->last_payload_len > 0) {
        char hex[2 * sizeof(state->last_payload) + 1];
        vsp_hex_format(state->last_payload, state->last_payload_len, hex, sizeof(hex));
        d[@"last_payload_hex"] = ns_string(hex);
    }
    if (state->has_value) d[@"last_value"] = @(state->value);
    else if (state->value_null) d[@"last_value"] = [NSNull null];
    if (state->label) d[@"last_label"] = ns_string(state->label);
    if (state->first_sample_ms > 0.0) d[@"first_sample_ms"] = @((long long)state->first_sample_ms);
    if (state->last_sample_ms > 0.0) d[@"last_sample_ms"] = @((long long)state->last_sample_ms);
    return d;
}

static NSString *mode_name(VSPProbeMode mode) {
    switch (mode) {
        case VSPProbeModeCAN: return @"can";
        case VSPProbeModeOBD: return @"obd";
        default: return @"auto";
    }
}

static NSDictionary *build_output(const vsp_profile_t *profile,
                                  const char *profilePath,
                                  const vsp_probe_options_t *opts,
                                  const vsp_decoder_t *dec,
                                  const vsp_probe_stats_t *stats,
                                  NSArray<NSDictionary *> *phases,
                                  bool planOnly,
                                  double actualDuration) {
    NSDictionary *json = vsp_profile_json(profile);
    NSMutableDictionary *out = [NSMutableDictionary dictionary];
    out[@"tool"] = @"vsp_probe";
    out[@"profile_path"] = [NSString stringWithUTF8String:profilePath];
    out[@"schema"] = json[@"schema"] ?: @"";
    out[@"vehicle"] = json[@"vehicle"] ?: @{};
    out[@"mode"] = mode_name(opts->mode);
    out[@"target_hz"] = @(opts->target_hz);
    out[@"duration_s"] = @(opts->duration_s);
    out[@"plan_only"] = @(planOnly);
    if (opts->device) out[@"device"] = [NSString stringWithUTF8String:opts->device];
    if (actualDuration > 0.0) out[@"actual_duration_s"] = @(((double)llround(actualDuration * 1000.0)) / 1000.0);
    if (phases) out[@"phases"] = phases;

    NSMutableArray *signalOut = [NSMutableArray array];
    int pass = 0;
    int under = 0;
    int limited = 0;
    int none = 0;
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        NSDictionary *d = planOnly ? plan_for_item(dec, i) :
                            result_for_item(dec, stats, i, opts->target_hz, actualDuration > 0.0 ? actualDuration : opts->duration_s);
        [signalOut addObject:d];
        if (!planOnly) {
            NSString *status = d[@"status"];
            if ([status isEqualToString:@"pass"]) pass++;
            else if ([status isEqualToString:@"under_target"]) under++;
            else if ([status isEqualToString:@"frame_limited"]) limited++;
            else none++;
        }
    }
    out[@"signals"] = signalOut;
    out[@"selected_count"] = @(signalOut.count);
    if (!planOnly) {
        out[@"summary"] = @{@"pass": @(pass), @"under_target": @(under), @"frame_limited": @(limited),
                            @"no_samples": @(none)};
    }
    return out;
}

static bool write_json(id obj, const char *path) {
    NSError *err = nil;
    NSData *data = [NSJSONSerialization dataWithJSONObject:obj
                                                   options:NSJSONWritingPrettyPrinted
                                                     error:&err];
    if (!data) {
        fprintf(stderr, "[ERROR] Failed to encode JSON: %s\n",
                err.localizedDescription.UTF8String ?: "unknown");
        return false;
    }
    if (path && *path) {
        NSString *p = [NSString stringWithUTF8String:path];
        if (![data writeToFile:p atomically:YES]) {
            fprintf(stderr, "[ERROR] Failed to write output: %s\n", path);
            return false;
        }
    } else {
        fwrite(data.bytes, 1, data.length, stdout);
        fputc('\n', stdout);
    }
    return true;
}

int cmd_probe_main(int argc, const char *argv[]) {
    @autoreleasepool {
        vsp_probe_options_t opts;
        if (!parse_options(argc, argv, &opts)) {
            usage();
            return 1;
        }
        bool pcan = opts.device && pcan_is_device(opts.device);
        if (pcan && !opts.plan_only && opts.mode != VSPProbeModeCAN) {
            /* auto may fall back to OBD queries; obd always sends them */
            cli_pcan_refuse(opts.mode == VSPProbeModeOBD ? "probe --mode obd" : "probe --mode auto");
            fprintf(stderr, "[ERROR] Use --mode can with a PCAN device\n");
            return 1;
        }
        pcan_bitrate_t pcanBitrate;
        if (opts.bitrate && !pcan) {
            fprintf(stderr, "[ERROR] --bitrate applies to PCAN devices only (pcan, pcan1..pcan8)\n");
            return 1;
        }
        if (pcan && !cli_pcan_parse_bitrate(opts.bitrate, &pcanBitrate)) return 1;

        char err[1024] = "";
        vsp_profile_t *profile = vsp_profile_load(opts.profile_path, err, sizeof(err));
        if (!profile) {
            fprintf(stderr, "[ERROR] %s\n", err);
            return 1;
        }

        vsp_select_t select = {
            .signals = opts.signals,
            .all_signals = false,
            .acquisition = acquisition_filter(opts.mode),
        };
        vsp_decoder_t *decoder = vsp_decoder_create(profile, &select);
        if (!decoder) {
            fprintf(stderr, "[ERROR] Out of memory\n");
            return 1;
        }
        for (int i = 0; i < vsp_decoder_skipped_count(decoder); i++) {
            const vsp_skipped_signal_t *sk = vsp_decoder_skipped(decoder, i);
            fprintf(stderr, "[INFO] %s: skipped, frame 0x%03X is a reassembled vendor stream\n",
                    sk->name, sk->frame_id);
        }
        int signalCount = vsp_decoder_signal_count(decoder);
        if (signalCount == 0) {
            fprintf(stderr, "[ERROR] No VSP signals selected for requested mode/filter\n");
            return 1;
        }
        vsp_probe_stats_t *stats = (vsp_probe_stats_t *)calloc((size_t)signalCount, sizeof(*stats));
        if (!stats) {
            fprintf(stderr, "[ERROR] Out of memory\n");
            return 1;
        }

        if (opts.plan_only) {
            NSDictionary *out = build_output(profile, opts.profile_path, &opts, decoder, stats,
                                             nil, true, 0.0);
            return write_json(out, opts.out_path) ? 0 : 1;
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        if (pcan) {
            /* listen-only CAN phase; no Bluetooth object is created */
            pcan_channel_t ch;
            if (!cli_pcan_open(opts.device, &pcanBitrate, probe_cancelled, &ch, NULL)) return 1;
            frame_source_t *src = frame_source_pcan_create(&ch);
            if (!src) {
                pcan_close(&ch);
                fprintf(stderr, "[ERROR] Out of memory\n");
                return 1;
            }
            NSMutableArray<NSDictionary *> *phases = [NSMutableArray array];
            int64_t startMs = now_ms();
            NSMutableDictionary *phase = [NSMutableDictionary dictionary];
            bool ok = run_can_phase_pcan(src, decoder, stats, opts.duration_s, phase);
            [phases addObject:phase];
            int64_t endMs = now_ms();
            frame_source_free(src);   /* CAN_Uninitialize */
            double actualDuration = (double)(endMs - startMs) / 1000.0;
            NSMutableDictionary *out = [[build_output(profile, opts.profile_path, &opts, decoder, stats,
                                                      phases, false, actualDuration) mutableCopy] autorelease];
            out[@"interface"] = @"pcan";
            bool written = write_json(out, opts.out_path);
            if (!written) return 1;
            return ok && !g_interrupted ? 0 : 1;
        }

        OBDAdapter *adapter = [[OBDAdapter alloc] initWithNameHints:[OBDAdapter defaultNameHints]
                                                              quiet:NO];
        obd_link_t link;
        obd_link_init(&link);
        if (!obd_connect_device(adapter, opts.device, &link)) {
            [adapter release];
            return 1;
        }

        int canCount = count_kind(decoder, VSP_SIGNAL_CAN_FRAME);
        bool obdSelected = count_kind(decoder, VSP_SIGNAL_DIAGNOSTIC_QUERY) > 0;

        NSMutableArray<NSDictionary *> *phases = [NSMutableArray array];
        int64_t startMs = now_ms();
        bool ok = true;

        int broadcastFrames = -1;
        if (canCount > 0 && (opts.mode == VSPProbeModeAuto || opts.mode == VSPProbeModeCAN)) {
            NSMutableDictionary *phase = [NSMutableDictionary dictionary];
            ok = run_can_phase(adapter, decoder, stats, opts.duration_s, phase);
            broadcastFrames = [phase[@"frames_received"] intValue];
            [phases addObject:phase];
        }
        /* Auto mode never transmits next to broadcast traffic: that may be a direct
         * tap of a vehicle bus (PT-CAN), where OBD requests must not be sent. On an
         * OBD port with broadcast, ask for the OBD phase with --mode obd. */
        if (ok && !g_interrupted && obdSelected && opts.mode == VSPProbeModeAuto) {
            /* The CAN phase only passes the profile's IDs; listen unfiltered when it heard none. */
            if (broadcastFrames <= 0) broadcastFrames = listen_for_broadcast(adapter, 1.5);
            if (broadcastFrames != 0) {
                fprintf(stderr, "[INFO] OBD phase skipped: %d broadcast frames heard (possible direct bus tap); "
                                "use --mode obd on an OBD port\n", broadcastFrames);
                [phases addObject:@{ @"name": @"obd", @"skipped": @YES,
                                     @"reason": @"broadcast traffic heard; auto mode does not transmit on a possible direct bus tap",
                                     @"broadcast_frames": @(broadcastFrames) }];
                obdSelected = false;
            }
        }
        if (ok && !g_interrupted &&
            obdSelected && (opts.mode == VSPProbeModeAuto || opts.mode == VSPProbeModeOBD)) {
            NSMutableDictionary *phase = [NSMutableDictionary dictionary];
            ok = run_obd_phase(adapter, profile, decoder, stats, &opts, phase);
            [phases addObject:phase];
        }

        int64_t endMs = now_ms();
        [adapter disconnect];

        double actualDuration = (double)(endMs - startMs) / 1000.0;
        NSDictionary *out = build_output(profile, opts.profile_path, &opts,
                                         decoder, stats, phases, false, actualDuration);
        bool written = write_json(out, opts.out_path);
        [adapter release];
        if (!written) return 1;
        return ok && !g_interrupted ? 0 : 1;
    }
}
