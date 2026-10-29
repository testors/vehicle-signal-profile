/**
 * @file discover_session.m
 * @brief Guided PT-CAN discover session + interactive CLI driver
 *
 * Moved from can_discover/main.m and can_reader/main.m, which carried
 * identical copies (run_discover / cli_discover and their helpers).
 * Objective-C only for NSFileManager and the run loop pump while waiting for
 * stdin; compiled without ARC like the rest of the project.
 */

#import <Foundation/Foundation.h>

#include "discover_session.h"
#include "cli_util.h"
#include "frame_tsv.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

/*******************************************************************************
 * Phase table
 ******************************************************************************/

/** Phase configuration */
static const struct {
    const char *name;
    int         duration_s;
    const char *instruction;
    const char *warning;        /**< NULL if no special warning */
} s_disc_phases[DISC_NUM_PHASES] = {
    { "Baseline",     5,
      "Keep engine running. Do NOT touch any controls.", NULL },
    { "Steering",     8,
      "Turn steering wheel to FULL LEFT, then FULL RIGHT (lock to lock). Repeat 2-3 times.",
      NULL },
    { "Throttle",     8,
      "Press accelerator pedal ALL THE WAY down, then release. Repeat 2-3 times.",
      NULL },
    { "Brake",        8,
      "Press brake pedal FIRMLY all the way down, then release. Repeat 2-3 times.",
      NULL },
    { "Gear",        10,
      "Shift through P -> R -> N -> D (hold brake, wait 1s per gear).",
      "Manual transmission vehicles may not broadcast gear position." },
    { "Wheel Speed", 15,
      "Drive slowly (30+ km/h) — include a turn if possible, then stop.",
      "Requires driving. ABS/ESC may be on a separate CAN bus." },
};

static const char *disc_phase_slug(int phase_idx) {
    static const char *kPhaseSlugs[DISC_NUM_PHASES] = {
        "baseline", "steering", "throttle", "brake", "gear", "wheel_speed"
    };
    if (phase_idx < 0 || phase_idx >= DISC_NUM_PHASES) return "phase";
    return kPhaseSlugs[phase_idx];
}

static const char *disc_phase_display_name(int phase_idx) {
    static const char *kPhaseNames[DISC_NUM_PHASES] = {
        "Baseline", "Steering", "Throttle", "Brake", "Gear", "Wheel Speed"
    };
    if (phase_idx < 0 || phase_idx >= DISC_NUM_PHASES) return "Phase";
    return kPhaseNames[phase_idx];
}

int disc_session_phase_count(void) {
    return DISC_NUM_PHASES;
}

const char *disc_session_phase_name(int phase_idx) {
    return disc_phase_display_name(phase_idx);
}

const char *disc_session_phase_slug(int phase_idx) {
    return disc_phase_slug(phase_idx);
}

const char *disc_session_phase_instruction(int phase_idx) {
    if (phase_idx < 0 || phase_idx >= DISC_NUM_PHASES) return "";
    return s_disc_phases[phase_idx].instruction;
}

const char *disc_session_phase_warning(int phase_idx) {
    if (phase_idx < 0 || phase_idx >= DISC_NUM_PHASES) return NULL;
    return s_disc_phases[phase_idx].warning;
}

int disc_session_phase_duration_s(int phase_idx) {
    if (phase_idx < 0 || phase_idx >= DISC_NUM_PHASES) return 0;
    return s_disc_phases[phase_idx].duration_s;
}

bool disc_session_phase_skippable(int phase_idx) {
    return phase_idx > DISC_PHASE_BASELINE && phase_idx < DISC_NUM_PHASES;
}

/*******************************************************************************
 * Session state
 ******************************************************************************/

struct disc_session {
    char *device_prefix;
    char *connected_name;
    char *monitor_cmd;
    disc_session_log_fn log;
    void *log_ctx;

    char dump_dir[DISC_PATH_MAX];
    char dbc_path[DISC_PATH_MAX];
    char vsp_path[DISC_PATH_MAX];
    char result_path[DISC_PATH_MAX];
    char manifest_path[DISC_PATH_MAX];
    bool dump_enabled;
    bool hw_timestamps;         /**< dumps carry the timestamp_us column */
    bool have_bus_mode;
    disc_bus_mode_t bus_mode;

    bool finished;
    bool dbc_written;
    bool vsp_written;
    bool manifest_written;

    disc_phase_t phases[DISC_NUM_PHASES];
    disc_raw_phase_t raw_phases[DISC_NUM_PHASES];
    disc_phase_dump_meta_t phase_meta[DISC_NUM_PHASES];
    disc_result_t result;

    frame_buffer_t capture;     /**< Frames of the phase being captured */
    int            active_phase;    /**< Phase being captured, -1 if none */
    int64_t        active_start_ms; /**< Source clock at disc_session_phase_begin */
};

static void session_log(const disc_session_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void session_log(const disc_session_t *s, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    if (s && s->log) {
        char text[2048];
        vsnprintf(text, sizeof(text), fmt, ap);
        s->log(text, s->log_ctx);
    } else {
        vfprintf(stderr, fmt, ap);
    }
    va_end(ap);
}

static char *dup_or_null(const char *s) {
    return s ? strdup(s) : NULL;
}

/*******************************************************************************
 * Session files
 ******************************************************************************/

static void disc_join_path(char *buf, size_t len,
                           const char *dir, const char *leaf) {
    if (!buf || len == 0) return;
    if (!dir || !dir[0]) {
        snprintf(buf, len, "%s", leaf ? leaf : "");
        return;
    }

    size_t dir_len = strlen(dir);
    if (dir_len > 0 && dir[dir_len - 1] == '/') {
        snprintf(buf, len, "%s%s", dir, leaf);
    } else {
        snprintf(buf, len, "%s/%s", dir, leaf);
    }
}

static void disc_make_session_dir_path(char *buf, size_t len,
                                       const char *dump_dir_override) {
    if (dump_dir_override && dump_dir_override[0]) {
        snprintf(buf, len, "%s", dump_dir_override);
        return;
    }

    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char stamp[32];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
    snprintf(buf, len, "captures/%s", stamp);
}

static bool disc_ensure_dir(const disc_session_t *s, const char *path) {
    if (!path || !path[0]) return false;

    NSFileManager *fm = [NSFileManager defaultManager];
    NSString *ns_path = [NSString stringWithUTF8String:path];
    NSError *error = nil;
    BOOL ok = [fm createDirectoryAtPath:ns_path
            withIntermediateDirectories:YES
                             attributes:nil
                                  error:&error];
    if (!ok) {
        session_log(s, "[WARN] Failed to create dump dir %s: %s\n",
                path, error.localizedDescription.UTF8String);
    }
    return ok == YES;
}

/** Draft output path: <dump_dir>/ptcan_discover_draft.<ext>, or
 *  ptcan_discover_draft_YYYYMMDD_HHMMSS.<ext> in the cwd without a dump dir. */
static void disc_make_draft_path(char *buf, size_t len,
                                 const char *dump_dir, const char *ext) {
    if (dump_dir && dump_dir[0]) {
        char leaf[64];
        snprintf(leaf, sizeof(leaf), "ptcan_discover_draft.%s", ext);
        disc_join_path(buf, len, dump_dir, leaf);
        return;
    }

    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char stamp[32];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
    snprintf(buf, len, "ptcan_discover_draft_%s.%s", stamp, ext);
}

static void disc_make_result_json_path(char *buf, size_t len, const char *dump_dir) {
    if (dump_dir && dump_dir[0]) {
        disc_join_path(buf, len, dump_dir, "result.json");
    } else {
        snprintf(buf, len, "result.json");
    }
}

static void disc_make_manifest_path(char *buf, size_t len, const char *dump_dir) {
    if (dump_dir && dump_dir[0]) {
        disc_join_path(buf, len, dump_dir, "manifest.json");
    } else {
        snprintf(buf, len, "manifest.json");
    }
}

static void disc_make_phase_dump_filename(char *buf, size_t len, int phase_idx) {
    snprintf(buf, len, "%02d_%s.frames.tsv", phase_idx, disc_phase_slug(phase_idx));
}

static void disc_make_phase_dump_path(char *buf, size_t len,
                                      const char *dump_dir, int phase_idx) {
    char leaf[64];
    disc_make_phase_dump_filename(leaf, sizeof(leaf), phase_idx);
    disc_join_path(buf, len, dump_dir, leaf);
}

static bool disc_write_phase_dump(const char *path, const disc_raw_phase_t *raw_phase, bool hw_timestamps) {
    if (!path || !raw_phase) return false;

    FILE *f = fopen(path, "w");
    if (!f) return false;

    /* a replay of a 6-column capture keeps its timestamp_us column too */
    for (int i = 0; i < raw_phase->frame_count && !hw_timestamps; i++) {
        hw_timestamps = raw_phase->frames[i].hw_timestamp;
    }
    frame_tsv_write_header_ex(f, hw_timestamps);
    for (int i = 0; i < raw_phase->frame_count; i++) {
        frame_tsv_write_row(f, &raw_phase->frames[i]);
    }

    fclose(f);
    return true;
}

static void disc_write_result_json_object(FILE *f,
                                          bool have_bus_mode,
                                          disc_bus_mode_t bus_mode,
                                          const char *dump_dir,
                                          const char *dbc_path,
                                          const char *vsp_path,
                                          const disc_result_t *result) {
    bool first = true;

    fprintf(f, "{\n");

    if (have_bus_mode) {
        fprintf(f, "  \"bus_mode\":");
        json_print_string(f, disc_bus_mode_name(bus_mode));
        first = false;
    }

    if (dump_dir && dump_dir[0]) {
        fprintf(f, "%s  \"dump_dir\":", first ? "" : ",\n");
        json_print_string(f, dump_dir);
        first = false;
    }

    if (dbc_path && dbc_path[0]) {
        fprintf(f, "%s  \"dbc_path\":", first ? "" : ",\n");
        json_print_string(f, dbc_path);
        first = false;
    }

    if (vsp_path && vsp_path[0]) {
        fprintf(f, "%s  \"vsp_path\":", first ? "" : ",\n");
        json_print_string(f, vsp_path);
        first = false;
    }

    for (int s = 0; result && s < DISC_SIG_COUNT; s++) {
        if (!result->found[s]) continue;

        const disc_candidate_t *c = &result->signals[s];
        fprintf(f, "%s  ", first ? "" : ",\n");
        json_print_string(f, disc_signal_name((disc_signal_t)s));
        fprintf(f, ":{\"can_id\":\"0x%03X\",\"dlc\":%d,\"hz\":%.1f",
                c->can_id, c->dlc, c->hz);
        if (c->byte_idx >= 0) fprintf(f, ",\"byte\":%d", c->byte_idx);
        if (c->byte2_idx >= 0) fprintf(f, ",\"byte2\":%d", c->byte2_idx);
        if (c->start_bit >= 0 && c->bit_length > 0) {
            fprintf(f, ",\"start_bit\":%d,\"bit_length\":%d,\"byte_order\":%d",
                    c->start_bit, c->bit_length, c->byte_order);
        }
        fprintf(f, ",\"score\":%.1f,\"confidence\":\"%s\"",
                c->score, disc_confidence_name(c->confidence));
        fprintf(f, ",\"evidence\":{\"phase\":%.1f,\"specificity\":%.1f,"
                   "\"shape\":%.1f,\"prior\":%.1f,\"prior_support\":%d,"
                   "\"ambiguity_margin\":%.1f}",
                c->evidence_score, c->specificity_score, c->shape_score,
                c->prior_score, c->prior_support, c->ambiguity_margin);
        if (c->byte2_idx >= 0 && c->endianness != DISC_ENDIAN_UNKNOWN) {
            fprintf(f, ",\"endian\":\"%s\"", disc_endian_name(c->endianness));
        }
        if (c->signedness != DISC_SIGN_UNKNOWN) {
            fprintf(f, ",\"signed\":%s",
                    c->signedness == DISC_SIGN_SIGNED ? "true" : "false");
            if (c->sign_char == 'S') fprintf(f, ",\"sign_bit\":%d", c->sign_bit);
            fprintf(f, ",\"raw_min\":%d,\"raw_max\":%d", c->raw_min, c->raw_max);
        }
        fprintf(f, "}");
        first = false;
    }

    fprintf(f, "\n}\n");
}

static bool disc_write_result_json(const char *path,
                                   bool have_bus_mode,
                                   disc_bus_mode_t bus_mode,
                                   const char *dump_dir,
                                   const char *dbc_path,
                                   const char *vsp_path,
                                   const disc_result_t *result) {
    if (!path) return false;

    FILE *f = fopen(path, "w");
    if (!f) return false;
    disc_write_result_json_object(f, have_bus_mode, bus_mode, dump_dir,
                                  dbc_path, vsp_path, result);
    fclose(f);
    return true;
}

static bool disc_write_manifest(const char *path,
                                const char *status,
                                const char *device_prefix,
                                const char *connected_name,
                                const char *monitor_cmd,
                                bool have_bus_mode,
                                disc_bus_mode_t bus_mode,
                                const char *dbc_path,
                                const char *vsp_path,
                                const char *result_path,
                                const disc_phase_dump_meta_t metas[DISC_NUM_PHASES]) {
    if (!path || !status || !metas) return false;

    FILE *f = fopen(path, "w");
    if (!f) return false;

    fprintf(f, "{\n");
    fprintf(f, "  \"format\":");
    json_print_string(f, "can_discover_dump/v1");
    fprintf(f, ",\n  \"status\":");
    json_print_string(f, status);
    fprintf(f, ",\n  \"device_prefix\":");
    if (device_prefix && device_prefix[0]) json_print_string(f, device_prefix);
    else fputs("null", f);
    fprintf(f, ",\n  \"connected_device\":");
    if (connected_name && connected_name[0]) json_print_string(f, connected_name);
    else fputs("null", f);
    fprintf(f, ",\n  \"monitor_cmd\":");
    json_print_string(f, monitor_cmd ? monitor_cmd : "");
    fprintf(f, ",\n  \"bus_mode\":");
    if (have_bus_mode) json_print_string(f, disc_bus_mode_name(bus_mode));
    else fputs("null", f);
    fprintf(f, ",\n  \"dbc_path\":");
    if (dbc_path && dbc_path[0]) json_print_string(f, dbc_path);
    else fputs("null", f);
    fprintf(f, ",\n  \"vsp_path\":");
    if (vsp_path && vsp_path[0]) json_print_string(f, vsp_path);
    else fputs("null", f);
    fprintf(f, ",\n  \"result_path\":");
    if (result_path && result_path[0]) json_print_string(f, result_path);
    else fputs("null", f);
    fprintf(f, ",\n  \"phases\":[\n");

    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        const disc_phase_dump_meta_t *meta = &metas[i];
        if (i > 0) fprintf(f, ",\n");
        fprintf(f, "    {\"index\":%d,\"name\":", i);
        json_print_string(f, disc_phase_display_name(i));
        fprintf(f, ",\"slug\":");
        json_print_string(f, disc_phase_slug(i));
        fprintf(f, ",\"planned_duration_s\":%d", meta->planned_duration_s);
        fprintf(f, ",\"skipped\":%s", meta->skipped ? "true" : "false");
        fprintf(f, ",\"captured\":%s", meta->captured ? "true" : "false");
        fprintf(f, ",\"actual_duration_s\":%.3f", meta->actual_duration_s);
        fprintf(f, ",\"analyzed_duration_s\":%.3f", meta->analyzed_duration_s);
        fprintf(f, ",\"frame_count\":%d", meta->total_frames);
        fprintf(f, ",\"id_count\":%d", meta->id_count);
        fprintf(f, ",\"dropped_frames\":%d", meta->dropped_frames);
        fprintf(f, ",\"capture_dropped_frames\":%d", meta->capture_dropped_frames);
        fprintf(f, ",\"dump_file\":");
        if (meta->dump_file[0]) json_print_string(f, meta->dump_file);
        else fputs("null", f);
        fprintf(f, "}");
    }

    fprintf(f, "\n  ]\n}\n");
    fclose(f);
    return true;
}

static void disc_free_raw_phases(disc_raw_phase_t phases[DISC_NUM_PHASES]) {
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        disc_raw_phase_free(&phases[i]);
    }
}

static void disc_free_phase_stats(disc_phase_t phases[DISC_NUM_PHASES]) {
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        disc_phase_free(&phases[i]);
    }
}

/*******************************************************************************
 * Session layer
 ******************************************************************************/

static bool valid_phase(int phase_idx) {
    return phase_idx >= 0 && phase_idx < DISC_NUM_PHASES;
}

disc_session_t *disc_session_create(const disc_session_config_t *cfg) {
    disc_session_t *s = (disc_session_t *)calloc(1, sizeof(*s));
    if (!s) return NULL;

    if (cfg) {
        s->device_prefix = dup_or_null(cfg->device_prefix);
        s->connected_name = dup_or_null(cfg->connected_name);
        s->monitor_cmd = dup_or_null(cfg->monitor_cmd);
        s->log = cfg->log;
        s->log_ctx = cfg->log_ctx;
        s->hw_timestamps = cfg->hw_timestamps;
    }
    s->bus_mode = DISC_BUS_DIRECT;
    s->active_phase = -1;

    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        disc_phase_init(&s->phases[i]);
        disc_raw_phase_init(&s->raw_phases[i]);
        memset(&s->phase_meta[i], 0, sizeof(s->phase_meta[i]));
        s->phase_meta[i].planned_duration_s = s_disc_phases[i].duration_s;
    }

    disc_make_session_dir_path(s->dump_dir, sizeof(s->dump_dir),
                               cfg ? cfg->dump_dir_override : NULL);
    if (disc_ensure_dir(s, s->dump_dir)) {
        s->dump_enabled = true;
        session_log(s, "[INFO] Capture dump dir: %s\n", s->dump_dir);
        disc_make_manifest_path(s->manifest_path, sizeof(s->manifest_path), s->dump_dir);
        disc_make_result_json_path(s->result_path, sizeof(s->result_path), s->dump_dir);
    }

    disc_result_init(&s->result);
    return s;
}

void disc_session_free(disc_session_t *s) {
    if (!s) return;
    disc_free_raw_phases(s->raw_phases);
    disc_free_phase_stats(s->phases);
    frame_buffer_free(&s->capture);
    free(s->device_prefix);
    free(s->connected_name);
    free(s->monitor_cmd);
    free(s);
}

const char *disc_session_dump_dir(const disc_session_t *s) {
    return (s && s->dump_enabled) ? s->dump_dir : NULL;
}

const disc_phase_t *disc_session_phase(const disc_session_t *s, int phase_idx) {
    return (s && valid_phase(phase_idx)) ? &s->phases[phase_idx] : NULL;
}

const disc_raw_phase_t *disc_session_raw_phase(const disc_session_t *s, int phase_idx) {
    return (s && valid_phase(phase_idx)) ? &s->raw_phases[phase_idx] : NULL;
}

const disc_phase_dump_meta_t *disc_session_phase_meta(const disc_session_t *s, int phase_idx) {
    return (s && valid_phase(phase_idx)) ? &s->phase_meta[phase_idx] : NULL;
}

bool disc_session_have_bus_mode(const disc_session_t *s) {
    return s && s->have_bus_mode;
}

disc_bus_mode_t disc_session_bus_mode(const disc_session_t *s) {
    return s ? s->bus_mode : DISC_BUS_DIRECT;
}

bool disc_session_phase_begin(disc_session_t *s, int phase_idx, int64_t start_ms) {
    if (!s || !valid_phase(phase_idx) || s->active_phase >= 0) return false;
    frame_buffer_reset(&s->capture);
    s->active_phase = phase_idx;
    s->active_start_ms = start_ms;
    return true;
}

void disc_session_phase_feed(disc_session_t *s, const can_frame_t *frame) {
    if (!s || !frame || s->active_phase < 0) return;
    frame_buffer_t *cap = &s->capture;
    int before = cap->count;
    frame_buffer_append(cap, frame);
    /* Statistics cover exactly the stored frames (a full buffer drops frames). */
    if (cap->count > before) {
        disc_phase_add_frame(&s->phases[s->active_phase],
                             frame->can_id, frame->data, frame->dlc);
    }
}

int disc_session_active_phase(const disc_session_t *s) {
    return s ? s->active_phase : -1;
}

int disc_session_phase_stored_frames(const disc_session_t *s) {
    return (s && s->active_phase >= 0) ? s->capture.count : 0;
}

bool disc_session_phase_end(disc_session_t *s, int64_t end_ms, bool source_open) {
    if (!s || s->active_phase < 0) return false;
    int phase_idx = s->active_phase;
    int64_t start_ms = s->active_start_ms;
    s->active_phase = -1;

    disc_phase_t *phase = &s->phases[phase_idx];
    disc_raw_phase_t *raw_phase = &s->raw_phases[phase_idx];
    frame_buffer_t *cap = &s->capture;

    /* Hz = stored frames / analyzed duration. If frames were dropped, the
       stored frames only cover capture start -> last stored frame. */
    double wall_s = (double)(end_ms - start_ms) / 1000.0;
    double analyzed_s = wall_s;
    int dropped = cap->dropped;
    if (dropped > 0 && cap->count > 0) {
        double covered_s = (double)(cap->last_stored_ms - start_ms) / 1000.0;
        if (covered_s > 0) analyzed_s = covered_s;
    }
    disc_phase_finalize(phase, analyzed_s);
    phase->dropped_frames += dropped;

    if (!disc_raw_phase_set(raw_phase, cap->frames, cap->count)) {
        session_log(s, "\n[WARN] Failed to store raw frames for DBC draft output\n");
    }

    if (!source_open) return false;

    disc_phase_dump_meta_t *meta = &s->phase_meta[phase_idx];
    meta->captured = true;
    meta->actual_duration_s = wall_s;
    meta->analyzed_duration_s = phase->duration_s;
    meta->total_frames = phase->total_frames;
    meta->id_count = phase->id_count;
    meta->dropped_frames = phase->dropped_frames;
    meta->capture_dropped_frames = dropped;

    if (phase_idx == DISC_PHASE_BASELINE) {
        /* Detect bus mode (D-CAN gateway vs direct PT-CAN) */
        s->bus_mode = disc_detect_bus_mode(phase);
        s->have_bus_mode = true;
        s->result.bus_mode = s->bus_mode;
    }
    return true;
}

static void session_feed_sink(const can_frame_t *frame, void *ctx) {
    disc_session_phase_feed((disc_session_t *)ctx, frame);
}

bool disc_session_capture_phase(disc_session_t *s, int phase_idx,
                                frame_source_t *src, int duration_s,
                                disc_capture_progress_fn progress, void *progress_ctx) {
    if (!s || !src || !valid_phase(phase_idx) || s->active_phase >= 0) return false;

    /* Same order as before the incremental API: empty buffer, start the
       source (frames may arrive while it starts), then read the start time. */
    disc_session_phase_begin(s, phase_idx, 0);
    frame_source_start(src, session_feed_sink, s);

    int64_t start_ms = frame_source_now_ms(src);
    s->active_start_ms = start_ms;
    int64_t end_target_ms = start_ms + (int64_t)duration_s * 1000;

    while (frame_source_now_ms(src) < end_target_ms &&
           frame_source_is_open(src) && !frame_source_exhausted(src)) {
        if (progress &&
            !progress((double)(frame_source_now_ms(src) - start_ms) / 1000.0,
                      (double)duration_s, s->capture.count, progress_ctx)) {
            break;
        }
        frame_source_pump(src, 0.01);
    }

    int64_t end_ms = frame_source_now_ms(src);

    frame_source_stop(src);

    return disc_session_phase_end(s, end_ms, frame_source_is_open(src));
}

bool disc_session_save_phase_dump(disc_session_t *s, int phase_idx) {
    if (!s || !valid_phase(phase_idx) || !s->dump_enabled) return false;

    char phase_path[DISC_PATH_MAX];
    disc_make_phase_dump_path(phase_path, sizeof(phase_path), s->dump_dir, phase_idx);
    if (disc_write_phase_dump(phase_path, &s->raw_phases[phase_idx], s->hw_timestamps)) {
        disc_make_phase_dump_filename(s->phase_meta[phase_idx].dump_file,
                                      sizeof(s->phase_meta[phase_idx].dump_file),
                                      phase_idx);
        return true;
    }
    session_log(s, "[WARN] Failed to write phase dump: %s\n", phase_path);
    return false;
}

void disc_session_skip_phase(disc_session_t *s, int phase_idx) {
    if (s && valid_phase(phase_idx)) s->phase_meta[phase_idx].skipped = true;
}

bool disc_session_finish(disc_session_t *s, disc_session_output_t *out) {
    if (!s) return false;

    disc_analyze_with_raw(s->phases, s->raw_phases, &s->result);

    disc_make_draft_path(s->dbc_path, sizeof(s->dbc_path),
                         s->dump_enabled ? s->dump_dir : NULL, "dbc");
    s->dbc_written = disc_write_draft_dbc(s->dbc_path, s->phases, s->raw_phases, &s->result);
    if (s->dbc_written) {
        session_log(s, "[INFO] Draft DBC written: %s\n", s->dbc_path);
    } else {
        session_log(s, "[WARN] Failed to write draft DBC: %s\n", s->dbc_path);
    }

    disc_make_draft_path(s->vsp_path, sizeof(s->vsp_path),
                         s->dump_enabled ? s->dump_dir : NULL, "vsp");
    s->vsp_written = disc_write_draft_vsp(s->vsp_path, s->phases, s->raw_phases, &s->result);
    if (s->vsp_written) {
        session_log(s, "[INFO] Draft VSP written: %s\n", s->vsp_path);
    } else {
        session_log(s, "[WARN] Failed to write draft VSP: %s\n", s->vsp_path);
    }

    if (s->dump_enabled) {
        if (!disc_write_result_json(s->result_path, s->have_bus_mode, s->bus_mode,
                                    s->dump_dir, s->dbc_written ? s->dbc_path : NULL,
                                    s->vsp_written ? s->vsp_path : NULL, &s->result)) {
            session_log(s, "[WARN] Failed to write result JSON: %s\n", s->result_path);
            s->result_path[0] = '\0';
        }
        s->manifest_written =
            disc_write_manifest(s->manifest_path, "completed", s->device_prefix,
                                s->connected_name, s->monitor_cmd,
                                s->have_bus_mode, s->bus_mode,
                                s->dbc_written ? s->dbc_path : NULL,
                                s->vsp_written ? s->vsp_path : NULL,
                                s->result_path[0] ? s->result_path : NULL,
                                s->phase_meta);
        if (!s->manifest_written) {
            session_log(s, "[WARN] Failed to write manifest: %s\n", s->manifest_path);
        }
    }
    s->finished = true;

    if (out) {
        memset(out, 0, sizeof(*out));
        out->result = &s->result;
        out->have_bus_mode = s->have_bus_mode;
        out->bus_mode = s->bus_mode;
        out->dump_dir = s->dump_enabled ? s->dump_dir : NULL;
        out->dbc_path = s->dbc_written ? s->dbc_path : NULL;
        out->vsp_path = s->vsp_written ? s->vsp_path : NULL;
        out->result_path = (s->dump_enabled && s->result_path[0]) ? s->result_path : NULL;
        out->manifest_path = s->manifest_written ? s->manifest_path : NULL;
    }
    return true;
}

void disc_session_print_result_json(const disc_session_t *s, FILE *f) {
    if (!s || !f) return;
    /* JSON output (stdout) — automatically inferred signals */
    disc_write_result_json_object(f, s->have_bus_mode, s->bus_mode,
                                  s->dump_enabled ? s->dump_dir : NULL,
                                  s->dbc_written ? s->dbc_path : NULL,
                                  s->vsp_written ? s->vsp_path : NULL, &s->result);
}

void disc_session_abort(disc_session_t *s, const char *status) {
    if (!s || !s->dump_enabled) return;
    disc_write_manifest(s->manifest_path, status ? status : "interrupted",
                        s->device_prefix, s->connected_name, s->monitor_cmd,
                        s->have_bus_mode, s->bus_mode,
                        NULL, NULL, NULL, s->phase_meta);
}

/*******************************************************************************
 * CLI driver
 ******************************************************************************/

/** Read a single char from stdin while pumping NSRunLoop for BLE.
 *  Drains any remaining characters on the line (up to and including '\n')
 *  so the next call doesn't accidentally consume a stale newline. */
static int disc_read_char(frame_source_t *src, volatile sig_atomic_t *running) {
    int ch = 0;
    while (ch == 0 && frame_source_is_open(src) && *running) {
        @autoreleasepool {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
            if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
                ch = fgetc(stdin);
            }
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }
    /* Drain rest of the line to prevent stale newline on next call */
    if (ch != '\n' && ch != 0) {
        int c;
        while ((c = fgetc(stdin)) != EOF && c != '\n')
            ;
    }
    return ch;
}

/** Finish the "Capturing..." console line (the session filled meta). */
static void disc_report_captured_phase(const disc_phase_t *phase,
                                       const disc_phase_dump_meta_t *meta) {
    double wall_duration_s = meta->actual_duration_s;
    int capture_dropped = meta->capture_dropped_frames;
    fprintf(stderr, " %d frames, %d CAN IDs", phase->total_frames, phase->id_count);
    if (phase->dropped_frames > 0) {
        fprintf(stderr, " (%d dropped)", phase->dropped_frames);
    }
    fprintf(stderr, "\n");
    if (capture_dropped > 0) {
        fprintf(stderr, "[WARN] Capture buffer full: %d frames dropped (%d not analyzed in total); "
                "Hz computed over %.3f s of %.3f s captured\n",
                capture_dropped, phase->dropped_frames,
                phase->duration_s, wall_duration_s);
    }
}

/** Print a candidate result line to stderr */
static void disc_print_candidate(const char *name, const disc_candidate_t *c) {
    fprintf(stderr, "    %-12s 0x%03X [%d] @ %.1f Hz  byte %d",
            name, c->can_id, c->dlc, c->hz, c->byte_idx);
    if (c->byte2_idx >= 0) fprintf(stderr, ",%d", c->byte2_idx);
    fprintf(stderr, "  score=%.1f  confidence=%s\n",
            c->score, disc_confidence_name(c->confidence));

    /* Characterization details (if available) */
    if (c->signedness != DISC_SIGN_UNKNOWN) {
        fprintf(stderr, "                 ");
        if (c->byte2_idx >= 0)
            fprintf(stderr, "%s-endian ", disc_endian_name(c->endianness));
        fprintf(stderr, "%s  raw=[%d..%d]\n",
                disc_sign_name(c->signedness), c->raw_min, c->raw_max);
    }
    if (c->start_bit >= 0 && c->bit_length > 0) {
        char sign_note[24] = "";
        if (c->sign_char == 'S') snprintf(sign_note, sizeof(sign_note), " sign_bit=%d", c->sign_bit);
        fprintf(stderr,
                "                 bit=%d|%d@%d%c%s  evidence=%.1f specificity=%.1f shape=%.1f prior=%.1f margin=%.1f\n",
                c->start_bit, c->bit_length, c->byte_order,
                c->sign_char == '-' ? '-' : '+', sign_note,
                c->evidence_score, c->specificity_score, c->shape_score,
                c->prior_score, c->ambiguity_margin);
    }
}

/* Capture progress: Ctrl+C during a capture ends the phase early. */
static bool disc_cli_progress(double elapsed_s, double duration_s, int frames, void *ctx) {
    (void)elapsed_s;
    (void)duration_s;
    (void)frames;
    const disc_cli_config_t *cfg = (const disc_cli_config_t *)ctx;
    return !(cfg->interrupt && *cfg->interrupt);
}

int disc_cli_run(const disc_cli_config_t *cfg) {
    frame_source_t *src = cfg->source;
    disc_session_config_t session_cfg = {
        .dump_dir_override = cfg->dump_dir_override,
        .device_prefix = cfg->device_prefix,
        .connected_name = cfg->connected_name,
        .monitor_cmd = cfg->monitor_cmd,
        .log = NULL,
        .log_ctx = NULL,
        .hw_timestamps = frame_source_has_hw_timestamps(src),
    };
    disc_session_t *session = disc_session_create(&session_cfg);
    if (!session) {
        fprintf(stderr, "[ERROR] Out of memory\n");
        return 1;
    }

    /* Baseline is mandatory; the other phases offer [s] skip. */
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        fprintf(stderr, "\n=== Phase %d/%d: %s ===\n",
                p + 1, DISC_NUM_PHASES, s_disc_phases[p].name);
        fprintf(stderr, "  %s\n", s_disc_phases[p].instruction);
        if (s_disc_phases[p].warning)
            fprintf(stderr, "  NOTE: %s\n", s_disc_phases[p].warning);
        if (disc_session_phase_skippable(p)) {
            fprintf(stderr, "  [Enter] proceed  [s] skip > ");
            fflush(stderr);
        } else {
            fprintf(stderr, "  Press Enter when ready...");
        }

        int ch = disc_read_char(src, cfg->running);
        if (!*cfg->running) goto interrupted;
        if (!frame_source_is_open(src)) goto disconnected;
        if (disc_session_phase_skippable(p) && (ch == 's' || ch == 'S')) {
            fprintf(stderr, "  Skipped.\n");
            disc_session_skip_phase(session, p);
            continue;
        }

        fprintf(stderr, "  Capturing %d seconds...", s_disc_phases[p].duration_s);
        fflush(stderr);
        bool still_open = disc_session_capture_phase(session, p, src,
                                                     s_disc_phases[p].duration_s,
                                                     disc_cli_progress, (void *)cfg);
        /* Reset interrupt so next capture isn't immediately skipped */
        if (cfg->interrupt) *cfg->interrupt = 0;
        if (!still_open) goto disconnected;
        disc_report_captured_phase(disc_session_phase(session, p),
                                   disc_session_phase_meta(session, p));

        if (p == DISC_PHASE_BASELINE) {
            disc_bus_mode_t bus_mode = disc_session_bus_mode(session);
            cli_log("Bus mode: %s (%s)",
                    bus_mode == DISC_BUS_GATEWAY ? "GATEWAY" : "DIRECT",
                    bus_mode == DISC_BUS_GATEWAY ? "D-CAN" : "PT-CAN");
        }
        disc_session_save_phase_dump(session, p);
    }

    fprintf(stderr, "\n[INFO] Analyzing captures with automatic scoring...\n");
    disc_session_output_t out;
    disc_session_finish(session, &out);

    disc_session_print_result_json(session, stdout);

    /* Summary to stderr */
    fprintf(stderr, "\n=== Final Results ===\n");
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        if (out.result->found[s]) {
            disc_print_candidate(disc_signal_name((disc_signal_t)s),
                                  &out.result->signals[s]);
        }
    }

    disc_session_free(session);
    if (cfg->disconnect) cfg->disconnect(cfg->disconnect_ctx);
    if (cfg->interrupt) *cfg->interrupt = 0;
    return 0;

interrupted:
    fprintf(stderr, "\n[INFO] Interrupted by user\n");
    disc_session_abort(session, "interrupted");
    disc_session_free(session);
    if (cfg->disconnect) cfg->disconnect(cfg->disconnect_ctx);
    return 130;

disconnected:
    fprintf(stderr, "\n[ERROR] Connection lost during discover\n");
    disc_session_abort(session, "disconnected");
    disc_session_free(session);
    return 1;
}
