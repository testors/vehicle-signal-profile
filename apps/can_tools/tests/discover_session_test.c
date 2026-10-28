/**
 * @file discover_session_test.c
 * @brief Discover session layer fed by replay frame sources (no hardware)
 *
 * Checks the plumbing around the analysis engine: phase capture from a frame
 * source, manifest records, phase dumps (read back with frame_tsv_load), bus
 * mode after the baseline, skip, early stop, capture-buffer drop accounting,
 * finish outputs and the log callback. The engine itself is covered by
 * discover_regression.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "discover_session.h"
#include "frame_source.h"
#include "frame_tsv.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);            \
            fprintf(stderr, __VA_ARGS__);                                   \
            fprintf(stderr, "\n");                                          \
        }                                                                   \
    } while (0)

#define OUT_DIR "discover_session_test_out"

/* Run inside a private temp dir (removed on exit) so the test never writes
 * into the directory it was started from. */
static char g_tmpdir[1024];

static void enter_tmpdir(const char *name) {
    const char *base = getenv("TMPDIR");
    if (!base || !*base) base = "/tmp";
    snprintf(g_tmpdir, sizeof(g_tmpdir), "%s/%s.XXXXXX", base, name);
    if (!mkdtemp(g_tmpdir) || chdir(g_tmpdir) != 0) {
        fprintf(stderr, "cannot create temp dir %s\n", g_tmpdir);
        exit(1);
    }
}

static void leave_tmpdir(void) {
    char cmd[1200];
    if (chdir("/") != 0) return;
    snprintf(cmd, sizeof(cmd), "rm -rf '%s'", g_tmpdir);
    if (system(cmd) != 0) fprintf(stderr, "cannot remove %s\n", g_tmpdir);
}

typedef struct {
    char text[16384];
    size_t len;
} log_buf_t;

static void log_sink(const char *text, void *ctx) {
    log_buf_t *log = (log_buf_t *)ctx;
    size_t n = strlen(text);
    if (log->len + n < sizeof(log->text)) {
        memcpy(log->text + log->len, text, n + 1);
        log->len += n;
    }
}

/* A small bus: 0x0A5 @100 Hz (bytes 2..3 move in the steering phase),
 * 0x0F3 @50 Hz, 0x316 @10 Hz. */
static can_frame_t *make_phase(int phase_idx, int seconds, int64_t t0, int *count_out) {
    int cap = seconds * 170 + 8;
    can_frame_t *frames = (can_frame_t *)calloc((size_t)cap, sizeof(*frames));
    int n = 0;
    for (int ms = 0; ms < seconds * 1000; ms += 10) {
        can_frame_t *f = &frames[n++];
        f->timestamp_ms = t0 + ms;
        f->can_id = 0x0A5;
        f->dlc = 8;
        int steer = phase_idx == DISC_PHASE_STEERING
                    ? (int)(4000.0 * sin((double)ms / 1000.0 * 2.0)) : 0;
        f->data[0] = (uint8_t)((ms / 10) & 0x0F);
        f->data[2] = (uint8_t)(steer & 0xFF);
        f->data[3] = (uint8_t)((steer >> 8) & 0xFF);
        if (ms % 20 == 0) {
            f = &frames[n++];
            f->timestamp_ms = t0 + ms + 1;
            f->can_id = 0x0F3;
            f->dlc = 8;
            f->data[4] = 0x2E;
            f->data[5] = 0xE0;
        }
        if (ms % 100 == 0) {
            f = &frames[n++];
            f->timestamp_ms = t0 + ms + 2;
            f->can_id = 0x316;
            f->dlc = 2;
        }
    }
    *count_out = n;
    return frames;
}

static bool file_exists(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static char *read_all(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)calloc((size_t)n + 1, 1);
    if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        buf = NULL;
    }
    fclose(f);
    return buf;
}

static bool stop_immediately(double elapsed_s, double duration_s, int frames, void *ctx) {
    (void)elapsed_s;
    (void)duration_s;
    (void)frames;
    (*(int *)ctx)++;
    return false;
}

static void test_session_flow(void) {
    log_buf_t log = { .len = 0 };
    disc_session_config_t cfg = {
        .dump_dir_override = OUT_DIR,
        .device_prefix = "vLinker",
        .connected_name = "vLinker MC-IOS",
        .monitor_cmd = "ATMA",
        .log = log_sink,
        .log_ctx = &log,
    };
    disc_session_t *s = disc_session_create(&cfg);
    CHECK(s != NULL, "create");
    if (!s) return;
    CHECK(disc_session_dump_dir(s) && strcmp(disc_session_dump_dir(s), OUT_DIR) == 0, "dump dir");
    CHECK(strstr(log.text, "[INFO] Capture dump dir: " OUT_DIR "\n") != NULL, "dump dir logged: %s", log.text);
    CHECK(!disc_session_have_bus_mode(s), "no bus mode before baseline");

    CHECK(disc_session_phase_count() == DISC_NUM_PHASES, "phase count");
    CHECK(strcmp(disc_session_phase_name(0), "Baseline") == 0 &&
          strcmp(disc_session_phase_slug(5), "wheel_speed") == 0 &&
          disc_session_phase_duration_s(4) == 10 && disc_session_phase_warning(0) == NULL &&
          disc_session_phase_warning(4) != NULL && !disc_session_phase_skippable(0) &&
          disc_session_phase_skippable(1) && strcmp(disc_session_phase_name(9), "Phase") == 0,
          "phase table");

    int64_t t0 = 1712560200000LL;
    int baseline_count = 0;
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        if (p == DISC_PHASE_GEAR) {
            disc_session_skip_phase(s, p);
            continue;
        }
        int seconds = disc_session_phase_duration_s(p);
        int count = 0;
        can_frame_t *frames = make_phase(p, seconds, t0 + p * 100000, &count);
        if (p == 0) baseline_count = count;
        frame_source_t *src = frame_source_replay_create(frames, count, false);
        CHECK(disc_session_capture_phase(s, p, src, seconds, NULL, NULL), "capture %d", p);
        CHECK(!frame_source_is_running(src), "source stopped after capture %d", p);
        CHECK(disc_session_save_phase_dump(s, p), "save %d", p);

        const disc_phase_dump_meta_t *meta = disc_session_phase_meta(s, p);
        const disc_phase_t *phase = disc_session_phase(s, p);
        double span = (double)(frames[count - 1].timestamp_ms - frames[0].timestamp_ms) / 1000.0;
        CHECK(meta->captured && !meta->skipped && meta->total_frames == count &&
              phase->total_frames == count && meta->id_count == 3 &&
              meta->capture_dropped_frames == 0 && meta->planned_duration_s == seconds,
              "meta %d: frames %d/%d ids %d", p, meta->total_frames, count, meta->id_count);
        CHECK(fabs(meta->actual_duration_s - span) < 1e-9 && fabs(meta->analyzed_duration_s - span) < 1e-9,
              "durations %d: %.3f %.3f vs %.3f", p, meta->actual_duration_s, meta->analyzed_duration_s, span);
        CHECK(disc_session_raw_phase(s, p)->frame_count == count, "raw frames %d", p);

        char path[256];
        snprintf(path, sizeof(path), OUT_DIR "/%02d_%s.frames.tsv", p, disc_session_phase_slug(p));
        frame_list_t back;
        CHECK(frame_tsv_load(path, path, &back) == 1 && back.count == count && back.malformed == 0,
              "dump %d read back", p);
        if (back.count == count) {
            CHECK(memcmp(&back.frames[count / 2].data, &frames[count / 2].data, 8) == 0 &&
                  back.frames[count / 2].timestamp_ms == frames[count / 2].timestamp_ms &&
                  back.frames[count / 2].can_id == frames[count / 2].can_id, "dump %d content", p);
        }
        frame_list_free(&back);
        CHECK(strcmp(meta->dump_file, path + strlen(OUT_DIR "/")) == 0, "dump_file %s", meta->dump_file);
        frame_source_free(src);
        free(frames);

        if (p == DISC_PHASE_BASELINE) {
            CHECK(disc_session_have_bus_mode(s), "bus mode after baseline");
            /* 100 Hz and 50 Hz IDs -> direct bus */
            CHECK(disc_session_bus_mode(s) == DISC_BUS_DIRECT, "bus mode direct");
        }
    }
    CHECK(disc_session_phase_meta(s, DISC_PHASE_GEAR)->skipped &&
          !disc_session_phase_meta(s, DISC_PHASE_GEAR)->captured, "gear skipped");
    CHECK(baseline_count > 0, "baseline frames");

    disc_session_output_t out;
    CHECK(disc_session_finish(s, &out), "finish");
    CHECK(out.result && out.have_bus_mode && out.bus_mode == DISC_BUS_DIRECT, "output bus mode");
    CHECK(out.dbc_path && strcmp(out.dbc_path, OUT_DIR "/ptcan_discover_draft.dbc") == 0 &&
          file_exists(out.dbc_path), "dbc");
    CHECK(out.vsp_path && strcmp(out.vsp_path, OUT_DIR "/ptcan_discover_draft.vsp") == 0 &&
          file_exists(out.vsp_path), "vsp");
    CHECK(out.result_path && file_exists(out.result_path), "result.json");
    CHECK(out.manifest_path && file_exists(out.manifest_path), "manifest.json");
    CHECK(strstr(log.text, "[INFO] Draft DBC written: " OUT_DIR "/ptcan_discover_draft.dbc\n") &&
          strstr(log.text, "[INFO] Draft VSP written: " OUT_DIR "/ptcan_discover_draft.vsp\n"),
          "finish logged: %s", log.text);

    char *manifest = out.manifest_path ? read_all(out.manifest_path) : NULL;
    CHECK(manifest && strstr(manifest, "\"format\":\"can_discover_dump/v1\"") &&
          strstr(manifest, "\"status\":\"completed\"") &&
          strstr(manifest, "\"device_prefix\":\"vLinker\"") &&
          strstr(manifest, "\"connected_device\":\"vLinker MC-IOS\"") &&
          strstr(manifest, "\"monitor_cmd\":\"ATMA\"") &&
          strstr(manifest, "\"bus_mode\":\"direct\"") &&
          strstr(manifest, "{\"index\":4,\"name\":\"Gear\",\"slug\":\"gear\",\"planned_duration_s\":10,"
                           "\"skipped\":true,\"captured\":false"),
          "manifest content");
    free(manifest);

    /* stdout JSON == result.json content */
    char *result = out.result_path ? read_all(out.result_path) : NULL;
    FILE *mem = tmpfile();
    disc_session_print_result_json(s, mem);
    long n = ftell(mem);
    rewind(mem);
    char *printed = (char *)calloc((size_t)n + 1, 1);
    CHECK(fread(printed, 1, (size_t)n, mem) == (size_t)n, "read printed JSON");
    fclose(mem);
    CHECK(result && strcmp(result, printed) == 0, "printed JSON equals result.json");
    CHECK(strstr(printed, "\"bus_mode\":\"direct\"") && strstr(printed, "\"dump_dir\":\"" OUT_DIR "\""),
          "printed JSON fields");
    free(result);
    free(printed);
    disc_session_free(s);
}

static void test_early_stop_and_drops(void) {
    disc_session_config_t cfg = {
        .dump_dir_override = OUT_DIR "/drops",
        .monitor_cmd = "STMA",
        .log = log_sink,
    };
    log_buf_t log = { .len = 0 };
    cfg.log_ctx = &log;
    disc_session_t *s = disc_session_create(&cfg);
    CHECK(s != NULL, "create 2");
    if (!s) return;

    /* progress returning false ends the phase before the first pump */
    int calls = 0;
    can_frame_t one = { .can_id = 0x100, .dlc = 1, .timestamp_ms = 1000 };
    frame_source_t *src = frame_source_replay_create(&one, 1, false);
    CHECK(disc_session_capture_phase(s, DISC_PHASE_STEERING, src, 8, stop_immediately, &calls),
          "early stop still open");
    CHECK(calls == 1 && disc_session_phase_meta(s, DISC_PHASE_STEERING)->total_frames == 0 &&
          !frame_source_exhausted(src), "early stop captured nothing (calls %d)", calls);
    frame_source_free(src);

    /* Past the capture buffer cap: drops are counted and Hz uses the covered time. */
    int extra = 50;
    int count = FRAME_BUFFER_MAX + extra;
    can_frame_t *frames = (can_frame_t *)calloc((size_t)count, sizeof(*frames));
    for (int i = 0; i < count; i++) {
        frames[i].can_id = 0x200 + (uint32_t)(i % 4);
        frames[i].dlc = 1;
        frames[i].data[0] = (uint8_t)i;
        frames[i].timestamp_ms = 10000 + i;   /* 1 kHz */
    }
    src = frame_source_replay_create(frames, count, false);
    free(frames);
    CHECK(disc_session_capture_phase(s, DISC_PHASE_BASELINE, src, 2000, NULL, NULL), "capture big");
    const disc_phase_dump_meta_t *meta = disc_session_phase_meta(s, DISC_PHASE_BASELINE);
    CHECK(meta->capture_dropped_frames == extra, "capture dropped %d", meta->capture_dropped_frames);
    CHECK(meta->dropped_frames >= extra, "dropped total %d", meta->dropped_frames);
    CHECK(meta->total_frames == FRAME_BUFFER_MAX, "stored %d", meta->total_frames);
    CHECK(fabs(meta->actual_duration_s - (count - 1) / 1000.0) < 1e-9, "wall %.3f", meta->actual_duration_s);
    CHECK(fabs(meta->analyzed_duration_s - (FRAME_BUFFER_MAX - 1) / 1000.0) < 1e-9,
          "analyzed %.3f", meta->analyzed_duration_s);
    frame_source_free(src);

    disc_session_abort(s, "interrupted");
    char *manifest = read_all(OUT_DIR "/drops/manifest.json");
    CHECK(manifest && strstr(manifest, "\"status\":\"interrupted\"") &&
          strstr(manifest, "\"dbc_path\":null") && strstr(manifest, "\"device_prefix\":null") &&
          strstr(manifest, "\"capture_dropped_frames\":50"), "abort manifest");
    free(manifest);
    disc_session_free(s);
}

/* Run one session over fixed phase frames, either through the blocking
 * capture (replay source) or the incremental begin/feed/end API, and return
 * the concatenated session files with the dump dir name replaced. */
static char *run_equivalence_session(const char *dir, bool incremental) {
    disc_session_config_t cfg = {
        .dump_dir_override = dir,
        .device_prefix = "vLinker",
        .connected_name = "vLinker FAKE",
        .monitor_cmd = "STMA",
        .log = log_sink,
    };
    log_buf_t log = { .len = 0 };
    cfg.log_ctx = &log;
    disc_session_t *s = disc_session_create(&cfg);
    if (!s) return NULL;
    int64_t t0 = 1712560200000LL;
    for (int p = 0; p < DISC_NUM_PHASES; p++) {
        if (p == DISC_PHASE_GEAR || p == DISC_PHASE_WHEEL_SPEED) {
            disc_session_skip_phase(s, p);
            continue;
        }
        int count = 0;
        can_frame_t *frames = make_phase(p, disc_session_phase_duration_s(p), t0 + p * 100000, &count);
        if (incremental) {
            disc_session_phase_begin(s, p, frames[0].timestamp_ms);
            CHECK(disc_session_active_phase(s) == p, "active phase %d", p);
            CHECK(!disc_session_phase_begin(s, p, 0), "no second begin while active");
            for (int i = 0; i < count; i++) disc_session_phase_feed(s, &frames[i]);
            CHECK(disc_session_phase_stored_frames(s) == count, "stored %d", p);
            CHECK(disc_session_phase_end(s, frames[count - 1].timestamp_ms, true), "end %d", p);
            CHECK(disc_session_active_phase(s) == -1, "no active phase after end");
        } else {
            frame_source_t *src = frame_source_replay_create(frames, count, false);
            CHECK(disc_session_capture_phase(s, p, src, disc_session_phase_duration_s(p), NULL, NULL),
                  "capture %d", p);
            frame_source_free(src);
        }
        disc_session_save_phase_dump(s, p);
        free(frames);
    }
    disc_session_output_t out;
    disc_session_finish(s, &out);

    static const char *files[] = {
        "00_baseline.frames.tsv", "01_steering.frames.tsv", "02_throttle.frames.tsv",
        "03_brake.frames.tsv", "manifest.json", "result.json",
        "ptcan_discover_draft.dbc", "ptcan_discover_draft.vsp",
    };
    size_t cap = 1 << 22, len = 0;
    char *all = (char *)calloc(cap, 1);
    for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
        char path[256];
        snprintf(path, sizeof(path), "%s/%s", dir, files[i]);
        char *content = read_all(path);
        CHECK(content != NULL, "%s written", path);
        if (!content) continue;
        len += (size_t)snprintf(all + len, cap - len, "=== %s\n%s", files[i], content);
        free(content);
    }
    FILE *mem = tmpfile();
    disc_session_print_result_json(s, mem);
    long n = ftell(mem);
    rewind(mem);
    len += (size_t)snprintf(all + len, cap - len, "=== stdout\n");
    if (len + (size_t)n < cap && fread(all + len, 1, (size_t)n, mem) == (size_t)n) len += (size_t)n;
    fclose(mem);
    disc_session_free(s);

    /* the dump dir name appears inside the files; neutralize it */
    for (char *p = strstr(all, dir); p; p = strstr(p, dir)) memset(p, '#', strlen(dir));
    return all;
}

static void test_incremental_equivalence(void) {
    char *blocking = run_equivalence_session("eq_block", false);
    char *incremental = run_equivalence_session("eq_incre", true);
    CHECK(blocking && incremental && strlen(blocking) > 1000, "both sessions produced files");
    if (blocking && incremental) {
        bool same = strcmp(blocking, incremental) == 0;
        CHECK(same, "incremental and blocking capture give identical files");
        if (!same) {
            size_t i = 0;
            while (blocking[i] && blocking[i] == incremental[i]) i++;
            fprintf(stderr, "first difference at byte %zu\n", i);
            const char *dump = getenv("EQ_DUMP_DIR");
            if (dump) {
                char path[512];
                snprintf(path, sizeof(path), "%s/blocking.txt", dump);
                FILE *f = fopen(path, "w"); if (f) { fputs(blocking, f); fclose(f); }
                snprintf(path, sizeof(path), "%s/incremental.txt", dump);
                f = fopen(path, "w"); if (f) { fputs(incremental, f); fclose(f); }
            }
        }
        CHECK(strstr(blocking, "=== manifest.json") && strstr(blocking, "\"status\":\"completed\"") &&
              strstr(blocking, "=== ptcan_discover_draft.vsp"), "files compared");
    }
    free(blocking);
    free(incremental);
}

int main(void) {
    enter_tmpdir("discover_session_test");
    test_session_flow();
    test_early_stop_and_drops();
    test_incremental_equivalence();

    leave_tmpdir();
    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
