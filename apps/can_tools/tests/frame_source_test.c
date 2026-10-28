/**
 * @file frame_source_test.c
 * @brief TSV reader, replay frame source and capture buffer (no hardware)
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

#define TSV_PATH "frame_source_test.frames.tsv"

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

/* Same format disc_write_phase_dump() writes, plus a blank and a bad row. */
static const char *kTsv =
    "timestamp_ms\tcan_id\tis_extended\tdlc\tdata_hex\n"
    "1712560212345\t0x0A5\t0\t8\t0011223344556677\n"
    "\n"
    "1712560212355\t0x18DAF110\t1\t3\tAABBCC\n"
    "garbage row\n"
    "1712560212365\t0x7FF\t0\t0\t\n"
    "1712560212375\t1A0\t0\t2\tdead\n"
    "1712560212385\t0x316\t0\t1\t0F\r\n";

typedef struct {
    can_frame_t frames[16];
    int count;
} collect_t;

static void collect_sink(const can_frame_t *frame, void *ctx) {
    collect_t *c = (collect_t *)ctx;
    if (c->count < 16) c->frames[c->count] = *frame;
    c->count++;
}

static bool write_file(const char *path, const char *text) {
    FILE *f = fopen(path, "w");
    if (!f) return false;
    fputs(text, f);
    return fclose(f) == 0;
}

static void check_frames(const can_frame_t *frames, int count) {
    CHECK(count == 5, "count %d", count);
    if (count != 5) return;
    CHECK(frames[0].timestamp_ms == 1712560212345LL && frames[0].can_id == 0x0A5 &&
          !frames[0].is_extended && frames[0].dlc == 8 &&
          frames[0].data[0] == 0x00 && frames[0].data[7] == 0x77, "frame 0");
    CHECK(frames[1].timestamp_ms == 1712560212355LL && frames[1].can_id == 0x18DAF110 &&
          frames[1].is_extended && frames[1].dlc == 3 &&
          frames[1].data[0] == 0xAA && frames[1].data[2] == 0xCC, "frame 1 (extended)");
    CHECK(frames[2].can_id == 0x7FF && frames[2].dlc == 0, "frame 2 (dlc 0)");
    CHECK(frames[3].can_id == 0x1A0 && frames[3].dlc == 2 &&
          frames[3].data[0] == 0xDE && frames[3].data[1] == 0xAD, "frame 3 (no 0x, lower-case hex)");
    CHECK(frames[4].timestamp_ms == 1712560212385LL && frames[4].can_id == 0x316 &&
          frames[4].data[0] == 0x0F, "frame 4 (CRLF)");
}

static void test_tsv_load(void) {
    frame_list_t list;
    CHECK(frame_tsv_load(TSV_PATH, "test.tsv", &list) == 1, "load");
    CHECK(list.malformed == 1, "malformed %d", list.malformed);
    check_frames(list.frames, list.count);
    frame_list_free(&list);

    CHECK(frame_tsv_load("does_not_exist.frames.tsv", "missing", &list) == 0, "missing file -> 0");

    char row[] = "12\t0x10\t0\t9\t00";
    can_frame_t f;
    CHECK(!frame_tsv_parse_row(row, &f), "dlc 9 rejected");
    char row2[] = "12\t0x10\t0\t2\t001";
    CHECK(!frame_tsv_parse_row(row2, &f), "odd hex rejected");
}

static void test_replay_unpaced(void) {
    frame_source_t *src = frame_source_replay_open(TSV_PATH, false);
    CHECK(src != NULL, "open");
    if (!src) return;

    CHECK(frame_source_is_open(src) && !frame_source_exhausted(src), "fresh source");
    CHECK(frame_source_now_ms(src) == 1712560212345LL, "clock starts at first frame");

    collect_t c = { .count = 0 };
    frame_source_pump(src, 0.01);
    CHECK(c.count == 0, "nothing before start");
    CHECK(frame_source_start(src, collect_sink, &c), "start");
    CHECK(frame_source_is_running(src), "running");
    frame_source_pump(src, 0.01);
    check_frames(c.frames, c.count);
    CHECK(frame_source_exhausted(src) && frame_source_is_open(src), "exhausted but open");
    CHECK(frame_source_now_ms(src) == 1712560212385LL, "clock at last frame");

    frame_source_pump(src, 0.01);
    CHECK(c.count == 5, "no frames after the end");
    frame_source_stop(src);
    CHECK(!frame_source_is_running(src), "stopped");
    frame_source_free(src);
}

static void test_replay_paced(void) {
    can_frame_t frames[3];
    memset(frames, 0, sizeof(frames));
    for (int i = 0; i < 3; i++) {
        frames[i].can_id = 0x100 + (uint32_t)i;
        frames[i].dlc = 1;
        frames[i].data[0] = (uint8_t)i;
        frames[i].timestamp_ms = 5000 + i * 300;   /* 0, 300, 600 ms */
    }
    frame_source_t *src = frame_source_replay_create(frames, 3, true);
    CHECK(src != NULL, "create");
    if (!src) return;

    collect_t c = { .count = 0 };
    frame_source_start(src, collect_sink, &c);
    frame_source_pump(src, 0.1);
    CHECK(c.count == 1, "paced: only the first frame after 100 ms (got %d)", c.count);
    int64_t clock = frame_source_now_ms(src);
    CHECK(clock >= 5090 && clock < 5300, "paced clock %lld", (long long)clock);

    frame_source_stop(src);
    int64_t frozen = frame_source_now_ms(src);
    usleep(50000);
    CHECK(frame_source_now_ms(src) == frozen, "clock frozen while stopped");

    frame_source_start(src, collect_sink, &c);
    frame_source_pump(src, 0.75);
    CHECK(c.count == 3, "paced: all frames after resuming (got %d)", c.count);
    CHECK(frame_source_exhausted(src), "paced exhausted");
    for (int i = 0; i < 3 && i < c.count; i++) {
        CHECK(c.frames[i].can_id == 0x100 + (uint32_t)i && c.frames[i].timestamp_ms == 5000 + i * 300,
              "paced order %d", i);
    }
    frame_source_free(src);

    /* Empty replay */
    src = frame_source_replay_create(NULL, 0, false);
    CHECK(src && frame_source_exhausted(src) && frame_source_now_ms(src) == 0, "empty replay");
    frame_source_free(src);
}

static void test_frame_buffer(void) {
    frame_buffer_t buf = {0};
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    for (int i = 0; i < FRAME_BUFFER_MAX + 10; i++) {
        f.timestamp_ms = i;
        frame_buffer_append(&buf, &f);
    }
    CHECK(buf.count == FRAME_BUFFER_MAX, "count at cap %d", buf.count);
    CHECK(buf.dropped == 10, "dropped %d", buf.dropped);
    CHECK(buf.last_stored_ms == FRAME_BUFFER_MAX - 1, "last stored %lld", (long long)buf.last_stored_ms);
    CHECK(buf.frames[12345].timestamp_ms == 12345, "content");

    frame_buffer_reset(&buf);
    CHECK(buf.count == 0 && buf.dropped == 0 && buf.last_stored_ms == 0 && buf.capacity > 0,
          "reset keeps the allocation");
    frame_buffer_sink(&f, &buf);
    CHECK(buf.count == 1, "sink appends");
    frame_buffer_free(&buf);
    CHECK(buf.frames == NULL && buf.capacity == 0 && buf.count == 0, "free");
}

int main(void) {
    enter_tmpdir("frame_source_test");
    if (!write_file(TSV_PATH, kTsv)) {
        fprintf(stderr, "cannot write %s\n", TSV_PATH);
        return 1;
    }
    test_tsv_load();
    test_replay_unpaced();
    test_replay_paced();
    test_frame_buffer();
    unlink(TSV_PATH);
    leave_tmpdir();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
