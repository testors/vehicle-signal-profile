/*
 * Profile index for the TUI picker (tui_profiles.c) over the fixtures in
 * tests/fixtures/tui/profiles. No hardware.
 *
 *   tui_profiles_test <tests/fixtures/tui>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "tui_profiles.h"

static int g_checks = 0, g_failures = 0;
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

static void copy_file(const char *from, const char *to) {
    FILE *a = fopen(from, "rb"), *b = fopen(to, "wb");
    char buf[8192];
    size_t n;
    while (a && b && (n = fread(buf, 1, sizeof(buf), a)) > 0) fwrite(buf, 1, n, b);
    if (a) fclose(a);
    if (b) fclose(b);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: tui_profiles_test FIXTURES_DIR\n");
        return 2;
    }
    char dir[1024];
    snprintf(dir, sizeof(dir), "%s/profiles", argv[1]);

    char found[1024];
    CHECK(tui_profiles_find_dir(dir, NULL, found, sizeof(found)) && strcmp(found, dir) == 0, "explicit dir");
    CHECK(!tui_profiles_find_dir("/nonexistent/profiles", NULL, found, sizeof(found)), "explicit missing dir");

    tui_profile_index_t ix;
    tui_profiles_open(&ix, dir, false);
    CHECK(ix.have_dir && ix.total == 3 && ix.indexed == 0, "3 files listed, none loaded: %d", ix.total);
    CHECK(strcmp(ix.entries[0].file, "other__roadster__r2__2009_2014.vsp") == 0, "sorted by name");
    CHECK(!tui_profiles_step(&ix, 2, 0) && ix.indexed == 2, "chunk of 2");
    CHECK(tui_profiles_step(&ix, 64, 0) && tui_profiles_done(&ix), "done");

    const tui_profile_entry_t *e = &ix.entries[0];
    CHECK(e->ok && strcmp(e->make, "Other") == 0 && strcmp(e->model, "Roadster") == 0 && strcmp(e->gen, "R2") == 0,
          "metadata %s %s %s", e->make, e->model, e->gen);
    CHECK(strcmp(e->years, "2009–2014") == 0, "years %s", e->years);
    const tui_profile_entry_t *obd = &ix.entries[2];
    CHECK(obd->signals == 2 && obd->can == 1 && obd->obd == 1, "counts %d %d %d", obd->signals, obd->can, obd->obd);
    CHECK(strcmp(obd->years, "2020–") == 0, "open-ended years %s", obd->years);

    int idx[8];
    CHECK(tui_profiles_filter(&ix, "", idx, 8) == 3, "empty filter");
    CHECK(tui_profiles_filter(&ix, "bench", idx, 8) == 2, "one token");
    int n = tui_profiles_filter(&ix, "BENCH obd", idx, 8);
    CHECK(n == 1 && idx[0] == 2, "two tokens, case-insensitive");
    CHECK(tui_profiles_filter(&ix, "bench nissan", idx, 8) == 0, "every token must match");
    CHECK(tui_profiles_find(&ix, "test__bench_car__t1__can.vsp") == 1, "find by file name");

    /* a recorded path that moved: same file name in the profile folder */
    char res[1024], want[1100];
    snprintf(want, sizeof(want), "%s/test__bench_car__t1__can.vsp", dir);
    CHECK(tui_profiles_resolve(want, NULL, res, sizeof(res)) && strcmp(res, want) == 0, "existing path kept");
    CHECK(tui_profiles_resolve("../old_checkout/profiles/test__bench_car__t1__can.vsp", dir, res, sizeof(res)) &&
          strcmp(res, want) == 0, "moved path found by file name: %s", res);
    CHECK(!tui_profiles_resolve("../old_checkout/profiles/test__bench_car__t1__can.vsp", NULL, res, sizeof(res)) &&
          res[0] == '\0', "moved path, no profile folder");
    CHECK(!tui_profiles_resolve("/nowhere/missing.vsp", dir, res, sizeof(res)), "unknown file name");
    CHECK(!tui_profiles_resolve("/nowhere/", dir, res, sizeof(res)), "no file name");
    char small[16];
    CHECK(!tui_profiles_resolve("x/test__bench_car__t1__can.vsp", dir, small, sizeof(small)), "out too small");

    char y[32];
    tui_profiles_years(y, sizeof(y), 0, 0, true);
    CHECK(strcmp(y, "-") == 0, "ascii unknown years");
    tui_profiles_years(y, sizeof(y), 2019, 0, true);
    CHECK(strcmp(y, "2019-") == 0, "ascii open years");

    /* drafts: newest first, at most 5 */
    char tmpl[] = "/tmp/tui_profiles_test.XXXXXX";
    char *tmp = mkdtemp(tmpl);
    CHECK(tmp != NULL, "temp dir");
    if (tmp) {
        char src[1100], d[1200], f[1300];
        snprintf(src, sizeof(src), "%s/test__bench_car__t1__can.vsp", dir);
        for (int i = 0; i < 7; i++) {
            snprintf(d, sizeof(d), "%s/2026102%d_100000", tmp, i);
            mkdir(d, 0755);
            snprintf(f, sizeof(f), "%s/ptcan_discover_draft.vsp", d);
            copy_file(src, f);
            char cmd[1400];
            snprintf(cmd, sizeof(cmd), "touch -t 2026102%d1000 '%s'", i, f);
            if (system(cmd) != 0) CHECK(0, "touch");
        }
        tui_profiles_scan_drafts(&ix, tmp);
        CHECK(ix.ndrafts == 5, "5 drafts: %d", ix.ndrafts);
        CHECK(ix.ndrafts > 0 && strstr(ix.drafts[0].path, "20261026_100000") != NULL, "newest first: %s",
              ix.ndrafts ? ix.drafts[0].path : "");
        CHECK(ix.ndrafts > 0 && ix.drafts[0].signals == 1, "draft signal count");
        char cmd[1200];
        snprintf(cmd, sizeof(cmd), "rm -r '%s'", tmp);
        if (system(cmd) != 0) CHECK(0, "cleanup");
    }
    tui_profiles_free(&ix);

    tui_profiles_open(&ix, NULL, false);
    CHECK(!ix.have_dir && ix.total == 0 && tui_profiles_done(&ix), "no folder");
    tui_profiles_free(&ix);

    /* Vehicle guess: the three fixture profiles plus test_profile.vsp (9 passive
       frames; its diagnostic ids 0x7DF / 0x7E8 are not broadcast frames). */
    char gtmpl[] = "/tmp/tui_profiles_guess.XXXXXX";
    char *gdir = mkdtemp(gtmpl);
    CHECK(gdir != NULL, "guess temp dir");
    if (gdir) {
        const char *names[] = { "other__roadster__r2__2009_2014.vsp", "test__bench_car__t1__can.vsp",
                                "test__bench_car__t1__obd.vsp" };
        char from[1200], to[1300];
        for (int i = 0; i < 3; i++) {
            snprintf(from, sizeof(from), "%s/%s", dir, names[i]);
            snprintf(to, sizeof(to), "%s/%s", gdir, names[i]);
            copy_file(from, to);
        }
        snprintf(from, sizeof(from), "%s/test_profile.vsp", argv[1]);
        snprintf(to, sizeof(to), "%s/test_profile.vsp", gdir);
        copy_file(from, to);

        tui_profiles_open(&ix, gdir, false);
        tui_profile_guess_t g[TUI_GUESS_MAX];
        tui_seen_frame_t one = { 0x0F3, false, 8 };
        CHECK(tui_profiles_guess(&ix, &one, 1, g, TUI_GUESS_MAX) == 0, "no guess before the index is complete");
        tui_profiles_step(&ix, 64, 0);
        int tp = tui_profiles_find(&ix, "test_profile.vsp");
        CHECK(tp >= 0 && ix.entries[tp].nframes == 9, "test_profile: 9 passive frames (%d)",
              tp >= 0 ? ix.entries[tp].nframes : -1);

        /* the stream_direct traffic: 8 of test_profile's 9 frames, plus others */
        tui_seen_frame_t seen[] = {
            { 0x0F3, false, 8 }, { 0x0A5, false, 8 }, { 0x1A0, false, 8 }, { 0x0D9, false, 6 },
            { 0x1E0, false, 8 }, { 0x3B0, false, 8 }, { 0x2C4, false, 8 }, { 0x3B2, false, 8 },
            { 0x316, false, 8 }, { 0x4F0, false, 2 }, { 0x18DAF110, true, 4 },
        };
        int ns = (int)(sizeof(seen) / sizeof(seen[0]));
        int n = tui_profiles_guess(&ix, seen, ns, g, TUI_GUESS_MAX);
        CHECK(n == 1 && g[0].entry == tp && g[0].hits == 8 && g[0].frames == 9,
              "stream_direct: test_profile alone (n %d, hits %d/%d)", n, n ? g[0].hits : 0, n ? g[0].frames : 0);
        seen[3].dlc = 8;   /* 0x0D9 is 6 bytes in the profile */
        n = tui_profiles_guess(&ix, seen, ns, g, TUI_GUESS_MAX);
        CHECK(n == 1 && g[0].hits == 7, "a dlc that differs is not a hit (hits %d)", n ? g[0].hits : 0);

        /* only 0x0F3: the two bench car profiles tie, test_profile misses 8 frames */
        n = tui_profiles_guess(&ix, &one, 1, g, TUI_GUESS_MAX);
        bool bench = n == 2;
        for (int i = 0; i < n && bench; i++) bench = strstr(ix.entries[g[i].entry].file, "test__bench_car") != NULL;
        CHECK(bench && g[0].score == g[1].score, "0x0F3 alone: both bench car profiles (n %d)", n);
        CHECK(tui_profiles_guess(&ix, &one, 1, g, 1) == 0, "more candidates than max: no list");
        tui_seen_frame_t ext = { 0x0F3, true, 8 };
        CHECK(tui_profiles_guess(&ix, &ext, 1, g, TUI_GUESS_MAX) == 0, "an extended id is another frame");
        tui_seen_frame_t other = { 0x123, false, 8 };
        CHECK(tui_profiles_guess(&ix, &other, 1, g, TUI_GUESS_MAX) == 0, "nothing matches");
        tui_profiles_free(&ix);

        char cmd[1200];
        snprintf(cmd, sizeof(cmd), "rm -r '%s'", gdir);
        if (system(cmd) != 0) CHECK(0, "guess cleanup");
    }

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
