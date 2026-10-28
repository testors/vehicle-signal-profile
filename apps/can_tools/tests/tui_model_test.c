/**
 * @file tui_model_test.c
 * @brief TUI model, grid, replay loading and recorder (plain C, no ncurses/ObjC/Bluetooth)
 */

#include <dirent.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utime.h>

#include "frame_tsv.h"
#include "tui_grid.h"
#include "tui_model.h"
#include "tui_recorder.h"
#include "tui_replay.h"
#include "tui_capture.h"
#include "tui_screens.h"

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

#define CHECK_STR(a, b) CHECK(strcmp((a), (b)) == 0, "\"%s\" != \"%s\"", (a), (b))

static char g_tmpdir[1024];

static void enter_tmpdir(void) {
    const char *base = getenv("TMPDIR");
    if (!base || !*base) base = "/tmp";
    snprintf(g_tmpdir, sizeof(g_tmpdir), "%s/tui_model_test.XXXXXX", base);
    if (!mkdtemp(g_tmpdir) || chdir(g_tmpdir) != 0) {
        fprintf(stderr, "cannot create temp dir\n");
        exit(1);
    }
}

static void leave_tmpdir(void) {
    char cmd[1200];
    if (chdir("/") != 0) return;
    snprintf(cmd, sizeof(cmd), "rm -rf '%s'", g_tmpdir);
    if (system(cmd) != 0) fprintf(stderr, "cannot remove %s\n", g_tmpdir);
}

static can_frame_t frame(uint32_t id, int64_t ts, int dlc, uint8_t b0) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.can_id = id;
    f.is_extended = id > 0x7FF;
    f.dlc = (uint8_t)dlc;
    f.timestamp_ms = ts;
    f.data[0] = b0;
    for (int i = 1; i < dlc; i++) f.data[i] = (uint8_t)(0x10 + i);
    return f;
}

/*******************************************************************************
 * Raw table
 ******************************************************************************/

static void test_raw(void) {
    tui_raw_table_t t;
    tui_raw_init(&t);
    /* 0x123 at 100 Hz with byte 0 counting; 0x0A5 at 20 Hz constant; ext at 2 Hz */
    for (int64_t ms = 0; ms < 3000; ms += 10) {
        can_frame_t f = frame(0x123, ms, 8, (uint8_t)(ms / 10));
        tui_raw_update(&t, &f, ms);
        if (ms % 50 == 0) {
            can_frame_t g = frame(0x0A5, ms, 4, 0x55);
            tui_raw_update(&t, &g, ms);
        }
        if (ms % 500 == 0) {
            can_frame_t h = frame(0x18DAF110, ms, 3, 0x01);
            tui_raw_update(&t, &h, ms);
        }
    }
    CHECK(t.count == 3, "count %d", t.count);
    const tui_raw_entry_t *list[8];
    int n = tui_raw_sorted(&t, TUI_SORT_HZ, 3000, NULL, list, 8);
    CHECK(n == 3 && list[0]->id == 0x123 && list[1]->id == 0x0A5 && list[2]->id == 0x18DAF110,
          "hz order");
    CHECK(fabs(tui_raw_hz(list[0], 3000) - 100.0) < 1e-9, "100 Hz: %f", tui_raw_hz(list[0], 3000));
    CHECK(fabs(tui_raw_hz(list[1], 3000) - 20.0) < 1e-9, "20 Hz: %f", tui_raw_hz(list[1], 3000));
    CHECK(fabs(tui_raw_hz(list[2], 3000) - 2.0) < 1e-9, "2 Hz: %f", tui_raw_hz(list[2], 3000));
    CHECK(tui_raw_hz(list[0], 6000) == 0.0, "rate decays to 0 after silence");

    n = tui_raw_sorted(&t, TUI_SORT_ID, 3000, NULL, list, 8);
    CHECK(n == 3 && list[0]->id == 0x0A5 && list[1]->id == 0x123 && list[2]->ext, "id order, ext last");
    n = tui_raw_sorted(&t, TUI_SORT_CHANGED, 3000, NULL, list, 8);
    CHECK(n == 3 && list[0]->id == 0x123, "changed order: counting id first");

    const tui_raw_entry_t *e = NULL;
    n = tui_raw_sorted(&t, TUI_SORT_HZ, 3000, NULL, list, 8);
    e = list[0];
    CHECK(e->byte_change_ms[0] == 2990 && e->byte_change_ms[1] == 0, "byte change times %lld %lld",
          (long long)e->byte_change_ms[0], (long long)e->byte_change_ms[1]);
    CHECK(e->last_change_ms == 2990 && e->count == 300 && e->first_ms == 0 && e->last_ms == 2990,
          "entry bookkeeping");

    CHECK(tui_raw_sorted(&t, TUI_SORT_HZ, 3000, "0x12", list, 8) == 1 && list[0]->id == 0x123, "filter 0x12");
    CHECK(tui_raw_sorted(&t, TUI_SORT_HZ, 3000, "0a", list, 8) == 1 && list[0]->id == 0x0A5, "filter 0a");
    CHECK(tui_raw_sorted(&t, TUI_SORT_HZ, 3000, "18DA", list, 8) == 1 && list[0]->ext, "filter ext");
    CHECK(tui_raw_sorted(&t, TUI_SORT_HZ, 3000, "2", list, 8) == 0, "prefix only");
    CHECK(tui_raw_sorted(&t, TUI_SORT_HZ, 3000, "rpm", list, 8) == 0, "non-hex filter matches no id");

    /* growth */
    for (uint32_t id = 0x200; id < 0x200 + 1000; id++) {
        can_frame_t f = frame(id, 4000, 1, 0);
        CHECK(tui_raw_update(&t, &f, 4000) != NULL, "insert %X", id);
    }
    CHECK(t.count == 1003, "count after growth %d", t.count);
    n = tui_raw_sorted(&t, TUI_SORT_ID, 4000, "5E7", list, 8);
    CHECK(n == 1 && list[0]->id == 0x5E7, "lookup after rehash");
    tui_raw_free(&t);
}

/*******************************************************************************
 * Signals: hz, age, stale, history
 ******************************************************************************/

static tui_signal_t make_signal(const char *name, const char *group, double cycle_ms, double factor,
                                bool values, bool diag) {
    tui_signal_desc_t d = {
        .name = name, .unit = "u", .group = group, .diagnostic = diag, .has_value_table = values,
        .factor = factor, .cycle_ms = cycle_ms, .frame_id = 0x123, .request_hex = "01 2F",
        .decoder_index = 0,
    };
    tui_signal_t s;
    tui_signal_init(&s, &d);
    return s;
}

static void test_signal_state(void) {
    tui_signal_t s = make_signal("Speed", "custom", 20, 0.01, false, false);
    CHECK(tui_signal_state(&s, 0) == TUI_SIG_NEVER, "never");
    for (int64_t ms = 1000; ms <= 2000; ms += 20) tui_signal_sample(&s, ms, true, false, (double)ms, NULL);
    CHECK(tui_signal_state(&s, 2010) == TUI_SIG_LIVE, "live");
    CHECK(fabs(tui_signal_hold_ms(&s, 2010) - 300.0) < 1e-9, "3 x 20 ms is below the 300 ms minimum hold");
    CHECK(tui_signal_state(&s, 2300) == TUI_SIG_LIVE, "age = 300 ms is still live");
    CHECK(tui_signal_state(&s, 2301) == TUI_SIG_STALE, "stale after the 300 ms minimum hold");
    CHECK(tui_signal_state(&s, 12000) == TUI_SIG_STALE, "stale up to 10 s");
    CHECK(tui_signal_state(&s, 12001) == TUI_SIG_DEAD, "dead after 10 s");
    CHECK(fabs(tui_signal_hz(&s, 2050) - 50.0) < 1e-9, "50 Hz from the first sample: %f", tui_signal_hz(&s, 2050));
    /* 50 Hz over Bluetooth: a 94 ms burst gap is not stale, 350 ms is */
    tui_signal_t bt = make_signal("Brake", "custom", 20, 0.1, false, false);
    for (int64_t ms = 0; ms <= 5000; ms += 20) tui_signal_sample(&bt, ms, true, false, 1.0, NULL);
    CHECK(tui_signal_state(&bt, 5094) == TUI_SIG_LIVE, "94 ms gap on a 50 Hz signal: live");
    CHECK(tui_signal_state(&bt, 5350) == TUI_SIG_STALE, "350 ms on a 50 Hz signal: stale");
    CHECK(strcmp(s.src, "0x123") == 0 && s.decimals == 2 && !s.discrete, "static info");

    /* no cycle: period from the measured rate (10 Hz -> 100 ms) */
    tui_signal_t t = make_signal("Slow", "custom", 0, 1, false, false);
    for (int64_t ms = 0; ms <= 5000; ms += 100) tui_signal_sample(&t, ms, true, false, 1.0, NULL);
    CHECK(fabs(tui_signal_period_ms(&t, 5050) - 100.0) < 1e-6, "measured period %f", tui_signal_period_ms(&t, 5050));
    /* the rate is the sample span, so a stopped signal keeps its true period */
    CHECK(tui_signal_state(&t, 5300) == TUI_SIG_LIVE && tui_signal_state(&t, 5301) == TUI_SIG_STALE,
          "stale after 3 x measured period");
    CHECK(fabs(tui_signal_hz(&t, 7000) - 10.0) < 1e-9, "stopped signal keeps its rate in the window");
    CHECK(tui_signal_hz(&t, 10001) < 0, "no samples in the last 5 s: unknown");
    /* 1 Hz: (n-1)/(t_last-t_first) gives exactly 1.0 at any time */
    tui_signal_t one = make_signal("One", "custom", 0, 1, false, false);
    for (int64_t ms = 0; ms <= 20000; ms += 1000) tui_signal_sample(&one, ms, true, false, 1.0, NULL);
    CHECK(fabs(tui_signal_hz(&one, 20000) - 1.0) < 1e-9 && fabs(tui_signal_hz(&one, 20999) - 1.0) < 1e-9,
          "1 Hz without jitter");
    /* declared 100 Hz but a gateway forwards it at 2 Hz: judged by the measured period */
    tui_signal_t gw = make_signal("Steer", "custom", 10, 0.1, false, false);
    for (int64_t ms = 0; ms <= 4000; ms += 500) tui_signal_sample(&gw, ms, true, false, 1.0, NULL);
    CHECK(fabs(tui_signal_period_ms(&gw, 4499) - 500.0) < 1e-6, "slower than declared: measured period");
    CHECK(tui_signal_state(&gw, 4499) == TUI_SIG_LIVE && tui_signal_state(&gw, 5500) == TUI_SIG_LIVE &&
          tui_signal_state(&gw, 5501) == TUI_SIG_STALE, "gateway-rate signal stale after 3 x 500 ms");
    CHECK(fabs(tui_signal_hold_ms(&gw, 4499) - 1500.0) < 1e-9, "slow signal: hold = 3 periods");
    /* arriving faster than declared: the declared cycle stays the limit */
    tui_signal_t fast = make_signal("Fast", "custom", 100, 1, false, false);
    for (int64_t ms = 0; ms <= 1000; ms += 10) tui_signal_sample(&fast, ms, true, false, 1.0, NULL);
    CHECK(fabs(tui_signal_period_ms(&fast, 1000) - 100.0) < 1e-6, "faster than declared: declared cycle");
    /* never measured: 1 s */
    tui_signal_t u = make_signal("Once", "custom", 0, 1, false, false);
    tui_signal_sample(&u, 1000, true, false, 1.0, NULL);
    CHECK(tui_signal_state(&u, 4000) == TUI_SIG_LIVE && tui_signal_state(&u, 4001) == TUI_SIG_STALE,
          "default 1 s period");

    tui_signal_t d = make_signal("Fuel", "custom", 0, 1, false, true);
    CHECK_STR(d.src, "obd");
    CHECK_STR(d.request, "OBD 01 2F");
    tui_signal_t g = make_signal("Gear", "gear", 50, 1, false, false);
    CHECK(g.discrete && g.group == TUI_GROUP_GEAR, "gear group is discrete");
}

static void test_ordering(void) {
    tui_signal_t s[5] = {
        make_signal("zeta", "custom", 0, 1, false, false),
        make_signal("Fuel", "custom", 0, 1, false, true),
        make_signal("brake", "brake", 0, 1, false, false),
        make_signal("Alpha", "custom", 0, 1, false, false),
        make_signal("rpm", "rpm", 0, 1, false, false),
    };
    qsort(s, 5, sizeof(s[0]), tui_signal_compare);
    CHECK_STR(s[0].name, "rpm");
    CHECK_STR(s[1].name, "brake");
    CHECK_STR(s[2].name, "Alpha");
    CHECK_STR(s[3].name, "zeta");
    CHECK_STR(s[4].name, "Fuel");
}

static void test_sparkline(void) {
    char out[512];
    /* one sample per 1.25 s cell, values 0..7 -> every level once */
    tui_signal_t s = make_signal("x", "custom", 20, 1, false, false);
    for (int i = 0; i < 8; i++) tui_signal_sample(&s, 10000 + i * 1250 + 100, true, false, (double)i, NULL);
    tui_signal_sparkline(&s, 20000, 8, false, out, sizeof(out));
    CHECK_STR(out, "▁▂▃▄▅▆▇█");
    tui_signal_sparkline(&s, 20000, 8, true, out, sizeof(out));
    CHECK_STR(out, " _.-~=*#");

    /* constant -> flat line; a 20 ms signal does not carry over 1.25 s gaps */
    tui_signal_t c = make_signal("c", "custom", 20, 1, false, false);
    for (int i = 0; i < 8; i++) tui_signal_sample(&c, 10000 + i * 1250 + 100, true, false, 5.0, NULL);
    tui_signal_sparkline(&c, 20000, 8, false, out, sizeof(out));
    CHECK_STR(out, "━━━━━━━━");

    /* only the last two cells have samples; the rest stay blank */
    tui_signal_t e = make_signal("e", "custom", 20, 1, false, false);
    tui_signal_sample(&e, 17600, true, false, 0.0, NULL);
    tui_signal_sample(&e, 18900, true, false, 1.0, NULL);
    tui_signal_sparkline(&e, 20000, 8, false, out, sizeof(out));
    CHECK_STR(out, "      ▁█");

    /* 1 Hz signal over 0.5 s cells: the value carries between samples */
    tui_signal_t slow = make_signal("slow", "custom", 1000, 1, false, false);
    for (int64_t ms = 10000; ms <= 20000; ms += 1000)
        tui_signal_sample(&slow, ms, true, false, (double)((ms / 1000) % 2), NULL);
    tui_signal_sparkline(&slow, 20000, 20, false, out, sizeof(out));
    CHECK(strchr(out, ' ') == NULL, "no gaps for a 1 Hz signal: \"%s\"", out);

    /* stopped signal: blank after 3 periods */
    tui_signal_t stop = make_signal("stop", "custom", 100, 1, false, false);
    for (int64_t ms = 10000; ms < 13000; ms += 100) tui_signal_sample(&stop, ms, true, false, 1.0, NULL);
    tui_signal_sparkline(&stop, 20000, 10, false, out, sizeof(out));
    CHECK_STR(out, "━━━       ");

    double mn = 0, mx = 0;
    CHECK(tui_signal_minmax(&s, 20000, &mn, &mx) && mn == 0.0 && mx == 7.0, "minmax %f %f", mn, mx);

    /* discrete text history, one token per second, oldest dropped to fit */
    static const char *labels[3] = { "P", "N", "D" };
    tui_signal_t g = make_signal("Gear", "gear", 50, 1, true, false);
    for (int64_t ms = 10000; ms < 20000; ms += 50) {
        int k = (int)((ms - 10000) / 4000);
        tui_signal_sample(&g, ms, false, true, 0, labels[k]);
    }
    tui_signal_discrete_history(&g, 20000, 40, out, sizeof(out));
    CHECK_STR(out, "P P P P N N N N D D");
    tui_signal_discrete_history(&g, 20000, 7, out, sizeof(out));
    CHECK_STR(out, "N N D D");
}

/* Braille timeline as code points; *_on: dots in the left / right column. */
static int timeline_cps(const tui_signal_t *s, int64_t now, int cells, unsigned int *cp) {
    char out[2048];
    unsigned char fg[256];
    int n = tui_signal_braille(s, now, cells, out, sizeof(out), fg);
    const char *p = out;
    for (int i = 0; i < n; i++) p += tui_utf8_decode(p, &cp[i]);
    return n;
}

static bool left_on(unsigned int cp) { return cp >= 0x2800 && ((cp - 0x2800) & 0x47) != 0; }
static bool right_on(unsigned int cp) { return cp >= 0x2800 && ((cp - 0x2800) & 0xB8) != 0; }

/* 50 Hz with a 94 ms burst gap every second (x.500 -> x.594 s): the samples in [from, to] */
static void feed_bursty(tui_signal_t *s, int64_t from, int64_t to) {
    for (int64_t t = from; t <= to; t++) {
        int64_t ph = t % 1000;
        bool grid = t % 20 == 0 && !(ph > 500 && ph < 600);
        if (grid || ph == 594) tui_signal_sample(s, t, true, false, (double)((t / 100) % 5), NULL);
    }
}

static void test_timeline_hold(void) {
    unsigned int c[256];

    /* 50 Hz over Bluetooth: no gap at any width or scroll position */
    static const int widths[] = { 40, 69, 100, 120, 160 };
    enum { NW = sizeof(widths) / sizeof(widths[0]) };
    int blanks[NW] = { 0 }, cells[NW] = { 0 };
    tui_signal_t bt = make_signal("Brake", "custom", 20, 0.1, false, false);
    feed_bursty(&bt, 0, 19993);
    for (int64_t now = 20000; now < 21000; now += 7) {
        feed_bursty(&bt, now - 6, now);
        for (int w = 0; w < NW; w++) {
            cells[w] = timeline_cps(&bt, now, widths[w], c);
            for (int i = 0; i < cells[w]; i++) blanks[w] += !left_on(c[i]) + !right_on(c[i]);
        }
    }
    for (int w = 0; w < NW; w++)
        CHECK(cells[w] == widths[w] && blanks[w] == 0, "94 ms gaps, %d cells: %d blank half cells",
              widths[w], blanks[w]);

    /* 50 Hz stopped for 1.5 s: held for 300 ms, then blank, drawn again from 15.5 s */
    tui_signal_t stop = make_signal("Brake", "custom", 20, 0.1, false, false);
    for (int64_t ms = 0; ms <= 20000; ms += 20) {
        if (ms > 14000 && ms < 15500) continue;
        tui_signal_sample(&stop, ms, true, false, 1.0 + (double)((ms / 100) % 5), NULL);
    }
    int n = timeline_cps(&stop, 20000, 100, c);   /* 100 ms cells */
    int bad = n == 100 ? -1 : 0;
    for (int i = 0; i < n; i++) {
        int64_t cs = 10000 + 100 * i, ce = cs + 100;
        bool both = left_on(c[i]) && right_on(c[i]), none = !left_on(c[i]) && !right_on(c[i]);
        if ((ce <= 14300 || cs >= 15500) && !both) bad = i;
        if (cs >= 14300 && ce <= 15500 && !none) bad = i;
    }
    CHECK(bad < 0, "1.5 s stop: drawn up to 300 ms after, blank to 15.5 s, drawn again (cell %d)", bad);

    /* 1 Hz: the leftmost cell holds the sample from 0.5 s before the window,
       and that value is part of the scale (0 at the bottom, 10 halfway) */
    tui_signal_t one = make_signal("Fuel", "custom", 1000, 1, false, false);
    for (int k = 0; k < 20; k++)
        tui_signal_sample(&one, 500 + 1000 * k, true, false, k < 10 ? 0.0 : (k % 2 ? 20.0 : 10.0), NULL);
    n = timeline_cps(&one, 20000, 20, c);         /* 500 ms cells */
    int blank = 0;
    for (int i = 0; i < n; i++) blank += !left_on(c[i]) + !right_on(c[i]);
    CHECK(n == 20 && blank == 0, "1 Hz: whole width drawn (%d blank half cells)", blank);
    CHECK(c[0] == tui_braille_char(1, 1), "leftmost cell: the 9.5 s sample at the bottom (%X)", c[0]);
    CHECK(c[1] == tui_braille_char(3, 3), "10 on the 0..20 scale: height 3 (%X)", c[1]);

    /* a sample older than the hold (3 s) is not carried into the window */
    tui_signal_t old = make_signal("Fuel", "custom", 1000, 1, false, false);
    tui_signal_sample(&old, 6500, true, false, 1.0, NULL);
    tui_signal_sample(&old, 13000, true, false, 1.0, NULL);
    timeline_cps(&old, 20000, 20, c);
    CHECK(c[0] == ' ' && c[6] != ' ' && c[19] == ' ', "3.5 s before the window: blank left edge");
}

/*******************************************************************************
 * Formatting
 ******************************************************************************/

static void test_format(void) {
    char b[64];
    CHECK(tui_decimals_for_factor(1) == 0 && tui_decimals_for_factor(10) == 0 &&
          tui_decimals_for_factor(0.25) == 1 && tui_decimals_for_factor(0.1) == 1 &&
          tui_decimals_for_factor(0.01) == 2 && tui_decimals_for_factor(0.016) == 2 &&
          tui_decimals_for_factor(0.0001) == 3 && tui_decimals_for_factor(0.392156862745098) == 1 &&
          tui_decimals_for_factor(2.5) == 0, "decimals");
    tui_format_value(b, sizeof(b), 6480, 0); CHECK_STR(b, "6,480");
    tui_format_value(b, sizeof(b), 142.6, 1); CHECK_STR(b, "142.6");
    tui_format_value(b, sizeof(b), -12.5, 1); CHECK_STR(b, "-12.5");
    tui_format_value(b, sizeof(b), -0.04, 1); CHECK_STR(b, "0.0");
    tui_format_value(b, sizeof(b), 999.96, 1); CHECK_STR(b, "1,000.0");
    tui_format_value(b, sizeof(b), -1234567.891, 2); CHECK_STR(b, "-1,234,567.89");
    tui_format_value(b, sizeof(b), 999.4, 0); CHECK_STR(b, "999");
    tui_format_signal_value(b, sizeof(b), 4431.8, 1); CHECK_STR(b, "4,432");
    tui_format_signal_value(b, sizeof(b), 999.4, 1); CHECK_STR(b, "999.4");
    tui_format_signal_value(b, sizeof(b), 999.96, 1); CHECK_STR(b, "1,000");
    tui_format_signal_value(b, sizeof(b), -2500.25, 2); CHECK_STR(b, "-2,500");
    tui_format_signal_value(b, sizeof(b), 142.6, 1); CHECK_STR(b, "142.6");
    /* gauge values: no "-0" / "-0.0", no thousands separators */
    tui_format_fixed(b, sizeof(b), 102 * 0.3 - 30.6, 1); CHECK_STR(b, "0.0");   /* -3.55e-15: released brake */
    tui_format_fixed(b, sizeof(b), -0.04, 1); CHECK_STR(b, "0.0");
    tui_format_fixed(b, sizeof(b), -0.4, 0); CHECK_STR(b, "0");
    tui_format_fixed(b, sizeof(b), -0.3, 1); CHECK_STR(b, "-0.3");
    tui_format_fixed(b, sizeof(b), -0.6, 0); CHECK_STR(b, "-1");
    tui_format_fixed(b, sizeof(b), 6480, 0); CHECK_STR(b, "6480");
    tui_format_age(b, sizeof(b), 10); CHECK_STR(b, "10ms");
    tui_format_age(b, sizeof(b), 800); CHECK_STR(b, "800ms");
    tui_format_age(b, sizeof(b), 1840); CHECK_STR(b, "1.8s");
    tui_format_age(b, sizeof(b), 12400); CHECK_STR(b, "12s");
    tui_format_hz(b, sizeof(b), 100.0, false); CHECK_STR(b, "100");
    tui_format_hz(b, sizeof(b), 1.0, false); CHECK_STR(b, "1.0");
    tui_format_hz(b, sizeof(b), -1, false); CHECK_STR(b, "—");
    tui_format_hz(b, sizeof(b), -1, true); CHECK_STR(b, "-");
    tui_format_count(b, sizeof(b), 1842); CHECK_STR(b, "1,842");
    tui_format_mmss(b, sizeof(b), 761000); CHECK_STR(b, "12:41");
    tui_format_hms(b, sizeof(b), 761000); CHECK_STR(b, "00:12:41");
    tui_format_can_id(b, sizeof(b), 0xF3, false); CHECK_STR(b, "0x0F3");
    tui_format_can_id(b, sizeof(b), 0x18DAF110, true); CHECK_STR(b, "0x18DAF110");
}

/*******************************************************************************
 * Bus classifier
 ******************************************************************************/

static void feed_ids(tui_classifier_t *c, int64_t from, int64_t to, int ids, int period_ms) {
    for (int64_t ms = from; ms < to; ms += period_ms) {
        for (int k = 0; k < ids; k++) {
            can_frame_t f = frame(0x100 + (uint32_t)k, ms, 8, (uint8_t)ms);
            tui_classifier_frame(c, &f, ms);
        }
        tui_classifier_tick(c, ms);
    }
}

static void test_classifier(void) {
    tui_classifier_t c;
    tui_classifier_init(&c);
    tui_classifier_start(&c, 0);
    CHECK(c.bus == TUI_BUS_DETECTING && strcmp(tui_bus_text(c.bus), "detecting…") == 0, "detecting");
    feed_ids(&c, 0, 2990, 3, 10);                     /* 3 IDs at 100 Hz */
    CHECK(c.bus == TUI_BUS_DETECTING, "no result before 3 s");
    CHECK(tui_classifier_tick(&c, 3000) && c.bus == TUI_BUS_DIRECT && c.direct_seen && c.last_fast_ids == 3,
          "DIRECT after the first window");
    /* next window [5 s, 8 s): slow IDs only */
    feed_ids(&c, 3000, 8000, 3, 1000);
    CHECK(tui_classifier_tick(&c, 8000), "second window evaluated at 8 s");
    CHECK(c.bus == TUI_BUS_GATEWAY && c.direct_seen && c.windows == 2, "GATEWAY later, DIRECT stays seen");
    /* window [10 s, 13 s): nothing */
    CHECK(!tui_classifier_tick(&c, 12999), "not yet");
    CHECK(tui_classifier_tick(&c, 13000) && c.bus == TUI_BUS_SILENT && c.direct_seen, "SILENT, still locked");
    CHECK(c.window_start_ms == 15000 && c.window_end_ms == 18000, "every 5 s over the last 3 s");
    /* frames in the 2 s gap are ignored */
    can_frame_t f = frame(0x100, 14000, 8, 0);
    tui_classifier_frame(&c, &f, 14000);
    CHECK(!c.accumulating, "gap frames ignored");
    tui_classifier_free(&c);

    tui_classifier_t s;
    tui_classifier_init(&s);
    tui_classifier_start(&s, 1000);
    CHECK(tui_classifier_tick(&s, 4000) && s.bus == TUI_BUS_SILENT && !s.direct_seen, "SILENT start");
    tui_classifier_note_direct(&s);
    CHECK(s.direct_seen, "discover baseline DIRECT locks");
    tui_classifier_free(&s);

    tui_classifier_t g;
    tui_classifier_init(&g);
    tui_classifier_start(&g, 0);
    feed_ids(&g, 0, 3000, 1, 10);                      /* one fast ID is not DIRECT */
    CHECK(tui_classifier_tick(&g, 3000) && g.bus == TUI_BUS_GATEWAY, "one 100 Hz ID: GATEWAY");
    tui_classifier_free(&g);
}

/*******************************************************************************
 * Log
 ******************************************************************************/

static void test_log(void) {
    static tui_log_t log;
    memset(&log, 0, sizeof(log));
    tui_log_add(&log, 1, "[INFO] connected");
    tui_log_add(&log, 2, "[WARN] Command timeout: ATZ");
    tui_log_add(&log, 3, "\n[ERROR] Connection lost\nsecond line\n");
    CHECK(log.count == 4, "count %d", log.count);
    CHECK(strcmp(tui_log_get(&log, 0)->text, "second line") == 0, "newest first");
    CHECK(tui_log_get(&log, 1)->level == TUI_LOG_ERROR, "error level");
    CHECK(tui_log_get(&log, 2)->level == TUI_LOG_WARN, "warn level");
    CHECK(strcmp(tui_log_get(&log, 3)->text, "connected") == 0 && tui_log_get(&log, 3)->level == TUI_LOG_INFO,
          "[INFO] stripped");
    for (int i = 0; i < TUI_LOG_LINES + 10; i++) tui_log_add(&log, i, "x");
    CHECK(log.count == TUI_LOG_LINES && tui_log_get(&log, TUI_LOG_LINES) == NULL, "ring bound");
}

/*******************************************************************************
 * Grid
 ******************************************************************************/

static void test_grid(void) {
    CHECK(tui_utf8_width("abc") == 3 && tui_utf8_width("가나") == 4 && tui_utf8_width("▁█") == 2 &&
          tui_utf8_width("—·…") == 3 && tui_utf8_width("é") == 1, "widths");
    tui_grid_t *g = tui_grid_new(6, 2);
    int used = tui_grid_put(g, 0, 0, "가나다", 'b', 5);
    CHECK(used == 5, "wide clip used %d", used);
    CHECK(tui_grid_cell(g, 0, 0)->width == 2 && tui_grid_cell(g, 1, 0)->width == 0 &&
          strcmp(tui_grid_cell(g, 4, 0)->ch, " ") == 0, "third wide char became a space");
    tui_grid_put(g, 1, 0, "x", '.', -1);
    CHECK(strcmp(tui_grid_cell(g, 0, 0)->ch, " ") == 0, "overwriting half of a wide char blanks it");
    tui_grid_put(g, 0, 1, "abcdefgh", 'd', -1);
    CHECK(strcmp(tui_grid_cell(g, 5, 1)->ch, "f") == 0, "clipped at the edge");

    char buf[64];
    tui_text_fit(buf, sizeof(buf), "Steering_Angle", 10, true, false);
    CHECK_STR(buf, "Steering_…");
    tui_text_fit(buf, sizeof(buf), "Steering_Angle", 10, true, true);
    CHECK_STR(buf, "Steering_.");
    tui_text_fit(buf, sizeof(buf), "short", 10, true, false);
    CHECK_STR(buf, "short");
    tui_text_fit(buf, sizeof(buf), "가나다라", 5, false, false);
    CHECK_STR(buf, "가나");

    FILE *f = tmpfile();
    tui_grid_dump_text(g, f);
    tui_grid_dump_roles(g, f);
    long n = ftell(f);
    rewind(f);
    char dump[256] = "";
    if (fread(dump, 1, (size_t)n, f) != (size_t)n) dump[0] = '\0';
    fclose(f);
    CHECK(strcmp(dump, " x나  \nabcdef\nb.bbb.\ndddddd\n") == 0, "dump: [%s]", dump);
    tui_grid_free(g);
}

/*******************************************************************************
 * Replay loading and the recorder
 ******************************************************************************/

static void write_tsv(const char *path, int64_t t0, int count, uint32_t id) {
    FILE *f = fopen(path, "w");
    frame_tsv_write_header(f);
    for (int i = 0; i < count; i++) {
        can_frame_t fr = frame(id, t0 + i * 10, 8, (uint8_t)i);
        frame_tsv_write_row(f, &fr);
    }
    fclose(f);
}

static void test_replay(void) {
    frame_list_t a = {0}, b = {0};
    for (int i = 0; i < 100; i++) {
        can_frame_t f = frame(0x100, 1000 + i * 10, 1, 0);
        frame_list_push(&a, &f);
    }
    for (int i = 0; i < 10; i++) {
        can_frame_t f = frame(0x200, 10000 + i * 10, 1, 0);
        frame_list_push(&b, &f);
    }
    int64_t shift = tui_replay_append(&a, &b, 1000);
    CHECK(shift == 7010 && a.count == 110 && a.frames[100].timestamp_ms == 2990 &&
          a.frames[109].timestamp_ms == 3080, "gap > 1 s shortened (shift %lld)", (long long)shift);
    frame_list_t c = {0};
    can_frame_t f = frame(0x300, 3500, 1, 0);
    frame_list_push(&c, &f);
    CHECK(tui_replay_append(&a, &c, 1000) == 0 && a.frames[110].timestamp_ms == 3500, "gap <= 1 s kept");
    frame_list_free(&a);
    frame_list_free(&b);
    frame_list_free(&c);

    mkdir("sess", 0777);
    write_tsv("sess/live.frames.tsv", 5000, 50, 0x0A5);          /* 5000..5490 */
    write_tsv("sess/live_02.frames.tsv", 20000, 50, 0x0F3);      /* gap 14.5 s -> 1 s */
    write_tsv("sess/00_baseline.frames.tsv", 90000, 5, 0x316);   /* ignored: live files exist */
    tui_replay_data_t d;
    char err[256];
    CHECK(tui_replay_load("sess", &d, err, sizeof(err)) == 0, "load session: %s", err);
    CHECK(d.files == 2 && d.frames.count == 100 && d.frames.frames[50].timestamp_ms == 6490 &&
          d.frames.frames[50].can_id == 0x0F3 && strcmp(d.name, "sess") == 0, "live segments concatenated");
    tui_replay_free(&d);

    mkdir("phases", 0777);
    write_tsv("phases/01_steering.frames.tsv", 50000, 3, 0x0A5);
    write_tsv("phases/00_baseline.frames.tsv", 10000, 4, 0x0F3);
    CHECK(tui_replay_load("phases", &d, err, sizeof(err)) == 0 && d.files == 2 && d.frames.count == 7 &&
          d.frames.frames[0].can_id == 0x0F3 && d.frames.frames[4].timestamp_ms == 11030,
          "phase dumps in name order");
    tui_replay_free(&d);
    CHECK(tui_replay_load("phases/01_steering.frames.tsv", &d, err, sizeof(err)) == 0 && d.files == 1 &&
          d.frames.count == 3 && strcmp(d.name, "01_steering.frames.tsv") == 0, "single file");
    tui_replay_free(&d);
    mkdir("empty", 0777);
    CHECK(tui_replay_load("empty", &d, err, sizeof(err)) != 0, "empty folder rejected");
    CHECK(tui_replay_load("nope", &d, err, sizeof(err)) != 0, "missing path rejected");
}

static void test_replay_list(void) {
    mkdir("caps", 0777);
    mkdir("caps/sess_a", 0777);
    write_tsv("caps/sess_a/live.frames.tsv", 0, 5, 0x100);
    mkdir("caps/sess_b", 0777);
    write_tsv("caps/sess_b/00_baseline.frames.tsv", 0, 3, 0x200);
    write_tsv("caps/loose.frames.tsv", 0, 2, 0x300);
    mkdir("caps/empty", 0777);

    struct utimbuf times;
    times.actime = times.modtime = 1000000000;
    utime("caps/sess_a", &times);
    times.actime = times.modtime = 1000000100;
    utime("caps/sess_b", &times);
    times.actime = times.modtime = 1000000200;
    utime("caps/loose.frames.tsv", &times);

    tui_replay_list_t list = {0};
    char err[256];
    CHECK(tui_replay_list_scan(&list, "caps", err, sizeof(err)) == 0, "scan: %s", err);
    CHECK(list.count == 3, "3 replayable entries: %d", list.count);
    CHECK(list.count == 3 && strcmp(list.entries[0].name, "loose.frames.tsv") == 0, "newest first: %s",
          list.count > 0 ? list.entries[0].name : "");
    CHECK(list.count == 3 && strcmp(list.entries[1].name, "sess_b") == 0, "second newest: %s",
          list.count > 1 ? list.entries[1].name : "");
    CHECK(list.count == 3 && strcmp(list.entries[2].name, "sess_a") == 0, "oldest last: %s",
          list.count > 2 ? list.entries[2].name : "");
    CHECK(list.count == 3 && list.entries[1].files == 1 && list.entries[2].files == 1, "frame file counts");
    CHECK(list.count == 3 && strcmp(list.entries[2].path, "caps/sess_a") == 0, "path: %s",
          list.count > 2 ? list.entries[2].path : "");
    tui_replay_list_free(&list);

    CHECK(tui_replay_list_scan(&list, "nope", err, sizeof(err)) != 0, "missing root rejected");
    tui_replay_list_free(&list);
}

static long long file_size(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 ? (long long)st.st_size : -1;
}

static const tui_replay_entry_t *find_entry(const tui_replay_list_t *list, const char *name) {
    for (int i = 0; i < list->count; i++) {
        if (strcmp(list->entries[i].name, name) == 0) return &list->entries[i];
    }
    return NULL;
}

/* Only captures with at least one frame are listed; the list and the loader
 * pick the same files. */
static void test_replay_list_frames(void) {
    mkdir("fcaps", 0777);
    mkdir("fcaps/hdr_live", 0777);                               /* link dropped before any frame */
    write_tsv("fcaps/hdr_live/live.frames.tsv", 0, 0, 0x100);
    write_tsv("fcaps/hdr_only.frames.tsv", 0, 0, 0x100);         /* loose, header only */
    mkdir("fcaps/hdr_phases", 0777);                             /* header-only live, phase dumps with frames */
    write_tsv("fcaps/hdr_phases/live.frames.tsv", 0, 0, 0x100);
    write_tsv("fcaps/hdr_phases/00_baseline.frames.tsv", 1000, 4, 0x200);
    write_tsv("fcaps/hdr_phases/01_steering.frames.tsv", 2000, 0, 0x201);   /* header only: skipped */
    write_tsv("fcaps/hdr_phases/02_throttle.frames.tsv", 3000, 3, 0x202);
    mkdir("fcaps/hdr_seg", 0777);                                /* empty first segment, frames in live_02 */
    write_tsv("fcaps/hdr_seg/live.frames.tsv", 0, 0, 0x100);
    write_tsv("fcaps/hdr_seg/live_02.frames.tsv", 5000, 6, 0x300);
    write_tsv("fcaps/hdr_seg/00_baseline.frames.tsv", 9000, 2, 0x301);      /* ignored: live has frames */
    mkdir("fcaps/hdr_all", 0777);                                /* only header-only dumps */
    write_tsv("fcaps/hdr_all/live.frames.tsv", 0, 0, 0x100);
    write_tsv("fcaps/hdr_all/00_baseline.frames.tsv", 0, 0, 0x100);
    write_tsv("fcaps/loose.frames.tsv", 0, 2, 0x400);

    tui_replay_list_t list = {0};
    char err[1200] = "";
    CHECK(tui_replay_list_scan(&list, "fcaps", err, sizeof(err)) == 0, "scan: %s", err);
    CHECK(list.count == 3, "3 captures with frames: %d", list.count);
    CHECK(!find_entry(&list, "hdr_live") && !find_entry(&list, "hdr_only.frames.tsv") && !find_entry(&list, "hdr_all"),
          "header-only captures not listed");

    const tui_replay_entry_t *ph = find_entry(&list, "hdr_phases");
    char want[32];
    snprintf(want, sizeof(want), "%lld B",
             file_size("fcaps/hdr_phases/00_baseline.frames.tsv") + file_size("fcaps/hdr_phases/02_throttle.frames.tsv"));
    CHECK(ph && ph->files == 2 && strcmp(ph->size, want) == 0, "phase dumps with frames: files %d size %s (want 2, %s)",
          ph ? ph->files : -1, ph ? ph->size : "", want);
    const tui_replay_entry_t *sg = find_entry(&list, "hdr_seg");
    snprintf(want, sizeof(want), "%lld B", file_size("fcaps/hdr_seg/live_02.frames.tsv"));
    CHECK(sg && sg->files == 1 && strcmp(sg->size, want) == 0, "live segment with frames: files %d size %s (want 1, %s)",
          sg ? sg->files : -1, sg ? sg->size : "", want);

    tui_replay_data_t d;
    CHECK(tui_replay_load("fcaps/hdr_phases", &d, err, sizeof(err)) == 0 && d.files == 2 && d.frames.count == 7 &&
          d.frames.frames[0].can_id == 0x200 && d.frames.frames[4].can_id == 0x202,
          "header-only live: the phase dumps replay (%s)", err);
    tui_replay_free(&d);
    CHECK(tui_replay_load("fcaps/hdr_seg", &d, err, sizeof(err)) == 0 && d.files == 1 && d.frames.count == 6 &&
          d.frames.frames[0].can_id == 0x300, "empty first segment skipped (%s)", err);
    tui_replay_free(&d);
    CHECK(tui_replay_load("fcaps/hdr_all", &d, err, sizeof(err)) != 0, "folder without frames rejected");

    /* list and loader agree: every listed capture loads its files, the others do not replay */
    for (int i = 0; i < list.count; i++) {
        const tui_replay_entry_t *e = &list.entries[i];
        int rc = tui_replay_load(e->path, &d, err, sizeof(err));
        CHECK(rc == 0 && d.files == e->files && d.frames.count > 0, "%s: loads %d of %d files, %d frames", e->name,
              rc == 0 ? d.files : -1, e->files, rc == 0 ? d.frames.count : 0);
        CHECK(tui_replay_has_frames(e->path, err, sizeof(err)), "%s: check agrees with the list", e->name);
        if (rc == 0) tui_replay_free(&d);
    }
    const char *empty[3] = { "fcaps/hdr_live", "fcaps/hdr_only.frames.tsv", "fcaps/hdr_all" };
    for (int i = 0; i < 3; i++) {
        bool frames = tui_replay_load(empty[i], &d, err, sizeof(err)) == 0 && d.frames.count > 0;
        if (frames) tui_replay_free(&d);
        CHECK(!frames && !tui_replay_has_frames(empty[i], err, sizeof(err)), "%s: nothing to replay", empty[i]);
    }
    tui_replay_list_free(&list);

    /* emptied or deleted after the scan */
    write_tsv("fcaps/loose.frames.tsv", 0, 0, 0x400);
    CHECK(!tui_replay_has_frames("fcaps/loose.frames.tsv", err, sizeof(err)) && strstr(err, "no frames"),
          "emptied capture refused: %s", err);
    CHECK(!tui_replay_has_frames("fcaps/gone", err, sizeof(err)) && strstr(err, "No such file"),
          "deleted capture refused: %s", err);
}

/* More than TUI_REPLAY_LIST_MAX captures: the newest are kept. The newest
 * mtimes go to the entries readdir returns last. */
static void test_replay_list_cap(void) {
    enum { N = TUI_REPLAY_LIST_MAX + 8 };
    mkdir("many", 0777);
    char path[64];
    for (int i = 0; i < N; i++) {
        snprintf(path, sizeof(path), "many/c%04d.frames.tsv", i);
        write_tsv(path, 0, 1, 0x100);
    }
    static char order[N][32];
    int n = 0;
    DIR *dir = opendir("many");
    struct dirent *de;
    while (dir && (de = readdir(dir)) != NULL && n < N) {
        if (de->d_name[0] != '.') snprintf(order[n++], sizeof(order[0]), "%s", de->d_name);
    }
    if (dir) closedir(dir);
    CHECK(n == N, "%d files created", n);
    struct utimbuf times;
    for (int i = 0; i < n; i++) {
        snprintf(path, sizeof(path), "many/%s", order[i]);
        times.actime = times.modtime = 1000000000 + i;
        utime(path, &times);
    }

    tui_replay_list_t list = {0};
    char err[256] = "";
    CHECK(tui_replay_list_scan(&list, "many", err, sizeof(err)) == 0, "scan: %s", err);
    CHECK(list.count == TUI_REPLAY_LIST_MAX, "capped at %d: %d", TUI_REPLAY_LIST_MAX, list.count);
    CHECK(list.count == TUI_REPLAY_LIST_MAX && strcmp(list.entries[0].name, order[n - 1]) == 0,
          "newest kept first: %s (want %s)", list.count > 0 ? list.entries[0].name : "", order[n - 1]);
    CHECK(list.count == TUI_REPLAY_LIST_MAX && list.entries[list.count - 1].mtime == 1000000000 + (n - TUI_REPLAY_LIST_MAX),
          "oldest kept is the %d-th newest", TUI_REPLAY_LIST_MAX);
    tui_replay_list_free(&list);
}

static void test_recorder(void) {
    mkdir("rec", 0777);
    tui_recorder_t r;
    tui_recorder_init(&r, "rec");
    char err[256] = "";
    CHECK(tui_recorder_open(&r, 1000, err, sizeof(err)), "open: %s", err);
    CHECK(strcmp(r.path, "rec/live.frames.tsv") == 0, "first segment %s", r.path);
    can_frame_t frames[4] = {
        frame(0x0A5, 1000, 8, 0x01), frame(0x18DAF110, 1005, 3, 0x22), frame(0x7FF, 1010, 0, 0),
        frame(0x123, 2100, 2, 0xFF),
    };
    for (int i = 0; i < 3; i++) tui_recorder_write(&r, &frames[i], frames[i].timestamp_ms);
    struct stat st;
    stat(r.path, &st);
    long before_flush = (long)st.st_size;
    tui_recorder_write(&r, &frames[3], 2100);              /* >= 1 s since open: flush */
    stat(r.path, &st);
    CHECK(before_flush == 0 && st.st_size > 0, "flushed about every second (%ld -> %lld)", before_flush,
          (long long)st.st_size);
    tui_recorder_close(&r);
    CHECK(!tui_recorder_is_open(&r), "closed");

    frame_list_t back;
    CHECK(frame_tsv_load("rec/live.frames.tsv", "rec", &back) == 1 && back.count == 4 && back.malformed == 0,
          "round trip load");
    for (int i = 0; i < back.count && i < 4; i++) {
        CHECK(back.frames[i].can_id == frames[i].can_id && back.frames[i].dlc == frames[i].dlc &&
              back.frames[i].timestamp_ms == frames[i].timestamp_ms &&
              back.frames[i].is_extended == frames[i].is_extended &&
              memcmp(back.frames[i].data, frames[i].data, frames[i].dlc) == 0, "frame %d identical", i);
    }
    frame_list_free(&back);

    CHECK(tui_recorder_open(&r, 3000, err, sizeof(err)) && strcmp(r.path, "rec/live_02.frames.tsv") == 0,
          "second segment %s", r.path);
    tui_recorder_close(&r);
    tui_recorder_t bad;
    tui_recorder_init(&bad, "no/such/dir");
    CHECK(!tui_recorder_open(&bad, 0, err, sizeof(err)) && strstr(err, "cannot write") != NULL, "open error");
}

static void test_screens(void) {
    char lines[4][256];
    int n = tui_wrap_text("Turn steering wheel to FULL LEFT, then FULL RIGHT (lock to lock).", 30, lines, 4);
    CHECK(n == 3, "wrap into 3 lines: %d", n);
    CHECK_STR(lines[0], "Turn steering wheel to FULL");
    CHECK_STR(lines[1], "LEFT, then FULL RIGHT (lock to");
    CHECK_STR(lines[2], "lock).");
    n = tui_wrap_text("one two three", 30, lines, 1);
    CHECK(n == 1 && strcmp(lines[0], "one two three") == 0, "single line");
    n = tui_wrap_text("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa b", 10, lines, 4);
    CHECK(n == 2 && tui_utf8_width(lines[0]) == 10, "long word cut to the width");

    tui_rect_t r = tui_modal_rect(120, 40, 82, 16);
    CHECK(r.w == 82 && r.h == 16 && r.x == 19 && r.y == 12, "centered modal %d,%d %dx%d", r.x, r.y, r.w, r.h);
    r = tui_modal_rect(60, 20, 82, 16);
    CHECK(r.w == 56 && r.x == 2, "modal narrower than the screen: %d at %d", r.w, r.x);

    /* picker paging matches what the picker draws */
    int rows = tui_picker_list_rows(120, 40, 500, 2);
    CHECK(rows > 10 && rows < 40, "picker list rows %d", rows);
    CHECK(tui_picker_list_rows(120, 40, 3, 0) == 3, "short list: one row per profile");

    /* the modal clears its rect and draws hints on its last inner row */
    tui_grid_t *g = tui_grid_new(60, 20);
    tui_grid_fill(g, 0, 5, 60, "x", TUI_ROLE_DEFAULT);
    tui_hint_t hints[1] = { { "esc", "close", 0 } };
    tui_rect_t c = tui_draw_modal(g, (tui_rect_t){ 10, 4, 30, 8 }, "T", NULL, hints, 1, false);
    CHECK(c.x == 11 && c.y == 5 && c.w == 28 && c.h == 4, "content rect");
    CHECK(strcmp(tui_grid_cell(g, 11, 5)->ch, " ") == 0, "interior cleared");
    CHECK(strcmp(tui_grid_cell(g, 10, 9)->ch, "├") == 0, "separator");
    CHECK(strcmp(tui_grid_cell(g, 12, 10)->ch, "e") == 0, "hint text");
    tui_dim_grid(g);
    CHECK(tui_grid_cell(g, 12, 10)->role == TUI_ROLE_DIM, "dimmed");
    tui_grid_free(g);
}

static void count_line(const char *line, void *ctx) {
    char *buf = (char *)ctx;
    strcat(buf, line);
    strcat(buf, "|");
}

static void test_capture(void) {
    char l1[] = "2026-10-28 03:01:02.123 can_tools[123:4567] [ERROR] Command timeout";
    tui_capture_strip_nslog(l1);
    CHECK_STR(l1, "[ERROR] Command timeout");
    char l2[] = "[INFO] plain line";
    tui_capture_strip_nslog(l2);
    CHECK_STR(l2, "[INFO] plain line");
    char l3[] = "2026 is a year [not nslog]";
    tui_capture_strip_nslog(l3);
    CHECK_STR(l3, "2026 is a year [not nslog]");

    tui_capture_t c;
    CHECK(tui_capture_begin(&c), "capture begins");
    char got[512] = "";
    fprintf(stderr, "[INFO] first\n[WARN] sec");
    tui_capture_poll(&c, count_line, got);
    CHECK_STR(got, "[INFO] first|");
    fprintf(stderr, "ond\n");
    tui_capture_poll(&c, count_line, got);
    CHECK_STR(got, "[INFO] first|[WARN] second|");
    fprintf(stderr, "[INFO] before switch\n");
    CHECK(tui_capture_switch(&c, "tui.log"), "switch to the session log");
    fprintf(stderr, "[INFO] after switch\n");
    tui_capture_poll(&c, count_line, got);
    CHECK_STR(got, "[INFO] first|[WARN] second|[INFO] before switch|[INFO] after switch|");
    tui_capture_end(&c);
    FILE *f = fopen("tui.log", "r");
    char all[512] = "";
    size_t n = f ? fread(all, 1, sizeof(all) - 1, f) : 0;
    all[n] = '\0';
    if (f) fclose(f);
    CHECK_STR(all, "[INFO] first\n[WARN] second\n[INFO] before switch\n[INFO] after switch\n");
}

int main(void) {
    enter_tmpdir();
    test_screens();
    test_capture();
    test_raw();
    test_signal_state();
    test_ordering();
    test_sparkline();
    test_timeline_hold();
    test_format();
    test_classifier();
    test_log();
    test_grid();
    test_replay();
    test_replay_list();
    test_replay_list_frames();
    test_replay_list_cap();
    test_recorder();
    leave_tmpdir();
    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
