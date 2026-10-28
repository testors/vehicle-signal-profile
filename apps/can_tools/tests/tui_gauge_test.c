/*
 * Gauges panel and braille timeline (step 4): encoding, sampling, gauge
 * selection, units, scales, steering, temperature zones, gear strip and the
 * 8-color mapping. Plain C against tui_core; no terminal, no adapter.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tui_gauges.h"
#include "tui_grid.h"
#include "tui_model.h"
#include "tui_render.h"

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

static unsigned int first_cp(const char *s) {
    unsigned int cp = 0;
    tui_utf8_decode(s, &cp);
    return cp;
}

/* code points of a UTF-8 string */
static int cps(const char *s, unsigned int *out, int max) {
    int n = 0;
    while (*s && n < max) {
        unsigned int cp;
        s += tui_utf8_decode(s, &cp);
        out[n++] = cp;
    }
    return n;
}

static tui_signal_t make_sig(const char *name, const char *canonical, const char *unit, double cycle_ms) {
    tui_signal_desc_t d = { .name = name, .unit = unit, .group = "custom", .factor = 1.0, .cycle_ms = cycle_ms,
                            .canonical = canonical };
    tui_signal_t s;
    tui_signal_init(&s, &d);
    return s;
}

/*******************************************************************************
 * Braille
 ******************************************************************************/

static void test_braille_encoding(void) {
    CHECK(tui_braille_char(1, 1) == 0x28C0, "⣀ = (1,1): %X", tui_braille_char(1, 1));
    CHECK(tui_braille_char(2, 2) == 0x28E4, "⣤ = (2,2)");
    CHECK(tui_braille_char(1, 3) == 0x28F0, "⣰ = (1,3)");
    CHECK(tui_braille_char(4, 1) == 0x28C7, "⣇ = (4,1)");
    CHECK(tui_braille_char(1, 0) == 0x2840, "⡀ = (1,0)");
    CHECK(tui_braille_char(4, 4) == 0x28FF, "⣿ = (4,4)");
    CHECK(tui_braille_char(0, 0) == 0, "blank");
    CHECK(tui_braille_level(0, 0, 30) == 1 && tui_braille_level(30, 0, 30) == 4, "levels 1..4");
    CHECK(tui_braille_level(15, 0, 30) == 3, "1 + round(1.5) = 3 (half to even: round(1.5) = 2)");
    CHECK(tui_braille_level(25, 0, 40) == 3, "1 + round(1.875) = 3");
    CHECK(tui_braille_level(5, 5, 5) == 0, "flat: no level");
}

static void test_braille_sampling(void) {
    /* 10 cells over 10 s: each cell 1 s, two halves of 500 ms */
    tui_signal_t s = make_sig("V", "", "", 100);
    int64_t now = 10000;
    for (int64_t ms = 0; ms <= now; ms += 100) {
        double v = (ms % 1000) < 500 ? 0.0 : 30.0;          /* low first half, high second half */
        tui_signal_sample(&s, ms, true, false, v, NULL);
    }
    char out[256];
    unsigned char fg[64];
    int n = tui_signal_braille(&s, now, 10, out, sizeof(out), fg);
    unsigned int c[64];
    int nc = cps(out, c, 64);
    CHECK(n == 10 && nc == 10, "10 cells");
    CHECK(c[3] == tui_braille_char(1, 4), "two samples per cell: low left, high right (%X)", c[3]);
    CHECK(fg[3] == 87, "color by the higher sample (generic palette level 4): %d", fg[3]);

    /* flat window -> ⠤ */
    tui_signal_t f = make_sig("F", "", "", 100);
    for (int64_t ms = 0; ms <= now; ms += 100) tui_signal_sample(&f, ms, true, false, 7.0, NULL);
    tui_signal_braille(&f, now, 10, out, sizeof(out), fg);
    cps(out, c, 64);
    CHECK(c[5] == 0x2824, "flat: dot 3 left + dot 6 right = ⠤ (%X)", c[5]);

    /* hold: a 1 Hz signal draws a continuous line; a stopped one goes blank */
    tui_signal_t h = make_sig("H", "", "", 1000);
    for (int64_t ms = 0; ms <= 6000; ms += 1000) tui_signal_sample(&h, ms, true, false, (double)(ms / 1000), NULL);
    tui_signal_braille(&h, now, 20, out, sizeof(out), fg);
    int nh = cps(out, c, 64);
    bool gap = false;
    for (int i = 0; i < 12 && i < nh; i++) gap = gap || c[i] == ' ';
    CHECK(!gap, "1 Hz signal held over its half-second slices");
    CHECK(nh == 20 && c[19] == ' ', "stopped 4 s ago: blank at the end");

    /* temperature: 1 °C noise stays low, not full height */
    tui_signal_t t = make_sig("Oil", "engine.oil.temperature", "°C", 1000);
    for (int64_t ms = 0; ms <= now; ms += 500) tui_signal_sample(&t, ms, true, false, (ms / 500) % 2 ? 91.0 : 90.0, NULL);
    tui_signal_braille(&t, now, 10, out, sizeof(out), fg);
    cps(out, c, 64);
    CHECK(c[4] == tui_braille_char(2, 3), "10 °C span: 90/91 -> heights 2/3 (%X)", c[4]);
    CHECK(fg[4] == 40, "oil 91 °C: ok zone cell color 40 (%d)", fg[4]);
    tui_signal_t tf = make_sig("Oil", "engine.oil.temperature", "F", 1000);
    for (int64_t ms = 0; ms <= now; ms += 500) tui_signal_sample(&tf, ms, true, false, (ms / 500) % 2 ? 200.0 : 190.0, NULL);
    tui_signal_braille(&tf, now, 10, out, sizeof(out), fg);
    cps(out, c, 64);
    CHECK(c[4] == tui_braille_char(2, 3), "°F: 18 °F span, 190/200 -> heights 2/3 (%X)", c[4]);
    CHECK(fg[4] == 40 && tui_temp_zone(TUI_TEMP_OIL, tui_to_celsius(200, "F")) == TUI_ZONE_OK, "F converted for the zone");

    /* palettes */
    tui_signal_t r = make_sig("RPM", "engine.speed", "rpm", 10);
    CHECK(r.palette == TUI_PAL_RPM && tui_palette_fg(TUI_PAL_RPM, 4) == 196, "rpm palette");
    tui_signal_t b = make_sig("B", "brake.pressure", "bar", 10);
    CHECK(b.palette == TUI_PAL_BRAKE && tui_palette_fg(TUI_PAL_BRAKE, 1) == 124, "brake palette");
    tui_signal_t th = make_sig("T", "accelerator.pedal.position", "%", 10);
    CHECK(th.palette == TUI_PAL_THROTTLE && tui_palette_fg(TUI_PAL_THROTTLE, 2) == 40, "throttle palette");
    tui_signal_t o = make_sig("Amb", "ambient.air.temperature", "°C", 1000);
    CHECK(o.temp == TUI_TEMP_OTHER && o.palette == TUI_PAL_GENERIC, "other temperatures: cyan, still widened");
}

/*******************************************************************************
 * Gauge selection
 ******************************************************************************/

static int sig_index(const tui_model_t *m, const char *name) {
    for (int i = 0; i < m->signal_count; i++) {
        if (strcmp(m->signals[i].name, name) == 0) return i;
    }
    return -1;
}

static void test_selection(void) {
    tui_signal_desc_t d[6] = {
        { .name = "Thr_Dup", .group = "throttle", .factor = 1, .cycle_ms = 20, .canonical = "throttle.position",
          .decoder_index = 0 },
        { .name = "Thr_Main", .group = "throttle", .factor = 1, .cycle_ms = 20, .canonical = "throttle.position",
          .decoder_index = 1 },
        { .name = "Thr_OBD", .group = "throttle", .factor = 1, .diagnostic = true, .canonical = "throttle.position",
          .request_hex = "01 11", .decoder_index = 2 },
        { .name = "Pedal", .group = "throttle", .factor = 1, .cycle_ms = 20,
          .canonical = "accelerator.pedal.position", .decoder_index = 3 },
        { .name = "Speed_FL", .group = "custom", .factor = 1, .cycle_ms = 20, .canonical = "wheel.speed.front_left",
          .decoder_index = 4 },
        { .name = "Speed_Diag", .group = "custom", .factor = 1, .diagnostic = true, .canonical = "vehicle.speed",
          .request_hex = "01 0D", .decoder_index = 5 },
    };
    tui_model_t m;
    tui_model_init(&m, 0);
    tui_model_set_signals(&m, d, 6);
    tui_gauges_t gs;
    tui_gauges_init(&gs);
    tui_gauges_bind(&gs, &m);
    CHECK(gs.bound == 2, "throttle and speed bound: %d", gs.bound);
    CHECK(gs.g[TUI_GAUGE_THROTTLE].ncand == 4, "4 throttle candidates");

    /* before samples: highest priority, CAN before diagnostic, profile order */
    tui_gauges_update(&gs, &m, 0);
    const tui_signal_t *s = tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE);
    CHECK(s && strcmp(s->name, "Pedal") == 0, "accelerator.pedal.position first: %s", s ? s->name : "-");
    s = tui_gauge_signal(&gs, &m, TUI_GAUGE_SPEED);
    CHECK(s && strcmp(s->name, "Speed_Diag") == 0, "vehicle.speed beats wheel.speed.front_left even as diagnostic");

    /* only the duplicates transmit: the freshest LIVE one is taken */
    tui_signal_t *main_sig = &m.signals[sig_index(&m, "Thr_Main")];
    tui_signal_t *obd_sig = &m.signals[sig_index(&m, "Thr_OBD")];
    tui_signal_t *pedal = &m.signals[sig_index(&m, "Pedal")];
    for (int64_t ms = 0; ms <= 1000; ms += 20) tui_signal_sample(main_sig, ms, true, false, 10, NULL);
    tui_gauges_update(&gs, &m, 1000);
    s = tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE);
    CHECK(s == main_sig, "never-sampled default replaced by the live duplicate");

    /* another LIVE candidate of the same priority: no flicker */
    for (int64_t ms = 1000; ms <= 2000; ms += 20) {
        tui_signal_sample(obd_sig, ms + 5, true, false, 11, NULL);
        tui_signal_sample(main_sig, ms, true, false, 10, NULL);
        tui_gauges_update(&gs, &m, ms + 5);
        CHECK(tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE) == main_sig, "keeps the current LIVE one at %lld",
              (long long)ms);
    }
    /* current goes stale while another is LIVE: switch */
    for (int64_t ms = 2000; ms <= 3000; ms += 100) tui_signal_sample(obd_sig, ms, true, false, 12, NULL);
    tui_gauges_update(&gs, &m, 3000);
    CHECK(tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE) == obd_sig, "stale current -> the LIVE one");
    /* a higher-priority id that becomes LIVE wins */
    for (int64_t ms = 3000; ms <= 3200; ms += 20) tui_signal_sample(pedal, ms, true, false, 50, NULL);
    tui_signal_sample(obd_sig, 3200, true, false, 12, NULL);
    tui_gauges_update(&gs, &m, 3200);
    CHECK(tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE) == pedal, "accelerator pedal wins once LIVE");
    /* nothing live: stays on the last one (stale) */
    tui_gauges_update(&gs, &m, 20000);
    CHECK(tui_gauge_signal(&gs, &m, TUI_GAUGE_THROTTLE) == pedal, "no live candidate: keep");

    tui_gauges_free(&gs);
    tui_model_free(&m);
}

/*******************************************************************************
 * Units, scales, zones
 ******************************************************************************/

static void test_units_scales(void) {
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_COOLANT, "F", 212) - 100) < 1e-9, "F -> °C");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_OIL, "°F", 32) - 0) < 1e-9, "°F -> °C");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_COOLANT, "°C", 90) - 90) < 1e-9, "°C stays");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_BRAKE, "kPa", 1850) - 18.5) < 1e-9, "kPa / 100");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_BRAKE, "psi", 100) - 6.89476) < 1e-9, "psi x 0.0689476");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_BRAKE, "", 18.5) - 18.5) < 1e-9, "bar / none");
    CHECK(fabs(tui_gauge_convert(TUI_GAUGE_SPEED, "mph", 100) - 160.9344) < 1e-9, "mph x 1.609344");
    CHECK(tui_unit_is_degrees("deg") && !tui_unit_is_degrees("%"), "throttle in degrees");
    CHECK(tui_brake_mode("brake.pedal.position") == TUI_BRAKE_PEDAL && tui_brake_mode("brake.switch") == TUI_BRAKE_SWITCH &&
          tui_brake_mode("brake.pressure.front") == TUI_BRAKE_PRESSURE, "brake modes");

    CHECK(tui_scale_grow(7000, 6999, 1000, 20000) == 7000, "inside the scale");
    CHECK(tui_scale_grow(7000, 8100, 1000, 20000) == 9000, "rpm grows to ceil(v / 1000) * 1000");
    CHECK(tui_scale_grow(9000, 8000, 1000, 20000) == 9000, "sticky: never shrinks");
    CHECK(tui_scale_grow(9000, 65535, 1000, 20000) == 9000, "ignored above 20,000");
    CHECK(tui_scale_grow(200, 251, 50, 500) == 300 && tui_scale_grow(200, 3276.7, 50, 500) == 200, "speed");
    CHECK(tui_scale_grow(100, 101, 20, 300) == 120 && tui_scale_grow(100, 301, 20, 300) == 100, "brake");

    /* growth through update, reset per data session */
    tui_signal_desc_t d = { .name = "RPM", .unit = "rpm", .group = "rpm", .factor = 1, .cycle_ms = 10,
                            .canonical = "engine.speed" };
    tui_model_t m;
    tui_model_init(&m, 0);
    tui_model_set_signals(&m, &d, 1);
    tui_gauges_t gs;
    tui_gauges_init(&gs);
    tui_gauges_bind(&gs, &m);
    tui_signal_sample(&m.signals[0], 100, true, false, 8400, NULL);
    tui_gauges_update(&gs, &m, 100);
    CHECK(gs.g[TUI_GAUGE_RPM].scale == 9000, "update grows the rpm scale");
    tui_signal_sample(&m.signals[0], 110, true, false, 65535, NULL);
    tui_gauges_update(&gs, &m, 110);
    CHECK(gs.g[TUI_GAUGE_RPM].scale == 9000, "invalid 0xFFFF sample ignored");
    tui_gauges_bind(&gs, &m);
    CHECK(gs.g[TUI_GAUGE_RPM].scale == TUI_SCALE_RPM_START && gs.g[TUI_GAUGE_SPEED].scale == TUI_SCALE_SPEED_START &&
          gs.g[TUI_GAUGE_BRAKE].scale == TUI_SCALE_BRAKE_START, "rebind (profile change) resets the scales");
    tui_gauges_free(&gs);
    tui_model_free(&m);

    /* zones at the boundaries */
    CHECK(tui_temp_zone(TUI_TEMP_COOLANT, 69.9) == TUI_ZONE_COLD && tui_temp_zone(TUI_TEMP_COOLANT, 70) == TUI_ZONE_OK,
          "coolant 70");
    CHECK(tui_temp_zone(TUI_TEMP_COOLANT, 109.9) == TUI_ZONE_OK && tui_temp_zone(TUI_TEMP_COOLANT, 110) == TUI_ZONE_HOT,
          "coolant 110");
    CHECK(tui_temp_zone(TUI_TEMP_OIL, 79) == TUI_ZONE_COLD && tui_temp_zone(TUI_TEMP_OIL, 80) == TUI_ZONE_OK &&
          tui_temp_zone(TUI_TEMP_OIL, 129) == TUI_ZONE_OK && tui_temp_zone(TUI_TEMP_OIL, 130) == TUI_ZONE_HOT, "oil");
    CHECK(tui_temp_zone(TUI_TEMP_GEARBOX, 49) == TUI_ZONE_COLD && tui_temp_zone(TUI_TEMP_GEARBOX, 50) == TUI_ZONE_OK &&
          tui_temp_zone(TUI_TEMP_GEARBOX, 119) == TUI_ZONE_OK && tui_temp_zone(TUI_TEMP_GEARBOX, 120) == TUI_ZONE_HOT,
          "gearbox");
    CHECK(tui_zone_text_fg(TUI_ZONE_COLD) == 39 && tui_zone_text_fg(TUI_ZONE_OK) == 46 && tui_zone_text_fg(TUI_ZONE_HOT) == 196,
          "zone text colors");
    const tui_temp_zones_t *z = tui_temp_zones(TUI_TEMP_GEARBOX);
    CHECK(z && z->lo == 20 && z->hi == 140, "gearbox bar 20..140");
}

/*******************************************************************************
 * Bars
 ******************************************************************************/

static int count_filled(const tui_bar_cell_t *c, int n) {
    int k = 0;
    for (int i = 0; i < n; i++) k += c[i].fg != TUI_FG_TRACK;
    return k;
}

static void test_steering(void) {
    tui_bar_style_t st = { 256, false };
    struct { double deg; int fill; } cases[] = { { 0, 0 }, { 4, 0 }, { 5, 1 }, { 92, 4 }, { 180, 7 }, { -180, 7 } };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        CHECK(tui_steer_fill(cases[i].deg, 7) == cases[i].fill, "%g° -> %d cells (got %d)", cases[i].deg,
              cases[i].fill, tui_steer_fill(cases[i].deg, 7));
    }
    tui_bar_cell_t c[15];
    tui_bar_steer(180, 7, false, &st, c);
    CHECK(strcmp(c[0].ch, "■") == 0 && c[0].fg == 123 && strcmp(c[7].ch, "┃") == 0 && c[7].fg == TUI_FG_CENTER,
          "180°: full left, no marker, outer cell 123");
    CHECK(count_filled(c, 7) == 7 && count_filled(c + 8, 7) == 0, "left side only");
    CHECK(c[6].fg == 24, "gradient from the center: next to center 24");
    tui_bar_steer(181, 7, false, &st, c);
    CHECK(strcmp(c[0].ch, "◀") == 0 && c[0].fg == TUI_FG_OVER, "181° left: ◀ in 51");
    tui_bar_steer(-181, 7, false, &st, c);
    CHECK(strcmp(c[14].ch, "▶") == 0 && c[14].fg == TUI_FG_OVER && count_filled(c, 7) == 0, "181° right: ▶");
    tui_bar_steer(449, 7, false, &st, c);
    CHECK(strcmp(c[0].ch, "◀") == 0 && c[0].fg == TUI_FG_OVER && c[1].fg == 87, "449°: still ◀, cyan bar");
    tui_bar_steer(450, 7, false, &st, c);
    {
        /* full lock: all 7 filled cells in the brake reds by distance from the center, no triangle */
        static const unsigned char want[7] = { 124, 160, 160, 196, 203, 203, 210 };   /* k = 0 .. 6 */
        bool ok = true;
        for (int k = 0; k < 7; k++) ok = ok && strcmp(c[6 - k].ch, "■") == 0 && c[6 - k].fg == want[k];
        CHECK(ok, "450°: whole left bar BRK gradient (%d %d %d %d %d %d %d)", c[6].fg, c[5].fg, c[4].fg, c[3].fg,
              c[2].fg, c[1].fg, c[0].fg);
        CHECK(count_filled(c + 8, 7) == 0, "450°: right side empty");
    }
    tui_bar_steer(-540, 7, false, &st, c);
    CHECK(strcmp(c[14].ch, "■") == 0 && c[14].fg == 210 && c[8].fg == 124, "540° right: BRK gradient to the right end");
    tui_bar_steer(92, 7, false, &st, c);
    CHECK(count_filled(c, 7) == 4 && c[6].fg != TUI_FG_TRACK && c[2].fg == TUI_FG_TRACK, "L 92°: 4 cells next to center");
    /* narrow: 4|4 */
    CHECK(tui_steer_fill(92, 4) == 2 && tui_steer_fill(180, 4) == 4, "half 4");
    tui_bar_cell_t n[9];
    tui_bar_steer(-300, 4, false, &st, n);
    CHECK(strcmp(n[8].ch, "▶") == 0 && strcmp(n[4].ch, "┃") == 0, "narrow over-range");
    /* ascii and stale */
    tui_bar_style_t as = { 256, true };
    tui_bar_steer(470, 7, false, &as, c);
    CHECK(strcmp(c[0].ch, "X") == 0 && strcmp(c[6].ch, "X") == 0 && strcmp(c[7].ch, "|") == 0 && strcmp(c[8].ch, ".") == 0,
          "ascii full lock: every filled cell X, center |, empty .");
    tui_bar_steer(90, 7, false, &as, c);
    CHECK(strcmp(c[6].ch, "#") == 0, "ascii filled #");
    tui_bar_steer(200, 7, false, &as, c);
    CHECK(strcmp(c[0].ch, "<") == 0, "ascii over-range <");
    tui_bar_steer(90, 7, true, &st, c);
    CHECK(c[6].fg == TUI_FG_STALE && c[7].fg == TUI_FG_CENTER, "stale: filled cells gray, center stays");
    /* no colors: empty cells "·" */
    tui_bar_style_t mono = { 0, false };
    tui_bar_steer(0, 7, false, &mono, c);
    CHECK(strcmp(c[0].ch, "·") == 0, "no colors: empty ·");
}

static void test_meter(void) {
    tui_bar_style_t st = { 256, false };
    tui_bar_cell_t c[15];
    CHECK(tui_bar_meter(0.62, 15, TUI_GRAD_THR, 5, false, &st, c) == 9, "62%% of 15 -> 9");
    CHECK(c[0].fg == 34 && c[8].fg == 46 && c[9].fg == TUI_FG_TRACK, "gradient by position, track 237");
    CHECK(tui_bar_meter(0.021, 15, TUI_GRAD_THR, 5, false, &st, c) == 1, "> 2%%: at least 1");
    CHECK(tui_bar_meter(0.02, 15, TUI_GRAD_THR, 5, false, &st, c) == 0, "2%%: empty");
    CHECK(tui_bar_meter(1.7, 15, TUI_GRAD_RPM, 11, false, &st, c) == 15 && c[14].fg == 196, "pegged");
    CHECK(tui_grad_pick(TUI_GRAD_HOT, 6, 7, 15) == 196, "round half to even: 2.5 -> 2 (stop 196)");
    tui_bar_meter(0.5, 10, TUI_GRAD_BRK, 5, true, &st, c);
    CHECK(c[0].fg == TUI_FG_STALE && c[0].role == 'g', "stale meter");
}

static void test_gear(void) {
    tui_bar_cell_t c[32];
    char text[40];
    int n = tui_gear_strip(7, "3", 20, c, 32);
    for (int i = 0; i < n; i++) text[i] = c[i].ch[0];
    text[n] = '\0';
    CHECK(strcmp(text, "R N 1 2 3 4 5 6 7") == 0, "7 gears spaced: '%s'", text);
    CHECK(c[8].fg == TUI_FG_GEAR_ON && c[8].role == 'b' && c[6].fg == TUI_FG_GEAR_OFF, "current gear 231 bold");
    n = tui_gear_strip(6, "N", 20, c, 32);
    CHECK(n == 15 && c[2].fg == TUI_FG_GEAR_ON, "6 gears, N");
    n = tui_gear_strip(7, "R", 14, c, 32);
    for (int i = 0; i < n; i++) text[i] = c[i].ch[0];
    text[n] = '\0';
    CHECK(strcmp(text, "RN1234567") == 0 && c[0].fg == TUI_FG_GEAR_ON, "compact when it does not fit: '%s'", text);
    n = tui_gear_strip(7, "P", 20, c, 32);
    bool any = false;
    for (int i = 0; i < n; i++) any = any || c[i].fg == TUI_FG_GEAR_ON;
    CHECK(!any, "P is not in the strip");
    CHECK(tui_gear_strip_max(7, 0) == 7 && tui_gear_strip_max(0, 4) == 6 && tui_gear_strip_max(0, 8) == 8 &&
          tui_gear_strip_max(12, 0) == 10, "strip length: table, observed, at least 6, at most 10");

    tui_signal_t s = make_sig("Gear", "transmission.gear.selected", "", 50);
    char lab[16];
    tui_signal_sample(&s, 0, true, false, -1, NULL);
    tui_gear_label(&s, lab, sizeof(lab));
    CHECK(strcmp(lab, "R") == 0, "-1 -> R");
    tui_signal_sample(&s, 10, true, false, 0, NULL);
    tui_gear_label(&s, lab, sizeof(lab));
    CHECK(strcmp(lab, "N") == 0, "0 -> N");
    tui_signal_sample(&s, 20, true, true, 0, "P");
    tui_gear_label(&s, lab, sizeof(lab));
    CHECK(strcmp(lab, "P") == 0, "null -> label");
}

/*******************************************************************************
 * Gauge cells: no negative pedal, pressure, rpm or speed
 ******************************************************************************/

/* Text of grid row y (cells concatenated). */
static void grid_row(const tui_grid_t *g, int y, char *out, size_t len) {
    size_t n = 0;
    out[0] = '\0';
    for (int x = 0; x < g->cols; x++) {
        const char *ch = tui_grid_cell(g, x, y)->ch;
        size_t l = strlen(ch);
        if (n + l + 1 > len) break;
        memcpy(out + n, ch, l);
        n += l;
    }
    out[n] = '\0';
}

/* The first row holding `label` (the Gauges panel is on top): the row from the
   label on, and the filled cells of the first ■ bar after it. */
static bool gauge_cell(const tui_grid_t *g, const char *label, char *text, size_t len, int *filled) {
    char row[1024];
    for (int y = 0; y < g->rows; y++) {
        grid_row(g, y, row, sizeof(row));
        const char *p = strstr(row, label);
        if (!p) continue;
        snprintf(text, len, "%s", p);
        int x = tui_utf8_width(row) - tui_utf8_width(p);   /* the label's column */
        while (x < g->cols && strcmp(tui_grid_cell(g, x, y)->ch, "■") != 0) x++;
        *filled = 0;
        for (; x < g->cols && strcmp(tui_grid_cell(g, x, y)->ch, "■") == 0; x++) {
            *filled += tui_grid_cell(g, x, y)->fg != TUI_FG_TRACK;
        }
        return true;
    }
    return false;
}

static void test_gauge_floor(void) {
    tui_signal_desc_t d[5] = {
        { .name = "Thr", .unit = "%", .group = "throttle", .factor = 0.4, .cycle_ms = 20,
          .canonical = "throttle.position", .decoder_index = 0 },
        { .name = "Brk", .unit = "bar", .group = "brake", .factor = 0.3, .cycle_ms = 20,
          .canonical = "brake.pressure", .decoder_index = 1 },
        { .name = "Str", .unit = "deg", .group = "steering", .factor = 0.1, .cycle_ms = 20,
          .canonical = "steering.wheel.angle", .decoder_index = 2 },
        { .name = "Rpm", .unit = "rpm", .group = "rpm", .factor = 1, .cycle_ms = 20,
          .canonical = "engine.speed", .decoder_index = 3 },
        { .name = "Spd", .unit = "km/h", .group = "custom", .factor = 0.1, .cycle_ms = 20,
          .canonical = "vehicle.speed", .decoder_index = 4 },
    };
    static const double val[5] = { -0.6, -0.3, -5.0, -3.0, -0.6 };
    tui_model_t m;
    tui_model_init(&m, 0);
    tui_model_set_signals(&m, d, 5);
    for (int i = 0; i < m.signal_count; i++) {
        tui_signal_t *s = &m.signals[i];
        for (int k = 0; k < 5; k++) {
            if (s->decoder_index != k) continue;
            for (int64_t ms = 0; ms <= 1000; ms += 20) tui_signal_sample(s, ms, true, false, val[k], NULL);
        }
    }
    tui_gauges_t gs;
    tui_gauges_init(&gs);
    tui_gauges_bind(&gs, &m);
    tui_gauges_update(&gs, &m, 1000);
    static const struct { int cols, rows; } sizes[2] = { { 120, 40 }, { 80, 24 } };   /* wide, narrow gauges */
    for (int k = 0; k < 2; k++) {
        tui_grid_t *g = tui_grid_new(sizes[k].cols, sizes[k].rows);
        tui_view_t v;
        memset(&v, 0, sizeof(v));
        v.cols = sizes[k].cols;
        v.rows = sizes[k].rows;
        v.now_ms = 1000;
        v.device = "gauge test";
        v.filter = "";
        v.tab = TUI_PANEL_GAUGES;
        v.focus = TUI_PANEL_SIGNALS;
        v.gauges = &gs;
        v.colors = 256;
        tui_render_dashboard(g, &m, &v);
        bool wide = k == 0;
        char t[1024];
        int filled = -1;
        /* -0.3 bar released brake, -0.6 % throttle, -0.6 km/h, -3 rpm: 0 with an empty bar */
        CHECK(gauge_cell(g, wide ? "Brake  " : "Brk ", t, sizeof(t), &filled) &&
              strstr(t, wide ? " 0.0 bar" : " 0.0bar") && !strstr(t, "-0.3") && filled == 0,
              "%s brake -0.3 bar shows 0.0 bar, empty bar (%d): [%.40s]", wide ? "wide" : "narrow", filled, t);
        CHECK(gauge_cell(g, wide ? "Throttle" : "Thr ", t, sizeof(t), &filled) && !strstr(t, "-1") &&
              filled == 0, "%s throttle -0.6 %% shows 0 (%d): [%.40s]", wide ? "wide" : "narrow", filled, t);
        CHECK(gauge_cell(g, wide ? "Speed" : "Spd ", t, sizeof(t), &filled) && !strstr(t, "-1") &&
              filled == 0, "%s speed -0.6 km/h shows 0 (%d): [%.40s]", wide ? "wide" : "narrow", filled, t);
        CHECK(gauge_cell(g, "RPM", t, sizeof(t), &filled) && !strstr(t, "-3") && filled == 0,
              "%s rpm -3 shows 0 (%d): [%.40s]", wide ? "wide" : "narrow", filled, t);
        /* steering stays signed: -5° is R 5 */
        CHECK(gauge_cell(g, wide ? "Steering" : "Str ", t, sizeof(t), &filled) && strstr(t, "R 5") != NULL,
              "%s steering -5° still R 5: [%.40s]", wide ? "wide" : "narrow", t);
        tui_grid_free(g);
    }
    /* a real pressure still fills the bar (the cell reader sees filled cells) */
    tui_signal_t *brk = &m.signals[sig_index(&m, "Brk")];
    for (int64_t ms = 1020; ms <= 1100; ms += 20) tui_signal_sample(brk, ms, true, false, 50.0, NULL);
    tui_gauges_update(&gs, &m, 1100);
    {
        tui_grid_t *g = tui_grid_new(120, 40);
        tui_view_t v;
        memset(&v, 0, sizeof(v));
        v.cols = 120;
        v.rows = 40;
        v.now_ms = 1100;
        v.device = "gauge test";
        v.filter = "";
        v.tab = TUI_PANEL_GAUGES;
        v.focus = TUI_PANEL_SIGNALS;
        v.gauges = &gs;
        v.colors = 256;
        tui_render_dashboard(g, &m, &v);
        char t[1024];
        int filled = 0;
        CHECK(gauge_cell(g, "Brake  ", t, sizeof(t), &filled) && strstr(t, " 50.0 bar") && filled > 0,
              "50 bar: value and a filled bar (%d)", filled);
        tui_grid_free(g);
    }
    tui_gauges_free(&gs);
    tui_model_free(&m);
}

static void test_colors(void) {
    CHECK(tui_color_basic(39) == 4 && tui_color_basic(33) == 4 && tui_color_basic(25) == 4, "cold -> blue");
    CHECK(tui_color_basic(46) == 2 && tui_color_basic(40) == 2 && tui_color_basic(34) == 2, "ok -> green");
    CHECK(tui_color_basic(196) == 1 && tui_color_basic(124) == 1, "hot -> red");
    CHECK(tui_color_basic(51) == 6 && tui_color_basic(87) == 6, "cyan");
    CHECK(tui_color_basic(221) == 3 && tui_color_basic(149) == 3, "rpm middle -> yellow");
    CHECK(tui_color_basic(237) == 8 && tui_color_basic(240) == 8, "track / stale -> gray");
    CHECK(tui_color_basic(231) == 7, "white");
    CHECK(tui_color_basic(93) != 0, "unknown index maps somewhere");
    tui_grid_t *g = tui_grid_new(4, 1);
    tui_grid_put_fg(g, 0, 0, "■■", 'b', 196, 2);
    tui_grid_map_colors(g, 8);
    CHECK(tui_grid_cell(g, 0, 0)->fg == 1, "grid mapped to 8 colors");
    tui_grid_map_colors(g, 0);
    CHECK(!tui_grid_has_fg(g), "no colors: cleared");
    tui_grid_put_fg(g, 0, 0, "x", '.', 45, 1);
    tui_grid_put(g, 0, 0, "y", '.', 1);
    CHECK(tui_grid_cell(g, 0, 0)->fg == 0, "a plain write resets fg");
    tui_grid_free(g);
    CHECK(first_cp("■") == 0x25A0, "■ is U+25A0");
}

int main(void) {
    test_braille_encoding();
    test_braille_sampling();
    test_selection();
    test_units_scales();
    test_steering();
    test_meter();
    test_gear();
    test_gauge_floor();
    test_colors();
    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
