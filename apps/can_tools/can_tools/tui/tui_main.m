/**
 * @file tui_main.m
 * @brief can_tools tui: entry, option parsing, replay loop, snapshot mode
 *
 * Single thread. Live mode (tui_live.m) shows the connect screen, or looks
 * for an explicit non-empty --device. Replay mode pumps a paced replay
 * source; snapshot mode drives a manual-clock replay source and never reads
 * the wall clock or touches the terminal.
 *
 * Test hook, compiled only into can_tools_fake (CAN_TOOLS_TEST_HOOKS):
 * CAN_TOOLS_TUI_HEADLESS=1 lets --snapshot-at / --keys run the live path
 * without a terminal on the wall clock. In the real can_tools the hook does
 * not exist: --snapshot-at without --replay always exits 2 during option
 * checking, before any adapter object is created (rules 0.4 / 0.5).
 */

#import <Foundation/Foundation.h>
#include <mach-o/dyld.h>

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "commands.h"
#include "frame_source.h"
#include "pcan.h"
#include "tui_app.h"
#include "tui_capture.h"
#include "tui_grid.h"
#include "tui_live.h"
#include "tui_replay.h"
#include "tui_term.h"
#include "vsp_engine.h"

typedef struct {
    const char *device;
    bool        device_given;
    const char *bitrate;     /* --bitrate (PCAN), NULL = 500k */
    const char *profile;
    const char *profiles_dir;
    const char *replay;
    double      speed;
    bool        no_tx, no_rec, mono, ascii, color_map;
    const char *snapshot_at;
    const char *size;
    const char *keys;
    const char *captures;
    int         colors;      /* --colors N, -1 = not given */
} tui_cli_t;

static void tui_usage(void) {
    fprintf(stderr,
            "Usage: can_tools tui [--device NAME] [--profile VSP|none] [--profiles DIR]\n"
            "                     [--no-tx] [--no-rec] [--captures DIR] [--mono] [--ascii]\n"
            "                     [--bitrate RATE|auto]\n"
            "       can_tools tui --replay PATH [--profile VSP|none] [--speed X]\n"
            "       can_tools tui --replay PATH [--profile VSP|none] --snapshot-at T[,T...]\n"
            "                     [--size COLSxROWS] [--keys \"T:key key;T:key\"] [--color-map]\n"
            "\n"
            "  Live dashboard. Without --replay: the connect screen (adapters found by a\n"
            "  scan; Enter connects). The adapter is set up receive-only (ATCSM1 silent\n"
            "  mode); nothing is sent on the bus unless you switch on TX mode (x), which\n"
            "  works only on a GATEWAY or SILENT bus and asks first.\n"
            "  --device NAME    connect to the first adapter whose name contains NAME;\n"
            "                   pcan / pcan1..pcan8: a PEAK PCAN-USB (PCBUSB library),\n"
            "                   opened listen-only (TX mode never available on it)\n"
            "  --bitrate RATE   PCAN bitrate: auto or 1M 800k 500k 250k 125k 100k 95k 83k\n"
            "                   50k 47k 33k 20k 10k 5k (default 500k)\n"
            "  --profile VSP    load this profile (none = raw CAN only); else the picker\n"
            "                   (replay: first the profile in the capture's session.json)\n"
            "  --profiles DIR   profile folder (default: VSP_PROFILES_DIR, ./profiles,\n"
            "                   the repository's profiles/)\n"
            "  --no-tx          TX mode stays off      --no-rec  start without recording\n"
            "  --captures DIR   session folders (default captures/)\n"
            "  --replay PATH    a *.frames.tsv file or a capture folder (live*.frames.tsv,\n"
            "                   else NN_*.frames.tsv); nothing is sent anywhere\n"
            "  --speed X        replay speed 0.25..16 (space pause, [ ] slower/faster)\n"
            "  --snapshot-at T  headless replay: print the screen at T seconds from the\n"
            "                   first frame (no terminal needed, simulated clock)\n"
            "  --keys SPEC      snapshot: keys at times, e.g. \"2:tab tab;4.5:s\"\n"
            "                   (enter esc tab space up down pgup pgdn backspace ctrl-n or one character)\n"
            "  --color-map      snapshot: also print the style role of every cell and, when\n"
            "                   cells carry 256-color indices, an fg-map (two hex digits per cell)\n"
            "  --colors N       use at most N colors: 256, 16, 8 or 0 (snapshot default 256)\n");
}

static bool parse_cli(int argc, const char *argv[], tui_cli_t *o) {
    memset(o, 0, sizeof(*o));
    o->speed = 1.0;
    o->colors = -1;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        bool has_next = i + 1 < argc;
        if (strcmp(a, "--device") == 0 && has_next) { o->device = argv[++i]; o->device_given = true; }
        else if (strcmp(a, "--bitrate") == 0 && has_next) o->bitrate = argv[++i];
        else if (strcmp(a, "--profile") == 0 && has_next) o->profile = argv[++i];
        else if (strcmp(a, "--profiles") == 0 && has_next) o->profiles_dir = argv[++i];
        else if (strcmp(a, "--replay") == 0 && has_next) o->replay = argv[++i];
        else if (strcmp(a, "--speed") == 0 && has_next) o->speed = atof(argv[++i]);
        else if (strcmp(a, "--snapshot-at") == 0 && has_next) o->snapshot_at = argv[++i];
        else if (strcmp(a, "--size") == 0 && has_next) o->size = argv[++i];
        else if (strcmp(a, "--keys") == 0 && has_next) o->keys = argv[++i];
        else if (strcmp(a, "--captures") == 0 && has_next) o->captures = argv[++i];
        else if (strcmp(a, "--colors") == 0 && has_next) {
            o->colors = atoi(argv[++i]);
            if (o->colors != 0 && o->colors != 8 && o->colors != 16 && o->colors != 256) {
                fprintf(stderr, "[ERROR] tui: --colors must be 256, 16, 8 or 0\n");
                return false;
            }
        }
        else if (strcmp(a, "--no-tx") == 0) o->no_tx = true;
        else if (strcmp(a, "--no-rec") == 0) o->no_rec = true;
        else if (strcmp(a, "--mono") == 0) o->mono = true;
        else if (strcmp(a, "--ascii") == 0) o->ascii = true;
        else if (strcmp(a, "--color-map") == 0) o->color_map = true;
        else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0 || strcmp(a, "help") == 0) return false;
        else {
            fprintf(stderr, "[ERROR] tui: unexpected argument: %s\n", a);
            return false;
        }
    }
    return true;
}

/*******************************************************************************
 * Snapshot mode
 ******************************************************************************/

static int parse_times(const char *spec, int64_t *out, int max) {
    int n = 0;
    char buf[1024];
    snprintf(buf, sizeof(buf), "%s", spec);
    for (char *t = strtok(buf, ","); t && n < max; t = strtok(NULL, ",")) {
        char *end = NULL;
        double v = strtod(t, &end);
        if (end == t || v < 0) return -1;
        out[n++] = (int64_t)llround(v * 1000.0);
    }
    return n;
}

static int parse_keys(const char *spec, tui_timed_key_t *out, int max) {
    int n = 0;
    char buf[4096];
    snprintf(buf, sizeof(buf), "%s", spec ? spec : "");
    char *save_group = NULL;
    for (char *g = strtok_r(buf, ";", &save_group); g; g = strtok_r(NULL, ";", &save_group)) {
        char *colon = strchr(g, ':');
        if (!colon) return -1;
        *colon = '\0';
        char *end = NULL;
        double t = strtod(g, &end);
        if (end == g || t < 0) return -1;
        char *save_key = NULL;
        for (char *k = strtok_r(colon + 1, " ", &save_key); k; k = strtok_r(NULL, " ", &save_key)) {
            int code = tui_key_from_name(k);
            if (code == TUI_KEY_NONE || n >= max) return -1;
            out[n].at_ms = (int64_t)llround(t * 1000.0);
            out[n].key = code;
            n++;
        }
    }
    return n;
}

/* The terminal's colors, lowered by --colors. */
static int clamp_colors(int term_colors, int requested) {
    if (requested >= 0 && requested < term_colors) return requested;
    return term_colors;
}

static int cmp_i64(const void *a, const void *b) {
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static int cmp_key(const void *a, const void *b) {
    const tui_timed_key_t *x = (const tui_timed_key_t *)a, *y = (const tui_timed_key_t *)b;
    return (x->at_ms > y->at_ms) - (x->at_ms < y->at_ms);
}

static int run_snapshot(tui_app_t *app, frame_source_t *src, int cols, int rows,
                        const int64_t *times, int ntimes, const tui_timed_key_t *keys, int nkeys,
                        bool color_map) {
    tui_grid_t *grid = tui_grid_new(cols, rows);
    if (!grid) return 1;
    int64_t first = 0, last = 0;
    frame_source_replay_range(src, &first, &last, NULL, NULL);
    tui_app_start(app);

    double clock = (double)first;
    int64_t sim = 0;
    int next_time = 0, next_key = 0;
    frame_source_replay_advance(src, first);
    for (;;) {
        while (next_key < nkeys && keys[next_key].at_ms <= sim) tui_app_key(app, keys[next_key++].key);
        tui_app_tick(app);
        while (next_time < ntimes && times[next_time] <= sim) {
            tui_app_render(app, grid);
            printf("--- snapshot T=%.3f %dx%d ---\n", (double)times[next_time] / 1000.0, cols, rows);
            tui_grid_dump_text(grid, stdout);
            if (color_map) {
                printf("--- color-map T=%.3f %dx%d ---\n", (double)times[next_time] / 1000.0, cols, rows);
                tui_grid_dump_roles(grid, stdout);
                if (tui_grid_has_fg(grid)) {
                    printf("--- fg-map T=%.3f %dx%d ---\n", (double)times[next_time] / 1000.0, cols, rows);
                    tui_grid_dump_fg(grid, stdout);
                }
            }
            next_time++;
        }
        if (next_time >= ntimes || tui_app_quit_requested(app)) break;
        sim += 10;
        if (!frame_source_replay_paused(src)) clock += 10.0 * frame_source_replay_speed(src);
        frame_source_replay_advance(src, (int64_t)clock);
    }
    tui_app_shutdown(app);
    tui_grid_free(grid);
    return 0;
}

/*******************************************************************************
 * Terminal, signals, stderr capture
 ******************************************************************************/

static volatile int g_tui_stop = 0;
static tui_capture_t g_capture;

static void tui_on_signal(int sig) {
    (void)sig;
    g_tui_stop = 1;
}

static void tui_atexit(void) {
    tui_term_close();
    tui_capture_end(&g_capture);
}

static void capture_to_app(const char *line, void *ctx) {
    tui_app_log((tui_app_t *)ctx, line);
}

static int run_interactive_replay(tui_app_t *app, frame_source_t *src) {
    tui_grid_t *grid = tui_grid_new(80, 24);
    if (!grid) return 1;
    int cols = 0, rows = 0;
    tui_term_size(&cols, &rows);
    tui_app_resize(app, cols, rows);
    tui_app_start(app);

    int64_t last_draw = 0;
    while (!g_tui_stop && !tui_app_quit_requested(app)) {
        frame_source_pump(src, 0.01);
        tui_capture_poll(&g_capture, capture_to_app, app);
        bool keyed = false;
        int k;
        while ((k = tui_term_key()) != TUI_KEY_NONE) {
            if (k == TUI_KEY_RESIZE) {
                tui_term_size(&cols, &rows);
                tui_app_resize(app, cols, rows);
            } else {
                tui_app_key(app, k);
            }
            keyed = true;
        }
        tui_app_tick(app);
        int64_t wall = frame_source_wall_ms();
        if (keyed || wall - last_draw >= 100) {
            tui_app_render(app, grid);
            tui_term_draw(grid);
            last_draw = wall;
        }
    }
    tui_app_shutdown(app);
    tui_grid_free(grid);
    return 0;
}

/* Replay inside an open terminal (from --replay, o on the connect screen, or
 * the replay list). Selecting another capture in the list reloads it without
 * leaving the terminal; if that one cannot be loaded, the previous capture
 * plays again with the reason in its Log panel. */
static int replay_in_terminal(const char *path, const tui_app_options_t *opt) {
    char next[1024], prev[1024] = "", warn[1200] = "";
    snprintf(next, sizeof(next), "%s", path);
    for (;;) {
        tui_replay_data_t data;
        char err[1024] = "";
        int loaded = tui_replay_load(next, &data, err, sizeof(err));
        if (loaded == 0 && data.frames.count == 0) {
            tui_replay_free(&data);
            snprintf(err, sizeof(err), "no frames in %s", next);
            loaded = -1;
        }
        if (loaded != 0) {
            if (prev[0]) {   /* picked in the list: back to the capture that played */
                snprintf(warn, sizeof(warn), "[WARN] cannot replay: %s", err);
                snprintf(next, sizeof(next), "%s", prev);
                prev[0] = '\0';
                continue;
            }
            tui_term_close();
            if (warn[0]) fprintf(stderr, "%s\n", warn);
            fprintf(stderr, "[ERROR] tui: %s\n", err);
            return 1;
        }
        frame_source_t *src = frame_source_replay_create(data.frames.frames, data.frames.count, true);
        char name[256];
        snprintf(name, sizeof(name), "%s", data.name);
        tui_replay_free(&data);
        if (!src) {
            tui_term_close();
            fprintf(stderr, "[ERROR] tui: out of memory\n");
            return 1;
        }
        int cols = 80, rows = 24;
        tui_term_size(&cols, &rows);
        tui_app_options_t ropt = *opt;
        ropt.replay_path = next;   /* its session.json names the profile */
        tui_app_t *app = tui_app_create_replay(&ropt, src, name, cols, rows, err, sizeof(err));
        if (!app) {
            tui_term_close();
            fprintf(stderr, "[ERROR] tui: %s\n", err);
            frame_source_free(src);
            return 1;
        }
        if (warn[0]) {
            tui_app_log(app, warn);
            warn[0] = '\0';
        }
        int rc = run_interactive_replay(app, src);
        const char *req = tui_app_replay_request(app);
        char req_copy[1024];
        req_copy[0] = '\0';
        if (req && req[0]) snprintf(req_copy, sizeof(req_copy), "%s", req);
        tui_app_free(app);
        frame_source_free(src);
        if (req_copy[0]) {
            snprintf(prev, sizeof(prev), "%s", next);
            snprintf(next, sizeof(next), "%s", req_copy);
            continue;
        }
        tui_term_close();
        return rc;
    }
}

/*******************************************************************************
 * Entry
 ******************************************************************************/

int cmd_tui_main(int argc, const char *argv[]) {
    @autoreleasepool {
        tui_cli_t o;
        if (!parse_cli(argc, argv, &o)) {
            tui_usage();
            return 2;
        }
        if (o.device_given && (!o.device || !o.device[0])) {
            fprintf(stderr, "[ERROR] tui: --device needs a non-empty adapter name\n");
            return 2;
        }
        if (o.replay && o.device_given) {
            fprintf(stderr, "[ERROR] tui: --device and --replay exclude each other\n");
            return 2;
        }
        pcan_bitrate_t bitrate;
        if (!pcan_parse_bitrate(o.bitrate, &bitrate)) {
            fprintf(stderr, "[ERROR] tui: bad --bitrate %s (auto or %s)\n", o.bitrate, pcan_bitrate_list());
            return 2;
        }
        if (o.replay && o.bitrate) {
            fprintf(stderr, "[ERROR] tui: --bitrate is for a live PCAN device, not --replay\n");
            return 2;
        }
        if (o.speed < FRAME_SOURCE_REPLAY_MIN_SPEED || o.speed > FRAME_SOURCE_REPLAY_MAX_SPEED) {
            fprintf(stderr, "[ERROR] tui: --speed must be 0.25..16\n");
            return 2;
        }
        bool snapshot = o.snapshot_at != NULL;
#ifdef CAN_TOOLS_TEST_HOOKS
        const char *hook = getenv("CAN_TOOLS_TUI_HEADLESS");
        bool headless_live = snapshot && !o.replay && hook && strcmp(hook, "1") == 0;
#else
        const bool headless_live = false;   /* real build: scripted keys never drive the live path */
#endif
        if (snapshot && !o.replay && !headless_live) {
            fprintf(stderr, "[ERROR] tui: --snapshot-at needs --replay\n");
            return 2;
        }
        if (!snapshot && (o.keys || o.size || o.color_map)) {
            fprintf(stderr, "[ERROR] tui: --keys, --size and --color-map need --snapshot-at\n");
            return 2;
        }
        if (o.profile && o.profile[0] && strcmp(o.profile, "none") != 0) {
            char perr[600] = "";
            vsp_profile_t *p = vsp_profile_load(o.profile, perr, sizeof(perr));
            if (!p) {
                fprintf(stderr, "[ERROR] tui: %s\n", perr);
                return 1;
            }
            vsp_profile_free(p);
        }

        char exe[1024] = "";
        uint32_t exe_len = sizeof(exe);
        if (_NSGetExecutablePath(exe, &exe_len) != 0) exe[0] = '\0';
        char state_dir[1024] = "";
        const char *home = getenv("HOME");
        if (home && home[0]) snprintf(state_dir, sizeof(state_dir), "%s/.can_tools", home);
        tui_app_options_t opt = {
            .profile_path = o.profile,
            .profiles_dir = o.profiles_dir,
            .exe_path = exe,
            .captures_root = o.captures ? o.captures : "captures",
            /* a replay snapshot never touches ~/.can_tools */
            .state_dir = (snapshot && o.replay) ? NULL : (state_dir[0] ? state_dir : NULL),
            .speed = o.speed,
            .no_tx = o.no_tx,
            .no_rec = o.no_rec,
            .ascii = o.ascii,
            .headless = snapshot && o.replay,
            .colors = o.colors >= 0 ? o.colors : 256,   /* terminals: replaced once one is open */
        };

        int cols = 120, rows = 40;
        int64_t times[64];
        tui_timed_key_t keys[256];
        int nt = 0, nk = 0;
        if (snapshot) {
            if (o.size && (sscanf(o.size, "%dx%d", &cols, &rows) != 2 || cols < 20 || rows < 8 ||
                           cols > 1000 || rows > 500)) {
                fprintf(stderr, "[ERROR] tui: --size must be COLSxROWS\n");
                return 2;
            }
            nt = parse_times(o.snapshot_at, times, 64);
            nk = o.keys ? parse_keys(o.keys, keys, 256) : 0;
            if (nt <= 0 || nk < 0) {
                fprintf(stderr, "[ERROR] tui: bad --snapshot-at or --keys\n");
                return 2;
            }
            qsort(times, (size_t)nt, sizeof(times[0]), cmp_i64);
            if (nk > 1) mergesort(keys, (size_t)nk, sizeof(keys[0]), cmp_key);
        }

        /* ---- live ---- */
        if (!o.replay) {
            if (!headless_live && (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO))) {
                fprintf(stderr, "[ERROR] tui: needs a terminal\n");
                return 2;
            }
            signal(SIGINT, tui_on_signal);
            signal(SIGTERM, tui_on_signal);
            signal(SIGHUP, tui_on_signal);
            atexit(tui_atexit);
            bool ascii = o.ascii;
            if (!headless_live && !tui_term_open(o.mono, o.ascii, &ascii)) {
                fprintf(stderr, "[ERROR] tui: cannot initialize the terminal\n");
                return 1;
            }
            tui_capture_begin(&g_capture);
            tui_live_options_t lo = {
                .device = o.device_given ? o.device : NULL,
                .bitrate = bitrate,
                .profile = o.profile,
                .profiles_dir = o.profiles_dir,
                .captures_root = opt.captures_root,
                .state_dir = opt.state_dir,
                .exe_path = exe,
                .no_tx = o.no_tx,
                .no_rec = o.no_rec,
                .ascii = ascii,
                .colors = headless_live ? opt.colors : clamp_colors(tui_term_colors(), o.colors),
                .capture = g_capture.active ? &g_capture : NULL,
                .stop = &g_tui_stop,
                .headless = headless_live,
                .snap_times = times,
                .nsnap = nt,
                .keys = keys,
                .nkeys = nk,
                .cols = cols,
                .rows = rows,
                .color_map = o.color_map,
            };
            char replay_path[1024] = "";
            int rc = tui_live_run(&lo, replay_path, sizeof(replay_path));
            if (rc == 3 && replay_path[0] && !headless_live) {
                opt.ascii = ascii;
                opt.colors = clamp_colors(tui_term_colors(), o.colors);
                opt.profile_path = NULL;
                rc = replay_in_terminal(replay_path, &opt);
            }
            tui_term_close();
            tui_capture_end(&g_capture);
            return rc == 3 ? 0 : rc;
        }

        /* ---- replay ---- */
        tui_replay_data_t data;
        char err[1024] = "";
        if (tui_replay_load(o.replay, &data, err, sizeof(err)) != 0) {
            fprintf(stderr, "[ERROR] tui: %s\n", err);
            return 1;
        }
        if (data.frames.count == 0) {
            fprintf(stderr, "[ERROR] tui: no frames in %s\n", o.replay);
            tui_replay_free(&data);
            return 1;
        }
        if (snapshot) {
            frame_source_t *src = frame_source_replay_create_manual(data.frames.frames, data.frames.count);
            char name[256];
            snprintf(name, sizeof(name), "%s", data.name);
            tui_replay_free(&data);
            if (!src) {
                fprintf(stderr, "[ERROR] tui: out of memory\n");
                return 1;
            }
            opt.replay_path = o.replay;
            tui_app_t *app = tui_app_create_replay(&opt, src, name, cols, rows, err, sizeof(err));
            if (!app) {
                fprintf(stderr, "[ERROR] tui: %s\n", err);
                frame_source_free(src);
                return 1;
            }
            int rc = run_snapshot(app, src, cols, rows, times, nt, keys, nk, o.color_map);
            tui_app_free(app);
            frame_source_free(src);
            return rc;
        }
        tui_replay_free(&data);
        if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
            fprintf(stderr, "[ERROR] tui: needs a terminal (use --snapshot-at for a text dump)\n");
            return 2;
        }
        signal(SIGINT, tui_on_signal);
        signal(SIGTERM, tui_on_signal);
        signal(SIGHUP, tui_on_signal);
        atexit(tui_atexit);
        bool ascii = o.ascii;
        if (!tui_term_open(o.mono, o.ascii, &ascii)) {
            fprintf(stderr, "[ERROR] tui: cannot initialize the terminal\n");
            return 1;
        }
        tui_capture_begin(&g_capture);
        opt.ascii = ascii;
        opt.colors = clamp_colors(tui_term_colors(), o.colors);
        int rc = replay_in_terminal(o.replay, &opt);
        tui_capture_end(&g_capture);
        return rc;
    }
}
