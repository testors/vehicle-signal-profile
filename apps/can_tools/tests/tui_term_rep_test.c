/*
 * tui_term_rep_test: the ncurses backend (tui_term.c) must write every
 * character of a run, also under a terminfo whose "rep" capability starts
 * with %p1%c, as xterm-ghostty's does ("%p1%c\E[%p2%{1}%-%db"). macOS ncurses
 * 6.0 sends runs of one character through rep even for multibyte
 * characters; %c of U+2500 is a NUL, so the run vanished (Ghostty bug).
 *
 * The test compiles its own terminfo entry with tic into a temporary folder
 * (xterm-256color + that rep), draws one fixed screen through tui_term in a
 * pseudo-terminal, once under plain xterm-256color and once under the test
 * entry, and requires every box / bar / sparkline glyph of the screen to
 * appear in the output exactly as often as in the grid, with no rep
 * sequence. No Ghostty, no terminal of its own, no adapter.
 *
 *   tui_term_rep_test            run the check
 *   tui_term_rep_test --draw     (child) draw the screen and exit
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <util.h>

#include <math.h>

#include "tui_gauges.h"
#include "tui_grid.h"
#include "tui_model.h"
#include "tui_render.h"
#include "tui_term.h"

#define COLS 80
#define ROWS 24

/* Screen 1: the dashboard frame of an empty model plus runs of bar and
   sparkline glyphs. */
static tui_grid_t *build_frame_screen(void) {
    tui_grid_t *g = tui_grid_new(COLS, ROWS);
    if (!g) return NULL;
    tui_model_t m;
    tui_model_init(&m, 0);
    tui_view_t v;
    memset(&v, 0, sizeof(v));
    v.cols = COLS;
    v.rows = ROWS;
    v.device = "rep test";
    v.filter = "";
    v.tab = TUI_PANEL_RAW;
    v.focus = TUI_PANEL_RAW;
    v.colors = 256;
    tui_render_dashboard(g, &m, &v);
    tui_model_free(&m);
    tui_grid_fill(g, 3, 4, 20, "█", TUI_ROLE_BRIGHT);
    tui_grid_fill(g, 23, 4, 20, "░", TUI_ROLE_DIM);
    tui_grid_fill(g, 3, 5, 30, "━", TUI_ROLE_DEFAULT);
    tui_grid_put(g, 3, 6, "▁▁▁▁▂▂▂▃▃▅▅▅▅▅▅▇▇▇▇██████▇▅▃▂▁", TUI_ROLE_DEFAULT, 40);
    return g;
}

/* Screen 2 (step 4): Gauges panel (■ meters with 256-color gradients,
   steering with ┃ and ◀, gear strip) and braille timelines. */
static tui_grid_t *build_gauge_screen(void) {
    tui_grid_t *g = tui_grid_new(COLS, ROWS);
    if (!g) return NULL;
    tui_signal_desc_t d[6] = {
        { .name = "Engine_RPM", .unit = "rpm", .group = "rpm", .factor = 0.25, .cycle_ms = 10,
          .canonical = "engine.speed", .decoder_index = 0 },
        { .name = "Throttle", .unit = "%", .group = "throttle", .factor = 0.1, .cycle_ms = 20,
          .canonical = "throttle.position", .decoder_index = 1 },
        { .name = "Steering", .unit = "deg", .group = "steering", .factor = 0.1, .cycle_ms = 10,
          .canonical = "steering.wheel.angle", .decoder_index = 2 },
        { .name = "Coolant", .unit = "°C", .group = "custom", .factor = 1, .cycle_ms = 1000,
          .canonical = "engine.coolant.temperature", .decoder_index = 3 },
        { .name = "Speed", .unit = "km/h", .group = "custom", .factor = 0.01, .cycle_ms = 20,
          .canonical = "vehicle.speed", .decoder_index = 4 },
        { .name = "Gear", .unit = "", .group = "gear", .factor = 1, .cycle_ms = 50,
          .canonical = "transmission.gear.selected", .decoder_index = 5, .max_gear = 7 },
    };
    tui_model_t m;
    tui_model_init(&m, 0);
    tui_model_set_signals(&m, d, 6);
    for (int64_t ms = 0; ms <= 10000; ms += 10) {
        double t = (double)ms / 1000.0;
        for (int i = 0; i < m.signal_count; i++) {
            tui_signal_t *s = &m.signals[i];
            double v;
            if (strcmp(s->name, "Engine_RPM") == 0) v = 3000 + 2500 * sin(t * 0.8) * sin(t * 0.8);
            else if (strcmp(s->name, "Throttle") == 0) v = 100 * fabs(sin(t * 0.9));
            else if (strcmp(s->name, "Steering") == 0) v = ms < 8000 ? 92 * sin(t) : 240;
            else if (strcmp(s->name, "Coolant") == 0) { if (ms % 1000) continue; v = 92 + (ms > 5000); }
            else if (strcmp(s->name, "Speed") == 0) v = 128 + 20 * sin(t * 0.3);
            else { if (ms % 50) continue; v = 3; }
            tui_signal_sample(s, ms, true, false, v, NULL);
        }
    }
    tui_gauges_t gs;
    tui_gauges_init(&gs);
    tui_gauges_bind(&gs, &m);
    tui_gauges_update(&gs, &m, 10000);
    tui_view_t v;
    memset(&v, 0, sizeof(v));
    v.cols = COLS;
    v.rows = ROWS;
    v.now_ms = 10000;
    v.device = "rep test";
    v.filter = "";
    v.tab = TUI_PANEL_GAUGES;
    v.focus = TUI_PANEL_SIGNALS;
    v.gauges = &gs;
    v.colors = 256;
    tui_render_dashboard(g, &m, &v);
    tui_gauges_free(&gs);
    tui_model_free(&m);
    return g;
}

static tui_grid_t *build_screen(int which) {
    return which == 2 ? build_gauge_screen() : build_frame_screen();
}

static int draw_child(int which) {
    bool ascii = false;
    if (!tui_term_open(false, false, &ascii)) return 3;
    tui_grid_t *g = build_screen(which);
    if (!g) return 4;
    tui_term_draw(g);
    tui_term_close();
    tui_grid_free(g);
    return 0;
}

/* Distinct non-ASCII glyphs of a grid with their counts. */
typedef struct {
    char glyph[8];
    int  count;
} glyph_count_t;

static int tally_glyphs(const tui_grid_t *g, glyph_count_t *out, int max) {
    int n = 0;
    for (int y = 0; y < g->rows; y++) {
        for (int x = 0; x < g->cols; x++) {
            const tui_cell_t *c = tui_grid_cell(g, x, y);
            if (c->width == 0 || (unsigned char)c->ch[0] < 0x80) continue;
            int k = 0;
            while (k < n && strcmp(out[k].glyph, c->ch) != 0) k++;
            if (k == n) {
                if (n == max) continue;
                snprintf(out[n].glyph, sizeof(out[n].glyph), "%s", c->ch);
                out[n++].count = 0;
            }
            out[k].count++;
        }
    }
    return n;
}

static int count_str(const char *buf, size_t len, const char *needle) {
    size_t n = strlen(needle);
    int count = 0;
    for (size_t i = 0; i + n <= len; i++) {
        if (memcmp(buf + i, needle, n) == 0) {
            count++;
            i += n - 1;
        }
    }
    return count;
}

/* "\E[<digits>b": the rep capability's tail */
static int count_rep(const char *buf, size_t len) {
    int count = 0;
    for (size_t i = 0; i + 3 < len; i++) {
        if (buf[i] != 0x1b || buf[i + 1] != '[') continue;
        size_t j = i + 2;
        while (j < len && buf[j] >= '0' && buf[j] <= '9') j++;
        if (j > i + 2 && j < len && buf[j] == 'b') count++;
    }
    return count;
}

/* Run "<self> --draw" in a pty with TERM (and TERMINFO); collect its output. */
static char *run_pty(const char *self, int which, const char *term, const char *terminfo, size_t *len_out,
                     int *status) {
    struct winsize ws = { .ws_row = ROWS, .ws_col = COLS };
    int master = -1;
    pid_t pid = forkpty(&master, NULL, NULL, &ws);
    if (pid < 0) return NULL;
    if (pid == 0) {
        setenv("TERM", term, 1);
        unsetenv("TERMINFO_DIRS");
        if (terminfo) setenv("TERMINFO", terminfo, 1);
        else unsetenv("TERMINFO");
        setenv("LC_ALL", "en_US.UTF-8", 1);
        unsetenv("NO_COLOR");
        execl(self, self, "--draw", which == 2 ? "2" : "1", (char *)NULL);
        _exit(127);
    }
    size_t cap = 1 << 16, len = 0;
    char *buf = (char *)malloc(cap);
    time_t start = time(NULL);
    for (;;) {
        struct pollfd p = { .fd = master, .events = POLLIN };
        int r = poll(&p, 1, 200);
        if (r > 0) {
            if (len + 4096 > cap) {
                cap *= 2;
                buf = (char *)realloc(buf, cap);
            }
            ssize_t n = read(master, buf + len, cap - len);
            if (n <= 0) break;            /* EOF / EIO: the child closed the pty */
            len += (size_t)n;
        }
        if (time(NULL) - start > 10) {
            kill(pid, SIGKILL);           /* only the child this test started */
            break;
        }
    }
    close(master);
    waitpid(pid, status, 0);
    *len_out = len;
    return buf;
}

int main(int argc, char **argv) {
    if (argc >= 3 && strcmp(argv[1], "--draw") == 0) return draw_child(atoi(argv[2]));

    int failures = 0, checks = 0;
    char tmpl[] = "/tmp/tui_term_rep_test.XXXXXX";
    char *dir = mkdtemp(tmpl);
    if (!dir) {
        perror("mkdtemp");
        return 1;
    }
    char src[256], cmd[1024];
    snprintf(src, sizeof(src), "%s/tui-rep-test.src", dir);
    FILE *f = fopen(src, "w");
    if (!f) return 1;
    /* xterm-256color plus xterm-ghostty's rep */
    fprintf(f, "tui-rep-test|xterm-256color with rep like xterm-ghostty,\n"
               "\trep=%%p1%%c\\E[%%p2%%{1}%%-%%db,\n"
               "\tuse=xterm-256color,\n");
    fclose(f);
    snprintf(cmd, sizeof(cmd), "env -u TERMINFO -u TERMINFO_DIRS tic -x -o '%s' '%s'", dir, src);
    if (system(cmd) != 0) {
        fprintf(stderr, "FAIL tic could not compile the test terminfo\n");
        return 1;
    }
    snprintf(cmd, sizeof(cmd), "infocmp -x -1 -A '%s' tui-rep-test | grep -q 'rep=%%p1%%c'", dir);
    checks++;
    if (system(cmd) != 0) {
        fprintf(stderr, "FAIL the test terminfo has no rep=%%p1%%c\n");
        failures++;
    }

    const char *terms[2] = { "xterm-256color", "tui-rep-test" };
    const char *infos[2] = { NULL, dir };
    for (int which = 1; which <= 2; which++) {
        tui_grid_t *g = build_screen(which);
        glyph_count_t expect[64];
        int nglyphs = g ? tally_glyphs(g, expect, 64) : 0;
        tui_grid_free(g);
        checks++;
        if (nglyphs < (which == 2 ? 10 : 15)) {
            fprintf(stderr, "FAIL screen %d: only %d distinct glyphs\n", which, nglyphs);
            failures++;
        }
        for (int t = 0; t < 2; t++) {
            size_t len = 0;
            int status = 0;
            char *out = run_pty(argv[0], which, terms[t], infos[t], &len, &status);
            checks++;
            if (!out || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
                fprintf(stderr, "FAIL screen %d %s: draw child failed (status %d)\n", which, terms[t], status);
                failures++;
                free(out);
                continue;
            }
            int reps = count_rep(out, len);
            checks++;
            if (reps != 0) {
                fprintf(stderr, "FAIL screen %d %s: %d rep sequences in the output\n", which, terms[t], reps);
                failures++;
            }
            for (int k = 0; k < nglyphs; k++) {
                int got = count_str(out, len, expect[k].glyph);
                checks++;
                if (got != expect[k].count) {
                    fprintf(stderr, "FAIL screen %d %s: %s written %d times, the screen has %d\n", which, terms[t],
                            expect[k].glyph, got, expect[k].count);
                    failures++;
                }
            }
            printf("screen %d %s: %zu bytes, %d glyph kinds, %d rep sequences\n", which, terms[t], len, nglyphs, reps);
            free(out);
        }
    }

    snprintf(cmd, sizeof(cmd), "rm -r '%s'", dir);
    if (system(cmd) != 0) fprintf(stderr, "[WARN] could not remove %s\n", dir);
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
