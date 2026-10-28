/**
 * @file tui_app.c
 * @brief TUI application state, frame fan-out, overlays and key dispatch (C)
 */

#include "tui_app.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>

#include "discover_session.h"
#include "tui_gauges.h"
#include "tui_model.h"
#include "tui_profiles.h"
#include "tui_recorder.h"
#include "tui_render.h"
#include "tui_replay.h"
#include "tui_screens.h"
#include "vsp_engine.h"

#define TX_MIN_INTERVAL_MS 100   /* at most 10 queries/s in total */
#define TX_MAX_FRAMES      16
#define HINT_EVAL_MS       500
#define MAX_CLS_CHANGES    64
#define MAX_REC_FILES      32

typedef enum {
    OVL_NONE = 0,
    OVL_PICKER,
    OVL_REPLAY,
    OVL_DISCOVER,
    OVL_RESULT,
    OVL_JSON,
    OVL_HELP,
    OVL_TX_CONFIRM,
    OVL_QUIT_CONFIRM,
} overlay_t;

struct tui_app {
    tui_app_options_t opt;
    char              profiles_dir[1024];
    char              exe_path[1024];
    char              captures_root[512];
    char              state_dir[1024];

    frame_source_t   *src;
    char              source_name[256];
    int               cols, rows;
    bool              replay;
    bool              live;
    tui_host_t        host;
    char              device_name[128];
    char              device_prefix[128];
    char              transport[16];
    char              stn[64];
    char              monitor_cmd[16];
    char              session_dir[512];
    char              started_at[32];

    /* profile */
    vsp_profile_t    *profile;
    vsp_decoder_t    *decoder;
    char              profile_path[1024];
    char              profile_short[160];
    int              *updated;
    int               updated_cap;
    int               diag_count;

    tui_model_t       model;
    tui_gauges_t      gauges;
    tui_view_t        view;
    char              filter[64];
    bool              filter_editing;

    int64_t           replay_first_ms;
    int64_t           replay_last_ms;
    bool              end_logged;
    bool              quit;

    overlay_t         overlay;
    overlay_t         overlay_back;   /* help / quit confirm / JSON return here */
    int               text_scroll;
    char            **text_lines;     /* JSON view */
    int               text_count;

    /* discover */
    disc_session_t   *disc;
    int               disc_phase;     /* capturing, -1 = none */
    int64_t           disc_start_ms;
    int               disc_next;
    tui_phase_state_t disc_state[DISC_NUM_PHASES];
    bool              disc_analyzing;
    bool              disc_rendered;
    tui_result_view_t result;
    bool              have_result;
    char              result_dir[600];
    char              result_json[1024];
    char              draft_vsp[1024];
    bool              hint_valid;
    uint32_t          hint_id;
    int               hint_byte;
    tui_signal_t      hint_sig;
    int64_t           hint_eval_ms;
    char              live_text[256];
    char              live_spark[256];

    /* profile picker */
    tui_profile_index_t pix;
    bool              pix_ready;
    char              pick_filter[64];
    int               pick_sel;
    int               pick_scroll;
    int              *pick_idx;
    int               pick_n;
    tui_picker_row_t *pick_rows;
    char              pick_want[1024]; /* preselect this path once indexed */

    /* replay list */
    tui_replay_list_t rlist;
    int               rlist_sel;
    int               rlist_scroll;
    char              replay_request[1024];

    /* recording + session.json (live) */
    tui_recorder_t    rec;
    char              rec_files[MAX_REC_FILES][64];
    int               rec_nfiles;
    struct {
        double    t_s;
        tui_bus_t bus;
    }                 cls_changes[MAX_CLS_CHANGES];
    int               ncls;

    /* TX mode */
    bool              tx_on;
    int              *tx_queries;
    int               tx_nq;
    int               tx_next;
    int64_t           tx_last_ms;
    uint64_t          tx_sent;
    uint64_t          tx_errors;

    bool              disconnected;
    bool              want_connect;
    bool              monitor_on;
};

static int64_t app_now(const tui_app_t *app) {
    return frame_source_now_ms(app->src);
}

static void app_log(tui_app_t *app, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void app_log(tui_app_t *app, const char *fmt, ...) {
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    tui_log_add(&app->model.log, app_now(app), line);
}

/*******************************************************************************
 * Keys
 ******************************************************************************/

int tui_key_from_name(const char *name) {
    if (!name || !name[0]) return TUI_KEY_NONE;
    if (strcmp(name, "enter") == 0) return TUI_KEY_ENTER;
    if (strcmp(name, "esc") == 0) return TUI_KEY_ESC;
    if (strcmp(name, "tab") == 0) return TUI_KEY_TAB;
    if (strcmp(name, "space") == 0) return ' ';
    if (strcmp(name, "up") == 0) return TUI_KEY_UP;
    if (strcmp(name, "down") == 0) return TUI_KEY_DOWN;
    if (strcmp(name, "pgup") == 0) return TUI_KEY_PGUP;
    if (strcmp(name, "pgdn") == 0) return TUI_KEY_PGDN;
    if (strcmp(name, "backspace") == 0) return TUI_KEY_BACKSPACE;
    if (strcmp(name, "ctrl-n") == 0) return TUI_KEY_CTRL_N;
    if (name[1] == '\0') return (unsigned char)name[0];
    return TUI_KEY_NONE;
}

/*******************************************************************************
 * tui_last
 ******************************************************************************/

static void trim_eol(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = '\0';
}

bool tui_last_read(const char *state_dir, char *device, size_t dlen, char *profile, size_t plen,
                   char *date, size_t datelen) {
    if (device && dlen) device[0] = '\0';
    if (profile && plen) profile[0] = '\0';
    if (date && datelen) date[0] = '\0';
    if (!state_dir || !state_dir[0]) return false;
    char path[1200];
    snprintf(path, sizeof(path), "%s/tui_last", state_dir);
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char line[1200];
    while (fgets(line, sizeof(line), f)) {
        trim_eol(line);
        if (strncmp(line, "device=", 7) == 0 && device) snprintf(device, dlen, "%s", line + 7);
        else if (strncmp(line, "profile=", 8) == 0 && profile) snprintf(profile, plen, "%s", line + 8);
        else if (strncmp(line, "date=", 5) == 0 && date) snprintf(date, datelen, "%s", line + 5);
    }
    fclose(f);
    return (device && device[0]) || (profile && profile[0]);
}

bool tui_last_write(const char *state_dir, const char *device, const char *profile) {
    if (!state_dir || !state_dir[0]) return false;
    char old_dev[256], old_prof[1024], old_date[32];
    tui_last_read(state_dir, old_dev, sizeof(old_dev), old_prof, sizeof(old_prof), old_date, sizeof(old_date));
    mkdir(state_dir, 0755);
    char path[1200], tmp[1300];
    snprintf(path, sizeof(path), "%s/tui_last", state_dir);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return false;
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char date[32];
    strftime(date, sizeof(date), "%Y-%m-%d", &tm_now);
    fprintf(f, "device=%s\n", device ? device : old_dev);
    fprintf(f, "profile=%s\n", profile ? profile : old_prof);
    fprintf(f, "date=%s\n", date);
    fclose(f);
    return rename(tmp, path) == 0;
}

static void save_last(tui_app_t *app) {
    if (!app->state_dir[0] || (app->opt.headless && !app->live)) return;
    const char *prof = app->profile_path[0] ? app->profile_path : "none";
    tui_last_write(app->state_dir, app->live ? app->device_name : NULL, prof);
}

/*******************************************************************************
 * session.json (live)
 ******************************************************************************/

static void json_str(FILE *f, const char *s) {
    if (!s) {
        fputs("null", f);
        return;
    }
    fputc('"', f);
    for (; *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') fprintf(f, "\\%c", c);
        else if (c == '\n') fputs("\\n", f);
        else if (c < 0x20) fprintf(f, "\\u%04x", c);
        else fputc(c, f);
    }
    fputc('"', f);
}

static const char *bus_json(tui_bus_t b) {
    switch (b) {
        case TUI_BUS_DIRECT:  return "DIRECT";
        case TUI_BUS_GATEWAY: return "GATEWAY";
        case TUI_BUS_SILENT:  return "SILENT";
        case TUI_BUS_DETECTING: break;
    }
    return "detecting";
}

static void wall_stamp(char *out, size_t len) {
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    strftime(out, len, "%Y-%m-%dT%H:%M:%S", &tm_now);
}

static void write_session_json(tui_app_t *app, bool ended) {
    if (!app->live || !app->session_dir[0]) return;
    char path[700], tmp[720];
    snprintf(path, sizeof(path), "%s/session.json", app->session_dir);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "{\n  \"format\": \"can_tools_tui_session/v1\",\n  \"started_at\": ");
    json_str(f, app->started_at);
    if (ended) {
        char stamp[32];
        wall_stamp(stamp, sizeof(stamp));
        fprintf(f, ",\n  \"ended_at\": ");
        json_str(f, stamp);
    }
    fprintf(f, ",\n  \"device\": ");
    json_str(f, app->device_name);
    fprintf(f, ",\n  \"device_prefix\": ");
    json_str(f, app->device_prefix[0] ? app->device_prefix : NULL);
    fprintf(f, ",\n  \"transport\": ");
    json_str(f, app->transport[0] ? app->transport : NULL);
    fprintf(f, ",\n  \"stn\": ");
    json_str(f, app->stn[0] ? app->stn : NULL);
    fprintf(f, ",\n  \"monitor_cmd\": ");
    json_str(f, app->monitor_cmd);
    fprintf(f, ",\n  \"profile\": ");
    json_str(f, app->profile_path[0] ? app->profile_path : NULL);
    fprintf(f, ",\n  \"recordings\": [");
    for (int i = 0; i < app->rec_nfiles; i++) {
        fprintf(f, "%s", i ? ", " : "");
        json_str(f, app->rec_files[i]);
    }
    fprintf(f, "],\n  \"classification\": [");
    for (int i = 0; i < app->ncls; i++) {
        fprintf(f, "%s\n    {\"t_s\": %.1f, \"bus\": \"%s\"}", i ? "," : "", app->cls_changes[i].t_s,
                bus_json(app->cls_changes[i].bus));
    }
    fprintf(f, "%s],\n  \"direct_seen\": %s\n}\n", app->ncls ? "\n  " : "",
            app->model.cls.direct_seen ? "true" : "false");
    fclose(f);
    rename(tmp, path);
}

/*******************************************************************************
 * Profile
 ******************************************************************************/

static void lower_join(char *out, size_t len, const char *s) {
    size_t n = 0;
    for (const char *p = s ? s : ""; *p && n + 1 < len; p++) {
        unsigned char c = (unsigned char)*p;
        out[n++] = c == ' ' ? '_' : (char)tolower(c);
    }
    out[n] = '\0';
}

static void profile_short_name(char *out, size_t len, const vsp_profile_t *p, const char *path) {
    vsp_profile_info_t info;
    vsp_profile_get_info(p, &info);
    char make[64] = "", model[64] = "", gen[32] = "";
    lower_join(make, sizeof(make), info.manufacturer);
    lower_join(model, sizeof(model), info.model);
    lower_join(gen, sizeof(gen), info.generation);
    if (strcmp(make, "unknown") == 0) make[0] = '\0';     /* discover drafts */
    if (strcmp(model, "unknown") == 0) model[0] = '\0';
    if (strcmp(gen, "unknown") == 0) gen[0] = '\0';
    if (make[0] || model[0]) {
        snprintf(out, len, "%s%s%s%s%s", make, model[0] ? " " : "", model, gen[0] ? " " : "", gen);
        return;
    }
    const char *base = strrchr(path, '/');
    snprintf(out, len, "%s", base ? base + 1 : path);
    char *dot = strstr(out, ".vsp");
    if (dot) *dot = '\0';
}

static void tx_leave(tui_app_t *app, bool restart_monitor);

static void unload_profile(tui_app_t *app) {
    vsp_decoder_free(app->decoder);
    vsp_profile_free(app->profile);
    free(app->updated);
    app->decoder = NULL;
    app->profile = NULL;
    app->updated = NULL;
    app->updated_cap = 0;
    app->diag_count = 0;
    app->profile_path[0] = '\0';
    app->profile_short[0] = '\0';
    tui_model_set_signals(&app->model, NULL, 0);
    tui_gauges_bind(&app->gauges, &app->model);
}

/* Highest forward gear (1..10) named in a value table, 0 if none. */
static int table_max_gear(const vsp_decoder_t *dec, int idx) {
    int best = 0;
    int n = vsp_decoder_value_count(dec, idx);
    for (int i = 0; i < n; i++) {
        long long raw = 0;
        const char *label = NULL;
        bool has_number = false;
        double value = 0;
        if (!vsp_decoder_value_entry(dec, idx, i, &raw, &label, &has_number, &value)) continue;
        int gear = 0;
        if (label && label[0]) {
            bool digits = true;
            for (const char *p = label; *p; p++) digits = digits && isdigit((unsigned char)*p);
            if (digits) gear = atoi(label);
        } else if (has_number && fabs(value - rint(value)) < 1e-9) {
            gear = (int)rint(value);
        }
        if (gear >= 1 && gear <= 10 && gear > best) best = gear;
    }
    return best;
}

static bool load_profile(tui_app_t *app, const char *path, char *err, size_t err_len) {
    char perr[512] = "";
    vsp_profile_t *profile = vsp_profile_load(path, perr, sizeof(perr));
    if (!profile) {
        snprintf(err, err_len, "%s", perr);
        return false;
    }
    vsp_select_t sel = { .signals = NULL, .all_signals = true, .acquisition = VSP_ACQ_ANY };
    vsp_decoder_t *decoder = vsp_decoder_create(profile, &sel);
    if (!decoder) {
        vsp_profile_free(profile);
        snprintf(err, err_len, "out of memory");
        return false;
    }
    if (app->tx_on) tx_leave(app, true);
    unload_profile(app);
    app->profile = profile;
    app->decoder = decoder;

    int n = vsp_decoder_signal_count(decoder);
    tui_signal_desc_t *descs = (tui_signal_desc_t *)calloc((size_t)n + 1, sizeof(*descs));
    app->updated = (int *)calloc((size_t)n + 1, sizeof(int));
    app->updated_cap = n;
    if (!descs || !app->updated) {
        free(descs);
        unload_profile(app);
        snprintf(err, err_len, "out of memory");
        return false;
    }
    for (int i = 0; i < n; i++) {
        const vsp_signal_info_t *info = vsp_decoder_signal_info(decoder, i);
        bool diag = info->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY;
        if (diag) app->diag_count++;
        descs[i] = (tui_signal_desc_t){
            .name = info->name,
            .unit = info->unit,
            .group = info->group,
            .diagnostic = diag,
            .has_value_table = info->has_value_table,
            .factor = info->factor,
            .lsb = info->lsb,
            .cycle_ms = info->cycle_ms,
            .frame_id = info->frame_id,
            .request_hex = info->request_hex,
            .decoder_index = i,
            .canonical = info->canonical_id,
            .max_gear = info->has_value_table ? table_max_gear(decoder, i) : 0,
        };
    }
    bool ok = tui_model_set_signals(&app->model, descs, n);
    free(descs);
    if (!ok) {
        unload_profile(app);
        snprintf(err, err_len, "out of memory");
        return false;
    }
    snprintf(app->profile_path, sizeof(app->profile_path), "%s", path);
    profile_short_name(app->profile_short, sizeof(app->profile_short), profile, path);

    int skipped = vsp_decoder_skipped_count(decoder);
    if (skipped > 0) app_log(app, "profile loaded: %d signals (%d skipped: reassembled)", n, skipped);
    else app_log(app, "profile loaded: %d signals", n);
    tui_gauges_bind(&app->gauges, &app->model);   /* also a new data session for the scales */
    if (n > 0) app_log(app, "gauges %d/%d (canonical id)", app->gauges.bound, TUI_GAUGE_COUNT);
    if (n > 0) {
        app->view.tab = app->gauges.bound > 0 ? TUI_PANEL_GAUGES : TUI_PANEL_SIGNALS;
        app->view.focus = TUI_PANEL_SIGNALS;
    }
    app->view.scroll[TUI_PANEL_SIGNALS] = 0;
    return true;
}

/* Picker / result / CLI: load, log failures, remember the choice. */
static void set_profile(tui_app_t *app, const char *path) {
    if (!path) {
        if (app->tx_on) tx_leave(app, true);
        unload_profile(app);
        app_log(app, "profile: none (raw CAN only)");
    } else {
        char err[600];
        if (!load_profile(app, path, err, sizeof(err))) {
            app_log(app, "[ERROR] profile: %s", err);
            return;
        }
    }
    save_last(app);
    write_session_json(app, false);
}

/*******************************************************************************
 * Recording (live)
 ******************************************************************************/

static void rec_start(tui_app_t *app) {
    if (!app->live || app->disconnected || tui_recorder_is_open(&app->rec)) return;
    char err[700];
    if (!tui_recorder_open(&app->rec, app_now(app), err, sizeof(err))) {
        app_log(app, "[ERROR] recording: %s", err);
        return;
    }
    const char *base = strrchr(app->rec.path, '/');
    base = base ? base + 1 : app->rec.path;
    if (app->rec_nfiles < MAX_REC_FILES) snprintf(app->rec_files[app->rec_nfiles++], 64, "%s", base);
    app_log(app, "rec on: %s", base);
    write_session_json(app, false);
}

static void rec_stop(tui_app_t *app) {
    if (!tui_recorder_is_open(&app->rec)) return;
    uint64_t frames = app->rec.frames;
    tui_recorder_close(&app->rec);
    char count[32];
    tui_format_count(count, sizeof(count), (long long)frames);
    app_log(app, "rec off (%s frames)", count);
    write_session_json(app, false);
}

/*******************************************************************************
 * TX mode (rule 0.2 / 0.3 / 0.6)
 ******************************************************************************/

const char *tui_app_tx_lock_reason(const tui_app_t *app) {
    if (app->model.cls.direct_seen || app->model.cls.bus == TUI_BUS_DIRECT) return "TX locked: DIRECT bus";
    if (app->replay || !app->live) return "TX unavailable in replay";
    if (app->opt.no_tx) return "TX disabled (--no-tx)";
    if (app->disconnected) return "TX unavailable: disconnected";
    if (app->model.cls.bus != TUI_BUS_GATEWAY && app->model.cls.bus != TUI_BUS_SILENT) {
        return "TX locked: bus not classified yet";
    }
    if (app->diag_count == 0) return "TX unavailable: the profile has no diagnostic signal";
    if (app->disc) return "TX unavailable during discover";
    return NULL;
}

bool tui_app_tx_on(const tui_app_t *app) {
    return app->tx_on;
}

bool tui_app_monitor_running(const tui_app_t *app) {
    return app->monitor_on;
}

static bool build_tx_queries(tui_app_t *app) {
    free(app->tx_queries);
    app->tx_queries = NULL;
    app->tx_nq = 0;
    int total = app->decoder ? vsp_decoder_signal_count(app->decoder) : 0;
    app->tx_queries = (int *)calloc((size_t)total + 1, sizeof(int));
    if (!app->tx_queries) return false;
    for (int i = 0; i < total; i++) {
        const vsp_signal_info_t *item = vsp_decoder_signal_info(app->decoder, i);
        if (item->kind != VSP_SIGNAL_DIAGNOSTIC_QUERY || !item->request_bare_hex || !item->request_bare_hex[0]) continue;
        bool seen = false;
        for (int j = 0; j < app->tx_nq && !seen; j++) {
            seen = strcmp(vsp_decoder_signal_info(app->decoder, app->tx_queries[j])->query_key, item->query_key) == 0;
        }
        if (!seen) app->tx_queries[app->tx_nq++] = i;
    }
    return app->tx_nq > 0;
}

static void tx_enter(tui_app_t *app) {
    const char *reason = tui_app_tx_lock_reason(app);
    if (reason) {
        app_log(app, "%s", reason);
        return;
    }
    if (!build_tx_queries(app)) {
        app_log(app, "TX unavailable: no OBD request in the profile");
        return;
    }
    frame_source_stop(app->src);
    app->monitor_on = false;
    app->model.cls.paused = true;
    app->tx_on = true;   /* from here on, leaving runs the PASSIVE re-init */
    if (!app->host.tx_setup || !app->host.tx_setup(app->host.ctx)) {
        app_log(app, "[ERROR] TX setup failed");
        tx_leave(app, true);
        return;
    }
    app->tx_next = 0;
    app->tx_last_ms = 0;
    app->tx_sent = 0;
    app->tx_errors = 0;
    app_log(app, "[WARN] TX ON: polling %d OBD request%s (at most 10/s)", app->tx_nq, app->tx_nq == 1 ? "" : "s");
}

static void tx_leave(tui_app_t *app, bool restart_monitor) {
    if (!app->tx_on) return;
    app->tx_on = false;
    bool connected = app->host.is_connected && app->host.is_connected(app->host.ctx);
    if (connected && app->host.passive_init) {
        if (app->host.passive_init(app->host.ctx)) app_log(app, "TX off: adapter back in silent mode (ATCSM1)");
        else app_log(app, "[ERROR] TX off: passive re-init failed");
    }
    if (restart_monitor && connected) {
        frame_source_start(app->src, tui_app_on_frame, app);
        app->monitor_on = true;
        tui_classifier_start(&app->model.cls, app_now(app));
    }
}

static void feed_decoded(tui_app_t *app, const can_frame_t *frame, int64_t ts, const char *query_key,
                         bool diagnostic_only) {
    if (!app->decoder) return;
    int n = diagnostic_only
        ? vsp_decoder_feed_diagnostic_collect(app->decoder, frame, (double)ts, query_key, app->updated, app->updated_cap)
        : vsp_decoder_feed_collect(app->decoder, frame, (double)ts, app->updated, app->updated_cap);
    if (n > app->updated_cap) n = app->updated_cap;
    for (int i = 0; i < n; i++) {
        int idx = app->updated[i];
        const vsp_signal_state_t *st = vsp_decoder_signal_state(app->decoder, idx);
        tui_signal_t *s = tui_model_signal_for_decoder(&app->model, idx);
        if (!st || !s) continue;
        tui_signal_sample(s, ts, st->has_value, st->value_null, st->value, st->label);
    }
}

static void tx_poll(tui_app_t *app) {
    if (!app->tx_on || app->tx_nq == 0 || !app->host.tx_query) return;
    int64_t now = app_now(app);
    if (app->tx_last_ms != 0 && now - app->tx_last_ms < TX_MIN_INTERVAL_MS) return;
    int idx = app->tx_queries[app->tx_next];
    app->tx_next = (app->tx_next + 1) % app->tx_nq;
    const vsp_signal_info_t *q = vsp_decoder_signal_info(app->decoder, idx);
    can_frame_t frames[TX_MAX_FRAMES];
    int64_t ts = now;
    int n = app->host.tx_query(app->host.ctx, q->tx_id, q->rx_id, q->request_bare_hex, frames, TX_MAX_FRAMES, &ts);
    app->tx_last_ms = app_now(app);
    app->tx_sent++;
    if (n < 0) {
        app->tx_errors++;
        return;
    }
    for (int i = 0; i < n; i++) {
        frames[i].timestamp_ms = ts;
        tui_model_frame(&app->model, &frames[i], ts);
        if (tui_recorder_is_open(&app->rec)) tui_recorder_write(&app->rec, &frames[i], ts);
        feed_decoded(app, &frames[i], ts, q->query_key, true);
    }
}

/*******************************************************************************
 * Discover
 ******************************************************************************/

static void disc_log_cb(const char *text, void *ctx) {
    tui_app_t *app = (tui_app_t *)ctx;
    tui_log_add(&app->model.log, app_now(app), text);
}

static void hint_reset(tui_app_t *app) {
    app->hint_valid = false;
    app->hint_eval_ms = 0;
    tui_signal_desc_t d = { .name = "hint", .unit = "", .group = "custom", .factor = 1.0 };
    tui_signal_init(&app->hint_sig, &d);
}

static bool disc_ensure_session(tui_app_t *app) {
    if (app->disc) return true;
    char dir[700];
    if (app->live) {
        snprintf(dir, sizeof(dir), "%s", app->session_dir);
    } else if (app->opt.headless) {
        snprintf(dir, sizeof(dir), "%s/snapshot_replay", app->captures_root);
    } else {
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        char stamp[32];
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
        snprintf(dir, sizeof(dir), "%s/%s_replay", app->captures_root, stamp);
    }
    disc_session_config_t cfg = {
        .dump_dir_override = dir,
        .device_prefix = app->live && app->device_prefix[0] ? app->device_prefix : NULL,
        .connected_name = app->live ? app->device_name : NULL,
        .monitor_cmd = app->live ? app->monitor_cmd : NULL,
        .log = disc_log_cb,
        .log_ctx = app,
    };
    app->disc = disc_session_create(&cfg);
    if (!app->disc) {
        app_log(app, "[ERROR] discover: out of memory");
        return false;
    }
    app->disc_phase = -1;
    app->disc_next = 0;
    app->disc_analyzing = false;
    for (int i = 0; i < DISC_NUM_PHASES; i++) app->disc_state[i] = TUI_PHASE_PENDING;
    app->have_result = false;
    return true;
}

static void disc_begin_analysis(tui_app_t *app) {
    app->disc_analyzing = true;
    app->disc_rendered = false;
}

static void disc_start_phase(tui_app_t *app) {
    int p = app->disc_next;
    if (!app->disc || p < 0 || p >= DISC_NUM_PHASES || app->disc_phase >= 0) return;
    if (app->replay && frame_source_exhausted(app->src)) {
        app_log(app, "[WARN] discover: the replay has ended, nothing to capture");
        return;
    }
    if (!disc_session_phase_begin(app->disc, p, app_now(app))) return;
    app->disc_phase = p;
    app->disc_start_ms = app_now(app);
    app->disc_state[p] = TUI_PHASE_ACTIVE;
    hint_reset(app);
    app_log(app, "discover %d/%d %s: capturing %d s", p + 1, DISC_NUM_PHASES, disc_session_phase_name(p),
            disc_session_phase_duration_s(p));
}

static void disc_end_phase(tui_app_t *app, bool source_open) {
    int p = app->disc_phase;
    if (!app->disc || p < 0) return;
    bool ok = disc_session_phase_end(app->disc, app_now(app), source_open);
    app->disc_phase = -1;
    app->hint_valid = false;
    if (!ok) return;
    app->disc_state[p] = TUI_PHASE_DONE;
    const disc_phase_dump_meta_t *meta = disc_session_phase_meta(app->disc, p);
    char count[32];
    tui_format_count(count, sizeof(count), meta ? meta->total_frames : 0);
    app_log(app, "discover %s: %s frames, %d IDs", disc_session_phase_name(p), count, meta ? meta->id_count : 0);
    if (p == DISC_PHASE_BASELINE && disc_session_have_bus_mode(app->disc)) {
        bool direct = disc_session_bus_mode(app->disc) == DISC_BUS_DIRECT;
        app_log(app, "discover baseline: bus %s", direct ? "DIRECT (PT-CAN)" : "GATEWAY (D-CAN)");
        if (direct) {
            tui_classifier_note_direct(&app->model.cls);   /* sticky TX lock (rule 0.3) */
            write_session_json(app, false);
        }
    }
    disc_session_save_phase_dump(app->disc, p);
    app->disc_next = p + 1;
    if (app->disc_next >= DISC_NUM_PHASES) disc_begin_analysis(app);
}

static void disc_skip(tui_app_t *app) {
    int p = app->disc_next;
    if (!app->disc || app->disc_phase >= 0 || app->disc_analyzing) return;
    if (p >= DISC_NUM_PHASES || !disc_session_phase_skippable(p)) return;
    disc_session_skip_phase(app->disc, p);
    app->disc_state[p] = TUI_PHASE_SKIPPED;
    app_log(app, "discover %s: skipped", disc_session_phase_name(p));
    app->disc_next = p + 1;
    if (app->disc_next >= DISC_NUM_PHASES) disc_begin_analysis(app);
}

static void disc_close_session(tui_app_t *app) {
    disc_session_free(app->disc);
    app->disc = NULL;
    app->disc_phase = -1;
    app->disc_analyzing = false;
    app->hint_valid = false;
}

static void disc_abort(tui_app_t *app, const char *status) {
    if (!app->disc) return;
    bool keep = strcmp(status, "disconnected") != 0;
    if (app->disc_phase >= 0) {
        int p = app->disc_phase;
        bool ok = disc_session_phase_end(app->disc, app_now(app), keep);
        app->disc_phase = -1;
        if (ok) {
            app->disc_state[p] = TUI_PHASE_DONE;
            disc_session_save_phase_dump(app->disc, p);
        }
    }
    disc_session_abort(app->disc, status);
    const char *dir = disc_session_dump_dir(app->disc);
    app_log(app, "[WARN] discover aborted (%s)%s%s", status, dir ? ", captured phases kept in " : "", dir ? dir : "");
    disc_close_session(app);
    if (app->overlay == OVL_DISCOVER) app->overlay = OVL_NONE;
}

static const disc_signal_t kResultOrder[DISC_SIG_COUNT] = {
    DISC_SIG_RPM, DISC_SIG_STEERING, DISC_SIG_THROTTLE, DISC_SIG_BRAKE, DISC_SIG_GEAR, DISC_SIG_WHEEL_SPEED,
};

static int phase_of_signal(disc_signal_t s) {
    switch (s) {
        case DISC_SIG_STEERING:    return DISC_PHASE_STEERING;
        case DISC_SIG_RPM:         return DISC_PHASE_THROTTLE;
        case DISC_SIG_THROTTLE:    return DISC_PHASE_THROTTLE;
        case DISC_SIG_BRAKE:       return DISC_PHASE_BRAKE;
        case DISC_SIG_GEAR:        return DISC_PHASE_GEAR;
        case DISC_SIG_WHEEL_SPEED: return DISC_PHASE_WHEEL_SPEED;
        default: break;
    }
    return DISC_PHASE_BASELINE;
}

static const char *result_signal_name(disc_signal_t s) {
    switch (s) {
        case DISC_SIG_STEERING:    return "steering";
        case DISC_SIG_RPM:         return "rpm";
        case DISC_SIG_THROTTLE:    return "throttle";
        case DISC_SIG_BRAKE:       return "brake";
        case DISC_SIG_GEAR:        return "gear";
        case DISC_SIG_WHEEL_SPEED: return "wheel";
        default: break;
    }
    return "?";
}

static const char *base_name(const char *path) {
    const char *b = strrchr(path, '/');
    return b ? b + 1 : path;
}

static void build_result(tui_app_t *app, const disc_session_output_t *out) {
    tui_result_view_t *v = &app->result;
    memset(v, 0, sizeof(*v));
    bool ascii = app->opt.ascii;
    snprintf(app->result_dir, sizeof(app->result_dir), "%s", out->dump_dir ? out->dump_dir : "(no session folder)");
    v->dir = app->result_dir;
    if (out->have_bus_mode) {
        snprintf(v->bus, sizeof(v->bus), "bus: %s", out->bus_mode == DISC_BUS_DIRECT ? "DIRECT" : "GATEWAY");
    }
    int found = 0;
    for (int k = 0; k < DISC_SIG_COUNT; k++) {
        disc_signal_t sig = kResultOrder[k];
        tui_result_row_t *row = &v->rows[v->count++];
        snprintf(row->signal, sizeof(row->signal), "%s", result_signal_name(sig));
        if (!out->result || !out->result->found[sig]) {
            int ph = phase_of_signal(sig);
            const disc_phase_dump_meta_t *meta = disc_session_phase_meta(app->disc, ph);
            snprintf(row->missing, sizeof(row->missing), "%s", meta && meta->skipped ? "skipped"
                     : (meta && meta->captured ? "not found" : "not captured"));
            continue;
        }
        found++;
        const disc_candidate_t *c = &out->result->signals[sig];
        row->found = true;
        tui_format_can_id(row->id, sizeof(row->id), c->can_id, c->can_id > 0x7FF);
        if (c->start_bit >= 0 && c->bit_length > 0) {
            if (c->sign_char == 'S') {
                snprintf(row->layout, sizeof(row->layout), "%d|%d@%d S%d", c->start_bit, c->bit_length,
                         c->byte_order, c->sign_bit);
            } else {
                snprintf(row->layout, sizeof(row->layout), "%d|%d@%d%c", c->start_bit, c->bit_length,
                         c->byte_order, c->sign_char == '-' ? '-' : '+');
            }
        } else if (c->byte2_idx >= 0) {
            snprintf(row->layout, sizeof(row->layout), "bytes %d,%d", c->byte_idx, c->byte2_idx);
        } else {
            snprintf(row->layout, sizeof(row->layout), "byte %d", c->byte_idx);
        }
        tui_format_hz(row->hz, sizeof(row->hz), c->hz, ascii);
        snprintf(row->conf, sizeof(row->conf), "%s", disc_confidence_name(c->confidence));
        snprintf(row->range, sizeof(row->range), "%d..%d", c->raw_min, c->raw_max);
        if (c->prior_support > 0) snprintf(row->prior, sizeof(row->prior), "%s%d", ascii ? "x" : "×", c->prior_support);
        else snprintf(row->prior, sizeof(row->prior), "%s", tui_glyph(TUI_G_DASH, ascii));
    }
    size_t n = (size_t)snprintf(v->written, sizeof(v->written), "written:");
    char sep[16];
    snprintf(sep, sizeof(sep), " %s ", tui_glyph(TUI_G_DOT, ascii));
    const char *files[3] = { out->dbc_path, out->vsp_path, out->result_path };
    int nf = 0;
    for (int i = 0; i < 3; i++) {
        if (!files[i] || n >= sizeof(v->written)) continue;
        n += (size_t)snprintf(v->written + n, sizeof(v->written) - n, "%s%s", nf ? sep : " ", base_name(files[i]));
        nf++;
    }
    if (nf == 0) snprintf(v->written, sizeof(v->written), "nothing written (no session folder)");
    v->can_load = out->vsp_path != NULL;
    snprintf(app->draft_vsp, sizeof(app->draft_vsp), "%s", out->vsp_path ? out->vsp_path : "");
    snprintf(app->result_json, sizeof(app->result_json), "%s", out->result_path ? out->result_path : "");
    app->have_result = true;
    app_log(app, "discover finished: %d of %d signals found", found, DISC_SIG_COUNT);
}

static void disc_finish(tui_app_t *app) {
    disc_session_output_t out;
    if (!disc_session_finish(app->disc, &out)) {
        app_log(app, "[ERROR] discover analysis failed");
        disc_close_session(app);
        app->overlay = OVL_NONE;
        return;
    }
    build_result(app, &out);
    disc_close_session(app);
    app->overlay = OVL_RESULT;
}

/* Profile signals that belong to a phase ("live" line). */
static bool signal_in_phase(const tui_signal_t *s, int phase) {
    switch (phase) {
        case DISC_PHASE_STEERING: return s->group == TUI_GROUP_STEERING;
        case DISC_PHASE_THROTTLE: return s->group == TUI_GROUP_THROTTLE || s->group == TUI_GROUP_RPM;
        case DISC_PHASE_BRAKE:    return s->group == TUI_GROUP_BRAKE;
        case DISC_PHASE_GEAR:     return s->group == TUI_GROUP_GEAR;
        case DISC_PHASE_WHEEL_SPEED:
            return s->group != TUI_GROUP_RPM && strcasestr(s->name, "speed") != NULL;
        default: break;
    }
    return false;
}

static const disc_id_t *find_id(const disc_phase_t *ph, uint32_t id) {
    for (int i = 0; i < ph->id_count; i++) {
        if (ph->ids[i].can_id == id) return &ph->ids[i];
    }
    return NULL;
}

/* The (id, byte) constant during the baseline with the most distinct values
   so far in the active phase. When no baseline-constant byte moved, the byte
   that gained the most distinct values over the baseline (at least 2), so a
   byte that was a little noisy at rest still shows. */
static void hint_evaluate(tui_app_t *app) {
    if (!app->disc || app->disc_phase <= DISC_PHASE_BASELINE) return;
    int64_t now = app_now(app);
    if (app->hint_eval_ms != 0 && now - app->hint_eval_ms < HINT_EVAL_MS) return;
    app->hint_eval_ms = now;
    const disc_phase_t *base = disc_session_phase(app->disc, DISC_PHASE_BASELINE);
    const disc_phase_t *cur = disc_session_phase(app->disc, app->disc_phase);
    if (!base || !cur || base->id_count == 0) return;
    int best = 1;
    uint32_t best_id = 0;
    int best_byte = -1;
    int gain_best = 1;
    uint32_t gain_id = 0;
    int gain_byte = -1;
    for (int i = 0; i < cur->id_count; i++) {
        const disc_id_t *c = &cur->ids[i];
        const disc_id_t *b = find_id(base, c->can_id);
        if (!b) continue;
        for (int k = 0; k < c->dlc && k < b->dlc && k < DISC_MAX_DLC; k++) {
            int u = c->bytes[k].unique_count;
            int gain = u - b->bytes[k].unique_count;
            if (gain > gain_best) {
                gain_best = gain;
                gain_id = c->can_id;
                gain_byte = k;
            }
            if (b->bytes[k].unique_count != 1) continue;
            if (u > best || (u == best && best_byte >= 0 && (c->can_id < best_id ||
                                                             (c->can_id == best_id && k < best_byte)))) {
                best = u;
                best_id = c->can_id;
                best_byte = k;
            }
        }
    }
    if (best_byte < 0) {
        best_id = gain_id;
        best_byte = gain_byte;
    }
    if (best_byte < 0) return;
    if (!app->hint_valid || best_id != app->hint_id || best_byte != app->hint_byte) {
        int64_t keep = app->hint_eval_ms;
        hint_reset(app);
        app->hint_eval_ms = keep;
        app->hint_valid = true;
        app->hint_id = best_id;
        app->hint_byte = best_byte;
    }
}

static void build_discover_view(tui_app_t *app, tui_discover_view_t *v) {
    memset(v, 0, sizeof(*v));
    bool ascii = app->opt.ascii;
    int64_t now = app_now(app);
    v->count = DISC_NUM_PHASES;
    bool capturing = app->disc && app->disc_phase >= 0;
    int next = app->disc ? app->disc_next : 0;
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        tui_discover_row_t *r = &v->rows[i];
        r->name = disc_session_phase_name(i);
        r->duration_s = disc_session_phase_duration_s(i);
        r->skippable = disc_session_phase_skippable(i);
        r->state = app->disc ? app->disc_state[i] : TUI_PHASE_PENDING;
        r->next = !capturing && !app->disc_analyzing && i == next;
        if (r->state == TUI_PHASE_ACTIVE) {
            r->elapsed_s = (double)(now - app->disc_start_ms) / 1000.0;
            r->frames = disc_session_phase_stored_frames(app->disc);
        } else if (r->state == TUI_PHASE_DONE) {
            const disc_phase_dump_meta_t *meta = disc_session_phase_meta(app->disc, i);
            r->frames = meta ? meta->total_frames : 0;
            r->ids = meta ? meta->id_count : 0;
            if (i == DISC_PHASE_BASELINE && disc_session_have_bus_mode(app->disc)) {
                r->note = disc_session_bus_mode(app->disc) == DISC_BUS_DIRECT ? "bus: DIRECT" : "bus: GATEWAY";
            }
        }
    }
    v->current = capturing ? app->disc_phase : (next < DISC_NUM_PHASES ? next : -1);
    if (v->current >= 0) {
        v->instruction = disc_session_phase_instruction(v->current);
        v->warning = disc_session_phase_warning(v->current);
    }
    v->session_active = app->disc != NULL;
    v->capturing = capturing;
    if (app->disc_analyzing) {
        v->status = ascii ? "analyzing..." : "analyzing…";
        return;
    }

    /* live line */
    app->live_text[0] = '\0';
    app->live_spark[0] = '\0';
    if (v->current == DISC_PHASE_BASELINE || v->current < 0) {
        char fps[32];
        tui_format_count(fps, sizeof(fps), (long long)(tui_model_frames_per_s(&app->model, now) + 0.5));
        snprintf(app->live_text, sizeof(app->live_text), "%s fr/s %s %d IDs", fps, tui_glyph(TUI_G_DOT, ascii),
                 app->model.raw.count);
    } else {
        size_t n = 0;
        int shown = 0;
        for (int i = 0; i < app->model.signal_count && shown < 3; i++) {
            const tui_signal_t *s = &app->model.signals[i];
            if (!signal_in_phase(s, v->current)) continue;
            char val[64];
            if (!s->sampled || s->value_null || !s->has_value) snprintf(val, sizeof(val), "%s", tui_glyph(TUI_G_DASH, ascii));
            else if (s->label) snprintf(val, sizeof(val), "%s", s->label);
            else tui_format_signal_value(val, sizeof(val), s->value, s->decimals);
            n += (size_t)snprintf(app->live_text + n, sizeof(app->live_text) - n, "%s%s %s", shown ? "   " : "", s->name, val);
            if (n >= sizeof(app->live_text)) n = sizeof(app->live_text) - 1;
            shown++;
        }
        if (capturing) {
            if (app->hint_valid) {
                char id[16];
                tui_format_can_id(id, sizeof(id), app->hint_id, app->hint_id > 0x7FF);
                snprintf(app->live_text + n, sizeof(app->live_text) - n, "%s%s b%d", shown ? "   " : "", id, app->hint_byte);
                tui_signal_sparkline(&app->hint_sig, now, 11, ascii, app->live_spark, sizeof(app->live_spark));
                v->live_spark = app->live_spark;
            } else {
                snprintf(app->live_text + n, sizeof(app->live_text) - n, "%sno byte has moved yet", shown ? "   " : "");
            }
        }
    }
    v->live_text = app->live_text[0] ? app->live_text : NULL;
}

static void open_discover(tui_app_t *app) {
    if (app->tx_on) {
        app_log(app, "discover unavailable in TX mode (x leaves it)");
        return;
    }
    if (app->disconnected) {
        app_log(app, "discover unavailable: disconnected");
        return;
    }
    app->overlay = OVL_DISCOVER;
}

static void discover_key(tui_app_t *app, int key) {
    bool capturing = app->disc && app->disc_phase >= 0;
    if (app->disc_analyzing) return;
    switch (key) {
        case TUI_KEY_ENTER:
        case '\r':
            if (capturing) break;
            if (!disc_ensure_session(app)) break;
            disc_start_phase(app);
            break;
        case 's':
            if (app->disc && !capturing) disc_skip(app);
            break;
        case 'a':
            if (app->disc) disc_abort(app, "interrupted");
            break;
        case TUI_KEY_ESC:
            if (!app->disc) app->overlay = OVL_NONE;
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * Result, JSON and help views
 ******************************************************************************/

static void free_text(tui_app_t *app) {
    for (int i = 0; i < app->text_count; i++) free(app->text_lines[i]);
    free(app->text_lines);
    app->text_lines = NULL;
    app->text_count = 0;
}

static void open_json(tui_app_t *app) {
    free_text(app);
    app->text_scroll = 0;
    FILE *f = app->result_json[0] ? fopen(app->result_json, "r") : NULL;
    if (!f) {
        app_log(app, "[WARN] result.json is not available");
        return;
    }
    char line[1024];
    int cap = 0;
    while (fgets(line, sizeof(line), f)) {
        trim_eol(line);
        for (char *p = line; *p; p++) if (*p == '\t') *p = ' ';
        if (app->text_count == cap) {
            int ncap = cap ? cap * 2 : 128;
            char **n = (char **)realloc(app->text_lines, (size_t)ncap * sizeof(char *));
            if (!n) break;
            app->text_lines = n;
            cap = ncap;
        }
        app->text_lines[app->text_count++] = strdup(line);
    }
    fclose(f);
    app->overlay_back = OVL_RESULT;
    app->overlay = OVL_JSON;
}

static const char *const kHelpLines[] = {
    "Dashboard",
    "  tab              focus the next panel (narrow screens: switch panel)",
    "  up down PgUp PgDn  scroll the focused panel",
    "  /                filter: signal name, Raw CAN id prefix (esc clears)",
    "  s                Raw CAN sort: hz, id, last change",
    "  d                discover: guided search for rpm, steering, throttle, ...",
    "  p                profile picker",
    "  r                recording on/off (live)",
    "  R                replay list (live, replay and the connect screen)",
    "  x                TX mode: poll the profile's OBD signals",
    "  space [ ]        replay: pause, slower, faster",
    "  v                last discover result",
    "  ?                this help",
    "  q                quit (asks while discover or TX mode runs); Ctrl+C quits now",
    "",
    "Discover",
    "  enter starts the next phase, s skips it, a aborts (captured phases are kept).",
    "  On the result screen enter loads the draft profile into the dashboard.",
    "",
    "Connect screen",
    "  enter connects the selected adapter, s rescans, R opens the replay list.",
    "",
    "Safety",
    "  The adapter listens in silent mode (ATCSM1): nothing is sent on the bus.",
    "  TX mode needs a GATEWAY or SILENT bus, a profile with OBD signals and a",
    "  y in the confirmation. A DIRECT (PT-CAN) bus locks TX until disconnect.",
};

static void open_help(tui_app_t *app) {
    if (app->overlay == OVL_HELP) return;
    app->overlay_back = app->overlay;
    app->overlay = OVL_HELP;
    app->text_scroll = 0;
}

static void text_scroll_key(tui_app_t *app, int key, int page) {
    switch (key) {
        case TUI_KEY_UP:   app->text_scroll--; break;
        case TUI_KEY_DOWN: app->text_scroll++; break;
        case TUI_KEY_PGUP: app->text_scroll -= page; break;
        case TUI_KEY_PGDN: app->text_scroll += page; break;
        default: break;
    }
    if (app->text_scroll < 0) app->text_scroll = 0;
}

/*******************************************************************************
 * Profile picker
 ******************************************************************************/

static void picker_refresh(tui_app_t *app) {
    if (!app->pick_idx) return;
    app->pick_n = tui_profiles_filter(&app->pix, app->pick_filter, app->pick_idx, app->pix.total);
    if (app->pick_want[0]) {
        for (int i = 0; i < app->pick_n; i++) {
            if (strcmp(app->pix.entries[app->pick_idx[i]].path, app->pick_want) == 0 ||
                strcmp(app->pix.entries[app->pick_idx[i]].file, base_name(app->pick_want)) == 0) {
                app->pick_sel = i;
                app->pick_want[0] = '\0';
                break;
            }
        }
        if (tui_profiles_done(&app->pix)) app->pick_want[0] = '\0';
    }
    int max = app->pick_n + app->pix.ndrafts - 1;
    if (app->pick_sel > max) app->pick_sel = max;
    if (app->pick_sel < 0) app->pick_sel = 0;
}

static void picker_open(tui_app_t *app, const char *preselect) {
    if (!app->pix_ready) {
        char dir[1024];
        bool found = tui_profiles_find_dir(app->profiles_dir[0] ? app->profiles_dir : NULL,
                                           app->exe_path, dir, sizeof(dir));
        tui_profiles_open(&app->pix, found ? dir : NULL, app->opt.ascii);
        app->pix_ready = true;
        app->pick_idx = (int *)calloc((size_t)app->pix.total + 1, sizeof(int));
        app->pick_rows = (tui_picker_row_t *)calloc((size_t)app->pix.total + 1, sizeof(tui_picker_row_t));
        if (!found) app_log(app, "[WARN] no profile folder found (--profiles DIR or VSP_PROFILES_DIR)");
        else app_log(app, "profile index: %d profile%s", app->pix.total, app->pix.total == 1 ? "" : "s");
    }
    tui_profiles_scan_drafts(&app->pix, app->captures_root);
    app->pick_filter[0] = '\0';
    app->pick_sel = 0;
    app->pick_scroll = 0;
    snprintf(app->pick_want, sizeof(app->pick_want), "%s", preselect ? preselect : "");
    app->overlay = OVL_PICKER;
    picker_refresh(app);
}

static void picker_step(tui_app_t *app) {
    if (!app->pix_ready || tui_profiles_done(&app->pix)) return;
    if (app->opt.headless) tui_profiles_step(&app->pix, 64, 0);
    else tui_profiles_step(&app->pix, 100000, 50.0);
    if (app->overlay == OVL_PICKER) picker_refresh(app);
}

static void picker_key(tui_app_t *app, int key) {
    size_t n = strlen(app->pick_filter);
    int total = app->pick_n + app->pix.ndrafts;
    int page = tui_picker_list_rows(app->cols, app->rows, app->pick_n, app->pix.ndrafts);
    switch (key) {
        case TUI_KEY_ESC:
            app->overlay = OVL_NONE;
            return;
        case TUI_KEY_CTRL_N:
            set_profile(app, NULL);
            app->overlay = OVL_NONE;
            return;
        case TUI_KEY_ENTER:
        case '\r':
            if (app->pick_sel < app->pick_n) {
                set_profile(app, app->pix.entries[app->pick_idx[app->pick_sel]].path);
                app->overlay = OVL_NONE;
            } else if (app->pick_sel - app->pick_n < app->pix.ndrafts) {
                set_profile(app, app->pix.drafts[app->pick_sel - app->pick_n].path);
                app->overlay = OVL_NONE;
            }
            return;
        case TUI_KEY_UP:   app->pick_sel--; break;
        case TUI_KEY_DOWN: app->pick_sel++; break;
        case TUI_KEY_PGUP: app->pick_sel -= page; break;
        case TUI_KEY_PGDN: app->pick_sel += page; break;
        case TUI_KEY_BACKSPACE:
        case 8:
            if (n > 0) app->pick_filter[n - 1] = '\0';
            app->pick_sel = 0;
            app->pick_want[0] = '\0';
            break;
        default:
            if (key >= 32 && key < 127 && n + 1 < sizeof(app->pick_filter)) {
                app->pick_filter[n] = (char)key;
                app->pick_filter[n + 1] = '\0';
                app->pick_sel = 0;
                app->pick_want[0] = '\0';
            }
            break;
    }
    if (app->pick_sel >= total) app->pick_sel = total - 1;
    if (app->pick_sel < 0) app->pick_sel = 0;
    picker_refresh(app);
}

static void draw_picker(tui_app_t *app, tui_grid_t *g) {
    for (int i = 0; i < app->pick_n; i++) {
        const tui_profile_entry_t *e = &app->pix.entries[app->pick_idx[i]];
        app->pick_rows[i] = (tui_picker_row_t){
            .make = e->make, .model = e->model, .gen = e->gen, .years = e->years, .file = e->file,
            .signals = e->signals, .can = e->can, .obd = e->obd,
            .current = app->profile_path[0] && strcmp(e->path, app->profile_path) == 0,
        };
    }
    tui_picker_row_t drafts[5];
    for (int i = 0; i < app->pix.ndrafts; i++) {
        const tui_profile_entry_t *e = &app->pix.drafts[i];
        drafts[i] = (tui_picker_row_t){ .file = e->file, .signals = e->signals, .can = e->can, .obd = e->obd };
    }
    tui_picker_view_t pv = {
        .filter = app->pick_filter,
        .total = app->pix.total,
        .indexed = app->pix.indexed,
        .indexing = !tui_profiles_done(&app->pix),
        .dir = app->pix.have_dir ? app->pix.dir : NULL,
        .rows = app->pick_rows,
        .nrows = app->pick_n,
        .drafts = drafts,
        .ndrafts = app->pix.ndrafts,
        .selected = app->pick_sel,
        .scroll = &app->pick_scroll,
        .obd_dim = app->model.cls.direct_seen || app->model.cls.bus == TUI_BUS_DIRECT,
    };
    tui_draw_picker(g, app->cols, app->rows, &pv, app->opt.ascii);
}

/*******************************************************************************
 * Replay list
 ******************************************************************************/

static void replay_list_open(tui_app_t *app) {
    tui_replay_list_free(&app->rlist);
    char err[256] = "";
    if (tui_replay_list_scan(&app->rlist, app->captures_root, err, sizeof(err)) != 0) {
        app_log(app, "[WARN] %s", err);
    }
    app->rlist_sel = 0;
    app->rlist_scroll = 0;
    app->overlay = OVL_REPLAY;
}

static void request_quit(tui_app_t *app);

static void replay_list_key(tui_app_t *app, int key) {
    int page = tui_replay_list_rows(app->cols, app->rows);
    switch (key) {
        case TUI_KEY_ESC:
            app->overlay = OVL_NONE;
            return;
        case TUI_KEY_ENTER:
        case '\r':
            if (app->rlist_sel >= 0 && app->rlist_sel < app->rlist.count) {
                const tui_replay_entry_t *e = &app->rlist.entries[app->rlist_sel];
                char err[256];
                if (!tui_replay_has_frames(e->path, err, sizeof(err))) {
                    app_log(app, "[WARN] cannot replay %s: %s", e->name, err);   /* deleted or emptied since the scan */
                    return;
                }
                snprintf(app->replay_request, sizeof(app->replay_request), "%s", e->path);
                request_quit(app);   /* asks first while discover or TX mode runs */
            }
            return;
        case TUI_KEY_UP:   app->rlist_sel--; break;
        case TUI_KEY_DOWN: app->rlist_sel++; break;
        case TUI_KEY_PGUP: app->rlist_sel -= page; break;
        case TUI_KEY_PGDN: app->rlist_sel += page; break;
        default: break;
    }
    if (app->rlist_sel >= app->rlist.count) app->rlist_sel = app->rlist.count - 1;
    if (app->rlist_sel < 0) app->rlist_sel = 0;
}

static void draw_replay_list(tui_app_t *app, tui_grid_t *g) {
    tui_replay_view_t rv = {
        .rows = app->rlist.entries,
        .nrows = app->rlist.count,
        .selected = app->rlist_sel,
        .scroll = &app->rlist_scroll,
        .root = app->captures_root,
    };
    tui_draw_replay_list(g, app->cols, app->rows, &rv, app->opt.ascii);
}

/*******************************************************************************
 * Lifecycle
 ******************************************************************************/

static tui_app_t *app_alloc(const tui_app_options_t *opt, frame_source_t *src, int cols, int rows) {
    tui_app_t *app = (tui_app_t *)calloc(1, sizeof(*app));
    if (!app) return NULL;
    app->opt = *opt;
    snprintf(app->profiles_dir, sizeof(app->profiles_dir), "%s", opt->profiles_dir ? opt->profiles_dir : "");
    snprintf(app->exe_path, sizeof(app->exe_path), "%s", opt->exe_path ? opt->exe_path : "");
    snprintf(app->captures_root, sizeof(app->captures_root), "%s",
             opt->captures_root && opt->captures_root[0] ? opt->captures_root : "captures");
    snprintf(app->state_dir, sizeof(app->state_dir), "%s", opt->state_dir ? opt->state_dir : "");
    app->opt.profiles_dir = NULL;
    app->opt.exe_path = NULL;
    app->opt.captures_root = NULL;
    app->opt.state_dir = NULL;
    app->opt.profile_path = NULL;
    app->src = src;
    app->cols = cols;
    app->rows = rows;
    app->disc_phase = -1;
    app->view.sort = TUI_SORT_HZ;
    tui_gauges_init(&app->gauges);
    hint_reset(app);
    return app;
}

static void initial_view(tui_app_t *app) {
    app->view.tab = app->model.signal_count > 0 ? TUI_PANEL_SIGNALS : TUI_PANEL_RAW;
    app->view.focus = app->view.tab;
    if (app->model.signal_count > 0 && app->gauges.bound > 0) app->view.tab = TUI_PANEL_GAUGES;
}

static bool has_gauges(const tui_app_t *app) {
    return app->model.signal_count > 0 && app->gauges.bound > 0;
}

tui_app_t *tui_app_create_replay(const tui_app_options_t *opt, frame_source_t *src,
                                 const char *source_name, int cols, int rows,
                                 char *err, size_t err_len) {
    tui_app_t *app = app_alloc(opt, src, cols, rows);
    if (!app) {
        snprintf(err, err_len, "out of memory");
        return NULL;
    }
    app->replay = true;
    snprintf(app->source_name, sizeof(app->source_name), "%s", source_name ? source_name : "replay");
    int delivered = 0, total = 0;
    frame_source_replay_range(src, &app->replay_first_ms, &app->replay_last_ms, &delivered, &total);
    tui_model_init(&app->model, frame_source_now_ms(src));
    if (opt->speed > 0) frame_source_replay_set_speed(src, opt->speed);

    char len[16];
    tui_format_mmss(len, sizeof(len), app->replay_last_ms - app->replay_first_ms);
    app_log(app, "replay %s: %d frames, %s", app->source_name, total, len);

    if (opt->profile_path && opt->profile_path[0] && strcmp(opt->profile_path, "none") != 0) {
        if (!load_profile(app, opt->profile_path, err, err_len)) {
            tui_app_free(app);
            return NULL;
        }
    }
    initial_view(app);
    return app;
}

tui_app_t *tui_app_create_live(const tui_app_options_t *opt, frame_source_t *src,
                               const tui_host_t *host, const char *session_dir,
                               const char *const *preface, int npreface,
                               int cols, int rows, char *err, size_t err_len) {
    tui_app_t *app = app_alloc(opt, src, cols, rows);
    if (!app) {
        snprintf(err, err_len, "out of memory");
        return NULL;
    }
    app->live = true;
    app->host = *host;
    snprintf(app->device_name, sizeof(app->device_name), "%s", host->device_name ? host->device_name : "adapter");
    snprintf(app->device_prefix, sizeof(app->device_prefix), "%s", host->device_prefix ? host->device_prefix : "");
    snprintf(app->transport, sizeof(app->transport), "%s", host->transport ? host->transport : "");
    snprintf(app->stn, sizeof(app->stn), "%s", host->stn ? host->stn : "");
    snprintf(app->monitor_cmd, sizeof(app->monitor_cmd), "%s", host->monitor_cmd ? host->monitor_cmd : "ATMA");
    app->host.device_name = app->device_name;
    app->host.device_prefix = app->device_prefix;
    app->host.transport = app->transport;
    app->host.stn = app->stn[0] ? app->stn : NULL;
    app->host.monitor_cmd = app->monitor_cmd;
    snprintf(app->source_name, sizeof(app->source_name), "%s", app->device_name);
    snprintf(app->session_dir, sizeof(app->session_dir), "%s", session_dir ? session_dir : "");
    wall_stamp(app->started_at, sizeof(app->started_at));
    tui_model_init(&app->model, frame_source_now_ms(src));
    tui_recorder_init(&app->rec, app->session_dir);
    for (int i = 0; i < npreface; i++) tui_app_log(app, preface[i]);

    if (opt->profile_path && opt->profile_path[0] && strcmp(opt->profile_path, "none") != 0) {
        if (!load_profile(app, opt->profile_path, err, err_len)) {
            tui_app_free(app);
            return NULL;
        }
        save_last(app);
    } else if (!opt->profile_path) {
        char dev[128], prof[1024], date[32];
        tui_last_read(app->state_dir, dev, sizeof(dev), prof, sizeof(prof), date, sizeof(date));
        picker_open(app, strcmp(prof, "none") == 0 ? NULL : prof);
    } else {
        save_last(app);
    }
    initial_view(app);
    write_session_json(app, false);
    return app;
}

void tui_app_free(tui_app_t *app) {
    if (!app) return;
    tui_recorder_close(&app->rec);
    disc_session_free(app->disc);
    tui_gauges_free(&app->gauges);
    tui_model_free(&app->model);
    vsp_decoder_free(app->decoder);
    vsp_profile_free(app->profile);
    free(app->updated);
    free(app->tx_queries);
    free(app->pick_idx);
    free(app->pick_rows);
    tui_replay_list_free(&app->rlist);
    if (app->pix_ready) tui_profiles_free(&app->pix);
    free_text(app);
    free(app);
}

void tui_app_on_frame(const can_frame_t *frame, void *ctx) {
    tui_app_t *app = (tui_app_t *)ctx;
    int64_t ts = frame->timestamp_ms;
    tui_model_frame(&app->model, frame, ts);
    if (tui_recorder_is_open(&app->rec)) tui_recorder_write(&app->rec, frame, ts);
    if (app->disc && app->disc_phase >= 0) disc_session_phase_feed(app->disc, frame);
    if (app->hint_valid && frame->can_id == app->hint_id && frame->dlc > app->hint_byte) {
        tui_signal_sample(&app->hint_sig, ts, true, false, (double)frame->data[app->hint_byte], NULL);
    }
    feed_decoded(app, frame, ts, NULL, false);
}

bool tui_app_start(tui_app_t *app) {
    tui_classifier_start(&app->model.cls, frame_source_now_ms(app->src));
    bool ok = frame_source_start(app->src, tui_app_on_frame, app);
    app->monitor_on = ok;
    if (app->live && !app->opt.no_rec) rec_start(app);
    else if (app->live) app_log(app, "rec off (--no-rec): r starts a recording");
    return ok;
}

void tui_app_log(tui_app_t *app, const char *text) {
    tui_log_add(&app->model.log, app_now(app), text);
}

static void handle_disconnect(tui_app_t *app) {
    if (app->disconnected) return;
    app->disconnected = true;
    app->monitor_on = false;
    app->tx_on = false;            /* nothing can be sent any more */
    app->model.cls.paused = true;
    app_log(app, "[ERROR] adapter disconnected: c returns to the connect screen");
    rec_stop(app);
    disc_abort(app, "disconnected");
    if (app->overlay == OVL_TX_CONFIRM) app->overlay = OVL_NONE;
    write_session_json(app, false);
}

void tui_app_tick(tui_app_t *app) {
    int64_t now = app_now(app);
    if (app->live && !app->disconnected && app->host.is_connected && !app->host.is_connected(app->host.ctx)) {
        handle_disconnect(app);
    }
    tui_classifier_t *c = &app->model.cls;
    if (tui_classifier_tick(c, now)) {
        if (c->bus == TUI_BUS_SILENT) {
            app_log(app, "bus mode SILENT (no frames in %d s)", TUI_CLS_WINDOW_MS / 1000);
        } else {
            app_log(app, "bus mode %s (%d IDs %s 20 Hz)", c->bus == TUI_BUS_DIRECT ? "DIRECT" : "GATEWAY",
                    c->last_fast_ids, app->opt.ascii ? ">=" : "≥");
        }
        if (app->ncls < MAX_CLS_CHANGES) {
            app->cls_changes[app->ncls].t_s = (double)(now - app->model.origin_ms) / 1000.0;
            app->cls_changes[app->ncls].bus = c->bus;
            app->ncls++;
        }
        write_session_json(app, false);
    }
    if (app->replay && !app->end_logged && frame_source_exhausted(app->src)) {
        app->end_logged = true;
        tui_log_add(&app->model.log, now, "replay end");
    }

    if (app->disc && app->disc_phase >= 0) {
        int p = app->disc_phase;
        int64_t dur = (int64_t)disc_session_phase_duration_s(p) * 1000;
        if (now - app->disc_start_ms >= dur) disc_end_phase(app, true);
        else if (app->replay && frame_source_exhausted(app->src)) disc_end_phase(app, true);
        else hint_evaluate(app);
    }
    if (app->disc && app->disc_analyzing && (app->opt.headless || app->disc_rendered)) disc_finish(app);

    tui_gauges_update(&app->gauges, &app->model, now);
    picker_step(app);
    tui_recorder_tick(&app->rec, now);
    tx_poll(app);
}

/*******************************************************************************
 * Keys
 ******************************************************************************/

static void current_layout(const tui_app_t *app, tui_layout_t *L) {
    tui_layout_compute(L, app->cols, app->rows, app->model.signal_count > 0,
                       app->model.signal_count, has_gauges(app), app->view.tab);
}

/* Tabs mode cycles Gauges too; in the split layout the gauges take no focus. */
static tui_panel_t next_panel(const tui_app_t *app, tui_panel_t p, bool tabs) {
    for (int i = 1; i <= TUI_PANEL_COUNT; i++) {
        tui_panel_t q = (tui_panel_t)(((int)p + i) % TUI_PANEL_COUNT);
        if (q == TUI_PANEL_SIGNALS && app->model.signal_count == 0) continue;
        if (q == TUI_PANEL_GAUGES && (!tabs || !has_gauges(app))) continue;
        return q;
    }
    return p;
}

static void request_quit(tui_app_t *app) {
    if (app->disc || app->tx_on) {
        app->overlay_back = app->overlay;
        app->overlay = OVL_QUIT_CONFIRM;
    } else {
        app->quit = true;
    }
}

static void dashboard_key(tui_app_t *app, int key) {
    tui_view_t *v = &app->view;
    if (app->filter_editing) {
        size_t n = strlen(app->filter);
        if (key == TUI_KEY_ESC) {
            app->filter[0] = '\0';
            app->filter_editing = false;
        } else if (key == TUI_KEY_ENTER || key == '\r') {
            app->filter_editing = false;
        } else if (key == TUI_KEY_BACKSPACE || key == 8) {
            if (n > 0) app->filter[n - 1] = '\0';
        } else if (key >= 32 && key < 127 && n + 1 < sizeof(app->filter)) {
            app->filter[n] = (char)key;
            app->filter[n + 1] = '\0';
        }
        for (int i = 0; i < TUI_PANEL_COUNT; i++) v->scroll[i] = 0;
        return;
    }

    tui_layout_t L;
    current_layout(app, &L);
    int page = tui_layout_content_rows(&L, v->focus);
    if (page < 1) page = 1;

    switch (key) {
        case TUI_KEY_TAB:
            if (L.narrow) {
                v->tab = next_panel(app, v->tab, true);
                v->focus = v->tab;
            } else {
                v->focus = next_panel(app, v->focus == TUI_PANEL_GAUGES ? TUI_PANEL_LOG : v->focus, false);
            }
            break;
        case TUI_KEY_UP:   v->scroll[v->focus] -= 1; break;
        case TUI_KEY_DOWN: v->scroll[v->focus] += 1; break;
        case TUI_KEY_PGUP: v->scroll[v->focus] -= page; break;
        case TUI_KEY_PGDN: v->scroll[v->focus] += page; break;
        case '/':
            app->filter_editing = true;
            break;
        case TUI_KEY_ESC:
            app->filter[0] = '\0';
            break;
        case 's':
            v->sort = (tui_sort_t)(((int)v->sort + 1) % TUI_SORT_COUNT);
            break;
        case ' ':
            if (app->replay) {
                bool paused = !frame_source_replay_paused(app->src);
                frame_source_replay_set_paused(app->src, paused);
                app_log(app, "%s", paused ? "replay paused" : "replay resumed");
            }
            break;
        case '[':
        case ']':
            if (app->replay) {
                double sp = frame_source_replay_speed(app->src) * (key == ']' ? 2.0 : 0.5);
                frame_source_replay_set_speed(app->src, sp);
                app_log(app, "replay speed %gx", frame_source_replay_speed(app->src));
            }
            break;
        case 'd':
            open_discover(app);
            break;
        case 'p':
            picker_open(app, app->profile_path[0] ? app->profile_path : NULL);
            break;
        case 'r':
            if (app->replay) app_log(app, "recording unavailable in replay");
            else if (app->disconnected) app_log(app, "recording unavailable: disconnected");
            else if (tui_recorder_is_open(&app->rec)) rec_stop(app);
            else rec_start(app);
            break;
        case 'R':
            replay_list_open(app);
            break;
        case 'x':
            if (app->tx_on) {
                tx_leave(app, true);
            } else {
                const char *reason = tui_app_tx_lock_reason(app);
                if (reason) app_log(app, "%s", reason);
                else app->overlay = OVL_TX_CONFIRM;
            }
            break;
        case 'c':
            if (app->disconnected) app->want_connect = true;
            break;
        case 'v':
            if (app->have_result) app->overlay = OVL_RESULT;
            break;
        case '?':
            open_help(app);
            break;
        case 'q':
            request_quit(app);
            break;
        default:
            break;
    }
    if (v->scroll[v->focus] < 0) v->scroll[v->focus] = 0;
}

void tui_app_key(tui_app_t *app, int key) {
    switch (app->overlay) {
        case OVL_NONE:
            dashboard_key(app, key);
            return;
        case OVL_PICKER:
            picker_key(app, key);
            return;
        case OVL_REPLAY:
            replay_list_key(app, key);
            return;
        case OVL_DISCOVER:
            if (key == '?') open_help(app);
            else if (key == 'q') request_quit(app);
            else discover_key(app, key);
            return;
        case OVL_RESULT:
            if ((key == TUI_KEY_ENTER || key == '\r') && app->result.can_load) {
                set_profile(app, app->draft_vsp);
                app->overlay = OVL_NONE;
            } else if (key == 'v') {
                open_json(app);
            } else if (key == TUI_KEY_ESC) {
                app->overlay = OVL_NONE;
            } else if (key == 'q') {
                request_quit(app);
            }
            return;
        case OVL_JSON:
        case OVL_HELP:
            if (key == TUI_KEY_ESC || (app->overlay == OVL_HELP && key == '?')) {
                app->overlay = app->overlay_back;
                app->text_scroll = 0;
            } else {
                text_scroll_key(app, key, app->rows > 12 ? app->rows - 10 : 1);
            }
            return;
        case OVL_TX_CONFIRM:
            app->overlay = OVL_NONE;
            if (key == 'y' || key == 'Y') tx_enter(app);
            else app_log(app, "TX cancelled");
            return;
        case OVL_QUIT_CONFIRM:
            if (key == 'y' || key == 'Y') {
                app->quit = true;
            } else {
                app->overlay = app->overlay_back;
                app->replay_request[0] = '\0';   /* back to the replay list: nothing picked */
            }
            return;
    }
}

void tui_app_resize(tui_app_t *app, int cols, int rows) {
    app->cols = cols;
    app->rows = rows;
}

bool tui_app_quit_requested(const tui_app_t *app) {
    return app->quit;
}

const char *tui_app_replay_request(const tui_app_t *app) {
    return app->replay_request[0] ? app->replay_request : NULL;
}

bool tui_app_wants_connect_screen(const tui_app_t *app) {
    return app->want_connect;
}

void tui_app_shutdown(tui_app_t *app) {
    if (app->tx_on) tx_leave(app, false);   /* PASSIVE re-init before the disconnect (rule 0.6) */
    if (app->disc) disc_abort(app, app->disconnected ? "disconnected" : "interrupted");
    rec_stop(app);
    write_session_json(app, true);
}

/*******************************************************************************
 * Render
 ******************************************************************************/

static const char *tx_footer(const tui_app_t *app) {
    if (!app->live) return NULL;
    if (app->tx_on) return "TX off";
    if (app->opt.no_tx) return "TX [disabled]";
    if (app->disconnected) return "TX [disconnected]";
    if (app->model.cls.direct_seen || app->model.cls.bus == TUI_BUS_DIRECT) return "TX mode [locked: DIRECT]";
    if (app->model.cls.bus == TUI_BUS_DETECTING) return "TX mode [locked: detecting]";
    if (app->diag_count == 0) return "TX mode [no OBD signals]";
    return "TX mode";
}

static const char *tx_hint(const tui_app_t *app) {
    if (!app->live || app->opt.no_tx) return "TX disabled";
    if (app->tx_on) return app->opt.ascii ? "polling..." : "polling…";
    if (app->model.cls.direct_seen || app->model.cls.bus == TUI_BUS_DIRECT) return "TX locked (DIRECT)";
    if (app->model.cls.bus == TUI_BUS_DETECTING) return "TX locked (detecting)";
    if (app->disconnected) return "TX unavailable";
    return "x to poll";
}

static void draw_overlay(tui_app_t *app, tui_grid_t *g) {
    bool ascii = app->opt.ascii;
    switch (app->overlay) {
        case OVL_NONE:
            return;
        case OVL_PICKER:
            draw_picker(app, g);
            return;
        case OVL_REPLAY:
            draw_replay_list(app, g);
            return;
        case OVL_DISCOVER: {
            tui_discover_view_t dv;
            build_discover_view(app, &dv);
            tui_draw_discover(g, app->cols, app->rows, &dv, ascii);
            return;
        }
        case OVL_RESULT:
            tui_draw_result(g, app->cols, app->rows, &app->result, ascii);
            return;
        case OVL_JSON: {
            tui_hint_t hints[2] = { { tui_glyph(TUI_G_UPDOWN, ascii), "scroll", 0 }, { "esc", "back", 0 } };
            tui_draw_text_view(g, app->cols, app->rows, "result.json", app->result_dir,
                               (const char *const *)app->text_lines, app->text_count, &app->text_scroll, 90,
                               hints, 2, ascii);
            return;
        }
        case OVL_HELP: {
            tui_hint_t hints[2] = { { tui_glyph(TUI_G_UPDOWN, ascii), "scroll", 0 }, { "esc", "back", 0 } };
            tui_draw_text_view(g, app->cols, app->rows, "Help", NULL, kHelpLines,
                               (int)(sizeof(kHelpLines) / sizeof(kHelpLines[0])), &app->text_scroll, 84,
                               hints, 2, ascii);
            return;
        }
        case OVL_TX_CONFIRM: {
            const char *gw[] = { "Send OBD requests on the bus?", "Bus: GATEWAY (OBD port). Continue? [y/N]" };
            const char *silent[] = { "No frames seen. Only continue on the car's OBD port,",
                                     "never on a PT-CAN tap. [y/N]" };
            bool is_silent = app->model.cls.bus == TUI_BUS_SILENT;
            tui_hint_t hints[2] = { { "y", "send", 0 }, { "any key", "cancel", 0 } };
            tui_draw_confirm(g, app->cols, app->rows, "TX mode", is_silent ? silent : gw, 2, TUI_ROLE_WARN,
                             hints, 2, ascii);
            return;
        }
        case OVL_QUIT_CONFIRM: {
            const char *lines[1] = { app->tx_on ? "TX mode is on. Quit? [y/N]" : "A discover session is running. Quit? [y/N]" };
            tui_hint_t hints[2] = { { "y", "quit", 0 }, { "any key", "back", 0 } };
            char line[400];
            if (app->replay_request[0]) {
                snprintf(line, sizeof(line), "%s and replay %s? [y/N]",
                         app->tx_on ? "TX mode is on. Leave TX mode" : "A discover session is running. Abort it",
                         base_name(app->replay_request));
                lines[0] = line;
                hints[0].desc = "replay";
            }
            tui_draw_confirm(g, app->cols, app->rows, app->replay_request[0] ? "Replay" : "Quit", lines, 1,
                             TUI_ROLE_DEFAULT, hints, 2, ascii);
            return;
        }
    }
}

void tui_app_render(tui_app_t *app, tui_grid_t *grid) {
    if (grid->cols != app->cols || grid->rows != app->rows) tui_grid_resize(grid, app->cols, app->rows);
    tui_view_t *v = &app->view;
    int64_t now = app_now(app);
    v->cols = app->cols;
    v->rows = app->rows;
    v->ascii = app->opt.ascii;
    v->now_ms = now;
    v->device = app->live ? app->device_name : app->source_name;
    v->live = app->live && !app->disconnected;
    v->tx_on = app->tx_on;
    v->disconnected = app->disconnected;
    v->replay = app->replay;
    if (app->replay) {
        v->speed = frame_source_replay_speed(app->src);
        v->paused = frame_source_replay_paused(app->src);
        v->ended = frame_source_exhausted(app->src);
        v->replay_pos_ms = now - app->replay_first_ms;
        v->replay_total_ms = app->replay_last_ms - app->replay_first_ms;
        if (v->replay_pos_ms > v->replay_total_ms) v->replay_pos_ms = v->replay_total_ms;
    }
    v->recording = tui_recorder_is_open(&app->rec);
    v->profile = app->profile_short[0] ? app->profile_short : NULL;
    v->filter = app->filter;
    v->filter_editing = app->filter_editing;
    v->tx_hint = tx_hint(app);
    v->rec_available = app->live && !app->disconnected;
    v->tx_footer = tx_footer(app);
    v->monitor_paused = app->tx_on;
    v->gauges = has_gauges(app) ? &app->gauges : NULL;
    v->colors = app->opt.colors;
    if (v->tab == TUI_PANEL_GAUGES && !v->gauges) v->tab = TUI_PANEL_SIGNALS;
    if (v->tab == TUI_PANEL_SIGNALS && app->model.signal_count == 0) v->tab = TUI_PANEL_RAW;
    if (v->focus == TUI_PANEL_GAUGES && !v->gauges) v->focus = TUI_PANEL_SIGNALS;
    if (v->focus == TUI_PANEL_SIGNALS && app->model.signal_count == 0) v->focus = TUI_PANEL_RAW;
    tui_render_dashboard(grid, &app->model, v);
    if (app->overlay != OVL_NONE) {
        tui_dim_grid(grid);
        draw_overlay(app, grid);
    }
    tui_grid_map_colors(grid, app->opt.colors);
    if (app->disc_analyzing) app->disc_rendered = true;
}
