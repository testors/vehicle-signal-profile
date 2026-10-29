/**
 * @file tui_live.m
 * @brief Live TUI: connect screen, PASSIVE connect, dashboard loop, TX host
 *        (Objective-C, compiled without ARC)
 */

#import <Foundation/Foundation.h>

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>

#import "obd_adapter.h"
#import "obd_connect.h"
#import "obd_poll.h"
#import "frame_source_live.h"
#include "frame_source.h"
#include "frame_source_pcan.h"
#include "pcan.h"
#include "tui_grid.h"
#include "tui_live.h"
#include "tui_replay.h"
#include "tui_screens.h"
#include "tui_term.h"

#define SCAN_RESTART_S   8.0
#define CONNECT_TIMEOUT_S 10.0
#define MAX_DEVICES      64
#define PENDING_LINES    64
#define TX_TIMEOUT_S     1.0
#define PCAN_POLL_S      1.0     /* hot-plug: re-read the PCAN channel conditions */

typedef enum {
    ST_CONNECT = 0,
    ST_CONNECTING,
    ST_DASHBOARD,
} live_state_t;

typedef struct {
    char                name[128];
    OBDAdapterTransport transport;
    int                 rssi;
} live_dev_t;

typedef struct {
    const tui_live_options_t *o;
    OBDAdapter     *adapter;
    obd_link_t      link;
    live_state_t    state;
    bool            quit;
    int             rc;

    /* connect screen */
    live_dev_t      devs[MAX_DEVICES];
    int             ndev;
    int             selected;
    char            target[128];       /* name to connect once it shows up */
    bool            target_exact;      /* false: --device prefix (contains) */
    double          scan_started;
    char            status[256];
    char            status_role;
    char            last_dev[128];
    char            last_prof[1024];
    char            last_date[32];
    char            last_text[400];
    bool            prompting;
    char            prompt[512];
    char           *replay_out;
    size_t          replay_len;
    tui_replay_list_t rlist;
    bool            rlist_open;
    int             rlist_sel;
    int             rlist_scroll;
    double          connect_started;
    int             connect_index;
    char            connect_name[128];

    /* PCAN-USB (rows before the Bluetooth ones on the connect screen) */
    const pcan_api_t   *pcan_api;          /* NULL: library missing, no PCAN rows */
    char                pcan_err[768];
    pcan_channel_info_t pcan[PCAN_MAX_CHANNELS];
    int                 npcan;
    double              pcan_polled;
    bool                target_pcan;       /* target is a PCAN channel */
    int                 target_pcan_index; /* 0 = the first available one */
    bool                target_busy_noted;
    bool                pcan_mode;         /* connecting to / connected over PCAN */
    int                 pcan_index;
    bool                autobaud_on;
    pcan_autobaud_t     autobaud;
    char                mode_text[48];     /* "listen-only 500k" */

    /* dashboard */
    frame_source_t *src;
    tui_app_t      *app;
    char            session_dir[600];
    char            stn_text[64];
    char            transport_text[16];
    char            monitor_cmd[32];

    /* captured stderr before the app exists */
    char            pending[PENDING_LINES][200];
    int             npending;

    /* drawing */
    tui_grid_t     *grid;
    int             cols, rows;
    double          start_wall;
    double          last_draw;
    int             next_snap;
    int             next_key;
} live_t;

static double wall_s(void) {
    return (double)frame_source_wall_ms() / 1000.0;
}

/*******************************************************************************
 * TX host (the only transmitting calls; reached through tui_app's TX guard)
 ******************************************************************************/

static bool host_is_connected(void *ctx) {
    return ((live_t *)ctx)->adapter.isConnected;
}

static bool host_tx_setup(void *ctx) {
    return obd_poll_setup(((live_t *)ctx)->adapter);
}

static int host_tx_query(void *ctx, uint32_t tx_id, uint32_t rx_id, const char *request_bare_hex,
                         can_frame_t *frames, int max_frames, int64_t *ts_ms) {
    live_t *L = (live_t *)ctx;
    int n = obd_poll_query(L->adapter, tx_id, rx_id, request_bare_hex, TX_TIMEOUT_S, frames, max_frames);
    *ts_ms = frame_source_wall_ms();
    return n;
}

static bool host_passive_init(void *ctx) {
    live_t *L = (live_t *)ctx;
    obd_init_config_t cfg = { .profile = OBD_INIT_PASSIVE, .filters = NULL, .filter_count = 0 };
    return obd_init_adapter(L->adapter, &cfg, &L->link);
}

/* PCAN host: connected while the channel answers; no TX callbacks exist. */
static bool host_pcan_is_connected(void *ctx) {
    live_t *L = (live_t *)ctx;
    return L->src && frame_source_is_open(L->src);
}

/*******************************************************************************
 * Captured stderr
 ******************************************************************************/

static void capture_line(const char *line, void *ctx) {
    live_t *L = (live_t *)ctx;
    if (L->app) {
        tui_app_log(L->app, line);
        return;
    }
    if (L->npending == PENDING_LINES) {
        memmove(L->pending[0], L->pending[1], sizeof(L->pending[0]) * (PENDING_LINES - 1));
        L->npending--;
    }
    snprintf(L->pending[L->npending++], sizeof(L->pending[0]), "%s", line);
}

static void poll_capture(live_t *L) {
    if (L->o->capture) tui_capture_poll(L->o->capture, capture_line, L);
}

/*******************************************************************************
 * Connect screen
 ******************************************************************************/

static void set_status(live_t *L, char role, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void set_status(live_t *L, char role, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(L->status, sizeof(L->status), fmt, ap);
    va_end(ap);
    L->status_role = role;
}

static void restart_scan(live_t *L, bool clear) {
    [L->adapter stopScan];
    if (clear) {
        L->ndev = 0;
        L->selected = 0;   /* with a "last:" line and no devices yet, that line */
    }
    [L->adapter startScan];
    L->scan_started = wall_s();
}

static void open_replay_list(live_t *L) {
    tui_replay_list_free(&L->rlist);
    char err[256] = "";
    if (tui_replay_list_scan(&L->rlist, L->o->captures_root, err, sizeof(err)) != 0) {
        set_status(L, TUI_ROLE_WARN, "%s", err);
    }
    L->rlist_open = true;
    L->rlist_sel = 0;
    L->rlist_scroll = 0;
}

static void merge_devices(live_t *L) {
    NSArray<NSString *> *names = L->adapter.discoveredNames;
    for (NSUInteger i = 0; i < names.count; i++) {
        const char *name = names[i].UTF8String ?: "";
        OBDAdapterTransport tr = (OBDAdapterTransport)[L->adapter.discoveredTransports[i] intValue];
        int rssi = [L->adapter.discoveredRSSIs[i] intValue];
        int found = -1;
        for (int k = 0; k < L->ndev; k++) {
            if (L->devs[k].transport == tr && strcmp(L->devs[k].name, name) == 0) found = k;
        }
        if (found < 0 && L->ndev < MAX_DEVICES) {
            found = L->ndev++;
            snprintf(L->devs[found].name, sizeof(L->devs[found].name), "%s", name);
            L->devs[found].transport = tr;
            /* keep the "last:" line selected while the list grows (PCAN rows come first) */
            if (L->last_dev[0] && L->selected == L->npcan + L->ndev - 1) L->selected = L->npcan + L->ndev;
        }
        if (found >= 0) L->devs[found].rssi = rssi;
    }
}

/* Index in the adapter's current scan of the target, -1 if not there yet. */
static int find_target(live_t *L) {
    NSArray<NSString *> *names = L->adapter.discoveredNames;
    NSString *want = [NSString stringWithUTF8String:L->target];
    for (NSUInteger i = 0; i < names.count; i++) {
        if (L->target_exact) {
            if ([names[i] isEqualToString:want]) return (int)i;
        } else if ([[names[i] uppercaseString] containsString:[want uppercaseString]]) {
            return (int)i;
        }
    }
    return -1;
}

/*******************************************************************************
 * PCAN-USB rows (hot-plug poll) and the PCAN target
 ******************************************************************************/

/* N of a "pcanN ..." row or last-device name, else 0. */
static int pcan_label_index(const char *name) {
    if (!name || strncasecmp(name, "pcan", 4) != 0) return 0;
    char tok[16];
    size_t n = strcspn(name, " ");
    if (n >= sizeof(tok)) return 0;
    memcpy(tok, name, n);
    tok[n] = '\0';
    int idx = pcan_parse_device(tok);
    return idx > 0 ? idx : 0;
}

/* Re-read the channel conditions (about once a second, or now). The
 * selection stays on the same Bluetooth row or "last:" line. */
static void pcan_poll(live_t *L, bool now) {
    if (!L->pcan_api) return;
    if (!now && wall_s() - L->pcan_polled < PCAN_POLL_S) return;
    L->pcan_polled = wall_s();
    int old = L->npcan;
    bool had_item = old + L->ndev + (L->last_dev[0] ? 1 : 0) > 0;
    L->npcan = pcan_enumerate(L->pcan_api, L->pcan, PCAN_MAX_CHANNELS);
    if (had_item && L->selected >= old) L->selected += L->npcan - old;
    int max_sel = L->npcan + L->ndev - 1 + (L->last_dev[0] ? 1 : 0);
    if (L->selected > max_sel) L->selected = max_sel;
    if (L->selected < 0) L->selected = 0;
}

/* Channel to open for the PCAN target; 0 while none is available. */
static int find_pcan_target(live_t *L) {
    int busy = 0;
    for (int i = 0; i < L->npcan; i++) {
        const pcan_channel_info_t *c = &L->pcan[i];
        if (L->target_pcan_index && c->index != L->target_pcan_index) continue;
        if (c->condition == PCAN_CHANNEL_AVAILABLE) return c->index;
        if (!busy) busy = c->index;
    }
    if (busy && !L->target_busy_noted) {
        set_status(L, TUI_ROLE_WARN, "pcan%d is in use by another application (waiting)", busy);
        L->target_busy_noted = true;
    }
    return 0;
}

/* Aim the connect at a PCAN channel (0 = first available). */
static void set_pcan_target(live_t *L, const char *name, int index) {
    if (!L->pcan_api) {
        set_status(L, TUI_ROLE_ALERT, "%s", L->pcan_err);
        L->target[0] = '\0';
        return;
    }
    snprintf(L->target, sizeof(L->target), "%s", name);
    L->target_exact = false;
    L->target_pcan = true;
    L->target_pcan_index = index;
    L->target_busy_noted = false;
}

static void render_now(live_t *L);

static void begin_connect(live_t *L, int index) {
    NSString *name = L->adapter.discoveredNames[(NSUInteger)index];
    snprintf(L->connect_name, sizeof(L->connect_name), "%s", name.UTF8String ?: "");
    OBDAdapterTransport tr = (OBDAdapterTransport)[L->adapter.discoveredTransports[(NSUInteger)index] intValue];
    L->target[0] = '\0';
    L->rlist_open = false;
    [L->adapter stopScan];
    if (tr == OBD_ADAPTER_TRANSPORT_BLE && ![L->adapter isBleReady]) {
        set_status(L, TUI_ROLE_ALERT, "%s needs BLE, but Bluetooth LE is not available", L->connect_name);
        restart_scan(L, false);
        return;
    }
    snprintf(L->transport_text, sizeof(L->transport_text), "%s", tr == OBD_ADAPTER_TRANSPORT_BLE ? "BLE" : "Classic");
    set_status(L, TUI_ROLE_BRIGHT, "connecting%s %s", L->o->ascii ? "..." : "…", L->connect_name);
    L->state = ST_CONNECTING;
    render_now(L);
    if (![L->adapter connectToDevice:index]) {
        set_status(L, TUI_ROLE_ALERT, "connect failed: %s", L->connect_name);
        L->state = ST_CONNECT;
        restart_scan(L, false);
        return;
    }
    L->connect_index = index;
    L->connect_started = wall_s();
}

static void cancel_connect(live_t *L, const char *why, char role) {
    if (L->pcan_mode) {
        if (L->autobaud_on) pcan_autobaud_cancel(&L->autobaud);   /* closes an open attempt */
        L->autobaud_on = false;
        L->pcan_mode = false;
    } else {
        [L->adapter disconnect];
    }
    set_status(L, role, "%s", why);
    L->state = ST_CONNECT;
    restart_scan(L, false);
}

static bool make_session_dir(live_t *L) {
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char stamp[32];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
    const char *root = L->o->captures_root && L->o->captures_root[0] ? L->o->captures_root : "captures";
    mkdir(root, 0755);
    for (int k = 1; k < 100; k++) {
        if (k == 1) snprintf(L->session_dir, sizeof(L->session_dir), "%s/%s", root, stamp);
        else snprintf(L->session_dir, sizeof(L->session_dir), "%s/%s_%d", root, stamp, k);
        if (mkdir(L->session_dir, 0755) == 0) return true;
        if (errno != EEXIST) break;
    }
    return false;
}

static void finish_connect(live_t *L) {
    set_status(L, TUI_ROLE_BRIGHT, "initializing adapter%s", L->o->ascii ? "..." : "…");
    render_now(L);
    obd_link_init(&L->link);
    L->link.device_index = L->connect_index;
    NSString *connected = L->adapter.connectedDeviceName;
    if (connected.length > 0) snprintf(L->link.device_name, sizeof(L->link.device_name), "%s", connected.UTF8String);
    else snprintf(L->link.device_name, sizeof(L->link.device_name), "%s", L->connect_name);

    /* Receive-only init: ATZ, ATE0, ATL0, STI, ATSP6, ATCSM1, ATS1, ATCAF0, ATH1 */
    obd_init_config_t cfg = { .profile = OBD_INIT_PASSIVE, .filters = NULL, .filter_count = 0 };
    if (!obd_init_adapter(L->adapter, &cfg, &L->link)) {
        poll_capture(L);
        cancel_connect(L, "adapter init failed (see the log in the session folder)", TUI_ROLE_ALERT);
        return;
    }
    snprintf(L->monitor_cmd, sizeof(L->monitor_cmd), "%s", obd_monitor_cmd(&L->link));
    snprintf(L->stn_text, sizeof(L->stn_text), "%s", L->link.stn_detected ? L->link.stn_info.device_id : "");

    if (!make_session_dir(L)) {
        cancel_connect(L, "cannot create the session folder", TUI_ROLE_ALERT);
        return;
    }
    if (L->o->capture) {
        poll_capture(L);
        char log_path[700];
        snprintf(log_path, sizeof(log_path), "%s/tui.log", L->session_dir);
        tui_capture_switch(L->o->capture, log_path);
    }

    L->src = frame_source_live_create(L->adapter, L->monitor_cmd);
    if (!L->src) {
        cancel_connect(L, "out of memory", TUI_ROLE_ALERT);
        return;
    }
    tui_host_t host = {
        .ctx = L,
        .device_name = L->link.device_name,
        .device_prefix = L->o->device,
        .transport = L->transport_text,
        .stn = L->stn_text[0] ? L->stn_text : NULL,
        .monitor_cmd = L->monitor_cmd,
        .is_connected = host_is_connected,
        .tx_setup = host_tx_setup,
        .tx_query = host_tx_query,
        .passive_init = host_passive_init,
    };
    tui_app_options_t opt = {
        .profile_path = L->o->profile,
        .profiles_dir = L->o->profiles_dir,
        .exe_path = L->o->exe_path,
        .captures_root = L->o->captures_root,
        .state_dir = L->o->state_dir,
        .no_tx = L->o->no_tx,
        .no_rec = L->o->no_rec,
        .ascii = L->o->ascii,
        .headless = false,
        .colors = L->o->colors,
    };
    /* Log preface, oldest first: connect, captured init lines, chip, folder */
    static char lines[PENDING_LINES + 4][200];
    const char *preface[PENDING_LINES + 4];
    int np = 0;
    snprintf(lines[np], sizeof(lines[np]), "connected %s (%s)", L->link.device_name, L->transport_text);
    preface[np] = lines[np];
    np++;
    for (int i = 0; i < L->npending; i++, np++) {
        snprintf(lines[np], sizeof(lines[np]), "%s", L->pending[i]);
        preface[np] = lines[np];
    }
    if (L->link.stn_detected) snprintf(lines[np], sizeof(lines[np]), "STN detected: %s", L->stn_text);
    else snprintf(lines[np], sizeof(lines[np]), "plain ELM327 (no STN): monitor with ATMA");
    preface[np] = lines[np];
    np++;
    snprintf(lines[np], sizeof(lines[np]), "session folder: %s", L->session_dir);
    preface[np] = lines[np];
    np++;
    L->npending = 0;

    char err[600] = "";
    L->app = tui_app_create_live(&opt, L->src, &host, L->session_dir, preface, np, L->cols, L->rows,
                                 err, sizeof(err));
    if (!L->app) {
        frame_source_free(L->src);
        L->src = NULL;
        cancel_connect(L, err[0] ? err : "cannot start the dashboard", TUI_ROLE_ALERT);
        return;
    }
    tui_last_write(L->o->state_dir, L->link.device_name, NULL);
    L->state = ST_DASHBOARD;
    tui_app_start(L->app);
}

/* Open the PCAN channel (pcan_open: listen-only set, init, read back) and
 * start the dashboard over its frame source. The host has no TX callbacks. */
static void finish_connect_pcan(live_t *L, int bitrate) {
    char err[768];
    pcan_channel_t ch;
    if (pcan_open(L->pcan_api, L->pcan_index, bitrate, &ch, err, sizeof(err)) != PCAN_OPEN_OK) {
        fprintf(stderr, "[ERROR] %s\n", err);
        poll_capture(L);
        cancel_connect(L, err, TUI_ROLE_ALERT);
        return;
    }
    obd_link_init(&L->link);
    snprintf(L->link.device_name, sizeof(L->link.device_name), "%s", ch.label);
    snprintf(L->mode_text, sizeof(L->mode_text), "listen-only %s", pcan_bitrate_name(bitrate));
    snprintf(L->monitor_cmd, sizeof(L->monitor_cmd), "pcan listen-only");
    L->stn_text[0] = '\0';
    char firmware[64];
    snprintf(firmware, sizeof(firmware), "%s", ch.firmware);

    if (!make_session_dir(L)) {
        pcan_close(&ch);
        cancel_connect(L, "cannot create the session folder", TUI_ROLE_ALERT);
        return;
    }
    if (L->o->capture) {
        poll_capture(L);
        char log_path[700];
        snprintf(log_path, sizeof(log_path), "%s/tui.log", L->session_dir);
        tui_capture_switch(L->o->capture, log_path);
    }
    L->src = frame_source_pcan_create(&ch);
    if (!L->src) {
        pcan_close(&ch);
        cancel_connect(L, "out of memory", TUI_ROLE_ALERT);
        return;
    }
    tui_host_t host = {
        .ctx = L,
        .device_name = L->link.device_name,
        .device_prefix = L->o->device,
        .transport = L->transport_text,
        .stn = NULL,
        .monitor_cmd = L->monitor_cmd,
        .is_connected = host_pcan_is_connected,
        .tx_setup = NULL,        /* nothing can be sent over PCAN */
        .tx_query = NULL,
        .passive_init = NULL,
        .listen_only = true,
        .mode_text = L->mode_text,
        .interface_name = "pcan",
        .library_version = pcan_api_version(),
        .bitrate = bitrate,
    };
    tui_app_options_t opt = {
        .profile_path = L->o->profile,
        .profiles_dir = L->o->profiles_dir,
        .exe_path = L->o->exe_path,
        .captures_root = L->o->captures_root,
        .state_dir = L->o->state_dir,
        .no_tx = L->o->no_tx,
        .no_rec = L->o->no_rec,
        .ascii = L->o->ascii,
        .headless = false,
        .colors = L->o->colors,
    };
    static char lines[PENDING_LINES + 5][200];
    const char *preface[PENDING_LINES + 5];
    int np = 0;
    snprintf(lines[np], sizeof(lines[np]), "connected %s (%s), PCBUSB \"%s\"", L->link.device_name,
             L->transport_text, pcan_api_version());
    preface[np] = lines[np];
    np++;
    for (int i = 0; i < L->npending; i++, np++) {
        snprintf(lines[np], sizeof(lines[np]), "%s", L->pending[i]);
        preface[np] = lines[np];
    }
    snprintf(lines[np], sizeof(lines[np]), "PCAN %s verified: no ACK, no error frames, nothing is sent; TX mode off",
             L->mode_text);
    preface[np] = lines[np];
    np++;
    snprintf(lines[np], sizeof(lines[np]), "timestamps: hardware %s (CAN controller clock)%s%s",
             L->o->ascii ? "us" : "µs", firmware[0] ? "; firmware " : "", firmware);
    preface[np] = lines[np];
    np++;
    snprintf(lines[np], sizeof(lines[np]), "session folder: %s", L->session_dir);
    preface[np] = lines[np];
    np++;
    L->npending = 0;

    L->app = tui_app_create_live(&opt, L->src, &host, L->session_dir, preface, np, L->cols, L->rows,
                                 err, sizeof(err));
    if (!L->app) {
        frame_source_free(L->src);   /* CAN_Uninitialize */
        L->src = NULL;
        cancel_connect(L, err[0] ? err : "cannot start the dashboard", TUI_ROLE_ALERT);
        return;
    }
    tui_last_write(L->o->state_dir, L->link.device_name, NULL);
    L->state = ST_DASHBOARD;
    tui_app_start(L->app);
}

static void begin_connect_pcan(live_t *L, int index) {
    L->target[0] = '\0';
    L->target_pcan = false;
    L->rlist_open = false;
    [L->adapter stopScan];
    L->pcan_mode = true;
    L->pcan_index = index;
    snprintf(L->transport_text, sizeof(L->transport_text), "USB");
    snprintf(L->connect_name, sizeof(L->connect_name), "pcan%d", index);
    L->state = ST_CONNECTING;
    const char *ell = L->o->ascii ? "..." : "…";
    if (L->o->bitrate.autodetect) {
        set_status(L, TUI_ROLE_BRIGHT, "pcan%d: detecting the bitrate, listen-only%s", index, ell);
        pcan_autobaud_begin(&L->autobaud, L->pcan_api, index, 0);
        L->autobaud_on = true;
        render_now(L);
        return;   /* step_connecting runs it in short slices */
    }
    set_status(L, TUI_ROLE_BRIGHT, "opening pcan%d listen-only %s%s", index, pcan_bitrate_name(L->o->bitrate.bitrate),
               ell);
    render_now(L);
    finish_connect_pcan(L, L->o->bitrate.bitrate);
}

static void close_dashboard(live_t *L) {
    if (!L->app) return;
    tui_app_shutdown(L->app);   /* TX off -> PASSIVE re-init (rule 0.6), discover abort, rec close */
    if (L->adapter.isConnected && tui_app_monitor_running(L->app)) frame_source_stop(L->src);
    poll_capture(L);
    tui_app_free(L->app);
    L->app = NULL;
    frame_source_free(L->src);   /* PCAN: CAN_Uninitialize */
    L->src = NULL;
    L->pcan_mode = false;
    if (L->adapter.isConnected) [L->adapter disconnect];
    if (L->o->capture) tui_capture_switch_temp(L->o->capture);
}

static void load_last(live_t *L) {
    tui_last_read(L->o->state_dir, L->last_dev, sizeof(L->last_dev), L->last_prof, sizeof(L->last_prof),
                  L->last_date, sizeof(L->last_date));
    L->last_text[0] = '\0';
    if (!L->last_dev[0]) return;
    const char *prof = L->last_prof[0] ? L->last_prof : "none";
    const char *base = strrchr(prof, '/');
    const char *dot = tui_glyph(TUI_G_DOT, L->o->ascii);
    int n = snprintf(L->last_text, sizeof(L->last_text), "last: %s %s %s", base ? base + 1 : prof, dot, L->last_dev);
    if (L->last_date[0] && n > 0 && (size_t)n < sizeof(L->last_text)) {
        snprintf(L->last_text + n, sizeof(L->last_text) - (size_t)n, " %s %s", dot, L->last_date);
    }
}

static void connect_key(live_t *L, int key) {
    if (L->prompting) {
        size_t n = strlen(L->prompt);
        if (key == TUI_KEY_ESC) {
            L->prompting = false;
        } else if (key == TUI_KEY_ENTER || key == '\r') {
            char err[256];
            if (n > 0 && L->replay_out && !tui_replay_has_frames(L->prompt, err, sizeof(err))) {
                set_status(L, TUI_ROLE_WARN, "cannot replay %s: %s", L->prompt, err);
            } else if (n > 0 && L->replay_out) {
                snprintf(L->replay_out, L->replay_len, "%s", L->prompt);
                L->rc = 3;
                L->quit = true;
            }
            L->prompting = false;
        } else if (key == TUI_KEY_BACKSPACE || key == 8) {
            if (n > 0) L->prompt[n - 1] = '\0';
        } else if (key >= 32 && key < 127 && n + 1 < sizeof(L->prompt)) {
            L->prompt[n] = (char)key;
            L->prompt[n + 1] = '\0';
        }
        return;
    }
    if (L->rlist_open) {
        int page = tui_replay_list_rows(L->cols, L->rows);
        switch (key) {
            case TUI_KEY_ESC:
                L->rlist_open = false;
                return;
            case TUI_KEY_ENTER:
            case '\r':
                if (L->rlist_sel >= 0 && L->rlist_sel < L->rlist.count && L->replay_out) {
                    const tui_replay_entry_t *e = &L->rlist.entries[L->rlist_sel];
                    char err[256];
                    if (!tui_replay_has_frames(e->path, err, sizeof(err))) {
                        set_status(L, TUI_ROLE_WARN, "cannot replay %s: %s", e->name, err);   /* the list stays open */
                        return;
                    }
                    snprintf(L->replay_out, L->replay_len, "%s", e->path);
                    L->rc = 3;
                    L->quit = true;
                }
                return;
            case TUI_KEY_UP:   L->rlist_sel--; break;
            case TUI_KEY_DOWN: L->rlist_sel++; break;
            case TUI_KEY_PGUP: L->rlist_sel -= page; break;
            case TUI_KEY_PGDN: L->rlist_sel += page; break;
            default: break;
        }
        if (L->rlist_sel >= L->rlist.count) L->rlist_sel = L->rlist.count - 1;
        if (L->rlist_sel < 0) L->rlist_sel = 0;
        return;
    }
    if (L->state == ST_CONNECTING) {
        if (key == TUI_KEY_ESC) cancel_connect(L, "connect cancelled", TUI_ROLE_DIM);
        return;
    }
    int max_sel = L->npcan + L->ndev - 1 + (L->last_dev[0] ? 1 : 0);
    switch (key) {
        case TUI_KEY_UP:
            if (L->selected > 0) L->selected--;
            break;
        case TUI_KEY_DOWN:
            if (L->selected < max_sel) L->selected++;
            break;
        case TUI_KEY_ENTER:
        case '\r':
            /* explicit Enter: the only way the connect screen connects (rule 0.4) */
            if (L->selected < L->npcan) {
                char name[64];
                const pcan_channel_info_t *c = &L->pcan[L->selected];
                pcan_channel_label(c->index, c->hw_name, name, sizeof(name));
                set_pcan_target(L, name, c->index);
            } else if (L->selected < L->npcan + L->ndev) {
                snprintf(L->target, sizeof(L->target), "%s", L->devs[L->selected - L->npcan].name);
                L->target_exact = true;
                L->target_pcan = false;
            } else if (L->last_dev[0]) {
                int pi = pcan_label_index(L->last_dev);
                if (pi > 0) {
                    set_pcan_target(L, L->last_dev, pi);
                } else {
                    snprintf(L->target, sizeof(L->target), "%s", L->last_dev);
                    L->target_exact = true;
                    L->target_pcan = false;
                }
            }
            if (L->target[0]) set_status(L, TUI_ROLE_BRIGHT, "waiting for %s%s", L->target, L->o->ascii ? "..." : "…");
            break;
        case TUI_KEY_ESC:
            if (L->target[0]) {
                L->target[0] = '\0';
                set_status(L, TUI_ROLE_DIM, "cancelled");
            }
            break;
        case 'r':
            set_status(L, TUI_ROLE_DIM, "recording starts after connecting");
            break;
        case 'R':
            open_replay_list(L);
            break;
        case 's':
            L->target[0] = '\0';
            L->status[0] = '\0';
            restart_scan(L, true);
            pcan_poll(L, true);
            break;
        case 'o':
            L->prompting = true;
            L->prompt[0] = '\0';
            break;
        case 'q':
            L->quit = true;
            break;
        default:
            break;
    }
}

static void step_connect(live_t *L) {
    [L->adapter runLoopFor:0.01];
    merge_devices(L);
    pcan_poll(L, false);
    if (wall_s() - L->scan_started >= SCAN_RESTART_S) restart_scan(L, false);
    if (L->target[0] && !L->rlist_open) {   /* a pending target waits until the replay list closes */
        if (L->target_pcan) {
            int ch = find_pcan_target(L);
            if (ch > 0) begin_connect_pcan(L, ch);
        } else {
            int idx = find_target(L);
            if (idx >= 0) begin_connect(L, idx);
        }
    }
}

/* PCAN connect: the bitrate detection runs in 20 ms slices (keys stay live). */
static void step_connecting_pcan(live_t *L) {
    if (!L->autobaud_on) return;
    if (!pcan_autobaud_step(&L->autobaud, 0.02)) return;
    L->autobaud_on = false;
    char report[512];
    pcan_autobaud_report(&L->autobaud, report, sizeof(report));
    if (!L->autobaud.bitrate) {
        fprintf(stderr, "[ERROR] pcan%d: bitrate detection failed: %s (tried: %s)\n", L->pcan_index,
                L->autobaud.error, report[0] ? report : "nothing");
        poll_capture(L);
        char why[300];
        snprintf(why, sizeof(why), "pcan%d: no bitrate detected (%s)", L->pcan_index, L->autobaud.error);
        cancel_connect(L, why, TUI_ROLE_ALERT);
        return;
    }
    fprintf(stderr, "[INFO] pcan%d: bitrate %s detected (tried: %s)\n", L->pcan_index,
            pcan_bitrate_name(L->autobaud.bitrate), report);
    poll_capture(L);
    set_status(L, TUI_ROLE_BRIGHT, "opening pcan%d listen-only %s%s", L->pcan_index,
               pcan_bitrate_name(L->autobaud.bitrate), L->o->ascii ? "..." : "…");
    render_now(L);
    finish_connect_pcan(L, L->autobaud.bitrate);
}

static void step_connecting(live_t *L) {
    if (L->pcan_mode) {
        step_connecting_pcan(L);
        return;
    }
    [L->adapter runLoopFor:0.01];
    if (L->adapter.serviceDiscoveryDone && L->adapter.isConnected) {
        finish_connect(L);
    } else if (wall_s() - L->connect_started > CONNECT_TIMEOUT_S) {
        cancel_connect(L, "connection / service discovery timeout", TUI_ROLE_ALERT);
    }
}

static void step_dashboard(live_t *L) {
    frame_source_pump(L->src, 0.01);
    poll_capture(L);
    tui_app_tick(L->app);
    if (tui_app_quit_requested(L->app)) {
        const char *req = tui_app_replay_request(L->app);
        if (req && req[0] && L->replay_out) {
            snprintf(L->replay_out, L->replay_len, "%s", req);
            L->rc = 3;
        }
        L->quit = true;
    } else if (tui_app_wants_connect_screen(L->app)) {
        close_dashboard(L);
        set_status(L, TUI_ROLE_DIM, "disconnected");
        load_last(L);
        L->rlist_open = false;
        L->state = ST_CONNECT;
        restart_scan(L, true);
    }
}

/*******************************************************************************
 * Drawing
 ******************************************************************************/

static void draw(live_t *L) {
    if (L->grid->cols != L->cols || L->grid->rows != L->rows) tui_grid_resize(L->grid, L->cols, L->rows);
    if (L->state == ST_DASHBOARD && L->app) {
        tui_app_render(L->app, L->grid);
        return;
    }
    bool ascii = L->o->ascii;
    tui_connect_row_t rows[PCAN_MAX_CHANNELS + MAX_DEVICES];
    char pcan_names[PCAN_MAX_CHANNELS][64];
    int nrows = 0;
    for (int i = 0; i < L->npcan; i++, nrows++) {
        pcan_channel_label(L->pcan[i].index, L->pcan[i].hw_name, pcan_names[i], sizeof(pcan_names[i]));
        rows[nrows].name = pcan_names[i];
        rows[nrows].transport = "USB";
        snprintf(rows[nrows].detail, sizeof(rows[nrows].detail), "%s", pcan_condition_text(L->pcan[i].condition));
    }
    for (int i = 0; i < L->ndev; i++, nrows++) {
        rows[nrows].name = L->devs[i].name;
        bool ble = L->devs[i].transport == OBD_ADAPTER_TRANSPORT_BLE;
        rows[nrows].transport = ble ? "BLE" : "Classic";
        if (ble) snprintf(rows[nrows].detail, sizeof(rows[nrows].detail), "%d dBm", L->devs[i].rssi);
        else snprintf(rows[nrows].detail, sizeof(rows[nrows].detail), "paired");
    }
    bool ble_ready = [L->adapter isBleReady];
    bool settling = !ble_ready && wall_s() - L->start_wall < 3.0;
    char status[300];
    const char *st = L->status[0] ? L->status : NULL;
    char st_role = L->status_role;
    if (!st && L->state == ST_CONNECT) {
        snprintf(status, sizeof(status), "scanning%s %d s", ascii ? "..." : "…", (int)(wall_s() - L->scan_started));
        st = status;
        st_role = TUI_ROLE_DIM;
    } else if (st && L->state == ST_CONNECT && L->target[0]) {
        snprintf(status, sizeof(status), "%s (scanning %d s)", L->status, (int)(wall_s() - L->scan_started));
        st = status;
    }
    tui_connect_view_t v = {
        .bt_state = ble_ready ? "BT ready" : (settling ? (ascii ? "BT ..." : "BT …") : "BT off"),
        .bt_note = (!ble_ready && !settling)
            ? (ascii ? "Bluetooth off - paired Classic adapters only" : "Bluetooth off — paired Classic adapters only")
            : NULL,
        .rows = rows,
        .nrows = nrows,
        .selected = L->selected,
        .last_text = L->last_text[0] ? L->last_text : NULL,
        .status = st,
        .status_role = st_role,
        .prompt_label = L->prompting ? "capture path: " : NULL,
        .prompt_text = L->prompt,
        .busy = L->state == ST_CONNECTING,
    };
    tui_draw_connect(L->grid, L->cols, L->rows, &v, ascii);
    if (L->rlist_open) {
        tui_dim_grid(L->grid);
        tui_replay_view_t rv = {
            .rows = L->rlist.entries,
            .nrows = L->rlist.count,
            .selected = L->rlist_sel,
            .scroll = &L->rlist_scroll,
            .root = L->o->captures_root,
        };
        tui_draw_replay_list(L->grid, L->cols, L->rows, &rv, ascii);
    }
}

static void render_now(live_t *L) {
    if (L->o->headless) return;
    draw(L);
    tui_term_draw(L->grid);
    L->last_draw = wall_s();
}

/*******************************************************************************
 * Loop
 ******************************************************************************/

static void handle_key(live_t *L, int key) {
    if (key == TUI_KEY_RESIZE) {
        tui_term_size(&L->cols, &L->rows);
        if (L->app) tui_app_resize(L->app, L->cols, L->rows);
        return;
    }
    if (L->state == ST_DASHBOARD && L->app) tui_app_key(L->app, key);
    else connect_key(L, key);
}

int tui_live_run(const tui_live_options_t *o, char *replay_out, size_t replay_len) {
    live_t *L = (live_t *)calloc(1, sizeof(*L));
    if (!L) return 1;
    L->o = o;
    L->replay_out = replay_out;
    L->replay_len = replay_len;
    L->cols = o->headless ? o->cols : 80;
    L->rows = o->headless ? o->rows : 24;
    if (!o->headless) tui_term_size(&L->cols, &L->rows);
    L->grid = tui_grid_new(L->cols, L->rows);
    obd_link_init(&L->link);
    L->adapter = [[OBDAdapter alloc] initWithNameHints:[OBDAdapter defaultNameHints] quiet:NO];
    L->adapter.logCommandErrors = YES;
    L->start_wall = wall_s();
    load_last(L);
    L->pcan_api = pcan_api_load(L->pcan_err, sizeof(L->pcan_err));   /* NULL: no PCAN rows */
    int pcan_dev = o->device ? pcan_parse_device(o->device) : PCAN_DEVICE_NONE;
    if (pcan_dev == PCAN_DEVICE_BAD) {
        set_status(L, TUI_ROLE_ALERT, "%s: PCAN channels are pcan1..pcan8", o->device);
    } else if (pcan_dev >= 0) {
        /* explicit --device pcan / pcanN: that PCAN channel (first available for "pcan") */
        set_pcan_target(L, o->device, pcan_dev);
        if (L->target[0]) set_status(L, TUI_ROLE_BRIGHT, "looking for %s%s", o->device, o->ascii ? "..." : "…");
    } else if (o->device && o->device[0]) {
        /* explicit --device: connect to the first adapter whose name contains it */
        snprintf(L->target, sizeof(L->target), "%s", o->device);
        L->target_exact = false;
        set_status(L, TUI_ROLE_BRIGHT, "looking for %s%s", o->device, o->ascii ? "..." : "…");
    }
    restart_scan(L, true);
    pcan_poll(L, true);

    while (!L->quit) {
        @autoreleasepool {
            if (o->stop && *o->stop) break;
            switch (L->state) {
                case ST_CONNECT:    step_connect(L); break;
                case ST_CONNECTING: step_connecting(L); break;
                case ST_DASHBOARD:  step_dashboard(L); break;
            }
            if (L->quit) break;
            if (L->state != ST_DASHBOARD) poll_capture(L);

            bool keyed = false;
            if (o->headless) {
                int64_t t = (int64_t)((wall_s() - L->start_wall) * 1000.0);
                while (L->next_key < o->nkeys && o->keys[L->next_key].at_ms <= t) {
                    handle_key(L, o->keys[L->next_key++].key);
                }
                while (L->next_snap < o->nsnap && o->snap_times[L->next_snap] <= t) {
                    draw(L);
                    printf("--- snapshot T=%.3f %dx%d ---\n", (double)o->snap_times[L->next_snap] / 1000.0,
                           L->cols, L->rows);
                    tui_grid_dump_text(L->grid, stdout);
                    if (o->color_map) {
                        printf("--- color-map T=%.3f %dx%d ---\n", (double)o->snap_times[L->next_snap] / 1000.0,
                               L->cols, L->rows);
                        tui_grid_dump_roles(L->grid, stdout);
                        if (tui_grid_has_fg(L->grid)) {
                            printf("--- fg-map T=%.3f %dx%d ---\n", (double)o->snap_times[L->next_snap] / 1000.0,
                                   L->cols, L->rows);
                            tui_grid_dump_fg(L->grid, stdout);
                        }
                    }
                    fflush(stdout);
                    L->next_snap++;
                }
                if (L->next_snap >= o->nsnap) L->quit = true;
            } else {
                int k;
                while ((k = tui_term_key()) != TUI_KEY_NONE) {
                    handle_key(L, k);
                    keyed = true;
                }
                if (keyed || wall_s() - L->last_draw >= 0.1) render_now(L);
            }
        }
    }

    close_dashboard(L);
    if (L->autobaud_on) pcan_autobaud_cancel(&L->autobaud);   /* quit during detection: close the attempt */
    [L->adapter stopScan];
    if (L->adapter.isConnected) [L->adapter disconnect];
    [L->adapter release];
    tui_grid_free(L->grid);
    tui_replay_list_free(&L->rlist);
    int rc = L->rc;
    free(L);
    return rc;
}
