/**
 * @file pcan_test.c
 * @brief PCAN listen-only open sequence, auto bitrate, clock mapping, PCAN
 *        frame source, message conversion and the 6-column TSV (no library,
 *        no hardware: a fake pcan_api_t that records every call)
 *
 * pcan_api_t has no write member, so no test (and no caller) can transmit
 * through it; test_symbols checks that the loader never looks up a write.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "frame_source.h"
#include "frame_source_pcan.h"
#include "frame_tsv.h"
#include "pcan.h"

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

/*******************************************************************************
 * Recording fake
 ******************************************************************************/

typedef enum { C_SET, C_INIT, C_GET_LO, C_UNINIT } call_kind_t;

typedef struct {
    call_kind_t   kind;
    pcan_handle_t ch;
    uint8_t       param;
    uint32_t      value;
    uint16_t      btr;
} call_t;

typedef struct {
    pcan_status_t    sts;
    pcan_msg_t       m;
    pcan_timestamp_t ts;
} queued_t;

static struct {
    call_t   calls[512];
    int      ncalls;
    bool     set_fail;
    bool     init_fail;
    bool     pre_readback_fail;          /* GetValue(LISTEN_ONLY) fails before init */
    bool     pre_readback_off;           /* ... reads OFF before init (the set was accepted) */
    bool     readback_fail;              /* after init */
    bool     readback_off;
    char     version[300];               /* PCAN_CHANNEL_VERSION answer */
    bool     version_fail;
    bool     version_unterminated;       /* fill all 256 bytes, no NUL */
    int      version_reads;
    uint16_t bus_btr;                    /* autobaud: frames only at this btr */
    uint32_t condition[PCAN_MAX_CHANNELS + 1];
    bool     initialized[PCAN_MAX_CHANNELS + 1];
    uint16_t init_btr[PCAN_MAX_CHANNELS + 1];
    uint32_t lo[PCAN_MAX_CHANNELS + 1];
    bool     generate;                   /* autobaud: endless data frames at bus_btr */
    uint32_t gen_n;
    queued_t queue[64];
    int      qhead, qtail;
    pcan_status_t empty_sts;             /* what a read returns with an empty queue */
    int      pipe_fd[2];
    bool     use_pipe;
} F;

static int ch_index(pcan_handle_t h) {
    int i = (int)h - (int)PCAN_USBBUS1 + 1;
    return (i >= 1 && i <= PCAN_MAX_CHANNELS) ? i : 0;
}

static void record(call_kind_t kind, pcan_handle_t ch, uint8_t param, uint32_t value, uint16_t btr) {
    if (F.ncalls < (int)(sizeof(F.calls) / sizeof(F.calls[0]))) {
        F.calls[F.ncalls++] = (call_t){ kind, ch, param, value, btr };
    }
}

static pcan_status_t f_initialize(pcan_handle_t ch, uint16_t btr, uint8_t t, uint32_t p, uint16_t i) {
    (void)t;
    (void)p;
    (void)i;
    record(C_INIT, ch, 0, 0, btr);
    int idx = ch_index(ch);
    if (F.init_fail || !idx) return PCAN_ERROR_ILLHW;
    F.initialized[idx] = true;
    F.init_btr[idx] = btr;
    return PCAN_ERROR_OK;
}

static pcan_status_t f_uninitialize(pcan_handle_t ch) {
    record(C_UNINIT, ch, 0, 0, 0);
    int idx = ch_index(ch);
    if (idx) {
        F.initialized[idx] = false;
        F.lo[idx] = 0;
    }
    return PCAN_ERROR_OK;
}

static pcan_status_t f_get_status(pcan_handle_t ch) {
    (void)ch;
    return PCAN_ERROR_OK;
}

static void drain_pipe(void) {
    if (!F.use_pipe) return;
    char buf[64];
    while (read(F.pipe_fd[0], buf, sizeof(buf)) > 0) {
    }
}

static pcan_status_t f_read(pcan_handle_t ch, pcan_msg_t *m, pcan_timestamp_t *ts) {
    int idx = ch_index(ch);
    if (!idx || !F.initialized[idx]) return PCAN_ERROR_INITIALIZE;
    memset(m, 0, sizeof(*m));
    memset(ts, 0, sizeof(*ts));
    if (F.generate) {
        if (F.init_btr[idx] != F.bus_btr) return PCAN_ERROR_QRCVEMPTY;
        uint32_t n = F.gen_n++;
        m->id = (n % 2) ? 0x200 : 0x100;
        m->len = 8;
        ts->millis = n;
        return PCAN_ERROR_OK;
    }
    if (F.qhead == F.qtail) {
        drain_pipe();
        return F.empty_sts;
    }
    queued_t *q = &F.queue[F.qhead++];
    *m = q->m;
    *ts = q->ts;
    return q->sts;
}

static pcan_status_t f_get_value(pcan_handle_t ch, uint8_t param, void *buf, uint32_t len) {
    int idx = ch_index(ch);
    if (!idx) return PCAN_ERROR_ILLCLIENT;
    uint32_t v = 0;
    switch (param) {
        case PCAN_LISTEN_ONLY: {
            bool pre = !F.initialized[idx];
            if (pre ? F.pre_readback_fail : F.readback_fail) {
                record(C_GET_LO, ch, param, 0xFFFFFFFFu, 0);
                return PCAN_ERROR_ILLOPERATION;
            }
            v = (pre ? F.pre_readback_off : F.readback_off) ? 0 : F.lo[idx];
            record(C_GET_LO, ch, param, v, 0);
            break;
        }
        case PCAN_CHANNEL_VERSION:
            F.version_reads++;
            if (ch != PCAN_USBBUS1 || F.version_fail) return PCAN_ERROR_ILLPARAMTYPE;
            if (F.version_unterminated) memset(buf, 'A', len);
            else snprintf((char *)buf, len, "%s", F.version);
            return PCAN_ERROR_OK;
        case PCAN_CHANNEL_CONDITION:
            v = F.condition[idx];
            break;
        case PCAN_RECEIVE_EVENT:
            if (!F.use_pipe || len < sizeof(int)) return PCAN_ERROR_ILLPARAMTYPE;
            memcpy(buf, &F.pipe_fd[0], sizeof(int));
            return PCAN_ERROR_OK;
        case PCAN_HARDWARE_NAME:
            if (F.condition[idx] == PCAN_CHANNEL_UNAVAILABLE) return PCAN_ERROR_ILLHW;
            snprintf((char *)buf, len, "PCAN-USB\n");
            return PCAN_ERROR_OK;
        case PCAN_DEVICE_ID:
            v = 7;
            break;
        default:
            return PCAN_ERROR_ILLPARAMTYPE;
    }
    if (len < sizeof(v)) return PCAN_ERROR_ILLPARAMVAL;
    memcpy(buf, &v, sizeof(v));
    return PCAN_ERROR_OK;
}

static pcan_status_t f_set_value(pcan_handle_t ch, uint8_t param, void *buf, uint32_t len) {
    uint32_t v = 0xFFFFFFFFu;
    if (len >= sizeof(v)) memcpy(&v, buf, sizeof(v));
    record(C_SET, ch, param, v, 0);
    int idx = ch_index(ch);
    if (F.set_fail || !idx) return PCAN_ERROR_ILLOPERATION;
    if (param == PCAN_LISTEN_ONLY) F.lo[idx] = v;
    return PCAN_ERROR_OK;
}

static pcan_status_t f_error_text(pcan_status_t e, uint16_t lang, char *buf) {
    (void)lang;
    snprintf(buf, 256, "fake 0x%X", e);
    return PCAN_ERROR_OK;
}

static const pcan_api_t kApi = {
    .initialize = f_initialize,
    .uninitialize = f_uninitialize,
    .get_status = f_get_status,
    .read = f_read,
    .get_value = f_get_value,
    .set_value = f_set_value,
    .get_error_text = f_error_text,
};

static void fake_reset(void) {
    if (F.use_pipe) {
        close(F.pipe_fd[0]);
        close(F.pipe_fd[1]);
    }
    memset(&F, 0, sizeof(F));
    F.empty_sts = PCAN_ERROR_QRCVEMPTY;
    snprintf(F.version, sizeof(F.version), "PCBUSB 0.13.0.10");
    for (int i = 1; i <= PCAN_MAX_CHANNELS; i++) F.condition[i] = PCAN_CHANNEL_UNAVAILABLE;
    F.condition[1] = PCAN_CHANNEL_AVAILABLE;
}

static void enqueue(pcan_status_t sts, uint32_t id, uint8_t type, uint8_t len, uint64_t hw_us) {
    queued_t *q = &F.queue[F.qtail++];
    memset(q, 0, sizeof(*q));
    q->sts = sts;
    q->m.id = id;
    q->m.msgtype = type;
    q->m.len = len;
    for (int i = 0; i < 8; i++) q->m.data[i] = (uint8_t)(0x10 + i);
    q->ts.micros = (uint16_t)(hw_us % 1000);
    q->ts.millis = (uint32_t)((hw_us / 1000) & 0xFFFFFFFFull);
    q->ts.millis_overflow = (uint16_t)((hw_us / 1000) >> 32);
    if (F.use_pipe) {
        char b = 1;
        if (write(F.pipe_fd[1], &b, 1) != 1) fprintf(stderr, "pipe write failed\n");
    }
}

/* Every CAN_SetValue the code made: PCAN_LISTEN_ONLY = ON, nothing else. */
static bool only_listen_only_on_sets(void) {
    for (int i = 0; i < F.ncalls; i++) {
        if (F.calls[i].kind == C_SET && (F.calls[i].param != PCAN_LISTEN_ONLY || F.calls[i].value != PCAN_PARAMETER_ON)) {
            return false;
        }
    }
    return true;
}

/* Every CAN_Initialize comes right after SetValue(LISTEN_ONLY, ON) on that
 * channel and a read-back of it that returned ON, and is followed by the
 * post-init read-back before anything else. */
static bool every_init_is_safe(void) {
    for (int i = 0; i < F.ncalls; i++) {
        if (F.calls[i].kind != C_INIT) continue;
        if (i < 2 || i + 1 >= F.ncalls) return false;
        const call_t *set = &F.calls[i - 2], *pre = &F.calls[i - 1], *post = &F.calls[i + 1];
        if (set->kind != C_SET || set->ch != F.calls[i].ch || set->param != PCAN_LISTEN_ONLY ||
            set->value != PCAN_PARAMETER_ON) {
            return false;
        }
        if (pre->kind != C_GET_LO || pre->ch != F.calls[i].ch || pre->value != PCAN_PARAMETER_ON) return false;
        if (post->kind != C_GET_LO || post->ch != F.calls[i].ch) return false;
    }
    return true;
}

static int count_calls(call_kind_t kind) {
    int n = 0;
    for (int i = 0; i < F.ncalls; i++) n += F.calls[i].kind == kind;
    return n;
}

/*******************************************************************************
 * Tests
 ******************************************************************************/

static void test_symbols(void) {
    int n = 0;
    const char *const *names = pcan_api_symbol_names(&n);
    CHECK(n == 7, "symbol count %d", n);
    bool init = false, read = false;
    for (int i = 0; i < n; i++) {
        CHECK(strstr(names[i], "Write") == NULL, "loader looks up %s", names[i]);
        CHECK(strstr(names[i], "Reset") == NULL, "loader looks up %s", names[i]);
        CHECK(strstr(names[i], "FD") == NULL, "loader looks up %s", names[i]);
        init |= strcmp(names[i], "CAN_Initialize") == 0;
        read |= strcmp(names[i], "CAN_Read") == 0;
    }
    CHECK(init && read, "receive symbols present");
    /* the table: 7 receive-side function pointers, no write member (by construction) */
    CHECK(sizeof(pcan_api_t) == 7 * sizeof(void (*)(void)), "pcan_api_t holds exactly the 7 receive functions");
}

static void test_layouts(void) {
    CHECK(sizeof(can_frame_t) == 32, "can_frame_t %zu bytes", sizeof(can_frame_t));
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    CHECK(!f.hw_timestamp && f.timestamp_us == 0, "zeroed frame has no hw timestamp");
    can_frame_set_hw_time_us(&f, 1712560212345678LL);
    CHECK(f.timestamp_ms == 1712560212345LL && f.timestamp_us == 678 && f.hw_timestamp, "set hw time");
    CHECK(can_frame_time_us(&f) == 1712560212345678LL, "time us round trip");
    can_frame_set_hw_time_us(&f, -1);
    CHECK(f.timestamp_ms == -1 && f.timestamp_us == 999, "negative floors: %lld %u", (long long)f.timestamp_ms,
          f.timestamp_us);
    can_frame_t g;
    memset(&g, 0, sizeof(g));
    g.timestamp_ms = 5;
    g.timestamp_us = 300;   /* ignored without hw_timestamp */
    CHECK(can_frame_time_us(&g) == 5000, "host frame time");
}

static void test_parse_device(void) {
    CHECK(pcan_parse_device("pcan") == 0, "pcan");
    CHECK(pcan_parse_device("PCAN") == 0, "PCAN");
    CHECK(pcan_parse_device("pcan1") == 1, "pcan1");
    CHECK(pcan_parse_device("PCan8") == 8, "PCan8");
    CHECK(pcan_parse_device("pcan0") == PCAN_DEVICE_BAD, "pcan0");
    CHECK(pcan_parse_device("pcan9") == PCAN_DEVICE_BAD, "pcan9");
    CHECK(pcan_parse_device("pcan10") == PCAN_DEVICE_BAD, "pcan10");
    CHECK(pcan_parse_device("pcanx") == PCAN_DEVICE_NONE, "pcanx");
    CHECK(pcan_parse_device("pcan1x") == PCAN_DEVICE_NONE, "pcan1x");
    CHECK(pcan_parse_device("vLinker") == PCAN_DEVICE_NONE, "vLinker");
    CHECK(pcan_parse_device("OBDLink") == PCAN_DEVICE_NONE, "OBDLink");
    CHECK(pcan_parse_device(NULL) == PCAN_DEVICE_NONE, "NULL");
    CHECK(pcan_is_device("pcan2") && pcan_is_device("pcan9") && !pcan_is_device("vLinker"), "is_device");
    CHECK(pcan_channel_handle(1) == 0x51 && pcan_channel_handle(8) == 0x58, "handles");
}

static void test_bitrates(void) {
    static const struct { const char *s; int bps; uint16_t btr; } ok[] = {
        { "1M", 1000000, 0x0014 }, { "800k", 800000, 0x0016 }, { "500k", 500000, 0x001C },
        { "250k", 250000, 0x011C }, { "125k", 125000, 0x031C }, { "100k", 100000, 0x432F },
        { "95k", 95238, 0xC34E }, { "83k", 83333, 0x852B }, { "50k", 50000, 0x472F },
        { "47k", 47619, 0x1414 }, { "33k", 33333, 0x8B2F }, { "20k", 20000, 0x532F },
        { "10k", 10000, 0x672F }, { "5k", 5000, 0x7F7F }, { "500K", 500000, 0x001C },
        { "1m", 1000000, 0x0014 }, { "83333", 83333, 0x852B }, { "500000", 500000, 0x001C },
    };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
        pcan_bitrate_t b;
        CHECK(pcan_parse_bitrate(ok[i].s, &b) && !b.autodetect && b.bitrate == ok[i].bps && b.btr0btr1 == ok[i].btr,
              "bitrate %s", ok[i].s);
    }
    pcan_bitrate_t b;
    CHECK(pcan_parse_bitrate(NULL, &b) && b.bitrate == 500000 && b.btr0btr1 == 0x001C && !b.autodetect, "default");
    CHECK(pcan_parse_bitrate("auto", &b) && b.autodetect, "auto");
    CHECK(pcan_parse_bitrate("AUTO", &b) && b.autodetect, "AUTO");
    const char *bad[] = { "", "499k", "abc", "500kk", "-500k", "5000001", "0" };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) CHECK(!pcan_parse_bitrate(bad[i], &b), "bad %s", bad[i]);
    CHECK(strcmp(pcan_bitrate_name(500000), "500k") == 0 && strcmp(pcan_bitrate_name(1000000), "1M") == 0 &&
          strcmp(pcan_bitrate_name(83333), "83k") == 0 && strcmp(pcan_bitrate_name(1), "?") == 0, "names");
    static const int order[] = { 500000, 250000, 1000000, 125000, 100000, 83333, 50000, 33333 };
    for (int i = 0; i < PCAN_AUTOBAUD_COUNT; i++) CHECK(pcan_autobaud_order[i] == order[i], "autobaud order %d", i);
}

static void test_open_sequence(void) {
    fake_reset();
    pcan_channel_t ch;
    char err[512] = "";
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_OK, "open: %s", err);
    CHECK(F.ncalls >= 4, "calls %d", F.ncalls);
    CHECK(F.calls[0].kind == C_SET && F.calls[0].ch == 0x51 && F.calls[0].param == PCAN_LISTEN_ONLY &&
          F.calls[0].value == PCAN_PARAMETER_ON, "1st: SetValue(LISTEN_ONLY, ON) before init");
    CHECK(F.calls[1].kind == C_GET_LO && F.calls[1].ch == 0x51 && F.calls[1].value == PCAN_PARAMETER_ON,
          "2nd: read back LISTEN_ONLY = ON before init");
    CHECK(F.calls[2].kind == C_INIT && F.calls[2].ch == 0x51 && F.calls[2].btr == 0x001C, "3rd: CAN_Initialize 500k");
    CHECK(F.calls[3].kind == C_GET_LO && F.calls[3].ch == 0x51, "4th: read back LISTEN_ONLY after init");
    CHECK(only_listen_only_on_sets() && every_init_is_safe(), "safe sequence");
    CHECK(ch.open && ch.index == 1 && ch.bitrate == 500000 && ch.rx_fd == -1, "channel state");
    CHECK(strcmp(ch.label, "pcan1 PCAN-USB") == 0 && strcmp(ch.hw_name, "PCAN-USB") == 0, "label %s", ch.label);
    CHECK(ch.has_device_id && ch.device_id == 7, "device id");
    CHECK(count_calls(C_UNINIT) == 0, "no uninit while open");
    pcan_close(&ch);
    CHECK(count_calls(C_UNINIT) == 1 && !ch.open, "close uninitializes once");
    pcan_close(&ch);
    CHECK(count_calls(C_UNINIT) == 1, "second close is ignored");
}

static void test_open_failures(void) {
    pcan_channel_t ch;
    char err[512];

    fake_reset();
    F.set_fail = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_NOT_LISTEN_ONLY, "set failure");
    CHECK(count_calls(C_INIT) == 0, "pre-init set failed: CAN_Initialize never called");
    CHECK(!ch.open && strstr(err, "listen-only"), "err: %s", err);

    fake_reset();
    F.pre_readback_off = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_NOT_LISTEN_ONLY, "pre-init read-back off");
    CHECK(count_calls(C_INIT) == 0 && count_calls(C_UNINIT) == 0 && F.ncalls == 2,
          "set accepted but reads OFF: CAN_Initialize never called (%d calls)", F.ncalls);
    CHECK(!ch.open && strstr(err, "before init"), "err: %s", err);

    fake_reset();
    F.pre_readback_fail = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_NOT_LISTEN_ONLY, "pre-init read-back error");
    CHECK(count_calls(C_INIT) == 0 && F.ncalls == 2, "pre-init GetValue fails: CAN_Initialize never called");
    CHECK(!ch.open && strstr(err, "before init"), "err: %s", err);

    fake_reset();
    F.readback_off = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_NOT_LISTEN_ONLY, "read-back off");
    CHECK(F.ncalls == 5 && F.calls[4].kind == C_UNINIT && F.calls[4].ch == 0x51,
          "read-back OFF after init: CAN_Uninitialize right away (%d calls)", F.ncalls);
    CHECK(!ch.open && strstr(err, "not active"), "err: %s", err);

    fake_reset();
    F.readback_fail = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_NOT_LISTEN_ONLY, "read-back error");
    CHECK(F.ncalls == 5 && F.calls[4].kind == C_UNINIT, "read-back error after init: uninitialized");

    fake_reset();
    F.init_fail = true;
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_FAILED, "init failure");
    CHECK(count_calls(C_GET_LO) == 1 && !ch.open, "no read-back after a failed init (only the one before)");
    CHECK(count_calls(C_UNINIT) == 1, "a failed init is uninitialized anyway");

    fake_reset();
    CHECK(pcan_open(&kApi, 1, 12345, &ch, err, sizeof(err)) == PCAN_OPEN_FAILED && F.ncalls == 0,
          "unsupported bitrate: nothing called");
    CHECK(pcan_open(&kApi, 9, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_FAILED && F.ncalls == 0,
          "bad channel: nothing called");
}

static void test_autobaud(void) {
    fake_reset();
    F.generate = true;
    F.bus_btr = 0x011C;   /* 250k */
    pcan_autobaud_t ab;
    pcan_autobaud_begin(&ab, &kApi, 1, 0.05);
    int steps = 0;
    while (!pcan_autobaud_step(&ab, 0.02) && steps < 1000) steps++;
    CHECK(ab.done && ab.bitrate == 250000 && !ab.aborted, "detected %d", ab.bitrate);
    CHECK(ab.ntried == 2 && ab.tried[0].bitrate == 500000 && ab.tried[0].frames == 0 &&
          ab.tried[1].bitrate == 250000 && ab.tried[1].frames >= PCAN_AUTOBAUD_MIN_FRAMES &&
          ab.tried[1].ids >= PCAN_AUTOBAUD_MIN_IDS, "tried %d", ab.ntried);
    CHECK(count_calls(C_INIT) == 2 && count_calls(C_UNINIT) == 2 && count_calls(C_GET_LO) == 4,
          "two full attempts (read-back before and after each init), both closed");
    CHECK(only_listen_only_on_sets() && every_init_is_safe(), "every attempt uses the safe sequence");
    CHECK(!F.initialized[1], "detection leaves the channel closed");
    char report[256];
    pcan_autobaud_report(&ab, report, sizeof(report));
    CHECK(strstr(report, "500k: no frames") && strstr(report, "250k: "), "report: %s", report);

    /* nothing on the bus: all eight tried, each the safe way, none left open */
    fake_reset();
    F.generate = true;
    F.bus_btr = 0;
    pcan_autobaud_begin(&ab, &kApi, 1, 0.01);
    steps = 0;
    while (!pcan_autobaud_step(&ab, 0.05) && steps < 1000) steps++;
    CHECK(ab.done && ab.bitrate == 0 && ab.ntried == PCAN_AUTOBAUD_COUNT, "none: %d tried", ab.ntried);
    CHECK(count_calls(C_INIT) == PCAN_AUTOBAUD_COUNT && count_calls(C_UNINIT) == PCAN_AUTOBAUD_COUNT,
          "eight attempts, eight closes");
    CHECK(only_listen_only_on_sets() && every_init_is_safe(), "safe sequence on every attempt");
    for (int i = 0; i < PCAN_AUTOBAUD_COUNT; i++) {
        uint16_t btr = 0;
        pcan_bitrate_btr(pcan_autobaud_order[i], &btr);
        int seen = 0;
        for (int k = 0; k < F.ncalls; k++) seen += F.calls[k].kind == C_INIT && F.calls[k].btr == btr;
        CHECK(seen == 1, "rate %d tried once", pcan_autobaud_order[i]);
    }

    /* read-back OFF on the first attempt: stop, nothing else is initialized */
    fake_reset();
    F.generate = true;
    F.bus_btr = 0x011C;
    F.readback_off = true;
    pcan_autobaud_begin(&ab, &kApi, 1, 0.05);
    steps = 0;
    while (!pcan_autobaud_step(&ab, 0.02) && steps < 1000) steps++;
    CHECK(ab.done && ab.aborted && ab.bitrate == 0, "aborted");
    CHECK(count_calls(C_INIT) == 1 && count_calls(C_UNINIT) == 1, "one attempt, closed, no retry");

    /* set failure: no init at all */
    fake_reset();
    F.generate = true;
    F.set_fail = true;
    pcan_autobaud_begin(&ab, &kApi, 1, 0.05);
    while (!pcan_autobaud_step(&ab, 0.02)) {
    }
    CHECK(ab.aborted && count_calls(C_INIT) == 0, "set failure: never initialized");

    /* pre-init read-back OFF or failing: no init at all, no other rate tried */
    for (int mode = 0; mode < 2; mode++) {
        fake_reset();
        F.generate = true;
        F.bus_btr = 0x011C;
        if (mode == 0) F.pre_readback_off = true;
        else F.pre_readback_fail = true;
        pcan_autobaud_begin(&ab, &kApi, 1, 0.05);
        steps = 0;
        while (!pcan_autobaud_step(&ab, 0.02) && steps < 1000) steps++;
        CHECK(ab.aborted && ab.bitrate == 0 && ab.ntried == 1 && count_calls(C_INIT) == 0,
              "pre-init read-back %s: never initialized", mode == 0 ? "OFF" : "error");
    }

    /* cancel closes an attempt in progress */
    fake_reset();
    F.generate = true;
    F.bus_btr = 0;
    pcan_autobaud_begin(&ab, &kApi, 1, 1.0);
    pcan_autobaud_step(&ab, 0.01);
    CHECK(F.initialized[1], "attempt open");
    pcan_autobaud_cancel(&ab);
    CHECK(!F.initialized[1] && ab.done && ab.bitrate == 0, "cancel closed it");
}

static void test_resolve_and_enumerate(void) {
    char err[256];
    fake_reset();
    F.condition[1] = PCAN_CHANNEL_OCCUPIED;
    F.condition[2] = PCAN_CHANNEL_AVAILABLE;
    F.condition[4] = PCAN_CHANNEL_PCANVIEW;
    CHECK(pcan_resolve_device(&kApi, "pcan", err, sizeof(err)) == 2, "first available");
    CHECK(pcan_resolve_device(&kApi, "pcan2", err, sizeof(err)) == 2, "pcan2");
    CHECK(pcan_resolve_device(&kApi, "pcan1", err, sizeof(err)) == 0 && strstr(err, "in use"), "occupied: %s", err);
    CHECK(pcan_resolve_device(&kApi, "pcan4", err, sizeof(err)) == 0 && strstr(err, "in use"), "PCAN-View: %s", err);
    CHECK(pcan_resolve_device(&kApi, "pcan3", err, sizeof(err)) == 0 && strstr(err, "not found"), "absent: %s", err);
    CHECK(pcan_resolve_device(&kApi, "pcan9", err, sizeof(err)) == 0 && strstr(err, "pcan1..pcan8"), "bad: %s", err);
    pcan_channel_info_t list[PCAN_MAX_CHANNELS];
    int n = pcan_enumerate(&kApi, list, PCAN_MAX_CHANNELS);
    CHECK(n == 3 && list[0].index == 1 && list[1].index == 2 && list[2].index == 4, "enumerate %d", n);
    CHECK(strcmp(list[1].hw_name, "PCAN-USB") == 0 && list[1].has_device_id, "details");
    CHECK(strcmp(pcan_condition_text(list[0].condition), "occupied") == 0, "condition text");
    CHECK(F.ncalls == 0, "enumeration and resolve never set, init or read back");
    F.condition[1] = PCAN_CHANNEL_OCCUPIED;
    F.condition[2] = PCAN_CHANNEL_UNAVAILABLE;
    F.condition[4] = PCAN_CHANNEL_UNAVAILABLE;
    CHECK(pcan_resolve_device(&kApi, "pcan", err, sizeof(err)) == 0 && strstr(err, "pcan1 is in use"), "%s", err);
}

static void test_timestamps_and_messages(void) {
    pcan_timestamp_t ts = { .millis = 1, .millis_overflow = 0, .micros = 500 };
    CHECK(pcan_timestamp_us(&ts) == 1500, "1.5 ms");
    ts = (pcan_timestamp_t){ .millis = 0xFFFFFFFFu, .millis_overflow = 0, .micros = 999 };
    CHECK(pcan_timestamp_us(&ts) == 4294967295999ull, "max millis");
    ts = (pcan_timestamp_t){ .millis = 5, .millis_overflow = 1, .micros = 7 };
    CHECK(pcan_timestamp_us(&ts) == 7ull + 5000ull + 0x100000000ull * 1000ull, "overflow %llu",
          (unsigned long long)pcan_timestamp_us(&ts));
    ts = (pcan_timestamp_t){ .millis = 0, .millis_overflow = 3, .micros = 0 };
    CHECK(pcan_timestamp_us(&ts) == 3ull * 0x100000000ull * 1000ull, "overflow only");

    pcan_msg_t m = { .id = 0x123, .msgtype = PCAN_MESSAGE_STANDARD, .len = 3, .data = { 1, 2, 3, 4 } };
    can_frame_t f;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_DATA, "data");
    pcan_msg_to_frame(&m, &f);
    CHECK(f.can_id == 0x123 && !f.is_extended && f.dlc == 3 && f.data[2] == 3 && f.data[3] == 0 && !f.hw_timestamp,
          "standard frame");
    m = (pcan_msg_t){ .id = 0x18DAF110, .msgtype = PCAN_MESSAGE_EXTENDED, .len = 8 };
    pcan_msg_to_frame(&m, &f);
    CHECK(f.can_id == 0x18DAF110 && f.is_extended && f.dlc == 8, "extended frame");
    m = (pcan_msg_t){ .id = 0x7F, .msgtype = PCAN_MESSAGE_EXTENDED, .len = 1 };
    pcan_msg_to_frame(&m, &f);
    CHECK(f.can_id == 0x7F && f.is_extended, "29-bit with a small id keeps the flag");
    m = (pcan_msg_t){ .id = 0xFFFFF923, .msgtype = PCAN_MESSAGE_STANDARD, .len = 12 };
    pcan_msg_to_frame(&m, &f);
    CHECK(f.can_id == 0x123 && f.dlc == 8, "11-bit masked, DLC clamped to 8");
    m = (pcan_msg_t){ .id = 0xFFFFFFFF, .msgtype = PCAN_MESSAGE_EXTENDED, .len = 0 };
    pcan_msg_to_frame(&m, &f);
    CHECK(f.can_id == 0x1FFFFFFF && f.dlc == 0, "29-bit masked, DLC 0");

    m.msgtype = PCAN_MESSAGE_RTR;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_RTR, "rtr");
    m.msgtype = PCAN_MESSAGE_RTR | PCAN_MESSAGE_EXTENDED;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_RTR, "ext rtr");
    m.msgtype = PCAN_MESSAGE_STATUS;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_STATUS, "status");
    m.msgtype = PCAN_MESSAGE_ERRFRAME;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_ERROR, "error frame");
    m.msgtype = PCAN_MESSAGE_FD;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_OTHER, "fd");
    m.msgtype = PCAN_MESSAGE_ECHO;
    CHECK(pcan_msg_kind(&m) == PCAN_KIND_OTHER, "echo");

    pcan_counts_t c = { 0 };
    pcan_counts_add(&c, PCAN_KIND_DATA);
    pcan_counts_add(&c, PCAN_KIND_RTR);
    pcan_counts_add(&c, PCAN_KIND_STATUS);
    pcan_counts_add(&c, PCAN_KIND_ERROR);
    pcan_counts_add(&c, PCAN_KIND_OTHER);
    CHECK(c.data == 1 && c.rtr == 1 && c.status == 1 && c.error == 1 && c.other == 1, "counts");

    CHECK(pcan_read_result(PCAN_ERROR_OK) == PCAN_READ_MSG, "ok");
    CHECK(pcan_read_result(PCAN_ERROR_QRCVEMPTY) == PCAN_READ_EMPTY, "empty");
    CHECK(pcan_read_result(PCAN_ERROR_QOVERRUN) == PCAN_READ_MSG, "overrun with a message");
    CHECK(pcan_read_result(PCAN_ERROR_OVERRUN | PCAN_ERROR_BUSLIGHT) == PCAN_READ_MSG, "bus bits with a message");
    CHECK(pcan_read_result(PCAN_ERROR_BUSOFF | PCAN_ERROR_QRCVEMPTY) == PCAN_READ_EMPTY, "bus-off, empty");
    CHECK(pcan_read_result(PCAN_ERROR_ILLHW) == PCAN_READ_ERROR, "ILLHW");
    CHECK(pcan_read_result(PCAN_ERROR_QRCVEMPTY | PCAN_ERROR_NODRIVER) == PCAN_READ_ERROR, "empty + driver error");
    CHECK(pcan_read_result(PCAN_ERROR_INITIALIZE) == PCAN_READ_ERROR, "not initialized");
    CHECK(pcan_read_result(PCAN_ERROR_ILLOPERATION) == PCAN_READ_ERROR, "illegal operation");

    pcan_msg_t st = { .msgtype = PCAN_MESSAGE_STATUS, .len = 4, .data = { 0, 0, 0, PCAN_ERROR_BUSOFF } };
    CHECK(pcan_status_msg_bits(&st) == PCAN_ERROR_BUSOFF, "status bits");
    CHECK(strcmp(pcan_bus_state_text(pcan_bus_bits(pcan_status_msg_bits(&st))), "bus-off") == 0, "bus-off text");
    st.data[3] = 0;
    st.data[1] = 0x04;   /* 0x40000 */
    CHECK(strcmp(pcan_bus_state_text(pcan_bus_bits(pcan_status_msg_bits(&st))), "error passive") == 0, "passive");
    CHECK(strcmp(pcan_bus_state_text(PCAN_ERROR_BUSHEAVY), "error warning") == 0, "warning");
    CHECK(strcmp(pcan_bus_state_text(0), "ok") == 0, "ok text");
}

static void test_clock(void) {
    pcan_clock_t c;
    pcan_clock_reset(&c);
    const int64_t base = 1712560000000000LL;   /* wall µs */
    /* first batch: offsets base+0 and base+100 -> anchor = base (the min) */
    pcan_clock_observe(&c, base + 500, 500);
    pcan_clock_observe(&c, base + 900, 800);
    pcan_clock_end_batch(&c);
    CHECK(c.anchored && c.anchor_us == base, "anchor min(host - hw): %lld", (long long)(c.anchor_us - base));
    CHECK(pcan_clock_map(&c, 800) == base + 800, "map");
    /* a later batch with a smaller offset does not move the anchor */
    pcan_clock_observe(&c, base + 1000, 1500);
    pcan_clock_end_batch(&c);
    CHECK(c.anchor_us == base, "anchor fixed after the first batch");
    int64_t a = pcan_clock_map(&c, 123456), b = pcan_clock_map(&c, 133456);
    CHECK(b - a == 10000, "intervals exact (monotonic mapping)");
    /* drift: host - hw grows by 300 µs over > 2 s */
    for (int i = 0; i < 60; i++) {
        int64_t hw = 2000 + (int64_t)i * 100000;
        pcan_clock_observe(&c, base + hw + 300 + 50, hw);
        pcan_clock_observe(&c, base + hw + 300, hw + 10);
        pcan_clock_end_batch(&c);
    }
    CHECK(c.have_drift && c.drift_us == 290, "drift %lld", (long long)c.drift_us);
    CHECK(c.anchor_us == base, "drift does not move the anchor");
    /* controller clock back by > 1 s: re-anchor on that batch */
    pcan_clock_observe(&c, base + 4000000, 100);
    pcan_clock_observe(&c, base + 4000100, 150);
    pcan_clock_end_batch(&c);
    CHECK(c.reanchors == 1 && c.anchor_us == base + 4000000 - 100, "re-anchored %lld", (long long)(c.anchor_us - base));
    /* empty batch: nothing changes */
    int64_t before = c.anchor_us;
    pcan_clock_end_batch(&c);
    CHECK(c.anchor_us == before, "empty batch");
}

typedef struct {
    can_frame_t frames[64];
    int         count;
} sink_t;

static void sink_fn(const can_frame_t *f, void *ctx) {
    sink_t *s = (sink_t *)ctx;
    if (s->count < 64) s->frames[s->count++] = *f;
}

static void test_frame_source(void) {
    fake_reset();
    pcan_channel_t ch;
    char err[256];
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_OK, "open");
    frame_source_t *src = frame_source_pcan_create(&ch);
    CHECK(src && !ch.open, "source owns the channel");
    if (!src) return;
    CHECK(frame_source_is_pcan(src) && frame_source_has_hw_timestamps(src), "hw timestamps");
    CHECK(frame_source_pcan_channel(src)->bitrate == 500000, "channel kept");

    /* queued before the start: dropped (they still anchor the clock) */
    enqueue(PCAN_ERROR_OK, 0x111, PCAN_MESSAGE_STANDARD, 8, 9999000);
    enqueue(PCAN_ERROR_OK, 0x111, PCAN_MESSAGE_STANDARD, 8, 9999500);
    sink_t s = { .count = 0 };
    CHECK(frame_source_start(src, sink_fn, &s), "start");
    CHECK(s.count == 0, "nothing delivered from before the start");

    enqueue(PCAN_ERROR_OK, 0x0A5, PCAN_MESSAGE_STANDARD, 8, 10000123);
    enqueue(PCAN_ERROR_OK, 0x300, PCAN_MESSAGE_RTR, 0, 10000200);
    enqueue(PCAN_ERROR_OK, 0, PCAN_MESSAGE_STATUS, 4, 10000300);
    enqueue(PCAN_ERROR_OK, 0x1, PCAN_MESSAGE_ERRFRAME, 4, 10000400);
    enqueue(PCAN_ERROR_OK, 0x18DAF110, PCAN_MESSAGE_EXTENDED, 8, 10000999);
    enqueue(PCAN_ERROR_QOVERRUN, 0x0F3, PCAN_MESSAGE_STANDARD, 12, 10001000);
    int64_t before = pcan_wall_us();
    frame_source_pump(src, 0.01);
    int64_t after = pcan_wall_us();
    CHECK(s.count == 3, "three data frames delivered (%d)", s.count);
    if (s.count == 3) {
        CHECK(s.frames[0].can_id == 0x0A5 && !s.frames[0].is_extended && s.frames[0].hw_timestamp, "std frame");
        CHECK(s.frames[1].can_id == 0x18DAF110 && s.frames[1].is_extended, "ext frame");
        CHECK(s.frames[2].can_id == 0x0F3 && s.frames[2].dlc == 8, "overrun-marked frame kept, DLC clamped");
        int64_t t0 = can_frame_time_us(&s.frames[0]), t1 = can_frame_time_us(&s.frames[1]),
                t2 = can_frame_time_us(&s.frames[2]);
        CHECK(t1 - t0 == 876 && t2 - t1 == 1, "controller intervals kept: %lld %lld", (long long)(t1 - t0),
              (long long)(t2 - t1));
        CHECK(t2 <= after && t0 >= before - 2000000, "mapped onto the wall clock");
        CHECK(s.frames[0].timestamp_us < 1000, "us within ms");
    }
    frame_source_pcan_stats_t st;
    CHECK(frame_source_pcan_stats(src, &st), "stats");
    CHECK(st.dropped_at_start == 2 && st.delivered == 3 && st.counts.rtr == 1 && st.counts.status == 1 &&
          st.counts.error == 1 && st.overruns == 1 && st.anchored, "stats: dropped %llu delivered %llu rtr %llu",
          (unsigned long long)st.dropped_at_start, (unsigned long long)st.delivered,
          (unsigned long long)st.counts.rtr);

    /* stop: frames queued meanwhile are dropped at the next start */
    frame_source_stop(src);
    enqueue(PCAN_ERROR_OK, 0x222, PCAN_MESSAGE_STANDARD, 1, 20000000);
    frame_source_pump(src, 0.001);
    CHECK(s.count == 3, "stopped: nothing delivered");
    frame_source_start(src, sink_fn, &s);
    CHECK(s.count == 3, "restart drops what queued while stopped");
    enqueue(PCAN_ERROR_OK, 0x333, PCAN_MESSAGE_STANDARD, 1, 20001000);
    frame_source_pump(src, 0.001);
    CHECK(s.count == 4 && s.frames[3].can_id == 0x333, "delivers again");

    /* unplugged: reads fail -> closed */
    F.empty_sts = PCAN_ERROR_ILLHW;
    frame_source_pump(src, 0.02);
    CHECK(!frame_source_is_open(src), "repeated read errors close the source");
    frame_source_pcan_stats(src, &st);
    CHECK(st.read_errors >= FRAME_SOURCE_PCAN_DEAD_READS, "read errors %llu", (unsigned long long)st.read_errors);
    CHECK(only_listen_only_on_sets() && count_calls(C_INIT) == 1, "the source never sets or initializes");
    int uninit_before = count_calls(C_UNINIT);
    frame_source_free(src);
    CHECK(count_calls(C_UNINIT) == uninit_before + 1, "free uninitializes");
}

static void test_wait_select(void) {
    fake_reset();
    CHECK(pipe(F.pipe_fd) == 0, "pipe");
    fcntl(F.pipe_fd[0], F_SETFL, O_NONBLOCK);
    F.use_pipe = true;
    pcan_channel_t ch;
    char err[256];
    CHECK(pcan_open(&kApi, 1, 500000, &ch, err, sizeof(err)) == PCAN_OPEN_OK && ch.rx_fd == F.pipe_fd[0],
          "receive event descriptor");
    int64_t t0 = pcan_mono_us();
    CHECK(pcan_wait(&ch, 30000), "wait");
    int64_t waited = pcan_mono_us() - t0;
    CHECK(waited >= 25000, "no event: waited %lld us", (long long)waited);
    enqueue(PCAN_ERROR_OK, 0x1, PCAN_MESSAGE_STANDARD, 1, 1);
    t0 = pcan_mono_us();
    CHECK(pcan_wait(&ch, 500000), "wait");
    waited = pcan_mono_us() - t0;
    CHECK(waited < 100000, "event wakes the wait: %lld us", (long long)waited);

    frame_source_t *src = frame_source_pcan_create(&ch);
    sink_t s = { .count = 0 };
    frame_source_start(src, sink_fn, &s);   /* drops the queued one, the pipe drains */
    enqueue(PCAN_ERROR_OK, 0x2, PCAN_MESSAGE_STANDARD, 1, 100);
    t0 = pcan_mono_us();
    frame_source_pump(src, 0.05);
    waited = pcan_mono_us() - t0;
    CHECK(s.count == 1 && s.frames[0].can_id == 0x2, "select path delivers");
    CHECK(waited < 200000, "pump keeps its budget: %lld us", (long long)waited);
    frame_source_free(src);
}

/*******************************************************************************
 * Library version gate
 ******************************************************************************/

static const pcan_api_t *test_loader(char *err, size_t err_len) {
    (void)err;
    (void)err_len;
    return &kApi;
}

static void test_version_gate(void) {
    static const struct { const char *s; bool ok; } cases[] = {
        { "PCBUSB 0.13.0.10", true },  { "PCBUSB 0.13", true },     { "x86_64 0.14.1", true },
        { "PCBUSB 0.130", true },      { "PCBUSB 0.12.2", false },  { "PCBUSB 0.9", false },
        { "PCBUSB 1.0", false },       { "PCBUSB 1.13", false },    { "garbage", false },
        { "", false },                 { "PCBUSB v0.13", false },   { "0.13", false },
        { "PCBUSB 0.13\nmore", true },
    };
    char ver[300], err[512];
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        fake_reset();
        snprintf(F.version, sizeof(F.version), "%s", cases[i].s);
        err[0] = '\0';
        bool ok = pcan_api_check_version(&kApi, ver, sizeof(ver), err, sizeof(err));
        CHECK(ok == cases[i].ok, "version \"%s\": %s (%s)", cases[i].s, ok ? "accepted" : "refused", err);
        if (!ok) {
            CHECK(strstr(err, "0.13") != NULL, "message names the tested version: %s", err);
            if (cases[i].s[0]) CHECK(strstr(err, cases[i].s) != NULL || strchr(cases[i].s, '\n'),
                                     "message names the found string: %s", err);
        }
        CHECK(F.ncalls == 0, "the gate only reads the version");
    }
    fake_reset();
    snprintf(F.version, sizeof(F.version), "PCBUSB 0.13\r\n");
    CHECK(pcan_api_check_version(&kApi, ver, sizeof(ver), err, sizeof(err)) && strcmp(ver, "PCBUSB 0.13") == 0,
          "line end stripped: '%s'", ver);
    fake_reset();
    F.version_fail = true;
    CHECK(!pcan_api_check_version(&kApi, ver, sizeof(ver), err, sizeof(err)) && strstr(err, "cannot read") &&
          strstr(err, "0.13"), "read failure refused: %s", err);
    fake_reset();
    F.version_unterminated = true;
    CHECK(!pcan_api_check_version(&kApi, ver, sizeof(ver), err, sizeof(err)) && strlen(ver) == PCAN_VERSION_MAX,
          "unterminated 256 bytes: terminator forced, refused");

    /* through the loader: once per load, cached; refused before any channel use */
    fake_reset();
    pcan_api_set_test_loader(test_loader);
    const pcan_api_t *api = pcan_api_load(err, sizeof(err));
    CHECK(api == &kApi && strcmp(pcan_api_version(), "PCBUSB 0.13.0.10") == 0, "loaded: %s", pcan_api_version());
    CHECK(pcan_api_load(err, sizeof(err)) == &kApi && F.version_reads == 1, "version read once (%d)", F.version_reads);
    fake_reset();
    snprintf(F.version, sizeof(F.version), "PCBUSB 0.12.2");
    pcan_api_set_test_loader(test_loader);   /* a new load */
    err[0] = '\0';
    CHECK(pcan_api_load(err, sizeof(err)) == NULL && strstr(err, "PCBUSB 0.12.2") && strstr(err, "0.13"),
          "old library refused: %s", err);
    CHECK(pcan_api_load(err, sizeof(err)) == NULL && F.version_reads == 1, "refusal cached");
    CHECK(F.ncalls == 0, "refused library: nothing set, initialized or read back");
    pcan_api_set_test_loader(NULL);
}

/*******************************************************************************
 * 6-column TSV
 ******************************************************************************/

static void test_tsv(void) {
    can_frame_t f;
    char row5[] = "1712560212345\t0x0A5\t0\t2\tABCD";
    CHECK(frame_tsv_parse_row(row5, &f) && !f.hw_timestamp && f.timestamp_us == 0 && f.timestamp_ms == 1712560212345LL,
          "5 columns");
    char row6[] = "1712560212345\t0x0A5\t0\t2\tABCD\t1712560212345678";
    CHECK(frame_tsv_parse_row(row6, &f) && f.hw_timestamp && f.timestamp_us == 678 &&
          f.timestamp_ms == 1712560212345LL && f.data[1] == 0xCD, "6 columns");
    char row6_0[] = "1712560212345\t0x0A5\t0\t2\tABCD\t1712560212345000";
    CHECK(frame_tsv_parse_row(row6_0, &f) && f.hw_timestamp && f.timestamp_us == 0, "6 columns, .000");
    char mismatch[] = "1712560212345\t0x0A5\t0\t2\tABCD\t1712560212346000";
    CHECK(!frame_tsv_parse_row(mismatch, &f), "6th column in another ms rejected");
    char before[] = "1712560212345\t0x0A5\t0\t2\tABCD\t1712560212344999";
    CHECK(!frame_tsv_parse_row(before, &f), "6th column in the ms before rejected");
    char empty6[] = "1712560212345\t0x0A5\t0\t2\tABCD\t";
    CHECK(!frame_tsv_parse_row(empty6, &f), "empty 6th column rejected");
    char junk6[] = "1712560212345\t0x0A5\t0\t2\tABCD\t17125602123456x";
    CHECK(!frame_tsv_parse_row(junk6, &f), "junk 6th column rejected");
    char row7[] = "1712560212345\t0x0A5\t0\t2\tABCD\t1712560212345678\t1";
    CHECK(!frame_tsv_parse_row(row7, &f), "7 columns rejected");
    char dlc0[] = "5\t0x10\t0\t0\t\t5123";
    CHECK(frame_tsv_parse_row(dlc0, &f) && f.dlc == 0 && f.timestamp_us == 123, "dlc 0 with 6 columns");

    char path[1024];
    const char *tmp = getenv("TMPDIR");
    snprintf(path, sizeof(path), "%s/pcan_test_XXXXXX", tmp && tmp[0] ? tmp : "/tmp");
    int fd = mkstemp(path);
    CHECK(fd >= 0, "temp file");
    if (fd < 0) return;
    FILE *out = fdopen(fd, "w");
    frame_tsv_write_header_ex(out, true);
    can_frame_t frames[3];
    memset(frames, 0, sizeof(frames));
    frames[0].can_id = 0x0A5;
    frames[0].dlc = 8;
    for (int i = 0; i < 8; i++) frames[0].data[i] = (uint8_t)(i * 17);
    can_frame_set_hw_time_us(&frames[0], 1712560212345678LL);
    frames[1].can_id = 0x18DAF110;
    frames[1].is_extended = true;
    frames[1].dlc = 0;
    can_frame_set_hw_time_us(&frames[1], 1712560212345999LL);
    frames[2].can_id = 0x316;   /* host-stamped: 5 columns */
    frames[2].dlc = 1;
    frames[2].data[0] = 0x0F;
    frames[2].timestamp_ms = 1712560212400LL;
    for (int i = 0; i < 3; i++) frame_tsv_write_row(out, &frames[i]);
    fclose(out);

    FILE *in = fopen(path, "r");
    char line[256];
    CHECK(fgets(line, sizeof(line), in) && strcmp(line, "timestamp_ms\tcan_id\tis_extended\tdlc\tdata_hex\ttimestamp_us\n") == 0,
          "6-column header");
    CHECK(fgets(line, sizeof(line), in) &&
          strcmp(line, "1712560212345\t0xA5\t0\t8\t0011223344556677\t1712560212345678\n") == 0, "hw row: %s", line);
    CHECK(fgets(line, sizeof(line), in) && strcmp(line, "1712560212345\t0x18DAF110\t1\t0\t\t1712560212345999\n") == 0,
          "hw dlc 0 row: %s", line);
    CHECK(fgets(line, sizeof(line), in) && strcmp(line, "1712560212400\t0x316\t0\t1\t0F\n") == 0, "host row: %s", line);
    fclose(in);

    frame_list_t list;
    CHECK(frame_tsv_load(path, "pcan_test", &list) == 1 && list.count == 3 && list.malformed == 0, "load");
    for (int i = 0; i < 3 && list.count == 3; i++) {
        CHECK(memcmp(&list.frames[i], &frames[i], sizeof(can_frame_t)) == 0, "round trip frame %d", i);
    }
    frame_list_free(&list);

    out = fopen(path, "w");
    frame_tsv_write_header(out);
    fprintf(out, "1712560212345\t0x0A5\t0\t1\t01\n");
    fclose(out);
    CHECK(frame_tsv_load(path, "pcan_test", &list) == 1 && list.count == 1 && !list.frames[0].hw_timestamp,
          "5-column capture still loads");
    frame_list_free(&list);
    unlink(path);

    frame_source_t *rsrc = frame_source_replay_create(frames, 3, false);
    CHECK(!frame_source_has_hw_timestamps(rsrc), "replay source: no hw timestamp column by default");
    frame_source_free(rsrc);
}

int main(void) {
    test_symbols();
    test_layouts();
    test_parse_device();
    test_bitrates();
    test_open_sequence();
    test_open_failures();
    test_autobaud();
    test_resolve_and_enumerate();
    test_timestamps_and_messages();
    test_clock();
    test_frame_source();
    test_wait_select();
    test_version_gate();
    test_tsv();
    fake_reset();
    if (g_failures) {
        fprintf(stderr, "%d of %d checks failed\n", g_failures, g_checks);
        return 1;
    }
    printf("pcan_test: %d checks passed\n", g_checks);
    return 0;
}
