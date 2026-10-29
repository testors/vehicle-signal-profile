/*
 * Fake PCBUSB for tests: linked only into can_tools_fake. A constructor
 * installs a test loader (pcan_api_set_test_loader), so this build never
 * dlopens the real library and cannot reach a PCAN-USB. The calls that
 * matter for the listen-only proof are logged to pcan_calls.txt in the
 * current directory (only when FAKE_PCAN is set).
 *
 * Environment:
 *   FAKE_PCAN                 unset: no channel (nothing logged); "missing": the
 *                             library is "not installed"; else the plugged-in
 *                             channels, comma-separated, e.g. "1" or "1,3"
 *   FAKE_PCAN_NAME            hardware name (default "PCAN-USB")
 *   FAKE_PCAN_OCCUPIED        a channel another application holds
 *   FAKE_PCAN_BITRATE         bus bitrate in bit/s (default 500000): frames only
 *                             reach a channel initialized at that rate
 *   FAKE_PCAN_VERSION         PCAN_CHANNEL_VERSION string, '_' for spaces
 *                             (default "PCBUSB 0.13.0.10 (fake)"); "fail": the read fails
 *   FAKE_PCAN_LO_SET_FAIL=1   setting PCAN_LISTEN_ONLY fails
 *   FAKE_PCAN_LO_PRE=off      the set is accepted, but it reads back OFF before init
 *   FAKE_PCAN_LO_PRE=fail     reading PCAN_LISTEN_ONLY before init fails
 *   FAKE_PCAN_LO_READBACK=0   PCAN_LISTEN_ONLY reads back OFF after init
 *   FAKE_PCAN_UNPLUG_AFTER_MS reads fail (ILLHW) this long after the first init
 *
 * Log lines: LOAD, VERSION, SETVALUE <ch> LISTEN_ONLY=<v> (any other parameter
 * is logged as SETVALUE <ch> OTHER), INIT <ch> btr=<btr> listen_only=<v>,
 * GETVALUE <ch> LISTEN_ONLY -> <v|error>, UNINIT <ch>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "pcan.h"

static FILE *fake_log(void) {
    static FILE *f = NULL;
    if (!getenv("FAKE_PCAN")) return NULL;
    if (!f) {
        f = fopen("pcan_calls.txt", "w");
        if (f) setvbuf(f, NULL, _IOLBF, 0);
    }
    return f;
}

#define LOG(...)                                   \
    do {                                           \
        FILE *lf_ = fake_log();                    \
        if (lf_) fprintf(lf_, __VA_ARGS__);        \
    } while (0)

static int64_t wall_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

typedef struct {
    uint32_t id;
    uint8_t  msgtype;
    uint8_t  dlc;
    int      period_ms;
    int64_t  next_us;
    uint32_t n;
} fake_stream_t;

typedef struct {
    bool          initialized;
    uint32_t      lo_pending;    /* PCAN_LISTEN_ONLY value; reset to OFF by uninit */
    uint16_t      btr;
    int64_t       init_us;
    fake_stream_t streams[8];
    int           nstreams;
    bool          status_sent;
} fake_channel_t;

static fake_channel_t g_ch[PCAN_MAX_CHANNELS + 1];
static int64_t g_first_init_us = 0;

static int index_of(pcan_handle_t h) {
    int idx = (int)h - (int)PCAN_USBBUS1 + 1;
    return (idx >= 1 && idx <= PCAN_MAX_CHANNELS) ? idx : 0;
}

static bool plugged(int idx) {
    const char *v = getenv("FAKE_PCAN");
    if (!v || !idx || strcmp(v, "missing") == 0) return false;
    char buf[64];
    snprintf(buf, sizeof(buf), "%s", v);
    for (char *t = strtok(buf, ","); t; t = strtok(NULL, ",")) {
        if (atoi(t) == idx) return true;
    }
    return false;
}

static bool unplugged_now(void) {
    const char *after = getenv("FAKE_PCAN_UNPLUG_AFTER_MS");
    return after && g_first_init_us && wall_us() - g_first_init_us >= atoll(after) * 1000;
}

static uint16_t bus_btr(void) {
    const char *v = getenv("FAKE_PCAN_BITRATE");
    uint16_t btr = 0x001C;
    if (v && !pcan_bitrate_btr(atoi(v), &btr)) btr = 0;
    return btr;
}

static void setup_streams(fake_channel_t *c) {
    c->nstreams = 0;
    c->streams[c->nstreams++] = (fake_stream_t){ 0x0A5, PCAN_MESSAGE_STANDARD, 8, 10, 0, 0 };
    c->streams[c->nstreams++] = (fake_stream_t){ 0x0F3, PCAN_MESSAGE_STANDARD, 8, 10, 0, 0 };
    c->streams[c->nstreams++] = (fake_stream_t){ 0x1A0, PCAN_MESSAGE_STANDARD, 8, 20, 0, 0 };
    c->streams[c->nstreams++] = (fake_stream_t){ 0x0D9, PCAN_MESSAGE_STANDARD, 6, 50, 0, 0 };
    c->streams[c->nstreams++] = (fake_stream_t){ 0x18DAF110, PCAN_MESSAGE_EXTENDED, 8, 100, 0, 0 };
    c->streams[c->nstreams++] = (fake_stream_t){ 0x300, PCAN_MESSAGE_RTR, 0, 500, 0, 0 };
    for (int i = 0; i < c->nstreams; i++) c->streams[i].next_us = c->init_us + 1000 * (i + 1);
    c->status_sent = false;
}

/* Controller clock: starts at 5 s when the channel is initialized. */
static void stamp(const fake_channel_t *c, int64_t at_us, pcan_timestamp_t *ts) {
    uint64_t hw = (uint64_t)(at_us - c->init_us) + 5000000ull;
    uint64_t ms = hw / 1000;
    ts->micros = (uint16_t)(hw % 1000);
    ts->millis = (uint32_t)(ms & 0xFFFFFFFFull);
    ts->millis_overflow = (uint16_t)(ms >> 32);
}

static pcan_status_t fake_initialize(pcan_handle_t ch, uint16_t btr, uint8_t hw_type, uint32_t io_port,
                                     uint16_t interrupt) {
    (void)hw_type;
    (void)io_port;
    (void)interrupt;
    int idx = index_of(ch);
    LOG("INIT 0x%02X btr=0x%04X listen_only=%u\n", ch, btr, idx ? g_ch[idx].lo_pending : 0);
    if (!plugged(idx)) return PCAN_ERROR_ILLHW;
    fake_channel_t *c = &g_ch[idx];
    if (c->initialized) return PCAN_ERROR_INITIALIZE;
    c->initialized = true;
    c->btr = btr;
    c->init_us = wall_us();
    if (!g_first_init_us) g_first_init_us = c->init_us;
    setup_streams(c);
    return PCAN_ERROR_OK;
}

static pcan_status_t fake_uninitialize(pcan_handle_t ch) {
    int idx = index_of(ch);
    LOG("UNINIT 0x%02X\n", ch);
    if (!idx) return PCAN_ERROR_ILLCLIENT;
    g_ch[idx].initialized = false;
    g_ch[idx].lo_pending = PCAN_PARAMETER_OFF;   /* the next init must set it again */
    return PCAN_ERROR_OK;
}

static pcan_status_t fake_get_status(pcan_handle_t ch) {
    int idx = index_of(ch);
    if (!idx || !g_ch[idx].initialized) return PCAN_ERROR_INITIALIZE;
    if (unplugged_now()) return PCAN_ERROR_ILLHW;
    return PCAN_ERROR_OK;
}

static pcan_status_t fake_read(pcan_handle_t ch, pcan_msg_t *msg, pcan_timestamp_t *ts) {
    int idx = index_of(ch);
    if (!idx || !g_ch[idx].initialized) return PCAN_ERROR_INITIALIZE;
    if (unplugged_now()) return PCAN_ERROR_ILLHW;
    fake_channel_t *c = &g_ch[idx];
    if (c->btr != bus_btr()) return PCAN_ERROR_QRCVEMPTY;   /* wrong bitrate: nothing decodes */
    int64_t now = wall_us();
    memset(msg, 0, sizeof(*msg));
    if (!c->status_sent && now - c->init_us >= 1000000) {
        c->status_sent = true;
        msg->msgtype = PCAN_MESSAGE_STATUS;
        msg->len = 4;
        stamp(c, now, ts);
        return PCAN_ERROR_OK;
    }
    fake_stream_t *due = NULL;
    for (int i = 0; i < c->nstreams; i++) {
        if (c->streams[i].next_us <= now && (!due || c->streams[i].next_us < due->next_us)) due = &c->streams[i];
    }
    if (!due) return PCAN_ERROR_QRCVEMPTY;
    msg->id = due->id;
    msg->msgtype = due->msgtype;
    msg->len = due->dlc;
    msg->data[0] = (uint8_t)(due->n & 0x0F);
    msg->data[2] = (uint8_t)((due->n * 7) & 0xFF);
    msg->data[4] = (uint8_t)(0x40 + (due->n % 16));
    msg->data[5] = (uint8_t)(due->n & 0xFF);
    stamp(c, due->next_us, ts);
    due->n++;
    due->next_us += (int64_t)due->period_ms * 1000;
    return PCAN_ERROR_OK;
}

static pcan_status_t fake_get_value(pcan_handle_t ch, uint8_t param, void *buf, uint32_t len) {
    int idx = index_of(ch);
    if (!idx) return PCAN_ERROR_ILLCLIENT;
    fake_channel_t *c = &g_ch[idx];
    switch (param) {
        case PCAN_CHANNEL_VERSION: {   /* pre-init, answered with or without a device */
            LOG("VERSION\n");
            const char *v = getenv("FAKE_PCAN_VERSION");
            if (v && strcmp(v, "fail") == 0) return PCAN_ERROR_ILLPARAMTYPE;
            char text[PCAN_VERSION_MAX];
            snprintf(text, sizeof(text), "%s", v ? v : "PCBUSB 0.13.0.10 (fake)");
            for (char *p = text; *p; p++) {
                if (*p == '_') *p = ' ';
            }
            snprintf((char *)buf, len, "%s", text);
            return PCAN_ERROR_OK;
        }
        case PCAN_CHANNEL_CONDITION: {
            if (len < sizeof(uint32_t)) return PCAN_ERROR_ILLPARAMVAL;
            uint32_t v = PCAN_CHANNEL_UNAVAILABLE;
            const char *occ = getenv("FAKE_PCAN_OCCUPIED");
            if (plugged(idx)) {
                v = (c->initialized || (occ && atoi(occ) == idx)) ? PCAN_CHANNEL_OCCUPIED : PCAN_CHANNEL_AVAILABLE;
            }
            memcpy(buf, &v, sizeof(v));
            return PCAN_ERROR_OK;
        }
        case PCAN_LISTEN_ONLY: {
            if (len < sizeof(uint32_t)) return PCAN_ERROR_ILLPARAMVAL;
            uint32_t v = c->lo_pending;
            if (!c->initialized) {
                const char *pre = getenv("FAKE_PCAN_LO_PRE");
                if (pre && strcmp(pre, "fail") == 0) {
                    LOG("GETVALUE 0x%02X LISTEN_ONLY -> error\n", ch);
                    return PCAN_ERROR_ILLOPERATION;
                }
                if (pre && strcmp(pre, "off") == 0) v = PCAN_PARAMETER_OFF;
            } else {
                const char *rb = getenv("FAKE_PCAN_LO_READBACK");
                if (rb && strcmp(rb, "0") == 0) v = PCAN_PARAMETER_OFF;
            }
            LOG("GETVALUE 0x%02X LISTEN_ONLY -> %u\n", ch, v);
            memcpy(buf, &v, sizeof(v));
            return PCAN_ERROR_OK;
        }
        case PCAN_HARDWARE_NAME: {
            if (!plugged(idx)) return PCAN_ERROR_ILLHW;
            const char *name = getenv("FAKE_PCAN_NAME");
            snprintf((char *)buf, len, "%s", name ? name : "PCAN-USB");
            return PCAN_ERROR_OK;
        }
        case PCAN_DEVICE_ID: {
            if (!plugged(idx) || len < sizeof(uint32_t)) return PCAN_ERROR_ILLHW;
            uint32_t v = (uint32_t)idx;
            memcpy(buf, &v, sizeof(v));
            return PCAN_ERROR_OK;
        }
        case PCAN_FIRMWARE_VERSION:
            if (!plugged(idx)) return PCAN_ERROR_ILLHW;
            snprintf((char *)buf, len, "8.4.0");
            return PCAN_ERROR_OK;
        default:
            return PCAN_ERROR_ILLPARAMTYPE;   /* RECEIVE_EVENT too: callers poll */
    }
}

static pcan_status_t fake_set_value(pcan_handle_t ch, uint8_t param, void *buf, uint32_t len) {
    int idx = index_of(ch);
    if (param != PCAN_LISTEN_ONLY) {
        LOG("SETVALUE 0x%02X OTHER param=0x%02X\n", ch, param);
        return PCAN_ERROR_ILLPARAMTYPE;
    }
    uint32_t v = 0xFFFFFFFFu;
    if (len >= sizeof(v)) memcpy(&v, buf, sizeof(v));
    LOG("SETVALUE 0x%02X LISTEN_ONLY=%u\n", ch, v);
    if (!idx) return PCAN_ERROR_ILLCLIENT;
    const char *fail = getenv("FAKE_PCAN_LO_SET_FAIL");
    if (fail && strcmp(fail, "1") == 0) return PCAN_ERROR_ILLOPERATION;
    g_ch[idx].lo_pending = v;
    return PCAN_ERROR_OK;
}

static pcan_status_t fake_get_error_text(pcan_status_t error, uint16_t language, char *buf) {
    (void)language;
    snprintf(buf, 256, "fake PCAN error 0x%X", error);
    return PCAN_ERROR_OK;
}

static const pcan_api_t kFakeApi = {
    .initialize = fake_initialize,
    .uninitialize = fake_uninitialize,
    .get_status = fake_get_status,
    .read = fake_read,
    .get_value = fake_get_value,
    .set_value = fake_set_value,
    .get_error_text = fake_get_error_text,
};

static const pcan_api_t *fake_loader(char *err, size_t err_len) {
    LOG("LOAD\n");
    const char *v = getenv("FAKE_PCAN");
    if (v && strcmp(v, "missing") == 0) {
        if (err && err_len) {
            snprintf(err, err_len, "PCAN driver library not found (fake): install PCBUSB from %s", PCAN_INSTALL_URL);
        }
        return NULL;
    }
    return &kFakeApi;
}

__attribute__((constructor)) static void fake_pcan_install(void) {
    pcan_api_set_test_loader(fake_loader);
}
