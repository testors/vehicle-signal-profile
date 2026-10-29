/**
 * @file pcan.c
 * @brief PEAK PCAN-USB through the PCBUSB library, receive only (Pure C)
 *
 * See pcan.h for the listen-only rules. The library is loaded with dlopen
 * from absolute paths only; only the receive-side symbols in kSymbols are
 * ever looked up, and the library is used only after the version gate.
 */

#include "pcan.h"

#include <ctype.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/*******************************************************************************
 * Loader
 ******************************************************************************/

/* Receive side only. There is deliberately no CAN_Write / CAN_WriteFD /
 * CAN_Reset here: pcan_api_t has no member to hold them. */
static const char *const kSymbols[] = {
    "CAN_Initialize",
    "CAN_Uninitialize",
    "CAN_GetStatus",
    "CAN_Read",
    "CAN_GetValue",
    "CAN_SetValue",
    "CAN_GetErrorText",
};

/* Absolute paths only: a bare name would also search DYLD_* paths and the
 * current directory. */
static const char *const kLibPaths[] = {
    "/usr/local/lib/libPCBUSB.dylib",
    "/opt/homebrew/lib/libPCBUSB.dylib",
};
#define NLIBPATHS (sizeof(kLibPaths) / sizeof(kLibPaths[0]))

static pcan_api_t        g_api;
static pcan_api_loader_fn g_test_loader = NULL;
static bool              g_tried = false;
static bool              g_loaded = false;
static char              g_path[512] = "";
static char              g_error[768] = "";
static char              g_version[PCAN_VERSION_MAX + 1] = "";
/* test loader: the version gate result of the table it returned last */
static const pcan_api_t *g_test_api = NULL;
static bool              g_test_ok = false;
static char              g_test_error[768] = "";

const char *const *pcan_api_symbol_names(int *count) {
    if (count) *count = (int)(sizeof(kSymbols) / sizeof(kSymbols[0]));
    return kSymbols;
}

void pcan_api_set_test_loader(pcan_api_loader_fn loader) {
    g_test_loader = loader;
    g_test_api = NULL;     /* gate the next table again */
    g_test_ok = false;
    g_version[0] = '\0';
}

const char *pcan_api_path(void) {
    if (g_test_loader) return "(test loader)";
    return g_path;
}

const char *pcan_api_version(void) {
    return g_version;
}

bool pcan_api_check_version(const pcan_api_t *api, char *version, size_t version_len,
                            char *err, size_t err_len) {
    char str[PCAN_VERSION_MAX + 1];
    memset(str, 0, sizeof(str));
    if (version && version_len) version[0] = '\0';
    pcan_status_t sts = api->get_value(PCAN_USBBUS1, PCAN_CHANNEL_VERSION, str, PCAN_VERSION_MAX);
    str[PCAN_VERSION_MAX] = '\0';   /* the library need not terminate it */
    if (sts != PCAN_ERROR_OK) {
        char text[300];
        pcan_status_text(api, sts, text, sizeof(text));
        snprintf(err, err_len, "PCAN: cannot read the PCBUSB version (PCAN_CHANNEL_VERSION: %s); "
                 "can_tools is tested with PCBUSB %s: library not used", text, PCAN_TESTED_VERSION);
        return false;
    }
    char *eol = strpbrk(str, "\r\n");
    if (eol) *eol = '\0';
    if (version && version_len) snprintf(version, version_len, "%s", str);
    char name[PCAN_VERSION_MAX + 1];
    unsigned int major = 0, minor = 0;
    if (sscanf(str, "%256s %u.%u", name, &major, &minor) != 3) {
        snprintf(err, err_len, "PCAN: unrecognized PCBUSB version \"%.120s\" (tested with %s): library not used",
                 str, PCAN_TESTED_VERSION);
        return false;
    }
    if (major != PCAN_TESTED_MAJOR || minor < PCAN_TESTED_MINOR) {
        snprintf(err, err_len, "PCAN: PCBUSB version \"%.120s\" (%u.%u) is not supported: can_tools is tested "
                 "with %s and needs %s or a later 0.x; library not used",
                 str, major, minor, PCAN_TESTED_VERSION, PCAN_TESTED_VERSION);
        return false;
    }
    return true;
}

/* Copy a dlsym result into a function pointer member without a cast between
 * object and function pointers. */
static bool resolve(void *lib, int sym_index, void *member, size_t member_size) {
    void *sym = dlsym(lib, kSymbols[sym_index]);
    if (!sym || member_size != sizeof(sym)) return false;
    memcpy(member, &sym, sizeof(sym));
    return true;
}

static bool load_from(const char *path, char *why, size_t why_len) {
    void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!lib) {
        const char *e = dlerror();
        snprintf(why, why_len, "%s", e ? e : "dlopen failed");
        return false;
    }
    pcan_api_t api;
    memset(&api, 0, sizeof(api));
    bool ok = resolve(lib, 0, &api.initialize, sizeof(api.initialize)) &&
              resolve(lib, 1, &api.uninitialize, sizeof(api.uninitialize)) &&
              resolve(lib, 2, &api.get_status, sizeof(api.get_status)) &&
              resolve(lib, 3, &api.read, sizeof(api.read)) &&
              resolve(lib, 4, &api.get_value, sizeof(api.get_value)) &&
              resolve(lib, 5, &api.set_value, sizeof(api.set_value)) &&
              resolve(lib, 6, &api.get_error_text, sizeof(api.get_error_text));
    if (!ok) {
        snprintf(why, why_len, "%s: a CAN_* symbol is missing (not PCBUSB?)", path);
        dlclose(lib);
        return false;
    }
    g_api = api;   /* the library stays loaded for the life of the process */
    snprintf(g_path, sizeof(g_path), "%s", path);
    return true;
}

const pcan_api_t *pcan_api_load(char *err, size_t err_len) {
    if (g_test_loader) {
        const pcan_api_t *api = g_test_loader(err, err_len);
        if (!api) return NULL;
        if (api != g_test_api) {   /* once per table, like once per library load */
            g_test_api = api;
            g_test_ok = pcan_api_check_version(api, g_version, sizeof(g_version), g_test_error,
                                               sizeof(g_test_error));
        }
        if (!g_test_ok) {
            if (err && err_len) snprintf(err, err_len, "%s", g_test_error);
            return NULL;
        }
        return api;
    }
    if (!g_tried) {
        g_tried = true;
        char why[NLIBPATHS][256];
        for (size_t i = 0; i < NLIBPATHS && !g_loaded; i++) {
            why[i][0] = '\0';
            g_loaded = load_from(kLibPaths[i], why[i], sizeof(why[i]));
        }
        if (!g_loaded) {
            int n = snprintf(g_error, sizeof(g_error),
                             "PCAN driver library not found (libPCBUSB.dylib in /usr/local/lib or "
                             "/opt/homebrew/lib): install PCBUSB %s from %s", PCAN_TESTED_VERSION, PCAN_INSTALL_URL);
            /* a file that is there but does not load says why */
            for (size_t i = 0; i < NLIBPATHS && n > 0 && (size_t)n < sizeof(g_error); i++) {
                if (access(kLibPaths[i], F_OK) == 0 && why[i][0]) {
                    n += snprintf(g_error + n, sizeof(g_error) - (size_t)n, " [%s]", why[i]);
                }
            }
        } else if (!pcan_api_check_version(&g_api, g_version, sizeof(g_version), g_error, sizeof(g_error))) {
            /* before any channel is touched; the library stays mapped but is never used */
            g_loaded = false;
        }
    }
    if (!g_loaded) {
        if (err && err_len) snprintf(err, err_len, "%s", g_error);
        return NULL;
    }
    return &g_api;
}

/*******************************************************************************
 * Status text
 ******************************************************************************/

static const struct {
    pcan_status_t code;
    const char   *name;
} kStatusNames[] = {
    { PCAN_ERROR_OK, "OK" },
    { PCAN_ERROR_XMTFULL, "XMTFULL" },
    { PCAN_ERROR_OVERRUN, "OVERRUN" },
    { PCAN_ERROR_BUSLIGHT, "BUSLIGHT" },
    { PCAN_ERROR_BUSHEAVY, "BUSHEAVY" },
    { PCAN_ERROR_BUSOFF, "BUSOFF" },
    { PCAN_ERROR_QRCVEMPTY, "QRCVEMPTY" },
    { PCAN_ERROR_QOVERRUN, "QOVERRUN" },
    { PCAN_ERROR_QXMTFULL, "QXMTFULL" },
    { PCAN_ERROR_REGTEST, "REGTEST" },
    { PCAN_ERROR_NODRIVER, "NODRIVER" },
    { PCAN_ERROR_HWINUSE, "HWINUSE" },
    { PCAN_ERROR_NETINUSE, "NETINUSE" },
    { PCAN_ERROR_ILLHW, "ILLHW" },
    { PCAN_ERROR_ILLNET, "ILLNET" },
    { PCAN_ERROR_ILLCLIENT, "ILLCLIENT" },
    { PCAN_ERROR_RESOURCE, "RESOURCE" },
    { PCAN_ERROR_ILLPARAMTYPE, "ILLPARAMTYPE" },
    { PCAN_ERROR_ILLPARAMVAL, "ILLPARAMVAL" },
    { PCAN_ERROR_UNKNOWN, "UNKNOWN" },
    { PCAN_ERROR_ILLDATA, "ILLDATA" },
    { PCAN_ERROR_BUSPASSIVE, "BUSPASSIVE" },
    { PCAN_ERROR_ILLMODE, "ILLMODE" },
    { PCAN_ERROR_CAUTION, "CAUTION" },
    { PCAN_ERROR_INITIALIZE, "INITIALIZE" },
    { PCAN_ERROR_ILLOPERATION, "ILLOPERATION" },
};

void pcan_status_text(const pcan_api_t *api, pcan_status_t sts, char *buf, size_t len) {
    if (!buf || !len) return;
    const char *name = NULL;
    for (size_t i = 0; i < sizeof(kStatusNames) / sizeof(kStatusNames[0]); i++) {
        if (kStatusNames[i].code == sts) name = kStatusNames[i].name;
    }
    char text[256] = "";
    if (api && api->get_error_text && api->get_error_text(sts, 0x09, text) == PCAN_ERROR_OK) {
        text[sizeof(text) - 1] = '\0';
    } else {
        text[0] = '\0';
    }
    if (name && text[0]) snprintf(buf, len, "PCAN_ERROR_%s (0x%05X): %s", name, sts, text);
    else if (name) snprintf(buf, len, "PCAN_ERROR_%s (0x%05X)", name, sts);
    else if (text[0]) snprintf(buf, len, "PCAN status 0x%05X: %s", sts, text);
    else snprintf(buf, len, "PCAN status 0x%05X", sts);
}

/*******************************************************************************
 * Device names and bitrates
 ******************************************************************************/

int pcan_parse_device(const char *s) {
    if (!s || strncasecmp(s, "pcan", 4) != 0) return PCAN_DEVICE_NONE;
    const char *p = s + 4;
    if (*p == '\0') return 0;
    for (const char *q = p; *q; q++) {
        if (!isdigit((unsigned char)*q)) return PCAN_DEVICE_NONE;
    }
    if (strlen(p) > 3) return PCAN_DEVICE_BAD;
    int n = atoi(p);
    if (n < 1 || n > PCAN_MAX_CHANNELS) return PCAN_DEVICE_BAD;
    return n;
}

bool pcan_is_device(const char *s) {
    return pcan_parse_device(s) != PCAN_DEVICE_NONE;
}

pcan_handle_t pcan_channel_handle(int index) {
    return (pcan_handle_t)(PCAN_USBBUS1 + (index - 1));
}

/* PCAN_BAUD_* BTR0BTR1 values (SJA1000 at 8 MHz, as PCAN-Basic defines them) */
static const struct {
    const char *name;
    int         bitrate;
    uint16_t    btr;
} kRates[] = {
    { "1M", 1000000, 0x0014 },
    { "800k", 800000, 0x0016 },
    { "500k", 500000, 0x001C },
    { "250k", 250000, 0x011C },
    { "125k", 125000, 0x031C },
    { "100k", 100000, 0x432F },
    { "95k", 95238, 0xC34E },
    { "83k", 83333, 0x852B },
    { "50k", 50000, 0x472F },
    { "47k", 47619, 0x1414 },
    { "33k", 33333, 0x8B2F },
    { "20k", 20000, 0x532F },
    { "10k", 10000, 0x672F },
    { "5k", 5000, 0x7F7F },
};
#define NRATES ((int)(sizeof(kRates) / sizeof(kRates[0])))

bool pcan_bitrate_btr(int bitrate, uint16_t *btr0btr1) {
    for (int i = 0; i < NRATES; i++) {
        if (kRates[i].bitrate == bitrate) {
            if (btr0btr1) *btr0btr1 = kRates[i].btr;
            return true;
        }
    }
    return false;
}

const char *pcan_bitrate_name(int bitrate) {
    for (int i = 0; i < NRATES; i++) {
        if (kRates[i].bitrate == bitrate) return kRates[i].name;
    }
    return "?";
}

const char *pcan_bitrate_list(void) {
    return "1M 800k 500k 250k 125k 100k 95k 83k 50k 47k 33k 20k 10k 5k";
}

bool pcan_parse_bitrate(const char *s, pcan_bitrate_t *out) {
    pcan_bitrate_t r = { .autodetect = false, .bitrate = PCAN_DEFAULT_BITRATE, .btr0btr1 = 0x001C };
    if (s && strcasecmp(s, "auto") == 0) {
        r.autodetect = true;
        r.bitrate = 0;
        r.btr0btr1 = 0;
    } else if (s) {
        int found = -1;
        for (int i = 0; i < NRATES && found < 0; i++) {
            if (strcasecmp(s, kRates[i].name) == 0) found = i;
        }
        if (found < 0 && s[0] && strspn(s, "0123456789") == strlen(s) && strlen(s) <= 7) {
            int bps = atoi(s);
            for (int i = 0; i < NRATES && found < 0; i++) {
                if (kRates[i].bitrate == bps) found = i;
            }
        }
        if (found < 0) return false;
        r.bitrate = kRates[found].bitrate;
        r.btr0btr1 = kRates[found].btr;
    }
    if (out) *out = r;
    return true;
}

/*******************************************************************************
 * Channels
 ******************************************************************************/

static bool get_u32(const pcan_api_t *api, pcan_handle_t h, uint8_t param, uint32_t *out) {
    uint32_t v = 0;
    if (api->get_value(h, param, &v, sizeof(v)) != PCAN_ERROR_OK) return false;
    *out = v;
    return true;
}

static void get_string(const pcan_api_t *api, pcan_handle_t h, uint8_t param, size_t buf_size,
                       char *out, size_t out_len) {
    char buf[PCAN_VERSION_MAX + 1];
    memset(buf, 0, sizeof(buf));
    if (buf_size > PCAN_VERSION_MAX) buf_size = PCAN_VERSION_MAX;
    out[0] = '\0';
    if (api->get_value(h, param, buf, (uint32_t)buf_size) != PCAN_ERROR_OK) return;
    buf[buf_size] = '\0';
    char *nl = strpbrk(buf, "\r\n");
    if (nl) *nl = '\0';
    snprintf(out, out_len, "%s", buf);
}

const char *pcan_condition_text(uint32_t condition) {
    switch (condition) {
        case PCAN_CHANNEL_AVAILABLE: return "available";
        case PCAN_CHANNEL_OCCUPIED:  return "occupied";
        case PCAN_CHANNEL_PCANVIEW:  return "in use (PCAN-View)";
        default:                     return "unavailable";
    }
}

void pcan_channel_label(int index, const char *hw_name, char *buf, size_t len) {
    if (hw_name && hw_name[0]) snprintf(buf, len, "pcan%d %s", index, hw_name);
    else snprintf(buf, len, "pcan%d", index);
}

int pcan_enumerate(const pcan_api_t *api, pcan_channel_info_t *out, int max) {
    if (!api || !out || max <= 0) return 0;
    int n = 0;
    for (int index = 1; index <= PCAN_MAX_CHANNELS && n < max; index++) {
        pcan_handle_t h = pcan_channel_handle(index);
        uint32_t cond = PCAN_CHANNEL_UNAVAILABLE;
        if (!get_u32(api, h, PCAN_CHANNEL_CONDITION, &cond) || cond == PCAN_CHANNEL_UNAVAILABLE) continue;
        pcan_channel_info_t *info = &out[n++];
        memset(info, 0, sizeof(*info));
        info->index = index;
        info->handle = h;
        info->condition = cond;
        /* best effort: some parameters may need an initialized channel */
        get_string(api, h, PCAN_HARDWARE_NAME, PCAN_HW_NAME_MAX, info->hw_name, sizeof(info->hw_name));
        info->has_device_id = get_u32(api, h, PCAN_DEVICE_ID, &info->device_id);
        get_string(api, h, PCAN_FIRMWARE_VERSION, PCAN_VERSION_MAX, info->firmware, sizeof(info->firmware));
        uint32_t features = 0;
        if (get_u32(api, h, PCAN_CHANNEL_FEATURES, &features)) info->fd_capable = (features & PCAN_FEATURE_FD_CAPABLE) != 0;
    }
    return n;
}

int pcan_resolve_device(const pcan_api_t *api, const char *device, char *err, size_t err_len) {
    int want = pcan_parse_device(device);
    if (want == PCAN_DEVICE_NONE) {
        snprintf(err, err_len, "%s is not a PCAN device (pcan, pcan1..pcan8)", device ? device : "(null)");
        return 0;
    }
    if (want == PCAN_DEVICE_BAD) {
        snprintf(err, err_len, "%s: PCAN channels are pcan1..pcan8", device);
        return 0;
    }
    if (!api) {
        snprintf(err, err_len, "PCAN library not loaded");
        return 0;
    }
    if (want == 0) {
        pcan_channel_info_t list[PCAN_MAX_CHANNELS];
        int n = pcan_enumerate(api, list, PCAN_MAX_CHANNELS);
        int busy = 0;
        for (int i = 0; i < n; i++) {
            if (list[i].condition == PCAN_CHANNEL_AVAILABLE) return list[i].index;
            if (!busy) busy = list[i].index;
        }
        if (busy) snprintf(err, err_len, "pcan%d is in use by another application", busy);
        else snprintf(err, err_len, "no PCAN-USB found (plugged in?)");
        return 0;
    }
    uint32_t cond = PCAN_CHANNEL_UNAVAILABLE;
    pcan_status_t sts = api->get_value(pcan_channel_handle(want), PCAN_CHANNEL_CONDITION, &cond, sizeof(cond));
    if (sts != PCAN_ERROR_OK) {
        char text[300];
        pcan_status_text(api, sts, text, sizeof(text));
        snprintf(err, err_len, "pcan%d: cannot read the channel condition: %s", want, text);
        return 0;
    }
    if (cond == PCAN_CHANNEL_AVAILABLE) return want;
    if (cond == PCAN_CHANNEL_UNAVAILABLE) snprintf(err, err_len, "pcan%d not found (plugged in?)", want);
    else snprintf(err, err_len, "pcan%d is in use by another application (%s)", want, pcan_condition_text(cond));
    return 0;
}

/*******************************************************************************
 * Safe open (the only CAN_Initialize / CAN_SetValue in the tool)
 ******************************************************************************/

/* The only CAN_SetValue call in the tool: listen-only ON, nothing else. */
static pcan_status_t set_listen_only_on(const pcan_api_t *api, pcan_handle_t h) {
    uint32_t on = PCAN_PARAMETER_ON;
    return api->set_value(h, PCAN_LISTEN_ONLY, &on, sizeof(on));
}

pcan_open_result_t pcan_open(const pcan_api_t *api, int index, int bitrate, pcan_channel_t *out,
                             char *err, size_t err_len) {
    char text[300];
    memset(out, 0, sizeof(*out));
    out->rx_fd = -1;
    if (!api || index < 1 || index > PCAN_MAX_CHANNELS) {
        snprintf(err, err_len, "PCAN: bad channel %d", index);
        return PCAN_OPEN_FAILED;
    }
    uint16_t btr = 0;
    if (!pcan_bitrate_btr(bitrate, &btr)) {
        snprintf(err, err_len, "PCAN: unsupported bitrate %d", bitrate);
        return PCAN_OPEN_FAILED;
    }
    pcan_handle_t h = pcan_channel_handle(index);

    /* 1. listen-only on the uninitialized channel: no ACK, no error frames, no TX */
    pcan_status_t sts = set_listen_only_on(api, h);
    if (sts != PCAN_ERROR_OK) {
        pcan_status_text(api, sts, text, sizeof(text));
        snprintf(err, err_len, "pcan%d: cannot set listen-only before init (%s); not initialized", index, text);
        return PCAN_OPEN_NOT_LISTEN_ONLY;
    }

    /* 2. it must read back ON while still uninitialized */
    uint32_t lo = 0xFFFFFFFFu;
    sts = api->get_value(h, PCAN_LISTEN_ONLY, &lo, sizeof(lo));
    if (sts != PCAN_ERROR_OK || lo != PCAN_PARAMETER_ON) {
        if (sts != PCAN_ERROR_OK) {
            pcan_status_text(api, sts, text, sizeof(text));
            snprintf(err, err_len, "pcan%d: listen-only cannot be read back before init (%s); not initialized",
                     index, text);
        } else {
            snprintf(err, err_len, "pcan%d: listen-only reads back %u, not ON, before init; not initialized",
                     index, lo);
        }
        return PCAN_OPEN_NOT_LISTEN_ONLY;
    }

    /* 3. initialize (classic CAN) */
    sts = api->initialize(h, btr, 0, 0, 0);
    if (sts != PCAN_ERROR_OK) {
        api->uninitialize(h);   /* in case it half-opened; harmless otherwise */
        pcan_status_text(api, sts, text, sizeof(text));
        snprintf(err, err_len, "pcan%d: CAN_Initialize at %s failed: %s", index, pcan_bitrate_name(bitrate), text);
        return PCAN_OPEN_FAILED;
    }

    /* 4. listen-only must still read back ON; otherwise close it at once */
    lo = 0xFFFFFFFFu;
    sts = api->get_value(h, PCAN_LISTEN_ONLY, &lo, sizeof(lo));
    if (sts != PCAN_ERROR_OK || lo != PCAN_PARAMETER_ON) {
        api->uninitialize(h);
        if (sts != PCAN_ERROR_OK) {
            pcan_status_text(api, sts, text, sizeof(text));
            snprintf(err, err_len, "pcan%d: listen-only read-back failed (%s); channel closed", index, text);
        } else {
            snprintf(err, err_len, "pcan%d: listen-only is not active after init (read back %u); channel closed",
                     index, lo);
        }
        return PCAN_OPEN_NOT_LISTEN_ONLY;
    }

    out->api = api;
    out->index = index;
    out->handle = h;
    out->bitrate = bitrate;
    out->btr0btr1 = btr;
    out->open = true;

    int fd = -1;
    if (api->get_value(h, PCAN_RECEIVE_EVENT, &fd, sizeof(fd)) == PCAN_ERROR_OK && fd >= 0) out->rx_fd = fd;
    get_string(api, h, PCAN_HARDWARE_NAME, PCAN_HW_NAME_MAX, out->hw_name, sizeof(out->hw_name));
    out->has_device_id = get_u32(api, h, PCAN_DEVICE_ID, &out->device_id);
    get_string(api, h, PCAN_FIRMWARE_VERSION, PCAN_VERSION_MAX, out->firmware, sizeof(out->firmware));
    pcan_channel_label(index, out->hw_name, out->label, sizeof(out->label));
    return PCAN_OPEN_OK;
}

void pcan_close(pcan_channel_t *ch) {
    if (!ch || !ch->open) return;
    ch->api->uninitialize(ch->handle);
    ch->open = false;
    ch->rx_fd = -1;
}

static bool sleep_us(int64_t us) {
    if (us <= 0) return true;
    struct timespec ts = { .tv_sec = (time_t)(us / 1000000), .tv_nsec = (long)(us % 1000000) * 1000L };
    return nanosleep(&ts, NULL) == 0;
}

bool pcan_wait(const pcan_channel_t *ch, int64_t timeout_us) {
    if (timeout_us < 0) timeout_us = 0;
    if (ch && ch->rx_fd >= 0 && ch->rx_fd < FD_SETSIZE) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(ch->rx_fd, &fds);
        struct timeval tv = { .tv_sec = (time_t)(timeout_us / 1000000),
                              .tv_usec = (suseconds_t)(timeout_us % 1000000) };
        int r = select(ch->rx_fd + 1, &fds, NULL, NULL, &tv);
        if (r >= 0) return true;
        if (errno == EINTR) return false;
        /* descriptor gone (unplugged?): poll instead; CAN_Read reports the error */
    }
    return sleep_us(timeout_us < 1000 ? timeout_us : 1000);
}

/*******************************************************************************
 * Messages and timestamps
 ******************************************************************************/

pcan_kind_t pcan_msg_kind(const pcan_msg_t *m) {
    if (m->msgtype & PCAN_MESSAGE_STATUS) return PCAN_KIND_STATUS;
    if (m->msgtype & PCAN_MESSAGE_ERRFRAME) return PCAN_KIND_ERROR;
    if (m->msgtype & (PCAN_MESSAGE_FD | PCAN_MESSAGE_ECHO)) return PCAN_KIND_OTHER;
    if (m->msgtype & PCAN_MESSAGE_RTR) return PCAN_KIND_RTR;
    return PCAN_KIND_DATA;
}

void pcan_msg_to_frame(const pcan_msg_t *m, can_frame_t *out) {
    memset(out, 0, sizeof(*out));
    out->is_extended = (m->msgtype & PCAN_MESSAGE_EXTENDED) != 0;
    out->can_id = m->id & (out->is_extended ? 0x1FFFFFFFu : 0x7FFu);
    out->dlc = m->len > 8 ? 8 : m->len;
    memcpy(out->data, m->data, out->dlc);
}

uint64_t pcan_timestamp_us(const pcan_timestamp_t *ts) {
    return (uint64_t)ts->micros + 1000ull * (uint64_t)ts->millis +
           0x100000000ull * 1000ull * (uint64_t)ts->millis_overflow;
}

uint32_t pcan_status_msg_bits(const pcan_msg_t *m) {
    return ((uint32_t)m->data[0] << 24) | ((uint32_t)m->data[1] << 16) |
           ((uint32_t)m->data[2] << 8) | (uint32_t)m->data[3];
}

uint32_t pcan_bus_bits(pcan_status_t sts) {
    return sts & (PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSPASSIVE | PCAN_ERROR_BUSOFF);
}

const char *pcan_bus_state_text(uint32_t status) {
    if (status & PCAN_ERROR_BUSOFF) return "bus-off";
    if (status & PCAN_ERROR_BUSPASSIVE) return "error passive";
    if (status & PCAN_ERROR_BUSHEAVY) return "error warning";
    if (status & PCAN_ERROR_BUSLIGHT) return "bus errors (light)";
    return "ok";
}

/* Bits that may come with a valid message (CAN_Read reports them alongside). */
#define PCAN_MSG_OK_BITS (PCAN_ERROR_OVERRUN | PCAN_ERROR_QOVERRUN | PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | \
                          PCAN_ERROR_BUSPASSIVE | PCAN_ERROR_BUSOFF | PCAN_ERROR_XMTFULL | PCAN_ERROR_QXMTFULL | \
                          PCAN_ERROR_CAUTION)

pcan_read_t pcan_read_result(pcan_status_t sts) {
    if (sts & ~(PCAN_MSG_OK_BITS | PCAN_ERROR_QRCVEMPTY)) return PCAN_READ_ERROR;
    if (sts & PCAN_ERROR_QRCVEMPTY) return PCAN_READ_EMPTY;
    return PCAN_READ_MSG;
}

void pcan_counts_add(pcan_counts_t *c, pcan_kind_t kind) {
    switch (kind) {
        case PCAN_KIND_DATA:   c->data++; break;
        case PCAN_KIND_RTR:    c->rtr++; break;
        case PCAN_KIND_STATUS: c->status++; break;
        case PCAN_KIND_ERROR:  c->error++; break;
        default:               c->other++; break;
    }
}

/*******************************************************************************
 * Clock mapping
 ******************************************************************************/

#define PCAN_JUMP_BACK_US    1000000
#define PCAN_DRIFT_WINDOW_US 2000000

void pcan_clock_reset(pcan_clock_t *c) {
    memset(c, 0, sizeof(*c));
}

void pcan_clock_observe(pcan_clock_t *c, int64_t host_us, int64_t hw_us) {
    int64_t off = host_us - hw_us;
    bool jump = c->anchored && hw_us + PCAN_JUMP_BACK_US < c->last_hw_us;
    if (!c->in_batch || jump) {
        /* a jump restarts the batch minimum: only times after it count */
        c->in_batch = true;
        c->batch_min = off;
    } else if (off < c->batch_min) {
        c->batch_min = off;
    }
    if (jump) c->jumped = true;
    c->batch_host_us = host_us;
    c->last_hw_us = hw_us;
}

void pcan_clock_end_batch(pcan_clock_t *c) {
    if (!c->in_batch) return;
    c->in_batch = false;
    if (!c->anchored || c->jumped) {
        if (c->anchored) c->reanchors++;
        c->anchored = true;
        c->anchor_us = c->batch_min;
        c->anchored_host_us = c->batch_host_us;
        c->jumped = false;
        c->win_valid = false;
        c->have_drift = false;
        return;
    }
    int64_t d = c->batch_min - c->anchor_us;
    if (!c->win_valid) {
        c->win_valid = true;
        c->win_start_us = c->batch_host_us;
        c->win_min = d;
    } else if (d < c->win_min) {
        c->win_min = d;
    }
    if (c->batch_host_us - c->win_start_us >= PCAN_DRIFT_WINDOW_US) {
        c->drift_us = c->win_min;
        c->drift_host_us = c->batch_host_us;
        c->have_drift = true;
        c->win_valid = false;
    }
}

int64_t pcan_clock_map(const pcan_clock_t *c, int64_t hw_us) {
    return hw_us + c->anchor_us;
}

/*******************************************************************************
 * Bitrate auto detection
 ******************************************************************************/

const int pcan_autobaud_order[PCAN_AUTOBAUD_COUNT] = {
    500000, 250000, 1000000, 125000, 100000, 83333, 50000, 33333,
};

int64_t pcan_mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int64_t pcan_wall_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

void pcan_autobaud_begin(pcan_autobaud_t *ab, const pcan_api_t *api, int index, double attempt_s) {
    memset(ab, 0, sizeof(*ab));
    ab->api = api;
    ab->index = index;
    ab->attempt_s = attempt_s > 0 ? attempt_s : PCAN_AUTOBAUD_ATTEMPT_S;
    ab->ch.rx_fd = -1;
}

static void autobaud_finish_attempt(pcan_autobaud_t *ab, bool success) {
    pcan_baud_try_t *t = &ab->tried[ab->ntried - 1];
    t->listened_s = (double)(pcan_mono_us() - ab->attempt_start_us) / 1e6;
    pcan_close(&ab->ch);
    if (success) {
        ab->done = true;
        ab->bitrate = t->bitrate;
    } else if (ab->next >= PCAN_AUTOBAUD_COUNT) {
        ab->done = true;
        snprintf(ab->error, sizeof(ab->error), "no bitrate showed %d frames from %d IDs",
                 PCAN_AUTOBAUD_MIN_FRAMES, PCAN_AUTOBAUD_MIN_IDS);
    }
}

bool pcan_autobaud_step(pcan_autobaud_t *ab, double budget_s) {
    if (ab->done) return true;
    int64_t until = pcan_mono_us() + (int64_t)(budget_s * 1e6);
    int64_t attempt_us = (int64_t)(ab->attempt_s * 1e6);
    for (;;) {
        if (!ab->ch.open) {
            if (ab->next >= PCAN_AUTOBAUD_COUNT) {
                ab->done = true;
                if (!ab->error[0]) snprintf(ab->error, sizeof(ab->error), "no bitrate could be opened");
                return true;
            }
            if (ab->ntried > 0 && pcan_mono_us() >= until) return false;
            pcan_baud_try_t *t = &ab->tried[ab->ntried++];
            memset(t, 0, sizeof(*t));
            t->bitrate = pcan_autobaud_order[ab->next++];
            ab->nids = 0;
            /* every attempt is the full safe sequence */
            char err[256];
            pcan_open_result_t r = pcan_open(ab->api, ab->index, t->bitrate, &ab->ch, err, sizeof(err));
            if (r == PCAN_OPEN_NOT_LISTEN_ONLY) {
                snprintf(t->error, sizeof(t->error), "%s", err);
                snprintf(ab->error, sizeof(ab->error), "%s", err);
                ab->aborted = true;
                ab->done = true;
                return true;
            }
            if (r != PCAN_OPEN_OK) {
                snprintf(t->error, sizeof(t->error), "%s", err);
                if (ab->next >= PCAN_AUTOBAUD_COUNT) {
                    ab->done = true;
                    snprintf(ab->error, sizeof(ab->error), "%s", err);
                    return true;
                }
                continue;
            }
            t->opened = true;
            ab->attempt_start_us = pcan_mono_us();
        }

        pcan_baud_try_t *t = &ab->tried[ab->ntried - 1];
        bool failed = false;
        for (;;) {
            pcan_msg_t m;
            pcan_timestamp_t ts;
            pcan_status_t sts = ab->api->read(ab->ch.handle, &m, &ts);
            pcan_read_t res = pcan_read_result(sts);
            if (res == PCAN_READ_EMPTY) break;
            if (res == PCAN_READ_ERROR) {
                char text[200];
                pcan_status_text(ab->api, sts, text, sizeof(text));
                snprintf(t->error, sizeof(t->error), "read: %s", text);
                failed = true;
                break;
            }
            pcan_kind_t kind = pcan_msg_kind(&m);
            if (kind == PCAN_KIND_STATUS || kind == PCAN_KIND_ERROR) t->errors++;
            if (kind != PCAN_KIND_DATA) continue;
            t->frames++;
            can_frame_t f;
            pcan_msg_to_frame(&m, &f);
            bool seen = false;
            for (int i = 0; i < ab->nids && !seen; i++) seen = ab->ids[i] == f.can_id;
            if (!seen && ab->nids < (int)(sizeof(ab->ids) / sizeof(ab->ids[0]))) ab->ids[ab->nids++] = f.can_id;
            t->ids = ab->nids;
            if (t->frames >= PCAN_AUTOBAUD_MIN_FRAMES && t->ids >= PCAN_AUTOBAUD_MIN_IDS) {
                autobaud_finish_attempt(ab, true);
                return true;
            }
        }
        int64_t now = pcan_mono_us();
        if (failed || now - ab->attempt_start_us >= attempt_us) {
            autobaud_finish_attempt(ab, false);
            if (ab->done) return true;
            continue;
        }
        if (now >= until) return false;
        int64_t wait = until - now;
        int64_t left = ab->attempt_start_us + attempt_us - now;
        if (left < wait) wait = left;
        if (!pcan_wait(&ab->ch, wait)) return false;   /* signal: let the caller look */
    }
}

void pcan_autobaud_cancel(pcan_autobaud_t *ab) {
    if (ab->ch.open) {
        pcan_baud_try_t *t = &ab->tried[ab->ntried - 1];
        t->listened_s = (double)(pcan_mono_us() - ab->attempt_start_us) / 1e6;
        pcan_close(&ab->ch);
    }
    if (!ab->done) {
        ab->done = true;
        ab->bitrate = 0;
        snprintf(ab->error, sizeof(ab->error), "cancelled");
    }
}

void pcan_autobaud_report(const pcan_autobaud_t *ab, char *buf, size_t len) {
    if (!buf || !len) return;
    buf[0] = '\0';
    size_t used = 0;
    for (int i = 0; i < ab->ntried && used + 1 < len; i++) {
        const pcan_baud_try_t *t = &ab->tried[i];
        int n;
        const char *sep = i ? "; " : "";
        if (!t->opened) {
            n = snprintf(buf + used, len - used, "%s%s: not opened", sep, pcan_bitrate_name(t->bitrate));
        } else if (t->frames == 0) {
            n = snprintf(buf + used, len - used, "%s%s: no frames%s", sep, pcan_bitrate_name(t->bitrate),
                         t->errors ? " (bus errors)" : "");
        } else {
            n = snprintf(buf + used, len - used, "%s%s: %d frame%s, %d ID%s", sep, pcan_bitrate_name(t->bitrate),
                         t->frames, t->frames == 1 ? "" : "s", t->ids, t->ids == 1 ? "" : "s");
        }
        if (n < 0) break;
        used += (size_t)n;
        if (used >= len) {
            buf[len - 1] = '\0';
            break;
        }
    }
}
