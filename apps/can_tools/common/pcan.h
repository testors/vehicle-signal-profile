/**
 * @file pcan.h
 * @brief PEAK PCAN-USB through the PCBUSB library, receive only (Pure C)
 *
 * PCBUSB (UV Software, https://github.com/mac-can/PCBUSB-Library) is the
 * PCAN-Basic compatible user-space driver for macOS. It is loaded with
 * dlopen at run time from /usr/local/lib or /opt/homebrew/lib only, so the
 * build and every other command work without it. A loaded library is used
 * only if its PCAN_CHANNEL_VERSION is 0.13 or a later 0.x (the tested one).
 * The declarations below are our own minimal subset of the PCAN-Basic
 * interface (types, layouts and constant values), not the library's header.
 *
 * Listen-only by construction:
 *  - pcan_api_t has no transmit member, and pcan_api_load() resolves only the
 *    receive-side symbols listed by pcan_api_symbol_names(). CAN_Write and
 *    CAN_WriteFD are never declared or looked up; CAN_Reset is not used.
 *  - pcan_open() is the only place a channel gets initialized. It sets
 *    PCAN_LISTEN_ONLY = ON on the uninitialized channel and reads it back
 *    (ON, or nothing is initialized), calls CAN_Initialize, then reads it
 *    back again; if that is not ON the channel is uninitialized at once and
 *    the open fails.
 *  - The only CAN_SetValue call in the tool sets PCAN_LISTEN_ONLY to ON.
 *  - Bitrate auto detection opens every candidate with pcan_open().
 *
 * Classic CAN only (CAN_Initialize, CAN_Read): CAN FD frames are not received.
 * Single-threaded; none of this is thread-safe.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCAN_INSTALL_URL "https://github.com/mac-can/PCBUSB-Library"

/*******************************************************************************
 * PCAN-Basic interface subset (values as defined by the PCAN-Basic API)
 ******************************************************************************/

typedef uint16_t pcan_handle_t;     /* TPCANHandle */
typedef uint32_t pcan_status_t;     /* TPCANStatus (32-bit in PCBUSB) */

#define PCAN_NONEBUS             0x00u
#define PCAN_USBBUS1             0x51u    /* ... PCAN_USBBUS8 = 0x58 */
#define PCAN_MAX_CHANNELS        8

/* Status / error bits */
#define PCAN_ERROR_OK            0x00000u
#define PCAN_ERROR_XMTFULL       0x00001u
#define PCAN_ERROR_OVERRUN       0x00002u
#define PCAN_ERROR_BUSLIGHT      0x00004u
#define PCAN_ERROR_BUSHEAVY      0x00008u
#define PCAN_ERROR_BUSOFF        0x00010u
#define PCAN_ERROR_QRCVEMPTY     0x00020u
#define PCAN_ERROR_QOVERRUN      0x00040u
#define PCAN_ERROR_QXMTFULL      0x00080u
#define PCAN_ERROR_REGTEST       0x00100u
#define PCAN_ERROR_NODRIVER      0x00200u
#define PCAN_ERROR_HWINUSE       0x00400u
#define PCAN_ERROR_NETINUSE      0x00800u
#define PCAN_ERROR_ILLHW         0x01400u
#define PCAN_ERROR_ILLNET        0x01800u
#define PCAN_ERROR_ILLCLIENT     0x01C00u
#define PCAN_ERROR_RESOURCE      0x02000u
#define PCAN_ERROR_ILLPARAMTYPE  0x04000u
#define PCAN_ERROR_ILLPARAMVAL   0x08000u
#define PCAN_ERROR_UNKNOWN       0x10000u
#define PCAN_ERROR_ILLDATA       0x20000u
#define PCAN_ERROR_BUSPASSIVE    0x40000u
#define PCAN_ERROR_ILLMODE       0x80000u
#define PCAN_ERROR_CAUTION       0x2000000u
#define PCAN_ERROR_INITIALIZE    0x4000000u
#define PCAN_ERROR_ILLOPERATION  0x8000000u

/* Parameters (CAN_GetValue; CAN_SetValue only ever with PCAN_LISTEN_ONLY) */
#define PCAN_DEVICE_ID           0x01u
#define PCAN_RECEIVE_EVENT       0x03u
#define PCAN_API_VERSION         0x05u
#define PCAN_CHANNEL_VERSION     0x06u    /* pre-init, any valid handle: "<name> <major>.<minor>..." */
#define PCAN_LISTEN_ONLY         0x08u
#define PCAN_CHANNEL_CONDITION   0x0Du
#define PCAN_HARDWARE_NAME       0x0Eu
#define PCAN_CHANNEL_FEATURES    0x16u
#define PCAN_FIRMWARE_VERSION    0x29u

#define PCAN_PARAMETER_OFF       0x00u
#define PCAN_PARAMETER_ON        0x01u

#define PCAN_CHANNEL_UNAVAILABLE 0x00u
#define PCAN_CHANNEL_AVAILABLE   0x01u
#define PCAN_CHANNEL_OCCUPIED    0x02u
#define PCAN_CHANNEL_PCANVIEW    0x03u

#define PCAN_FEATURE_FD_CAPABLE  0x01u

#define PCAN_HW_NAME_MAX         33      /* 32 characters + NUL */
#define PCAN_VERSION_MAX         256

/* Message types (bits of pcan_msg_t.msgtype) */
#define PCAN_MESSAGE_STANDARD    0x00u
#define PCAN_MESSAGE_RTR         0x01u
#define PCAN_MESSAGE_EXTENDED    0x02u
#define PCAN_MESSAGE_FD          0x04u
#define PCAN_MESSAGE_BRS         0x08u
#define PCAN_MESSAGE_ESI         0x10u
#define PCAN_MESSAGE_ECHO        0x20u
#define PCAN_MESSAGE_ERRFRAME    0x40u
#define PCAN_MESSAGE_STATUS      0x80u

/* TPCANMsg */
typedef struct {
    uint32_t id;          /* 11/29-bit identifier */
    uint8_t  msgtype;     /* PCAN_MESSAGE_* */
    uint8_t  len;         /* DLC 0..8 */
    uint8_t  data[8];
} pcan_msg_t;

/* TPCANTimestamp: total µs = micros + 1000 * millis + 0x100000000 * 1000 * millis_overflow */
typedef struct {
    uint32_t millis;
    uint16_t millis_overflow;
    uint16_t micros;      /* 0..999 */
} pcan_timestamp_t;

_Static_assert(sizeof(pcan_msg_t) == 16 && offsetof(pcan_msg_t, data) == 6, "TPCANMsg layout");
_Static_assert(sizeof(pcan_timestamp_t) == 8 && offsetof(pcan_timestamp_t, micros) == 6, "TPCANTimestamp layout");

/*******************************************************************************
 * API table
 ******************************************************************************/

/**
 * The library functions this tool may call. Receive side only: there is no
 * write member, so nothing that holds a pcan_api_t can transmit through it.
 * set_value is called from exactly one place (pcan.c: set_listen_only_on).
 */
typedef struct {
    pcan_status_t (*initialize)(pcan_handle_t channel, uint16_t btr0btr1, uint8_t hw_type,
                                uint32_t io_port, uint16_t interrupt);              /* CAN_Initialize */
    pcan_status_t (*uninitialize)(pcan_handle_t channel);                          /* CAN_Uninitialize */
    pcan_status_t (*get_status)(pcan_handle_t channel);                            /* CAN_GetStatus */
    pcan_status_t (*read)(pcan_handle_t channel, pcan_msg_t *msg, pcan_timestamp_t *ts); /* CAN_Read */
    pcan_status_t (*get_value)(pcan_handle_t channel, uint8_t param, void *buf, uint32_t len); /* CAN_GetValue */
    pcan_status_t (*set_value)(pcan_handle_t channel, uint8_t param, void *buf, uint32_t len); /* CAN_SetValue */
    pcan_status_t (*get_error_text)(pcan_status_t error, uint16_t language, char *buf); /* CAN_GetErrorText */
} pcan_api_t;

/** The symbols pcan_api_load() resolves, in pcan_api_t order. */
const char *const *pcan_api_symbol_names(int *count);

/** The PCBUSB version this tool was tested with; older ones and other majors are refused. */
#define PCAN_TESTED_MAJOR 0
#define PCAN_TESTED_MINOR 13
#define PCAN_TESTED_VERSION "0.13"

/**
 * Load the library once per process (dlopen /usr/local/lib/libPCBUSB.dylib,
 * then /opt/homebrew/lib/libPCBUSB.dylib; no other path) and resolve the
 * table, then gate on the version (pcan_api_check_version), before any
 * channel is touched. The result is cached. NULL with a message (missing:
 * names PCAN_INSTALL_URL; wrong version: names the found string and 0.13).
 * With a test loader set, that loader is asked instead (no dlopen at all);
 * the version gate still runs, once per table it returns.
 */
const pcan_api_t *pcan_api_load(char *err, size_t err_len);
/** Path the library was loaded from ("" before a load, "(test loader)" for a fake). */
const char *pcan_api_path(void);
/** PCAN_CHANNEL_VERSION of the loaded library ("" before a successful load). */
const char *pcan_api_version(void);

/**
 * Read PCAN_CHANNEL_VERSION from PCAN_USBBUS1 (a pre-init parameter, no device
 * needed; 256-byte buffer, terminator forced) and parse it as "%s %u.%u".
 * True only for major == 0 && minor >= 13. version gets the string found.
 */
bool pcan_api_check_version(const pcan_api_t *api, char *version, size_t version_len,
                            char *err, size_t err_len);

/** Tests only: a loader that returns a fake table (or NULL with err) instead
 *  of dlopen. Called on every pcan_api_load(); NULL restores the library. */
typedef const pcan_api_t *(*pcan_api_loader_fn)(char *err, size_t err_len);
void pcan_api_set_test_loader(pcan_api_loader_fn loader);

/** "PCAN_ERROR_ILLHW (0x01400): <library text>" style text for logs. */
void pcan_status_text(const pcan_api_t *api, pcan_status_t sts, char *buf, size_t len);

/*******************************************************************************
 * Device names and bitrates
 ******************************************************************************/

#define PCAN_DEVICE_NONE  (-1)   /* not a PCAN device name */
#define PCAN_DEVICE_BAD   (-2)   /* "pcanN" with N outside 1..8 */

/** "pcan" -> 0 (first available channel), "pcan1".."pcan8" -> 1..8,
 *  case-insensitive; PCAN_DEVICE_BAD for another number, else PCAN_DEVICE_NONE. */
int  pcan_parse_device(const char *s);
/** The name is meant for PCAN (valid or PCAN_DEVICE_BAD). */
bool pcan_is_device(const char *s);
/** Channel handle of pcanN (N = 1..8). */
pcan_handle_t pcan_channel_handle(int index);

#define PCAN_DEFAULT_BITRATE 500000

typedef struct {
    bool     autodetect;     /* "auto" */
    int      bitrate;        /* bit/s, e.g. 500000, 83333 */
    uint16_t btr0btr1;       /* PCAN_BAUD_* register value */
} pcan_bitrate_t;

/** "1M 800k 500k 250k 125k 100k 95k 83k 50k 47k 33k 20k 10k 5k" (any case),
 *  the bit/s number (500000, 83333, ...) or "auto". NULL = the 500k default. */
bool pcan_parse_bitrate(const char *s, pcan_bitrate_t *out);
/** BTR0BTR1 of a supported bitrate (bit/s). */
bool pcan_bitrate_btr(int bitrate, uint16_t *btr0btr1);
/** "500k", "1M", "83k"; "?" for an unsupported value. */
const char *pcan_bitrate_name(int bitrate);
/** The list for usage text: "1M 800k 500k ... 5k". */
const char *pcan_bitrate_list(void);

/*******************************************************************************
 * Channels
 ******************************************************************************/

typedef struct {
    int           index;                         /* N of pcanN */
    pcan_handle_t handle;
    uint32_t      condition;                     /* PCAN_CHANNEL_* */
    char          hw_name[PCAN_HW_NAME_MAX + 1]; /* "PCAN-USB", "" if not readable */
    bool          has_device_id;
    uint32_t      device_id;
    char          firmware[64];                  /* "" if not readable */
    bool          fd_capable;
} pcan_channel_info_t;

/** Channels that are not UNAVAILABLE, in channel order (condition first; the
 *  name, device id and firmware where the library answers before init). */
int pcan_enumerate(const pcan_api_t *api, pcan_channel_info_t *out, int max);
/** "available", "occupied", "in use (PCAN-View)", "unavailable" */
const char *pcan_condition_text(uint32_t condition);
/** Row / header name: "pcan1 PCAN-USB" ("pcan1" when the name is unknown). */
void pcan_channel_label(int index, const char *hw_name, char *buf, size_t len);

/**
 * Pick the channel for a device name: "pcan" = the first AVAILABLE channel,
 * "pcanN" = that channel if it is AVAILABLE. Returns 1..8, or 0 with a
 * message (not found, occupied, bad name).
 */
int pcan_resolve_device(const pcan_api_t *api, const char *device, char *err, size_t err_len);

/** An open (initialized, listen-only verified) channel. */
typedef struct {
    const pcan_api_t *api;
    int           index;
    pcan_handle_t handle;
    int           bitrate;
    uint16_t      btr0btr1;
    bool          open;
    int           rx_fd;                         /* PCAN_RECEIVE_EVENT descriptor, -1 = poll */
    char          hw_name[PCAN_HW_NAME_MAX + 1];
    char          label[64];                     /* "pcan1 PCAN-USB" */
    bool          has_device_id;
    uint32_t      device_id;
    char          firmware[64];
} pcan_channel_t;

typedef enum {
    PCAN_OPEN_OK = 0,
    PCAN_OPEN_FAILED,           /* not initialized (bad channel/bitrate, CAN_Initialize failed) */
    PCAN_OPEN_NOT_LISTEN_ONLY,  /* listen-only could not be set or verified: never retry */
} pcan_open_result_t;

/**
 * The one way to initialize a channel (see the file comment):
 *   1. CAN_SetValue(PCAN_LISTEN_ONLY, ON) on the uninitialized channel;
 *      failure -> PCAN_OPEN_NOT_LISTEN_ONLY, nothing initialized
 *   2. CAN_GetValue(PCAN_LISTEN_ONLY) must return OK and ON, still before
 *      init; else PCAN_OPEN_NOT_LISTEN_ONLY, nothing initialized
 *   3. CAN_Initialize(btr0btr1); failure -> CAN_Uninitialize, PCAN_OPEN_FAILED
 *   4. CAN_GetValue(PCAN_LISTEN_ONLY) == ON, else CAN_Uninitialize at once
 *      -> PCAN_OPEN_NOT_LISTEN_ONLY
 * Then the receive event descriptor and the device details (best effort).
 */
pcan_open_result_t pcan_open(const pcan_api_t *api, int index, int bitrate, pcan_channel_t *out,
                             char *err, size_t err_len);
/** CAN_Uninitialize (once; a closed channel is ignored). */
void pcan_close(pcan_channel_t *ch);

/** Wait up to timeout_us for the receive event (select on rx_fd; without a
 *  descriptor a sleep of at most 1 ms). False if a signal ended the wait. */
bool pcan_wait(const pcan_channel_t *ch, int64_t timeout_us);

/*******************************************************************************
 * Messages and timestamps
 ******************************************************************************/

typedef enum {
    PCAN_KIND_DATA = 0,
    PCAN_KIND_RTR,
    PCAN_KIND_STATUS,
    PCAN_KIND_ERROR,     /* error frame */
    PCAN_KIND_OTHER,     /* FD or echo frame (never expected here) */
} pcan_kind_t;

pcan_kind_t pcan_msg_kind(const pcan_msg_t *m);
/** Data message -> frame: 11/29-bit ID, is_extended, DLC clamped to 0..8,
 *  data; timestamps left zero. */
void pcan_msg_to_frame(const pcan_msg_t *m, can_frame_t *out);
/** Controller time in µs. */
uint64_t pcan_timestamp_us(const pcan_timestamp_t *ts);
/** Status message -> PCAN_ERROR_* bus bits (DATA[0..3], most significant first). */
uint32_t pcan_status_msg_bits(const pcan_msg_t *m);
/** Bus state of PCAN_ERROR_* bits: "bus-off", "error passive", "error warning",
 *  "bus errors (light)", "ok". */
const char *pcan_bus_state_text(uint32_t status);
/** Bus-state bits of a status word (BUSLIGHT | BUSHEAVY | BUSPASSIVE | BUSOFF). */
uint32_t pcan_bus_bits(pcan_status_t sts);

typedef enum {
    PCAN_READ_MSG = 0,   /* a message (overrun / bus-state bits may be set) */
    PCAN_READ_EMPTY,     /* receive queue empty */
    PCAN_READ_ERROR,     /* anything else: unplugged, not initialized, ... */
} pcan_read_t;

/** Classify a CAN_Read return value. */
pcan_read_t pcan_read_result(pcan_status_t sts);

/** Counts of received messages by kind. */
typedef struct {
    uint64_t data;
    uint64_t rtr;
    uint64_t status;
    uint64_t error;
    uint64_t other;
} pcan_counts_t;

void pcan_counts_add(pcan_counts_t *c, pcan_kind_t kind);

/**
 * Controller time -> wall clock. wall_us = hw_us + anchor; the anchor is
 * min(host_wall_us_at_read - hw_us) over the first drained batch and stays
 * fixed (a monotonic mapping keeps per-ID intervals exact). Later batches
 * only measure the drift. A controller clock that jumps back by more than
 * 1 s re-anchors.
 */
typedef struct {
    bool    anchored;
    int64_t anchor_us;
    int64_t anchored_host_us;    /* host time of the anchoring batch */
    bool    in_batch;
    int64_t batch_min;           /* min(host - hw) of the batch being drained */
    int64_t batch_host_us;       /* last host time of that batch */
    int64_t last_hw_us;
    bool    jumped;              /* clock went back in this batch */
    int     reanchors;
    /* drift: min(host - hw) - anchor over 2 s windows */
    int64_t win_start_us;
    int64_t win_min;
    bool    win_valid;
    int64_t drift_us;
    int64_t drift_host_us;       /* host time of the drift value */
    bool    have_drift;
} pcan_clock_t;

void    pcan_clock_reset(pcan_clock_t *c);
/** One message of a drained batch: wall-clock µs right after the read, controller µs. */
void    pcan_clock_observe(pcan_clock_t *c, int64_t host_us, int64_t hw_us);
/** End of a drained batch; the first one fixes the anchor. */
void    pcan_clock_end_batch(pcan_clock_t *c);
/** Wall-clock µs of a controller time (anchored only). */
int64_t pcan_clock_map(const pcan_clock_t *c, int64_t hw_us);

/*******************************************************************************
 * Bitrate auto detection (listen-only, one candidate at a time)
 ******************************************************************************/

#define PCAN_AUTOBAUD_COUNT      8
#define PCAN_AUTOBAUD_MIN_FRAMES 20
#define PCAN_AUTOBAUD_MIN_IDS    2
#define PCAN_AUTOBAUD_ATTEMPT_S  0.5

/** Tried in this order: 500k, 250k, 1M, 125k, 100k, 83k, 50k, 33k. */
extern const int pcan_autobaud_order[PCAN_AUTOBAUD_COUNT];

typedef struct {
    int    bitrate;
    bool   opened;
    int    frames;          /* valid data frames */
    int    ids;             /* distinct IDs among them */
    int    errors;          /* error / status messages */
    double listened_s;
    char   error[128];      /* open failure */
} pcan_baud_try_t;

typedef struct {
    const pcan_api_t *api;
    int             index;
    double          attempt_s;
    int             next;            /* next candidate in pcan_autobaud_order */
    pcan_channel_t  ch;              /* open during an attempt */
    int64_t         attempt_start_us;
    uint32_t        ids[64];         /* distinct IDs of the current attempt */
    int             nids;
    pcan_baud_try_t tried[PCAN_AUTOBAUD_COUNT];
    int             ntried;
    bool            done;
    int             bitrate;         /* detected; 0 = none */
    bool            aborted;         /* listen-only failed: stop, nothing more is opened */
    char            error[256];
} pcan_autobaud_t;

/** attempt_s <= 0 uses PCAN_AUTOBAUD_ATTEMPT_S. */
void pcan_autobaud_begin(pcan_autobaud_t *ab, const pcan_api_t *api, int index, double attempt_s);
/** Work for up to budget_s; true once finished (ab->bitrate, ab->tried). Every
 *  attempt is a full pcan_open() and ends with pcan_close(). */
bool pcan_autobaud_step(pcan_autobaud_t *ab, double budget_s);
/** Close an attempt in progress and finish without a result. */
void pcan_autobaud_cancel(pcan_autobaud_t *ab);
/** "500k: 0 frames; 250k: 31 frames, 4 IDs" */
void pcan_autobaud_report(const pcan_autobaud_t *ab, char *buf, size_t len);

/** Monotonic clock in µs. */
int64_t pcan_mono_us(void);
/** Wall clock in µs since the epoch. */
int64_t pcan_wall_us(void);

#ifdef __cplusplus
}
#endif
