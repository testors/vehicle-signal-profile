/**
 * @file vsp_engine.h
 * @brief VSP profile loading, signal selection and CAN frame decoding
 *
 * C-callable API over vehicle-signal-profile (*.vsp) JSON. The profile is
 * parsed once with NSJSONSerialization; a decoder then holds the selected
 * signals with their decode/match rules precomputed, so feeding frames is
 * plain C. Rules (VSP_RUNTIME_SPEC.md):
 *
 *  - CAN signal: frame id must match under can_frames[].frame_id_mask, then
 *    decode.mux (if positional) must match.
 *  - Diagnostic signal: response id must equal rx_id (if known). Each CAN
 *    frame is decoded as is, ISO-TP PCI included, no reassembly. A positional
 *    decode.mux selects the frame; without one, the single/first frame payload
 *    must start with the positive-response marker (41 PID, 62 DID, 61 LID).
 *  - Value: little-endian or DBC big-endian bits, value_type integer / float32
 *    / float64, signed (two's complement) or sign_bit (sign-magnitude),
 *    factor/offset or polynomial, value table (label, numeric override, null).
 *  - Reassembled frames (vendor stream buffers) are not decodable: their
 *    signals are skipped and listed by vsp_decoder_skipped().
 *
 * Strings returned by the API are owned by the profile/decoder and stay valid
 * until it is freed. Single-threaded.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "elm327.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Profile
 ******************************************************************************/

typedef struct vsp_profile vsp_profile_t;

/** Load a .vsp file. On failure returns NULL and writes a message such as
 *  "Failed to read profile: <path>" or "Invalid VSP JSON: <reason>" to err. */
vsp_profile_t *vsp_profile_load(const char *path, char *err, size_t err_len);

/** Same from an in-memory JSON document. */
vsp_profile_t *vsp_profile_load_json(const char *json, size_t len, char *err, size_t err_len);

void vsp_profile_free(vsp_profile_t *profile);

/** Display metadata. Strings are NULL when absent or not a string. */
typedef struct {
    const char *path;           /**< NULL for vsp_profile_load_json */
    const char *schema;         /**< e.g. "vehicle_data_profile.v1" */
    const char *manufacturer;   /**< vehicle.manufacturer */
    const char *model;          /**< vehicle.model */
    const char *generation;     /**< applicability.generation */
    const char *platform;       /**< applicability.platform */
    const char *market;         /**< applicability.market */
    const char *trim;           /**< applicability.trim */
    const char *powertrain;     /**< applicability.powertrain */
    int         year_start;     /**< applicability.year_start, 0 = unknown */
    int         year_end;       /**< applicability.year_end, 0 = unknown */
    int         bus_count;
    int         frame_count;    /**< can_frames[] objects */
    int         signal_count;   /**< signals[] objects */
    int         query_count;    /**< diagnostic_queries[] objects */
} vsp_profile_info_t;

void vsp_profile_get_info(const vsp_profile_t *profile, vsp_profile_info_t *out);

typedef struct {
    const char *id;             /**< e.g. "can0" */
    const char *type;           /**< e.g. "can" */
    int         bitrate;        /**< 0 = unknown */
} vsp_bus_info_t;

bool vsp_profile_bus(const vsp_profile_t *profile, int idx, vsp_bus_info_t *out);

typedef struct {
    uint32_t    frame_id;
    uint32_t    frame_id_mask;  /**< 0 = full id */
    const char *name;
    const char *bus;
    int         dlc;            /**< -1 = unknown */
    double      cycle_ms;       /**< 0 = unknown */
    bool        extended;       /**< id_format == "extended" */
    bool        reassembled;    /**< vendor stream buffer, not one CAN frame */
} vsp_frame_info_t;

bool vsp_profile_frame(const vsp_profile_t *profile, int idx, vsp_frame_info_t *out);

typedef struct {
    const char *id;
    const char *kind;           /**< e.g. "obd_mode_01", "isotp_flow_control" */
    const char *role;           /**< e.g. "channel_query", "flow_control" */
    bool        has_tx_id;
    uint32_t    tx_id;
    uint32_t    rx_id;          /**< 0 = unknown */
    const char *request_hex;    /**< as written, e.g. "22 50 75" */
    const char *payload_hex;    /**< full CAN payload, e.g. "03 22 50 75 AA AA AA AA" */
} vsp_query_info_t;

bool vsp_profile_query(const vsp_profile_t *profile, int idx, vsp_query_info_t *out);

/*******************************************************************************
 * Decoder (selected signals + live state)
 ******************************************************************************/

typedef enum {
    VSP_ACQ_ANY = 0,            /**< can_frame and diagnostic_query */
    VSP_ACQ_CAN,                /**< can_frame only */
    VSP_ACQ_DIAGNOSTIC,         /**< diagnostic_query only */
} vsp_acq_filter_t;

typedef struct {
    /** Comma list of groups (rpm, gear, steering, throttle, brake and their
     *  aliases), canonical ids (contain a '.') or VSP signal names, matched
     *  case-insensitively. NULL or "" = the five default groups. */
    const char      *signals;
    /** Select every signal (signals is ignored). */
    bool             all_signals;
    vsp_acq_filter_t acquisition;
} vsp_select_t;

typedef enum {
    VSP_SIGNAL_CAN_FRAME = 0,
    VSP_SIGNAL_DIAGNOSTIC_QUERY,
} vsp_signal_kind_t;

typedef struct {
    const char       *name;             /**< "" if absent */
    const char       *canonical_id;     /**< "" if absent */
    const char       *group;            /**< rpm, gear, steering, throttle, brake, custom */
    const char       *unit;             /**< decode.unit, NULL if absent / not a string */
    const char       *acquisition_type; /**< "can_frame" / "diagnostic_query" */
    vsp_signal_kind_t kind;
    bool              has_mux;          /**< positional decode.mux */
    /* CAN */
    uint32_t          frame_id;
    uint32_t          frame_id_mask;    /**< 0 = full id */
    double            cycle_ms;         /**< can_frames[].cycle_ms, 0 = unknown */
    /* Diagnostic */
    uint32_t          tx_id;
    uint32_t          rx_id;            /**< 0 = any response id */
    const char       *query_id;         /**< NULL if not a string */
    const char       *request_hex;      /**< as written, "" if absent */
    const char       *request_bare_hex; /**< hex digits only, upper case */
    const char       *positive_marker_hex; /**< "41PP" / "62DDDD" / "61LL", NULL if none */
    const char       *query_key;        /**< "TTT:RRR:REQUEST", groups signals of one request */
    const char       *query_payload_hex;/**< diagnostic_queries[].payload_hex, NULL if absent */
    double            factor;           /**< decode.factor (1 if absent) */
    double            lsb;              /**< value of one raw step: |factor| for an integer field
                                             of 6+ bits with a linear decode; 0 = none (under 6
                                             bits, float32/float64, polynomial, numeric
                                             value-table override, no decode) */
    bool              has_value_table;  /**< decode.values has entries */
} vsp_signal_info_t;

typedef struct {
    int         samples;            /**< frames that matched */
    double      first_sample_ms;    /**< 0 = none yet */
    double      last_sample_ms;
    bool        has_value;          /**< last decode produced a number */
    bool        value_null;         /**< last decode hit a value-table entry with value null */
    double      value;              /**< last number (valid if has_value) */
    const char *label;              /**< last value-table label, NULL if none */
    uint8_t     last_payload[8];    /**< data bytes of the last matched frame */
    int         last_payload_len;
} vsp_signal_state_t;

typedef struct {
    const char *name;
    uint32_t    frame_id;
} vsp_skipped_signal_t;

typedef struct vsp_decoder vsp_decoder_t;

/** Select signals (sorted by group, canonical id, acquisition type, name).
 *  NULL only on allocation failure; zero selected signals is not an error. */
vsp_decoder_t *vsp_decoder_create(const vsp_profile_t *profile, const vsp_select_t *select);
void vsp_decoder_free(vsp_decoder_t *decoder);

int vsp_decoder_signal_count(const vsp_decoder_t *decoder);
const vsp_signal_info_t  *vsp_decoder_signal_info(const vsp_decoder_t *decoder, int idx);
const vsp_signal_state_t *vsp_decoder_signal_state(const vsp_decoder_t *decoder, int idx);

/** Signals that matched the selection but sit on a reassembled frame, in
 *  profile order. */
int vsp_decoder_skipped_count(const vsp_decoder_t *decoder);
const vsp_skipped_signal_t *vsp_decoder_skipped(const vsp_decoder_t *decoder, int idx);

/** Apply the CAN rules to the CAN signals. Returns the number updated. */
int vsp_decoder_feed_can(vsp_decoder_t *decoder, const can_frame_t *frame, double timestamp_ms);

/** Apply the diagnostic rules to the diagnostic signals; query_key (or NULL
 *  for any) restricts them to the signals of one request. */
int vsp_decoder_feed_diagnostic(vsp_decoder_t *decoder, const can_frame_t *frame,
                                double timestamp_ms, const char *query_key);

/** Both of the above (a passive monitor hearing broadcast and responses). */
int vsp_decoder_feed(vsp_decoder_t *decoder, const can_frame_t *frame, double timestamp_ms);

/** vsp_decoder_feed() that also writes the indices of the updated signals to
 *  updated[] (at most max_updated). Returns the number updated. */
int vsp_decoder_feed_collect(vsp_decoder_t *decoder, const can_frame_t *frame,
                             double timestamp_ms, int *updated, int max_updated);

/** Value table of a signal (decode.values): number of entries, 0 if none. */
int vsp_decoder_value_count(const vsp_decoder_t *decoder, int idx);
/** Entry i: raw code, label (NULL if none), has_number (false for a null
 *  override) and the decoded number (override, else raw * factor + offset). */
bool vsp_decoder_value_entry(const vsp_decoder_t *decoder, int idx, int i, long long *raw,
                             const char **label, bool *has_number, double *value);

/** vsp_decoder_feed_diagnostic() that also collects the updated indices. */
int vsp_decoder_feed_diagnostic_collect(vsp_decoder_t *decoder, const can_frame_t *frame,
                                        double timestamp_ms, const char *query_key,
                                        int *updated, int max_updated);

/** Clear every signal's state. */
void vsp_decoder_reset(vsp_decoder_t *decoder);

/*******************************************************************************
 * Hex helpers
 ******************************************************************************/

/** Copy the hex digits of in, upper-cased. */
void vsp_hex_strip(const char *in, char *out, size_t out_len);

/** "%02X" per byte. */
void vsp_hex_format(const uint8_t *bytes, int len, char *out, size_t out_len);

/** 8-byte ISO-TP single frame for a request (PCI length, request bytes, filler),
 *  as hex. False if the request is empty or not hex. Requests longer than 7
 *  bytes are cut to 7. */
bool vsp_single_frame_payload_hex(const char *request_hex, uint8_t filler,
                                  char *out, size_t out_len);

/*******************************************************************************
 * Objective-C access to the parsed JSON (for output that must keep JSON types)
 ******************************************************************************/

#ifdef __OBJC__
@class NSDictionary;
/** The whole profile document. */
NSDictionary *vsp_profile_json(const vsp_profile_t *profile);
/** signals[] entry of a selected signal. */
NSDictionary *vsp_decoder_signal_json(const vsp_decoder_t *decoder, int idx);
/** diagnostic_queries[] entry (or the query built from the signal's
 *  acquisition) of a diagnostic signal; nil for CAN signals. */
NSDictionary *vsp_decoder_query_json(const vsp_decoder_t *decoder, int idx);
#endif

#ifdef __cplusplus
}
#endif
