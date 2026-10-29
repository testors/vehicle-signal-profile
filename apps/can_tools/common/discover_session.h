/**
 * @file discover_session.h
 * @brief Guided PT-CAN discover session (capture phases -> analysis -> files)
 *
 * Two layers:
 *
 *  - Session layer (no stdin, no prompts): create a session, capture or skip
 *    each phase from any frame source, then finish = analyze + write draft
 *    DBC/VSP + result.json + manifest.json into the session dir. A TUI drives
 *    this layer phase by phase.
 *
 *  - CLI driver: the interactive stdin ENTER / [s] skip / Ctrl+C loop with the
 *    stderr report and the result JSON on stdout, used by `can_discover` and
 *    `can_reader discover`.
 *
 * Session files: <dump_dir>/NN_<slug>.frames.tsv, manifest.json, result.json,
 * ptcan_discover_draft.dbc, ptcan_discover_draft.vsp. dump_dir defaults to
 * captures/YYYYMMDD_HHMMSS. If it cannot be created the session runs without
 * dumps and the drafts go to ptcan_discover_draft_YYYYMMDD_HHMMSS.<ext>.
 *
 * Messages ("[INFO] ...", "[WARN] ...") go to the config's log callback, or
 * to stderr when it is NULL.
 *
 * Single-threaded: call from the thread that pumps the frame source.
 */

#pragma once

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>

#include "can_discover.h"
#include "frame_source.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Max path buffer length for dump artifacts */
#define DISC_PATH_MAX 512

/** Per-phase record written to manifest.json */
typedef struct {
    bool   skipped;
    bool   captured;
    int    planned_duration_s;
    double actual_duration_s;       /**< Wall-clock capture duration */
    double analyzed_duration_s;     /**< Duration used for Hz (covered by stored frames) */
    int    total_frames;
    int    id_count;
    int    dropped_frames;          /**< Total frames not analyzed */
    int    capture_dropped_frames;  /**< Frames the capture buffer could not store */
    char   dump_file[64];
} disc_phase_dump_meta_t;

/*******************************************************************************
 * Phase table (DISC_NUM_PHASES entries, index = DISC_PHASE_*)
 ******************************************************************************/

int         disc_session_phase_count(void);
/** "Baseline", "Steering", ... ("Phase" if out of range) */
const char *disc_session_phase_name(int phase_idx);
/** "baseline", "steering", ... ("phase" if out of range) */
const char *disc_session_phase_slug(int phase_idx);
/** Operator instruction ("" if out of range) */
const char *disc_session_phase_instruction(int phase_idx);
/** Extra note, or NULL */
const char *disc_session_phase_warning(int phase_idx);
/** Planned capture length in seconds (0 if out of range) */
int         disc_session_phase_duration_s(int phase_idx);
/** Every phase except the baseline may be skipped */
bool        disc_session_phase_skippable(int phase_idx);

/*******************************************************************************
 * Session layer
 ******************************************************************************/

/** Receives one message, newline(s) included, e.g. "[INFO] Draft DBC written: x\n". */
typedef void (*disc_session_log_fn)(const char *text, void *ctx);

typedef struct {
    const char *dump_dir_override;  /**< NULL/"" = captures/YYYYMMDD_HHMMSS */
    const char *device_prefix;      /**< manifest "device_prefix" (may be NULL) */
    const char *connected_name;     /**< manifest "connected_device" (may be NULL) */
    const char *monitor_cmd;        /**< manifest "monitor_cmd", e.g. "STMA" */
    disc_session_log_fn log;        /**< NULL = stderr */
    void *log_ctx;
    bool hw_timestamps;             /**< source stamps in hardware: dumps get the timestamp_us column */
} disc_session_config_t;

typedef struct disc_session disc_session_t;

/** Creates the dump dir (logging "[INFO] Capture dump dir: ..." or a
 *  "[WARN]" if it fails). Strings are copied. NULL only on allocation failure. */
disc_session_t *disc_session_create(const disc_session_config_t *cfg);
void disc_session_free(disc_session_t *s);

/** Session dir, or NULL when dumps are disabled (dir could not be created). */
const char *disc_session_dump_dir(const disc_session_t *s);
const disc_phase_t *disc_session_phase(const disc_session_t *s, int phase_idx);
const disc_raw_phase_t *disc_session_raw_phase(const disc_session_t *s, int phase_idx);
const disc_phase_dump_meta_t *disc_session_phase_meta(const disc_session_t *s, int phase_idx);
/** Bus mode is known once the baseline has been captured. */
bool disc_session_have_bus_mode(const disc_session_t *s);
disc_bus_mode_t disc_session_bus_mode(const disc_session_t *s);

/**
 * Called before every pump of a capture (about every 10 ms on a live source).
 * Return false to end the phase early (e.g. Ctrl+C). elapsed_s is measured on
 * the source clock; frames is the number stored so far.
 */
typedef bool (*disc_capture_progress_fn)(double elapsed_s, double duration_s,
                                         int frames, void *ctx);

/**
 * Capture one phase: start the source, pump it until duration_s has passed on
 * the source clock, the source closes, a finite source runs out or progress
 * returns false; then stop the source. Fills the phase statistics and raw
 * frames. Hz uses the time covered by stored frames when the capture buffer
 * dropped frames. If the source is still open afterwards the phase's manifest
 * record is filled and, for the baseline, the bus mode is detected.
 *
 * Returns false if the source closed (live: connection lost); the manifest
 * record is then left unfilled, as before.
 */
bool disc_session_capture_phase(disc_session_t *s, int phase_idx,
                                frame_source_t *src, int duration_s,
                                disc_capture_progress_fn progress, void *progress_ctx);

/*
 * Incremental capture, for callers that own a continuous frame stream (the
 * TUI keeps its monitor running between phases). disc_session_capture_phase()
 * is begin + feed (as its source delivers) + end.
 *
 *   begin: start phase_idx at start_ms (source clock); false if a phase is
 *          already active or the index is invalid.
 *   feed:  store one frame (capture buffer, up to FRAME_BUFFER_MAX) and add it
 *          to the phase statistics; ignored when no phase is active.
 *   end:   finish at end_ms exactly as disc_session_capture_phase() does
 *          (Hz over stored frames, drop accounting, raw frames). With
 *          source_open the manifest record is filled and, for the baseline,
 *          the bus mode detected; returns source_open.
 */
bool disc_session_phase_begin(disc_session_t *s, int phase_idx, int64_t start_ms);
void disc_session_phase_feed(disc_session_t *s, const can_frame_t *frame);
bool disc_session_phase_end(disc_session_t *s, int64_t end_ms, bool source_open);
/** Phase being captured, or -1. */
int  disc_session_active_phase(const disc_session_t *s);
/** Frames stored so far in the active phase (0 if none). */
int  disc_session_phase_stored_frames(const disc_session_t *s);

/** Write NN_<slug>.frames.tsv for a captured phase and record it in the
 *  manifest. Logs "[WARN] Failed to write phase dump: ..." on failure.
 *  False if dumps are disabled or the write failed. */
bool disc_session_save_phase_dump(disc_session_t *s, int phase_idx);

/** Mark a phase as skipped in the manifest. */
void disc_session_skip_phase(disc_session_t *s, int phase_idx);

/** What finish produced. Pointers stay valid until disc_session_free(). */
typedef struct {
    const disc_result_t *result;
    bool                 have_bus_mode;
    disc_bus_mode_t      bus_mode;
    const char          *dump_dir;       /**< NULL when dumps are disabled */
    const char          *dbc_path;       /**< NULL if the draft DBC was not written */
    const char          *vsp_path;       /**< NULL if the draft VSP was not written */
    const char          *result_path;    /**< NULL if result.json was not written */
    const char          *manifest_path;  /**< NULL if manifest.json was not written */
} disc_session_output_t;

/**
 * Analyze every captured phase and write the draft DBC/VSP, result.json and
 * manifest.json (status "completed"), logging each file. out may be NULL.
 * Returns true if the analysis ran (individual file failures are logged).
 */
bool disc_session_finish(disc_session_t *s, disc_session_output_t *out);

/** Print the result JSON object (the stdout output of discover) after finish. */
void disc_session_print_result_json(const disc_session_t *s, FILE *f);

/** The result JSON object (result.json, the discover / replay stdout): bus
 *  mode when known, the paths that are set, and every found signal. The one
 *  writer of that shape, so a live session and its replay print the same. */
void disc_write_result_json_object(FILE *f, bool have_bus_mode, disc_bus_mode_t bus_mode,
                                   const char *dump_dir, const char *dbc_path,
                                   const char *vsp_path, const disc_result_t *result);

/** End without analysis: write manifest.json with the given status
 *  ("interrupted", "disconnected") and no output paths. */
void disc_session_abort(disc_session_t *s, const char *status);

/*******************************************************************************
 * CLI driver
 ******************************************************************************/

typedef struct {
    frame_source_t *source;            /**< Live source; is_open = still connected */
    const char *device_prefix;
    const char *connected_name;
    const char *monitor_cmd;
    const char *dump_dir_override;
    /** Cleared by the SIGINT handler outside a capture: ends the session. */
    volatile sig_atomic_t *running;
    /** Set by the SIGINT handler during a capture: ends that phase early. */
    volatile sig_atomic_t *interrupt;
    /** Called on normal completion and on Ctrl+C (not on connection loss). */
    void (*disconnect)(void *ctx);
    void *disconnect_ctx;
} disc_cli_config_t;

/**
 * Interactive guided discover: per phase, print the instruction, wait for
 * Enter (or 's' to skip), capture, report on stderr; then analyze, write the
 * session files and print the result JSON on stdout.
 *
 * Returns 0 on success, 130 when interrupted, 1 when the connection is lost.
 */
int disc_cli_run(const disc_cli_config_t *cfg);

#ifdef __cplusplus
}
#endif
