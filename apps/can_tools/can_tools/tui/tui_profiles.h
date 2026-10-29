/**
 * @file tui_profiles.h
 * @brief Profile index for the TUI picker: lazy, chunked loading of *.vsp
 *        metadata, filtering, and discover drafts (C over the vsp_engine API)
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A passive CAN frame of a profile (can_frames[] on a CAN bus, not a
 *  reassembled stream, not a diagnostic request / response id). */
typedef struct {
    uint32_t id;
    bool     ext;
    int      dlc;   /* -1 = unknown */
} tui_profile_frame_t;

typedef struct {
    char path[1024];
    char file[256];
    char make[64];
    char model[96];
    char gen[64];
    char years[24];
    char hay[640];      /* lower case "make model gen file" for the filter */
    int  signals;       /* signals[] in the profile */
    int  can;           /* decodable CAN signals */
    int  obd;           /* diagnostic (OBD / UDS) signals */
    bool ok;            /* loaded */
    bool indexed;
    tui_profile_frame_t *frames;   /* passive CAN frames (owned), for tui_profiles_guess */
    int  nframes;
} tui_profile_entry_t;

typedef struct {
    char                 dir[1024];
    bool                 have_dir;
    bool                 ascii;
    tui_profile_entry_t *entries;   /* sorted by file name */
    int                  total;
    int                  indexed;
    tui_profile_entry_t  drafts[5]; /* newest first */
    int                  ndrafts;
} tui_profile_index_t;

/** Profile folder: explicit_dir, else $VSP_PROFILES_DIR, ./profiles, the
 *  repository's profiles/ (CAN_TOOLS_PROFILES_DIR, an absolute path set by
 *  CMake), then profiles/ in the executable's folder and up to three parents
 *  (apps/can_tools/build/can_tools reaches the repository root). False when
 *  none exists. */
bool tui_profiles_find_dir(const char *explicit_dir, const char *exe_path, char *out, size_t len);

/** List the .vsp files in dir (names only, nothing loaded yet). dir NULL = no folder. */
void tui_profiles_open(tui_profile_index_t *ix, const char *dir, bool ascii);
void tui_profiles_free(tui_profile_index_t *ix);
/** Index the next entries: at most max_count, and with budget_ms > 0 stop
 *  once that much monotonic time passed. True when everything is indexed. */
bool tui_profiles_step(tui_profile_index_t *ix, int max_count, double budget_ms);
bool tui_profiles_done(const tui_profile_index_t *ix);
/** Newest 5 <captures_root>/<dir>/ptcan_discover_draft.vsp with signal counts. */
void tui_profiles_scan_drafts(tui_profile_index_t *ix, const char *captures_root);
/** Indexed entries matching every space-separated token (case-insensitive
 *  substrings of make, model, generation and file name). */
int  tui_profiles_filter(const tui_profile_index_t *ix, const char *filter, int *out, int max);
/** Index of the entry with this path (or file name), -1 if none. */
int  tui_profiles_find(const tui_profile_index_t *ix, const char *path);
/** A profile path recorded earlier (session.json, tui_last) that may have
 *  moved: the path itself when that file exists, else the file of the same
 *  name in dir (the profile folder; NULL = none). False when neither exists. */
bool tui_profiles_resolve(const char *recorded, const char *dir, char *out, size_t len);

/** A frame seen on the bus. */
typedef struct {
    uint32_t id;
    bool     ext;
    int      dlc;
} tui_seen_frame_t;

typedef struct {
    int    entry;    /* index into ix->entries */
    double score;
    int    hits;     /* profile frames seen (with a matching dlc) */
    int    frames;   /* profile frames */
} tui_profile_guess_t;

#define TUI_GUESS_MAX 5

/** Profiles whose passive CAN frames match the frames seen on the bus. Score:
 *  every profile frame weighs log(N / profiles that list its id), so an id few
 *  profiles share says more; a frame seen (dlc matching when the profile gives
 *  one) adds its weight, one not seen costs 0.3 of it (a profile can list a
 *  frame of another bus, or an OBD-port gateway can hide it). The short list
 *  keeps the profiles scoring at least 60 % of the best with at least half of
 *  their frames (and min(3, frames)) seen, best first. Returns its length; 0
 *  while the index is incomplete, when nothing matches, or when more than max
 *  qualify (the traffic does not narrow it down). */
int tui_profiles_guess(const tui_profile_index_t *ix, const tui_seen_frame_t *seen, int nseen,
                       tui_profile_guess_t *out, int max);

/** "2009–2014", "2019–", "—" (ASCII "-"). */
void tui_profiles_years(char *out, size_t len, int start, int end, bool ascii);

#ifdef __cplusplus
}
#endif
