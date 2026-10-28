/**
 * @file tui_profiles.h
 * @brief Profile index for the TUI picker: lazy, chunked loading of *.vsp
 *        metadata, filtering, and discover drafts (C over the vsp_engine API)
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/** Profile folder: explicit_dir, else $VSP_PROFILES_DIR, ./profiles,
 *  ../vehicle-signal-profile/profiles, then the same two below each parent
 *  folder of the executable. False when none exists. */
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

/** "2009–2014", "2019–", "—" (ASCII "-"). */
void tui_profiles_years(char *out, size_t len, int start, int end, bool ascii);

#ifdef __cplusplus
}
#endif
