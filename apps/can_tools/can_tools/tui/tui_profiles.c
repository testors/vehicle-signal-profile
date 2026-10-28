/**
 * @file tui_profiles.c
 * @brief Profile index for the TUI picker (C over the vsp_engine API)
 */

#include "tui_profiles.h"

#include <ctype.h>
#include <dirent.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "vsp_engine.h"

static bool is_dir(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

static bool try_dir(const char *path, char *out, size_t len) {
    if (!path || !path[0] || !is_dir(path)) return false;
    snprintf(out, len, "%s", path);
    return true;
}

bool tui_profiles_find_dir(const char *explicit_dir, const char *exe_path, char *out, size_t len) {
    if (explicit_dir && explicit_dir[0]) return try_dir(explicit_dir, out, len);
    if (try_dir(getenv("VSP_PROFILES_DIR"), out, len)) return true;
    if (try_dir("profiles", out, len)) return true;
    if (try_dir("../vehicle-signal-profile/profiles", out, len)) return true;
    if (!exe_path || !exe_path[0]) return false;

    char real[PATH_MAX];
    if (!realpath(exe_path, real)) return false;
    char *slash = strrchr(real, '/');
    for (int depth = 0; slash && depth < 4; depth++) {
        *slash = '\0';
        char cand[PATH_MAX + 64];
        snprintf(cand, sizeof(cand), "%s/profiles", real);
        if (try_dir(cand, out, len)) return true;
        snprintf(cand, sizeof(cand), "%s/../vehicle-signal-profile/profiles", real);
        if (try_dir(cand, out, len)) return true;
        slash = strrchr(real, '/');
    }
    return false;
}

static bool has_suffix(const char *s, const char *suffix) {
    size_t n = strlen(s), m = strlen(suffix);
    return n >= m && strcmp(s + n - m, suffix) == 0;
}

static int cmp_entry(const void *a, const void *b) {
    return strcmp(((const tui_profile_entry_t *)a)->file, ((const tui_profile_entry_t *)b)->file);
}

void tui_profiles_open(tui_profile_index_t *ix, const char *dir, bool ascii) {
    memset(ix, 0, sizeof(*ix));
    ix->ascii = ascii;
    if (!dir || !dir[0]) return;
    snprintf(ix->dir, sizeof(ix->dir), "%s", dir);
    DIR *d = opendir(dir);
    if (!d) return;
    ix->have_dir = true;
    int cap = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.' || !has_suffix(e->d_name, ".vsp")) continue;
        if (ix->total == cap) {
            int ncap = cap ? cap * 2 : 256;
            tui_profile_entry_t *n = (tui_profile_entry_t *)realloc(ix->entries, (size_t)ncap * sizeof(*n));
            if (!n) break;
            ix->entries = n;
            cap = ncap;
        }
        tui_profile_entry_t *p = &ix->entries[ix->total++];
        memset(p, 0, sizeof(*p));
        snprintf(p->file, sizeof(p->file), "%s", e->d_name);
        snprintf(p->path, sizeof(p->path), "%s/%s", dir, e->d_name);
    }
    closedir(d);
    if (ix->total > 1) qsort(ix->entries, (size_t)ix->total, sizeof(ix->entries[0]), cmp_entry);
}

void tui_profiles_free(tui_profile_index_t *ix) {
    free(ix->entries);
    memset(ix, 0, sizeof(*ix));
}

void tui_profiles_years(char *out, size_t len, int start, int end, bool ascii) {
    const char *dash = ascii ? "-" : "–";
    if (start > 0 && end > 0) snprintf(out, len, "%d%s%d", start, dash, end);
    else if (start > 0) snprintf(out, len, "%d%s", start, dash);
    else if (end > 0) snprintf(out, len, "%s%d", dash, end);
    else snprintf(out, len, "%s", ascii ? "-" : "—");
}

static void lower_into(char *out, size_t len, const char *s) {
    size_t n = strlen(out);
    for (const char *p = s ? s : ""; *p && n + 1 < len; p++) out[n++] = (char)tolower((unsigned char)*p);
    out[n] = '\0';
}

static void index_entry(tui_profile_entry_t *p, bool ascii) {
    p->indexed = true;
    char err[256];
    vsp_profile_t *prof = vsp_profile_load(p->path, err, sizeof(err));
    if (!prof) {
        snprintf(p->years, sizeof(p->years), "%s", ascii ? "-" : "—");
        snprintf(p->hay, sizeof(p->hay), "%s", p->file);
        return;
    }
    vsp_profile_info_t info;
    vsp_profile_get_info(prof, &info);
    snprintf(p->make, sizeof(p->make), "%s", info.manufacturer ? info.manufacturer : "");
    snprintf(p->model, sizeof(p->model), "%s", info.model ? info.model : "");
    snprintf(p->gen, sizeof(p->gen), "%s", info.generation ? info.generation : "");
    tui_profiles_years(p->years, sizeof(p->years), info.year_start, info.year_end, ascii);
    p->signals = info.signal_count;
    vsp_select_t sel = { .signals = NULL, .all_signals = true, .acquisition = VSP_ACQ_ANY };
    vsp_decoder_t *dec = vsp_decoder_create(prof, &sel);
    if (dec) {
        for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
            if (vsp_decoder_signal_info(dec, i)->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY) p->obd++;
            else p->can++;
        }
        vsp_decoder_free(dec);
    }
    vsp_profile_free(prof);
    p->ok = true;
    p->hay[0] = '\0';
    lower_into(p->hay, sizeof(p->hay), p->make);
    lower_into(p->hay, sizeof(p->hay), " ");
    lower_into(p->hay, sizeof(p->hay), p->model);
    lower_into(p->hay, sizeof(p->hay), " ");
    lower_into(p->hay, sizeof(p->hay), p->gen);
    lower_into(p->hay, sizeof(p->hay), " ");
    lower_into(p->hay, sizeof(p->hay), p->file);
}

static double mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

bool tui_profiles_step(tui_profile_index_t *ix, int max_count, double budget_ms) {
    double start = budget_ms > 0 ? mono_ms() : 0;
    int n = 0;
    while (ix->indexed < ix->total && n < max_count) {
        index_entry(&ix->entries[ix->indexed], ix->ascii);
        ix->indexed++;
        n++;
        if (budget_ms > 0 && mono_ms() - start >= budget_ms) break;
    }
    return ix->indexed >= ix->total;
}

bool tui_profiles_done(const tui_profile_index_t *ix) {
    return ix->indexed >= ix->total;
}

typedef struct {
    char   path[1024];
    time_t mtime;
} draft_cand_t;

static int cmp_draft(const void *a, const void *b) {
    const draft_cand_t *x = (const draft_cand_t *)a, *y = (const draft_cand_t *)b;
    if (x->mtime != y->mtime) return x->mtime > y->mtime ? -1 : 1;
    return strcmp(y->path, x->path);
}

void tui_profiles_scan_drafts(tui_profile_index_t *ix, const char *captures_root) {
    ix->ndrafts = 0;
    if (!captures_root || !captures_root[0]) return;
    DIR *d = opendir(captures_root);
    if (!d) return;
    draft_cand_t *c = NULL;
    int n = 0, cap = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') continue;
        char path[1024];
        snprintf(path, sizeof(path), "%s/%s/ptcan_discover_draft.vsp", captures_root, e->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
        if (n == cap) {
            int ncap = cap ? cap * 2 : 16;
            draft_cand_t *nc = (draft_cand_t *)realloc(c, (size_t)ncap * sizeof(*nc));
            if (!nc) break;
            c = nc;
            cap = ncap;
        }
        snprintf(c[n].path, sizeof(c[n].path), "%s", path);
        c[n].mtime = st.st_mtime;
        n++;
    }
    closedir(d);
    if (n > 1) qsort(c, (size_t)n, sizeof(c[0]), cmp_draft);
    for (int i = 0; i < n && ix->ndrafts < 5; i++) {
        tui_profile_entry_t *p = &ix->drafts[ix->ndrafts++];
        memset(p, 0, sizeof(*p));
        snprintf(p->path, sizeof(p->path), "%s", c[i].path);
        snprintf(p->file, sizeof(p->file), "%s", c[i].path);
        index_entry(p, ix->ascii);
    }
    free(c);
}

int tui_profiles_filter(const tui_profile_index_t *ix, const char *filter, int *out, int max) {
    char tokens[8][64];
    int nt = 0;
    char buf[256];
    snprintf(buf, sizeof(buf), "%s", filter ? filter : "");
    for (char *t = strtok(buf, " "); t && nt < 8; t = strtok(NULL, " ")) {
        tokens[nt][0] = '\0';
        lower_into(tokens[nt], sizeof(tokens[nt]), t);
        nt++;
    }
    int n = 0;
    for (int i = 0; i < ix->indexed && n < max; i++) {
        const tui_profile_entry_t *p = &ix->entries[i];
        bool ok = true;
        for (int k = 0; k < nt && ok; k++) ok = strstr(p->hay, tokens[k]) != NULL;
        if (ok) out[n++] = i;
    }
    return n;
}

int tui_profiles_find(const tui_profile_index_t *ix, const char *path) {
    if (!path || !path[0]) return -1;
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    for (int i = 0; i < ix->total; i++) {
        if (strcmp(ix->entries[i].path, path) == 0 || strcmp(ix->entries[i].file, base) == 0) return i;
    }
    return -1;
}
