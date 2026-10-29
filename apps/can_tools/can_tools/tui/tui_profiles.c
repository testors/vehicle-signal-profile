/**
 * @file tui_profiles.c
 * @brief Profile index for the TUI picker (C over the vsp_engine API)
 */

#include "tui_profiles.h"

#include <ctype.h>
#include <dirent.h>
#include <limits.h>
#include <math.h>
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

static bool is_file(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
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
#ifdef CAN_TOOLS_PROFILES_DIR
    if (try_dir(CAN_TOOLS_PROFILES_DIR, out, len)) return true;  /* this repository's profiles/ (CMake) */
#endif
    if (!exe_path || !exe_path[0]) return false;

    char real[PATH_MAX];
    if (!realpath(exe_path, real)) return false;
    char *slash = strrchr(real, '/');
    for (int depth = 0; slash && depth < 4; depth++) {
        *slash = '\0';
        char cand[PATH_MAX + 64];
        snprintf(cand, sizeof(cand), "%s/profiles", real);
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
    for (int i = 0; i < ix->total; i++) free(ix->entries[i].frames);
    for (int i = 0; i < 5; i++) free(ix->drafts[i].frames);
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

/* The passive CAN frames: can_frames[] on a CAN bus (or one the profile does
 * not type), not reassembled streams, not the ids of diagnostic requests and
 * responses (those answer queries, they are not broadcast). */
static void index_frames(tui_profile_entry_t *p, const vsp_profile_t *prof, const vsp_profile_info_t *info) {
    free(p->frames);
    p->frames = NULL;
    p->nframes = 0;
    if (info->frame_count <= 0) return;
    p->frames = (tui_profile_frame_t *)calloc((size_t)info->frame_count, sizeof(*p->frames));
    if (!p->frames) return;
    for (int i = 0; i < info->frame_count; i++) {
        vsp_frame_info_t f;
        if (!vsp_profile_frame(prof, i, &f) || f.reassembled || f.frame_id_mask) continue;
        bool can_bus = true;
        for (int b = 0; f.bus && b < info->bus_count; b++) {
            vsp_bus_info_t bus;
            if (vsp_profile_bus(prof, b, &bus) && bus.id && strcmp(bus.id, f.bus) == 0) {
                can_bus = !bus.type || strcmp(bus.type, "can") == 0;
            }
        }
        bool diag = false;
        for (int q = 0; q < info->query_count && !diag; q++) {
            vsp_query_info_t qi;
            if (!vsp_profile_query(prof, q, &qi)) continue;
            diag = (qi.has_tx_id && qi.tx_id == f.frame_id) || (qi.rx_id && qi.rx_id == f.frame_id);
        }
        if (!can_bus || diag) continue;
        bool dup = false;
        for (int k = 0; k < p->nframes && !dup; k++) dup = p->frames[k].id == f.frame_id && p->frames[k].ext == f.extended;
        if (dup) continue;
        p->frames[p->nframes++] = (tui_profile_frame_t){ f.frame_id, f.extended, f.dlc > 0 ? f.dlc : -1 };
    }
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
    index_frames(p, prof, &info);
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
    for (int i = 0; i < 5; i++) {   /* frame lists of the last scan */
        free(ix->drafts[i].frames);
        ix->drafts[i].frames = NULL;
        ix->drafts[i].nframes = 0;
    }
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

bool tui_profiles_resolve(const char *recorded, const char *dir, char *out, size_t len) {
    if (out && len) out[0] = '\0';
    if (!recorded || !recorded[0] || !out || len == 0) return false;
    if (is_file(recorded)) {
        snprintf(out, len, "%s", recorded);
        return true;
    }
    if (!dir || !dir[0]) return false;
    const char *base = strrchr(recorded, '/');
    base = base ? base + 1 : recorded;
    if (!base[0]) return false;
    char cand[PATH_MAX + 256];
    int n = snprintf(cand, sizeof(cand), "%s/%s", dir, base);
    if (n < 0 || (size_t)n >= sizeof(cand) || (size_t)n >= len || !is_file(cand)) return false;
    snprintf(out, len, "%s", cand);
    return true;
}

/*******************************************************************************
 * Vehicle guess from the frames on the bus
 ******************************************************************************/

typedef struct {
    uint64_t key;   /* ext << 32 | id */
    int      dlc;
    int      count; /* profiles listing it (df), or unused */
} frame_key_t;

static uint64_t frame_key(uint32_t id, bool ext) {
    return ((uint64_t)(ext ? 1 : 0) << 32) | id;
}

static int cmp_frame_key(const void *a, const void *b) {
    uint64_t x = ((const frame_key_t *)a)->key, y = ((const frame_key_t *)b)->key;
    return (x > y) - (x < y);
}

static const frame_key_t *find_key(const frame_key_t *keys, int n, uint64_t key) {
    frame_key_t probe = { key, 0, 0 };
    return (const frame_key_t *)bsearch(&probe, keys, (size_t)n, sizeof(*keys), cmp_frame_key);
}

static int cmp_guess(const void *a, const void *b) {
    const tui_profile_guess_t *x = (const tui_profile_guess_t *)a, *y = (const tui_profile_guess_t *)b;
    if (x->score != y->score) return x->score > y->score ? -1 : 1;
    return x->entry - y->entry;
}

#define GUESS_MISS_COST  0.3
#define GUESS_KEEP_RATIO 0.6

int tui_profiles_guess(const tui_profile_index_t *ix, const tui_seen_frame_t *seen, int nseen,
                       tui_profile_guess_t *out, int max) {
    if (!ix || !tui_profiles_done(ix) || !seen || nseen <= 0 || !out || max <= 0) return 0;

    /* document frequency of every frame id over the profiles that list frames */
    int total = 0, profiles = 0;
    for (int i = 0; i < ix->total; i++) {
        total += ix->entries[i].nframes;
        profiles += ix->entries[i].nframes > 0;
    }
    if (total == 0) return 0;
    frame_key_t *df = (frame_key_t *)malloc((size_t)total * sizeof(*df));
    frame_key_t *obs = (frame_key_t *)malloc((size_t)nseen * sizeof(*obs));
    tui_profile_guess_t *all = (tui_profile_guess_t *)malloc((size_t)ix->total * sizeof(*all));
    if (!df || !obs || !all) {
        free(df);
        free(obs);
        free(all);
        return 0;
    }
    int n = 0;
    for (int i = 0; i < ix->total; i++) {
        for (int k = 0; k < ix->entries[i].nframes; k++) {
            df[n++] = (frame_key_t){ frame_key(ix->entries[i].frames[k].id, ix->entries[i].frames[k].ext), 0, 1 };
        }
    }
    qsort(df, (size_t)n, sizeof(*df), cmp_frame_key);
    int nd = 0;
    for (int i = 0; i < n; i++) {
        if (nd > 0 && df[nd - 1].key == df[i].key) df[nd - 1].count++;
        else df[nd++] = df[i];
    }
    for (int i = 0; i < nseen; i++) obs[i] = (frame_key_t){ frame_key(seen[i].id, seen[i].ext), seen[i].dlc, 0 };
    qsort(obs, (size_t)nseen, sizeof(*obs), cmp_frame_key);

    int na = 0;
    double best = 0.0;
    for (int i = 0; i < ix->total; i++) {
        const tui_profile_entry_t *e = &ix->entries[i];
        if (e->nframes == 0) continue;
        double score = 0.0;
        int hits = 0;
        for (int k = 0; k < e->nframes; k++) {
            uint64_t key = frame_key(e->frames[k].id, e->frames[k].ext);
            const frame_key_t *d = find_key(df, nd, key);
            double w = log((double)profiles / (double)(d ? d->count : 1));
            const frame_key_t *o = find_key(obs, nseen, key);
            if (o && (e->frames[k].dlc < 0 || e->frames[k].dlc == o->dlc)) {
                score += w;
                hits++;
            } else {
                score -= GUESS_MISS_COST * w;
            }
        }
        all[na++] = (tui_profile_guess_t){ i, score, hits, e->nframes };
        if (score > best) best = score;
    }
    free(df);
    free(obs);

    int kept = 0;
    if (best > 0.0) {
        qsort(all, (size_t)na, sizeof(*all), cmp_guess);
        for (int i = 0; i < na; i++) {
            const tui_profile_guess_t *g = &all[i];
            int need = g->frames < 3 ? g->frames : 3;
            if (g->score <= 0.0 || g->score < GUESS_KEEP_RATIO * best) break;
            if (g->hits < need || 2 * g->hits < g->frames) continue;
            if (kept == max) {   /* more than max qualify: not narrowed down */
                kept = 0;
                break;
            }
            out[kept++] = *g;
        }
    }
    free(all);
    return kept;
}
