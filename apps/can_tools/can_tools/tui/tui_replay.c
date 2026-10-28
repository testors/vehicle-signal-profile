/**
 * @file tui_replay.c
 * @brief Load a capture for TUI replay (Pure C)
 */

#include "tui_replay.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

int64_t tui_replay_append(frame_list_t *dst, const frame_list_t *src, int64_t max_gap_ms) {
    int64_t shift = 0;
    if (dst->count > 0 && src->count > 0) {
        int64_t gap = src->frames[0].timestamp_ms - dst->frames[dst->count - 1].timestamp_ms;
        if (gap > max_gap_ms) shift = gap - max_gap_ms;
    }
    for (int i = 0; i < src->count; i++) {
        can_frame_t f = src->frames[i];
        f.timestamp_ms -= shift;
        if (!frame_list_push(dst, &f)) break;
    }
    return shift;
}

static int cmp_str(const void *a, const void *b) {
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

static bool ends_with(const char *s, const char *suffix) {
    size_t n = strlen(s), m = strlen(suffix);
    return n >= m && strcmp(s + n - m, suffix) == 0;
}

static bool is_phase_dump(const char *name) {
    return name[0] >= '0' && name[0] <= '9' && name[1] >= '0' && name[1] <= '9' &&
           name[2] == '_' && ends_with(name, ".frames.tsv");
}

static bool is_live_dump(const char *name) {
    return strncmp(name, "live", 4) == 0 && ends_with(name, ".frames.tsv");
}

static const char *base_name(const char *path) {
    size_t n = strlen(path);
    while (n > 1 && path[n - 1] == '/') n--;
    const char *end = path + n;
    const char *p = end;
    while (p > path && p[-1] != '/') p--;
    static char buf[256];
    size_t len = (size_t)(end - p);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, p, len);
    buf[len] = '\0';
    return buf;
}

static bool join_path(char *buf, size_t len, const char *dir, const char *leaf) {
    int n = snprintf(buf, len, "%s/%s", dir, leaf);
    return n >= 0 && (size_t)n < len;
}

#define REPLAY_MAX_FILES 256   /* frame files per folder */
#define PROBE_LINES      32    /* lines read to find a frame row */

/* A frame row among the first PROBE_LINES lines (a header-only recording has
 * none). Reads only those lines. */
static bool file_has_frames(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    char line[FRAME_TSV_LINE_MAX];
    bool found = false;
    for (int i = 0; i < PROBE_LINES && !found && fgets(line, sizeof(line), f); i++) {
        line[strcspn(line, "\r\n")] = '\0';
        can_frame_t frame;
        found = frame_tsv_parse_row(line, &frame);
    }
    fclose(f);
    return found;
}

typedef struct {
    char   *names[REPLAY_MAX_FILES];   /* name order */
    int     count;
    int64_t bytes;
} replay_files_t;

static void files_free(replay_files_t *f) {
    for (int i = 0; i < f->count; i++) free(f->names[i]);
    f->count = 0;
    f->bytes = 0;
}

/* Sort f and drop the files without a frame row. */
static void files_keep_frames(const char *dir, replay_files_t *f) {
    qsort(f->names, (size_t)f->count, sizeof(char *), cmp_str);
    int n = 0;
    f->bytes = 0;
    for (int i = 0; i < f->count; i++) {
        char file[1024];
        struct stat st;
        if (join_path(file, sizeof(file), dir, f->names[i]) && stat(file, &st) == 0 && S_ISREG(st.st_mode) &&
            file_has_frames(file)) {
            f->names[n++] = f->names[i];
            f->bytes += st.st_size;
        } else {
            free(f->names[i]);
        }
    }
    f->count = n;
}

/* The files a session folder replays (the list and the loader both use this):
 * its live dumps that have frames if any, else its phase dumps that have
 * frames. -1 when the folder cannot be read. */
static int folder_files(const char *dir, replay_files_t *out) {
    replay_files_t phase;
    memset(out, 0, sizeof(*out));
    memset(&phase, 0, sizeof(phase));
    DIR *d = opendir(dir);
    if (!d) return -1;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        replay_files_t *f = is_live_dump(de->d_name) ? out : (is_phase_dump(de->d_name) ? &phase : NULL);
        if (!f || f->count >= REPLAY_MAX_FILES) continue;
        char *name = strdup(de->d_name);
        if (name) f->names[f->count++] = name;
    }
    closedir(d);
    files_keep_frames(dir, out);
    if (out->count > 0) {
        files_free(&phase);
    } else {
        files_keep_frames(dir, &phase);
        *out = phase;
    }
    return 0;
}

static int load_one(const char *path, const char *display, tui_replay_data_t *out,
                    char *err, size_t err_len) {
    frame_list_t list;
    int rc = frame_tsv_load(path, display, &list);
    if (rc <= 0) {
        snprintf(err, err_len, rc == 0 ? "not found: %s" : "cannot read %s", path);
        return -1;
    }
    tui_replay_append(&out->frames, &list, TUI_REPLAY_MAX_GAP_MS);
    out->malformed += list.malformed;
    out->files++;
    frame_list_free(&list);
    return 0;
}

int tui_replay_load(const char *path, tui_replay_data_t *out, char *err, size_t err_len) {
    memset(out, 0, sizeof(*out));
    if (!path || !path[0]) {
        snprintf(err, err_len, "no replay path");
        return -1;
    }
    snprintf(out->name, sizeof(out->name), "%s", base_name(path));

    struct stat st;
    if (stat(path, &st) != 0) {
        snprintf(err, err_len, "%s: %s", path, strerror(errno));
        return -1;
    }
    if (!S_ISDIR(st.st_mode)) return load_one(path, base_name(path), out, err, err_len);

    replay_files_t files;
    if (folder_files(path, &files) != 0) {
        snprintf(err, err_len, "%s: %s", path, strerror(errno));
        return -1;
    }
    int rc = 0;
    if (files.count == 0) {
        snprintf(err, err_len, "%s: no live*.frames.tsv or NN_*.frames.tsv with frames", path);
        rc = -1;
    }
    for (int i = 0; i < files.count && rc == 0; i++) {
        char file[1024];
        snprintf(file, sizeof(file), "%s/%s", path, files.names[i]);
        rc = load_one(file, files.names[i], out, err, err_len);
    }
    files_free(&files);
    if (rc != 0) tui_replay_free(out);
    return rc;
}

void tui_replay_free(tui_replay_data_t *data) {
    frame_list_free(&data->frames);
    data->files = 0;
}

bool tui_replay_has_frames(const char *path, char *err, size_t err_len) {
    struct stat st;
    if (!path || !path[0]) {
        snprintf(err, err_len, "no replay path");
        return false;
    }
    if (stat(path, &st) != 0) {
        snprintf(err, err_len, "%s", strerror(errno));
        return false;
    }
    bool ok = false;
    if (S_ISDIR(st.st_mode)) {
        replay_files_t files;
        if (folder_files(path, &files) != 0) {
            snprintf(err, err_len, "%s", strerror(errno));
            return false;
        }
        ok = files.count > 0;
        files_free(&files);
    } else {
        ok = S_ISREG(st.st_mode) && file_has_frames(path);
    }
    if (!ok) snprintf(err, err_len, "no frames");
    return ok;
}

/*******************************************************************************
 * Replay list
 ******************************************************************************/

static void format_bytes(char *out, size_t len, int64_t bytes) {
    if (bytes >= 1024 * 1024) snprintf(out, len, "%.1f MB", (double)bytes / (1024.0 * 1024.0));
    else if (bytes >= 1024) snprintf(out, len, "%.1f KB", (double)bytes / 1024.0);
    else snprintf(out, len, "%lld B", (long long)bytes);
}

static void format_when(char *out, size_t len, int64_t mtime) {
    time_t t = (time_t)mtime;
    struct tm tm_local;
    if (!localtime_r(&t, &tm_local) || strftime(out, len, "%Y-%m-%d %H:%M", &tm_local) == 0) {
        snprintf(out, len, "?");
    }
}

static bool list_push(tui_replay_list_t *list, const tui_replay_entry_t *entry) {
    if (list->count == list->cap) {
        int cap = list->cap ? list->cap * 2 : 16;
        tui_replay_entry_t *n = (tui_replay_entry_t *)realloc(list->entries, (size_t)cap * sizeof(*n));
        if (!n) return false;
        list->entries = n;
        list->cap = cap;
    }
    list->entries[list->count++] = *entry;
    return true;
}

static int cmp_entry(const void *a, const void *b) {
    const tui_replay_entry_t *x = (const tui_replay_entry_t *)a;
    const tui_replay_entry_t *y = (const tui_replay_entry_t *)b;
    if (x->mtime != y->mtime) return (x->mtime < y->mtime) - (x->mtime > y->mtime);
    return strcmp(x->name, y->name);
}

int tui_replay_list_scan(tui_replay_list_t *list, const char *root, char *err, size_t err_len) {
    if (!list) return -1;
    if (!root || !root[0]) {
        snprintf(err, err_len, "no captures folder");
        return -1;
    }
    snprintf(list->root, sizeof(list->root), "%s", root);

    DIR *d = opendir(root);
    if (!d) {
        snprintf(err, err_len, "%s: %s", root, strerror(errno));
        return -1;
    }
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char path[1024];
        if (!join_path(path, sizeof(path), root, de->d_name)) continue;
        struct stat st;
        if (stat(path, &st) != 0) continue;

        tui_replay_entry_t e;
        memset(&e, 0, sizeof(e));
        snprintf(e.path, sizeof(e.path), "%s", path);
        snprintf(e.name, sizeof(e.name), "%s", de->d_name);
        e.mtime = (int64_t)st.st_mtime;
        format_when(e.when, sizeof(e.when), e.mtime);

        if (S_ISDIR(st.st_mode)) {
            replay_files_t files;
            if (folder_files(path, &files) != 0) continue;
            e.files = files.count;
            format_bytes(e.size, sizeof(e.size), files.bytes);
            files_free(&files);
            if (e.files == 0) continue;
        } else if (S_ISREG(st.st_mode) && ends_with(de->d_name, ".frames.tsv") && file_has_frames(path)) {
            e.files = 1;
            format_bytes(e.size, sizeof(e.size), (int64_t)st.st_size);
        } else {
            continue;
        }
        if (!list_push(list, &e)) {
            closedir(d);
            snprintf(err, err_len, "out of memory");
            return -1;
        }
    }
    closedir(d);

    if (list->count > 1) qsort(list->entries, (size_t)list->count, sizeof(list->entries[0]), cmp_entry);
    if (list->count > TUI_REPLAY_LIST_MAX) list->count = TUI_REPLAY_LIST_MAX;   /* keep the newest */
    return 0;
}

void tui_replay_list_free(tui_replay_list_t *list) {
    if (!list) return;
    free(list->entries);
    list->entries = NULL;
    list->count = 0;
    list->cap = 0;
    list->root[0] = '\0';
}
