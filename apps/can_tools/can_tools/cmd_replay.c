/**
 * @file cmd_replay.c
 * @brief Offline replay of a saved can_discover capture session
 *
 * Re-runs PT-CAN signal discovery on a capture session folder written by
 * can_discover / can_reader discover (NN_<slug>.frames.tsv + manifest.json),
 * so the engine can be validated on real captures without the car.
 *
 * Usage: can_tools replay <session_dir> [--out-dir DIR]
 *        discover_replay <session_dir> [--out-dir DIR]   (alias)
 *
 * Moved from tools/discover_replay.c (main renamed cmd_replay_main).
 *
 * Writes result.json, ptcan_discover_draft.dbc and ptcan_discover_draft.vsp
 * into the output directory (default: <session_dir>/replay). The result JSON
 * is also printed on stdout; logs and the per-signal summary go to stderr.
 *
 * Plain C11 + POSIX; depends only on can_discover_core and common/frame_tsv.c.
 */

#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "can_discover.h"
#include "frame_tsv.h"
#include "commands.h"

#define REPLAY_PATH_MAX       1024

/*******************************************************************************
 * Phase naming (must match disc_phase_slug() in common/discover_session.m)
 ******************************************************************************/

static const char *const kPhaseSlugs[DISC_NUM_PHASES] = {
    "baseline", "steering", "throttle", "brake", "gear", "wheel_speed"
};

/*******************************************************************************
 * Small helpers
 ******************************************************************************/

/** Join dir and leaf into buf. Returns false if the result was truncated. */
static bool join_path(char *buf, size_t len, const char *dir, const char *leaf) {
    size_t dir_len = strlen(dir);
    const char *sep = (dir_len > 0 && dir[dir_len - 1] == '/') ? "" : "/";
    int n = snprintf(buf, len, "%s%s%s", dir, sep, leaf);
    return n >= 0 && (size_t)n < len;
}

static bool is_directory(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

/** mkdir -p */
static bool make_dirs(const char *path) {
    char buf[REPLAY_PATH_MAX];
    size_t len = strlen(path);
    if (len == 0 || len >= sizeof(buf)) return false;
    memcpy(buf, path, len + 1);

    for (size_t i = 1; i <= len; i++) {
        if (buf[i] != '/' && buf[i] != '\0') continue;
        char saved = buf[i];
        buf[i] = '\0';
        if (mkdir(buf, 0777) != 0 && errno != EEXIST) return false;
        buf[i] = saved;
    }
    return is_directory(path);
}

static int compare_int64(const void *a, const void *b) {
    int64_t x = *(const int64_t *)a;
    int64_t y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static void json_print_string(FILE *f, const char *s) {
    fputc('"', f);
    for (; *s; s++) {
        switch (*s) {
            case '"':  fputs("\\\"", f); break;
            case '\\': fputs("\\\\", f); break;
            case '\n': fputs("\\n", f); break;
            case '\r': fputs("\\r", f); break;
            case '\t': fputs("\\t", f); break;
            default:
                if ((unsigned char)*s < 0x20) {
                    fprintf(f, "\\u%04x", (unsigned char)*s);
                } else {
                    fputc(*s, f);
                }
                break;
        }
    }
    fputc('"', f);
}

/*******************************************************************************
 * Frame TSV loading (frame_list_t, frame_tsv_load: common/frame_tsv.c)
 ******************************************************************************/

/**
 * Time covered by the frames: (last - first) plus one median inter-frame
 * interval, in seconds. Returns 0 if fewer than two frames.
 */
static double frames_covered_s(const frame_list_t *list) {
    if (list->count < 2) return 0.0;

    int64_t first = list->frames[0].timestamp_ms;
    int64_t last = first;
    int64_t *gaps = (int64_t *)malloc((size_t)(list->count - 1) * sizeof(*gaps));
    if (!gaps) return 0.0;

    for (int i = 0; i < list->count; i++) {
        int64_t ts = list->frames[i].timestamp_ms;
        if (ts < first) first = ts;
        if (ts > last) last = ts;
        if (i > 0) {
            int64_t gap = ts - list->frames[i - 1].timestamp_ms;
            gaps[i - 1] = gap > 0 ? gap : 0;
        }
    }
    qsort(gaps, (size_t)(list->count - 1), sizeof(*gaps), compare_int64);
    int64_t median_gap = gaps[(list->count - 1) / 2];
    free(gaps);

    return (double)(last - first + median_gap) / 1000.0;
}

/*******************************************************************************
 * Manifest (hand-rolled scan of the writer's flat per-phase objects)
 ******************************************************************************/

typedef struct {
    bool   present;
    bool   has_analyzed;
    double analyzed_duration_s;
    bool   has_actual;
    double actual_duration_s;
    bool   has_capture_dropped;
    int    capture_dropped_frames;
} manifest_phase_t;

static char *read_text_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;

    size_t cap = 4096, len = 0;
    char *buf = (char *)malloc(cap);
    while (buf) {
        if (len + 1 >= cap) {
            char *grown = (char *)realloc(buf, cap * 2);
            if (!grown) {
                free(buf);
                buf = NULL;
                break;
            }
            buf = grown;
            cap *= 2;
        }
        size_t got = fread(buf + len, 1, cap - len - 1, f);
        len += got;
        if (got == 0) break;
    }
    fclose(f);
    if (buf) buf[len] = '\0';
    return buf;
}

/** Find `"key"` in a NUL-terminated JSON object and return the value start. */
static const char *json_find_value(const char *obj, const char *key) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(obj, pattern);
    if (!p) return NULL;
    p += strlen(pattern);
    while (isspace((unsigned char)*p)) p++;
    if (*p != ':') return NULL;
    p++;
    while (isspace((unsigned char)*p)) p++;
    return p;
}

static bool json_find_number(const char *obj, const char *key, double *out) {
    const char *v = json_find_value(obj, key);
    if (!v) return false;
    char *end = NULL;
    double d = strtod(v, &end);
    if (end == v) return false;  /* null or not a number */
    *out = d;
    return true;
}

/** Return the closing brace matching the '{' at p (string-aware), or NULL. */
static const char *json_object_end(const char *p) {
    int depth = 0;
    bool in_string = false;
    for (; *p; p++) {
        if (in_string) {
            if (*p == '\\' && p[1]) p++;
            else if (*p == '"') in_string = false;
            continue;
        }
        if (*p == '"') in_string = true;
        else if (*p == '{') depth++;
        else if (*p == '}' && --depth == 0) return p;
    }
    return NULL;
}

/** Returns false if the manifest is absent or has no usable "phases" array. */
static bool load_manifest(const char *path, manifest_phase_t out[DISC_NUM_PHASES]) {
    memset(out, 0, sizeof(manifest_phase_t) * DISC_NUM_PHASES);

    char *text = read_text_file(path);
    if (!text) return false;

    bool any = false;
    const char *p = strstr(text, "\"phases\"");
    if (p) p = strchr(p, '[');
    if (p) p++;

    while (p && *p) {
        while (*p && (isspace((unsigned char)*p) || *p == ',')) p++;
        if (*p != '{') break;  /* ']' or unexpected content */

        const char *end = json_object_end(p);
        if (!end) break;

        size_t obj_len = (size_t)(end - p) + 1;
        char *obj = (char *)malloc(obj_len + 1);
        if (!obj) break;
        memcpy(obj, p, obj_len);
        obj[obj_len] = '\0';

        double index = -1.0;
        if (json_find_number(obj, "index", &index) &&
            index >= 0 && index < DISC_NUM_PHASES && index == (double)(int)index) {
            manifest_phase_t *mp = &out[(int)index];
            double v = 0.0;
            mp->present = true;
            if (json_find_number(obj, "analyzed_duration_s", &v)) {
                mp->has_analyzed = true;
                mp->analyzed_duration_s = v;
            }
            if (json_find_number(obj, "actual_duration_s", &v)) {
                mp->has_actual = true;
                mp->actual_duration_s = v;
            }
            if (json_find_number(obj, "capture_dropped_frames", &v)) {
                mp->has_capture_dropped = true;
                mp->capture_dropped_frames = (int)v;
            }
            any = true;
        }
        free(obj);
        p = end + 1;
    }

    free(text);
    return any;
}

/*******************************************************************************
 * Output
 ******************************************************************************/

/** Same object shape as disc_write_result_json_object() in common/discover_session.m */
static void write_result_json_object(FILE *f,
                                     bool have_bus_mode,
                                     disc_bus_mode_t bus_mode,
                                     const char *dump_dir,
                                     const char *dbc_path,
                                     const char *vsp_path,
                                     const disc_result_t *result) {
    bool first = true;

    fprintf(f, "{\n");

    if (have_bus_mode) {
        fprintf(f, "  \"bus_mode\":");
        json_print_string(f, disc_bus_mode_name(bus_mode));
        first = false;
    }

    if (dump_dir && dump_dir[0]) {
        fprintf(f, "%s  \"dump_dir\":", first ? "" : ",\n");
        json_print_string(f, dump_dir);
        first = false;
    }

    if (dbc_path && dbc_path[0]) {
        fprintf(f, "%s  \"dbc_path\":", first ? "" : ",\n");
        json_print_string(f, dbc_path);
        first = false;
    }

    if (vsp_path && vsp_path[0]) {
        fprintf(f, "%s  \"vsp_path\":", first ? "" : ",\n");
        json_print_string(f, vsp_path);
        first = false;
    }

    for (int s = 0; result && s < DISC_SIG_COUNT; s++) {
        if (!result->found[s]) continue;

        const disc_candidate_t *c = &result->signals[s];
        fprintf(f, "%s  ", first ? "" : ",\n");
        json_print_string(f, disc_signal_name((disc_signal_t)s));
        fprintf(f, ":{\"can_id\":\"0x%03X\",\"dlc\":%d,\"hz\":%.1f",
                c->can_id, c->dlc, c->hz);
        if (c->byte_idx >= 0) fprintf(f, ",\"byte\":%d", c->byte_idx);
        if (c->byte2_idx >= 0) fprintf(f, ",\"byte2\":%d", c->byte2_idx);
        if (c->start_bit >= 0 && c->bit_length > 0) {
            fprintf(f, ",\"start_bit\":%d,\"bit_length\":%d,\"byte_order\":%d",
                    c->start_bit, c->bit_length, c->byte_order);
        }
        fprintf(f, ",\"score\":%.1f,\"confidence\":\"%s\"",
                c->score, disc_confidence_name(c->confidence));
        fprintf(f, ",\"evidence\":{\"phase\":%.1f,\"specificity\":%.1f,"
                   "\"shape\":%.1f,\"prior\":%.1f,\"prior_support\":%d,"
                   "\"ambiguity_margin\":%.1f}",
                c->evidence_score, c->specificity_score, c->shape_score,
                c->prior_score, c->prior_support, c->ambiguity_margin);
        if (c->byte2_idx >= 0 && c->endianness != DISC_ENDIAN_UNKNOWN) {
            fprintf(f, ",\"endian\":\"%s\"", disc_endian_name(c->endianness));
        }
        if (c->signedness != DISC_SIGN_UNKNOWN) {
            fprintf(f, ",\"signed\":%s",
                    c->signedness == DISC_SIGN_SIGNED ? "true" : "false");
            if (c->sign_char == 'S') fprintf(f, ",\"sign_bit\":%d", c->sign_bit);
            fprintf(f, ",\"raw_min\":%d,\"raw_max\":%d", c->raw_min, c->raw_max);
        }
        fprintf(f, "}");
        first = false;
    }

    fprintf(f, "\n}\n");
}

static void print_signal_summary(const disc_result_t *result) {
    fprintf(stderr, "\n=== Replay Results (bus mode: %s) ===\n",
            disc_bus_mode_name(result->bus_mode));
    for (int s = 0; s < DISC_SIG_COUNT; s++) {
        const char *name = disc_signal_name((disc_signal_t)s);
        if (!result->found[s]) {
            fprintf(stderr, "  %-12s not found\n", name);
            continue;
        }

        const disc_candidate_t *c = &result->signals[s];
        char layout[48];
        if (c->start_bit >= 0 && c->bit_length > 0) {
            if (c->sign_char == 'S') {
                snprintf(layout, sizeof(layout), "%d|%d@%d+ sign %d",
                         c->start_bit, c->bit_length, c->byte_order, c->sign_bit);
            } else {
                snprintf(layout, sizeof(layout), "%d|%d@%d%c",
                         c->start_bit, c->bit_length, c->byte_order, c->sign_char);
            }
        } else if (c->byte2_idx >= 0) {
            snprintf(layout, sizeof(layout), "byte %d,%d", c->byte_idx, c->byte2_idx);
        } else {
            snprintf(layout, sizeof(layout), "byte %d", c->byte_idx);
        }

        fprintf(stderr, "  %-12s 0x%03X [%d]  %-12s  %7.1f Hz  score=%.1f  confidence=%s\n",
                name, c->can_id, c->dlc, layout, c->hz, c->score,
                disc_confidence_name(c->confidence));
        fprintf(stderr, "  %-12s evidence=%.1f specificity=%.1f shape=%.1f prior=%.1f "
                        "(support %d) margin=%.1f\n",
                "", c->evidence_score, c->specificity_score, c->shape_score,
                c->prior_score, c->prior_support, c->ambiguity_margin);
        if (c->signedness != DISC_SIGN_UNKNOWN) {
            fprintf(stderr, "  %-12s ", "");
            if (c->byte2_idx >= 0 && c->endianness != DISC_ENDIAN_UNKNOWN) {
                fprintf(stderr, "%s-endian ", disc_endian_name(c->endianness));
            }
            fprintf(stderr, "%s  raw=[%d..%d]\n",
                    disc_sign_name(c->signedness), c->raw_min, c->raw_max);
        }
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/

static void print_usage(void) {
    fprintf(stderr,
            "Usage: discover_replay <session_dir> [--out-dir DIR]\n"
            "\n"
            "  Re-runs PT-CAN signal discovery on a saved capture session\n"
            "  (NN_<phase>.frames.tsv + manifest.json from can_discover).\n"
            "\n"
            "  --out-dir DIR  Where to write result.json and the draft DBC/VSP.\n"
            "                 Default: <session_dir>/replay\n"
            "\n"
            "  Phase durations come from manifest.json (analyzed_duration_s,\n"
            "  else actual_duration_s), or from frame timestamps if absent.\n"
            "  Result JSON on stdout, logs on stderr.\n");
}

int cmd_replay_main(int argc, char *argv[]) {
    const char *session_dir = NULL;
    const char *out_dir_arg = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage();
            return 0;
        } else if (strcmp(argv[i], "--out-dir") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "[ERROR] --out-dir requires a directory path\n");
                return 2;
            }
            out_dir_arg = argv[++i];
        } else if (strncmp(argv[i], "--out-dir=", 10) == 0) {
            out_dir_arg = argv[i] + 10;
        } else if (argv[i][0] == '-' && argv[i][1] != '\0') {
            fprintf(stderr, "[ERROR] Unknown option: %s\n", argv[i]);
            print_usage();
            return 2;
        } else if (!session_dir) {
            session_dir = argv[i];
        } else {
            fprintf(stderr, "[ERROR] Unexpected argument: %s\n", argv[i]);
            print_usage();
            return 2;
        }
    }

    if (!session_dir || !session_dir[0] || (out_dir_arg && !out_dir_arg[0])) {
        print_usage();
        return 2;
    }
    if (!is_directory(session_dir)) {
        fprintf(stderr, "[ERROR] Not a directory: %s\n", session_dir);
        return 1;
    }

    char out_dir[REPLAY_PATH_MAX];
    if (out_dir_arg) {
        snprintf(out_dir, sizeof(out_dir), "%s", out_dir_arg);
    } else if (!join_path(out_dir, sizeof(out_dir), session_dir, "replay")) {
        fprintf(stderr, "[ERROR] Path too long: %s\n", session_dir);
        return 1;
    }

    /* Manifest durations */
    char manifest_path[REPLAY_PATH_MAX];
    manifest_phase_t manifest[DISC_NUM_PHASES];
    bool have_manifest = false;
    if (join_path(manifest_path, sizeof(manifest_path), session_dir, "manifest.json")) {
        have_manifest = load_manifest(manifest_path, manifest);
    } else {
        memset(manifest, 0, sizeof(manifest));
    }
    if (!have_manifest) {
        fprintf(stderr, "[WARN] No usable manifest.json; durations will be "
                        "estimated from frame timestamps\n");
    }

    disc_phase_t phases[DISC_NUM_PHASES];
    disc_raw_phase_t raw_phases[DISC_NUM_PHASES];
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        disc_phase_init(&phases[i]);
        disc_raw_phase_init(&raw_phases[i]);
    }

    int exit_code = 0;
    int loaded_phases = 0;
    bool baseline_loaded = false;

    fprintf(stderr, "[INFO] Session: %s\n", session_dir);
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        char leaf[64];
        char path[REPLAY_PATH_MAX];
        snprintf(leaf, sizeof(leaf), "%02d_%s.frames.tsv", i, kPhaseSlugs[i]);
        if (!join_path(path, sizeof(path), session_dir, leaf)) {
            fprintf(stderr, "[ERROR] Path too long: %s\n", session_dir);
            exit_code = 1;
            goto cleanup;
        }

        frame_list_t frames;
        int rc = frame_tsv_load(path, leaf, &frames);
        if (rc < 0) {
            exit_code = 1;
            goto cleanup;
        }
        if (rc == 0) {
            fprintf(stderr, "  %-22s skipped (file not found)\n", leaf);
            continue;
        }

        /* Duration: manifest analyzed -> manifest actual -> frame timestamps */
        const manifest_phase_t *mp = &manifest[i];
        double covered_s = frames_covered_s(&frames);
        double duration_s = 0.0;
        const char *source = "frame timestamps";
        if (mp->present && mp->has_analyzed && mp->analyzed_duration_s > 0) {
            duration_s = mp->analyzed_duration_s;
            source = "manifest analyzed_duration_s";
        } else if (mp->present && mp->has_actual && mp->actual_duration_s > 0) {
            duration_s = mp->actual_duration_s;
            source = "manifest actual_duration_s";
            /* Sessions recorded before analyzed_duration_s existed used a
             * fixed-size capture buffer that silently stopped storing frames.
             * If the frames cover clearly less than the wall-clock time, use
             * the covered time so Hz is not underestimated. */
            if (frames.count >= 2 && covered_s > 0 && covered_s < 0.8 * duration_s) {
                fprintf(stderr, "[WARN] %s: frames span only %.3f s of %.3f s (old capture "
                                "buffer was full); using the covered time for Hz\n",
                        leaf, covered_s, duration_s);
                duration_s = covered_s;
                source = "frame timestamps (truncated old capture)";
            }
        } else {
            duration_s = covered_s;
        }
        if (duration_s <= 0 && frames.count > 0) {
            duration_s = 1.0;
            source = "default (could not infer)";
            fprintf(stderr, "[WARN] %s: cannot infer duration from %d frame(s); using 1.0 s\n",
                    leaf, frames.count);
        }

        for (int f = 0; f < frames.count; f++) {
            disc_phase_add_frame(&phases[i], frames.frames[f].can_id,
                                 frames.frames[f].data, frames.frames[f].dlc);
        }
        disc_phase_finalize(&phases[i], duration_s);
        bool stored = disc_raw_phase_set(&raw_phases[i], frames.frames, frames.count);
        int frame_count = frames.count;
        int malformed = frames.malformed;
        frame_list_free(&frames);
        if (!stored) {
            fprintf(stderr, "[ERROR] Out of memory storing raw frames for %s\n", leaf);
            exit_code = 1;
            goto cleanup;
        }

        fprintf(stderr, "  %-22s %7d frames, %4d IDs, %8.3f s (%s), %.1f frames/s",
                leaf, frame_count, phases[i].id_count, duration_s, source,
                duration_s > 0 ? (double)phases[i].total_frames / duration_s : 0.0);
        if (malformed > 0) fprintf(stderr, ", %d malformed", malformed);
        if (phases[i].dropped_frames > 0) {
            fprintf(stderr, ", %d dropped (ID table full)", phases[i].dropped_frames);
        }
        fprintf(stderr, "\n");

        if (mp->has_capture_dropped && mp->capture_dropped_frames > 0) {
            fprintf(stderr, "[WARN] %s: live capture dropped %d frames (buffer full)\n",
                    leaf, mp->capture_dropped_frames);
        }
        if (frame_count >= 2 && covered_s < 0.8 * duration_s) {
            fprintf(stderr, "[WARN] %s: frames span only %.3f s of %.3f s; the capture may "
                            "have been truncated, so Hz will be underestimated\n",
                    leaf, covered_s, duration_s);
        }

        loaded_phases++;
        if (i == DISC_PHASE_BASELINE) baseline_loaded = true;
    }

    if (loaded_phases == 0) {
        fprintf(stderr, "[ERROR] No phase dumps (NN_<phase>.frames.tsv) found in %s\n",
                session_dir);
        exit_code = 1;
        goto cleanup;
    }
    if (!baseline_loaded) {
        fprintf(stderr, "[WARN] Baseline phase missing; results will be unreliable\n");
    }

    if (!make_dirs(out_dir)) {
        fprintf(stderr, "[ERROR] Cannot create output dir %s: %s\n", out_dir, strerror(errno));
        exit_code = 1;
        goto cleanup;
    }

    fprintf(stderr, "[INFO] Analyzing %d phase(s)...\n", loaded_phases);
    disc_result_t result;
    disc_result_init(&result);
    disc_analyze_with_raw(phases, raw_phases, &result);

    char dbc_path[REPLAY_PATH_MAX];
    char vsp_path[REPLAY_PATH_MAX];
    char result_path[REPLAY_PATH_MAX];
    if (!join_path(dbc_path, sizeof(dbc_path), out_dir, "ptcan_discover_draft.dbc") ||
        !join_path(vsp_path, sizeof(vsp_path), out_dir, "ptcan_discover_draft.vsp") ||
        !join_path(result_path, sizeof(result_path), out_dir, "result.json")) {
        fprintf(stderr, "[ERROR] Path too long: %s\n", out_dir);
        exit_code = 1;
        goto cleanup;
    }

    bool dbc_written = disc_write_draft_dbc(dbc_path, phases, raw_phases, &result);
    if (dbc_written) {
        fprintf(stderr, "[INFO] Draft DBC written: %s\n", dbc_path);
    } else {
        fprintf(stderr, "[ERROR] Failed to write draft DBC: %s\n", dbc_path);
        exit_code = 1;
    }

    bool vsp_written = disc_write_draft_vsp(vsp_path, phases, raw_phases, &result);
    if (vsp_written) {
        fprintf(stderr, "[INFO] Draft VSP written: %s\n", vsp_path);
    } else {
        fprintf(stderr, "[ERROR] Failed to write draft VSP: %s\n", vsp_path);
        exit_code = 1;
    }

    FILE *rf = fopen(result_path, "w");
    if (rf) {
        write_result_json_object(rf, baseline_loaded, result.bus_mode, session_dir,
                                 dbc_written ? dbc_path : NULL,
                                 vsp_written ? vsp_path : NULL, &result);
        if (fclose(rf) != 0) {
            fprintf(stderr, "[ERROR] Failed to write result JSON: %s\n", result_path);
            exit_code = 1;
        } else {
            fprintf(stderr, "[INFO] Result JSON written: %s\n", result_path);
        }
    } else {
        fprintf(stderr, "[ERROR] Cannot open %s: %s\n", result_path, strerror(errno));
        exit_code = 1;
    }

    write_result_json_object(stdout, baseline_loaded, result.bus_mode, session_dir,
                             dbc_written ? dbc_path : NULL,
                             vsp_written ? vsp_path : NULL, &result);
    print_signal_summary(&result);

cleanup:
    for (int i = 0; i < DISC_NUM_PHASES; i++) {
        disc_raw_phase_free(&raw_phases[i]);
        disc_phase_free(&phases[i]);
    }
    return exit_code;
}
