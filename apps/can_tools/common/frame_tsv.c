/**
 * @file frame_tsv.c
 * @brief Capture dump (*.frames.tsv) reader (Pure C)
 *
 * Moved from tools/discover_replay.c.
 */

#include "frame_tsv.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void frame_tsv_write_header(FILE *f) {
    frame_tsv_write_header_ex(f, false);
}

void frame_tsv_write_header_ex(FILE *f, bool with_us) {
    fprintf(f, "timestamp_ms\tcan_id\tis_extended\tdlc\tdata_hex%s\n", with_us ? "\ttimestamp_us" : "");
}

void frame_tsv_write_row(FILE *f, const can_frame_t *frame) {
    fprintf(f, "%lld\t0x%X\t%d\t%u\t",
            (long long)frame->timestamp_ms,
            frame->can_id,
            frame->is_extended ? 1 : 0,
            frame->dlc);
    for (int b = 0; b < frame->dlc; b++) {
        fprintf(f, "%02X", frame->data[b]);
    }
    if (frame->hw_timestamp) fprintf(f, "\t%lld", (long long)can_frame_time_us(frame));
    fputc('\n', f);
}

bool frame_list_push(frame_list_t *list, const can_frame_t *frame) {
    if (list->count >= list->capacity) {
        int new_capacity = list->capacity > 0 ? list->capacity * 2 : 4096;
        can_frame_t *grown = (can_frame_t *)realloc(list->frames,
                                                    (size_t)new_capacity * sizeof(*grown));
        if (!grown) return false;
        list->frames = grown;
        list->capacity = new_capacity;
    }
    list->frames[list->count++] = *frame;
    return true;
}

void frame_list_free(frame_list_t *list) {
    free(list->frames);
    memset(list, 0, sizeof(*list));
}

bool frame_tsv_parse_row(char *line, can_frame_t *out) {
    char *fields[6];
    int field_count = 0;
    char *p = line;
    fields[field_count++] = p;
    for (; *p; p++) {
        if (*p != '\t') continue;
        if (field_count >= 6) return false;
        *p = '\0';
        fields[field_count++] = p + 1;
    }
    if (field_count != 5 && field_count != 6) return false;

    memset(out, 0, sizeof(*out));

    /* timestamp_ms: decimal */
    char *end = NULL;
    if (!isdigit((unsigned char)fields[0][0]) && fields[0][0] != '-') return false;
    errno = 0;
    long long ts = strtoll(fields[0], &end, 10);
    if (errno != 0 || *end != '\0') return false;
    out->timestamp_ms = (int64_t)ts;

    /* can_id: hex, optional 0x prefix */
    const char *id_str = fields[1];
    if (id_str[0] == '0' && (id_str[1] == 'x' || id_str[1] == 'X')) id_str += 2;
    if (!isxdigit((unsigned char)id_str[0])) return false;
    errno = 0;
    unsigned long can_id = strtoul(id_str, &end, 16);
    if (errno != 0 || *end != '\0' || can_id > 0x1FFFFFFFUL) return false;
    out->can_id = (uint32_t)can_id;

    /* is_extended: 0 / 1 */
    if (strcmp(fields[2], "0") == 0) out->is_extended = false;
    else if (strcmp(fields[2], "1") == 0) out->is_extended = true;
    else return false;

    /* dlc: 0..8 */
    if (fields[3][0] < '0' || fields[3][0] > '8' || fields[3][1] != '\0') return false;
    out->dlc = (uint8_t)(fields[3][0] - '0');

    /* data_hex: exactly 2 * dlc hex digits */
    const char *hex = fields[4];
    if (strlen(hex) != (size_t)out->dlc * 2) return false;
    for (int b = 0; b < out->dlc; b++) {
        int hi = hex_nibble(hex[2 * b]);
        int lo = hex_nibble(hex[2 * b + 1]);
        if (hi < 0 || lo < 0) return false;
        out->data[b] = (uint8_t)((hi << 4) | lo);
    }

    /* timestamp_us (optional): decimal, in the millisecond of column 1 */
    if (field_count == 6) {
        const char *us_str = fields[5];
        const char *digits = us_str[0] == '-' ? us_str + 1 : us_str;
        if (!isdigit((unsigned char)digits[0])) return false;
        errno = 0;
        long long us = strtoll(us_str, &end, 10);
        if (errno != 0 || *end != '\0') return false;
        can_frame_t check = *out;
        can_frame_set_hw_time_us(&check, (int64_t)us);
        if (check.timestamp_ms != out->timestamp_ms) return false;
        *out = check;
    }
    return true;
}

/** Strip trailing CR/LF in place. Tabs are kept: a dlc=0 row ends in "\t". */
static void strip_eol(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[--n] = '\0';
    }
}

static bool is_blank(const char *s) {
    for (; *s; s++) {
        if (!isspace((unsigned char)*s)) return false;
    }
    return true;
}

int frame_tsv_load(const char *path, const char *display_name, frame_list_t *list) {
    memset(list, 0, sizeof(*list));

    FILE *f = fopen(path, "r");
    if (!f) {
        if (errno == ENOENT) return 0;
        fprintf(stderr, "[ERROR] Cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }

    char line[FRAME_TSV_LINE_MAX];
    int line_no = 0;
    bool saw_content = false;
    while (fgets(line, sizeof(line), f)) {
        line_no++;
        size_t n = strlen(line);
        bool too_long = (n == sizeof(line) - 1 && line[n - 1] != '\n' && !feof(f));
        if (too_long) {
            int c;
            while ((c = fgetc(f)) != EOF && c != '\n')
                ;
        }

        strip_eol(line);
        if (!too_long && is_blank(line)) continue;

        /* Header: first non-blank line naming the columns */
        if (!saw_content) {
            saw_content = true;
            if (!too_long && strncmp(line, "timestamp_ms", 12) == 0) continue;
        }

        can_frame_t frame;
        if (too_long || !frame_tsv_parse_row(line, &frame)) {
            if (list->malformed < FRAME_TSV_MAX_ROW_WARNINGS) {
                fprintf(stderr, "[WARN] %s:%d: malformed row skipped\n",
                        display_name, line_no);
            }
            list->malformed++;
            continue;
        }
        if (!frame_list_push(list, &frame)) {
            fprintf(stderr, "[ERROR] Out of memory loading %s\n", path);
            fclose(f);
            frame_list_free(list);
            return -1;
        }
    }

    bool read_error = ferror(f) != 0;
    fclose(f);
    if (read_error) {
        fprintf(stderr, "[ERROR] Read error on %s\n", path);
        frame_list_free(list);
        return -1;
    }
    if (list->malformed > FRAME_TSV_MAX_ROW_WARNINGS) {
        fprintf(stderr, "[WARN] %s: %d malformed rows skipped in total\n",
                display_name, list->malformed);
    }
    return 1;
}
