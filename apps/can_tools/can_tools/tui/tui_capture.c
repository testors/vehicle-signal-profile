/**
 * @file tui_capture.c
 * @brief stderr capture into a file for the TUI Log panel (C)
 */

#include "tui_capture.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static bool open_target(tui_capture_t *c, const char *path, bool temp, int *wfd_out) {
    int wfd;
    if (temp) {
        const char *tmp = getenv("TMPDIR");
        char tmpl[1024];
        snprintf(tmpl, sizeof(tmpl), "%s/can_tools_tui.XXXXXX", tmp && tmp[0] ? tmp : "/tmp");
        wfd = mkstemp(tmpl);
        if (wfd < 0) return false;
        snprintf(c->path, sizeof(c->path), "%s", tmpl);
    } else {
        wfd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (wfd < 0) return false;
        snprintf(c->path, sizeof(c->path), "%s", path);
    }
    *wfd_out = wfd;
    c->temp = temp;
    return true;
}

bool tui_capture_begin(tui_capture_t *c) {
    memset(c, 0, sizeof(*c));
    c->saved_fd = -1;
    c->read_fd = -1;
    fflush(stderr);
    int saved = dup(STDERR_FILENO);
    if (saved < 0) return false;
    int wfd;
    if (!open_target(c, NULL, true, &wfd)) {
        close(saved);
        return false;
    }
    int rfd = open(c->path, O_RDONLY);
    if (rfd < 0 || dup2(wfd, STDERR_FILENO) < 0) {
        if (rfd >= 0) close(rfd);
        close(wfd);
        unlink(c->path);
        close(saved);
        return false;
    }
    close(wfd);
    c->saved_fd = saved;
    c->read_fd = rfd;
    c->active = true;
    setvbuf(stderr, NULL, _IOLBF, 0);
    return true;
}

static bool switch_to(tui_capture_t *c, const char *path, bool temp) {
    if (!c->active) return false;
    fflush(stderr);
    char old_path[1024];
    snprintf(old_path, sizeof(old_path), "%s", c->path);
    bool old_temp = c->temp;
    int wfd;
    if (!open_target(c, path, temp, &wfd)) {
        snprintf(c->path, sizeof(c->path), "%s", old_path);
        c->temp = old_temp;
        return false;
    }
    off_t consumed = lseek(c->read_fd, 0, SEEK_CUR);
    off_t copied = 0;
    if (old_temp && !temp) {
        /* keep the earlier lines in tui.log */
        int src = open(old_path, O_RDONLY);
        if (src >= 0) {
            char buf[8192];
            ssize_t n;
            while ((n = read(src, buf, sizeof(buf))) > 0) {
                if (write(wfd, buf, (size_t)n) != n) break;
                copied += n;
            }
            close(src);
        }
    }
    int rfd = open(c->path, O_RDONLY);
    if (rfd < 0 || dup2(wfd, STDERR_FILENO) < 0) {
        if (rfd >= 0) close(rfd);
        close(wfd);
        return false;
    }
    close(wfd);
    /* position the reader after what was already delivered */
    off_t start = lseek(rfd, 0, SEEK_END);
    if (old_temp && !temp && consumed <= copied) start = start - copied + consumed;
    lseek(rfd, start, SEEK_SET);
    close(c->read_fd);
    c->read_fd = rfd;
    if (old_temp) unlink(old_path);
    return true;
}

bool tui_capture_switch(tui_capture_t *c, const char *path) {
    return switch_to(c, path, false);
}

bool tui_capture_switch_temp(tui_capture_t *c) {
    if (c->active && c->temp) return true;
    return switch_to(c, NULL, true);
}

void tui_capture_strip_nslog(char *line) {
    /* YYYY-MM-DD HH:MM:SS.mmm name[pid:tid] */
    const char *p = line;
    if (!(isdigit((unsigned char)p[0]) && isdigit((unsigned char)p[1]) && isdigit((unsigned char)p[2]) &&
          isdigit((unsigned char)p[3]) && p[4] == '-')) return;
    const char *br = strchr(p, '[');
    if (!br || br - p > 120) return;
    const char *close_br = strchr(br, ']');
    if (!close_br || !strchr(br, ':') || strchr(br, ':') > close_br) return;
    const char *rest = close_br + 1;
    while (*rest == ' ') rest++;
    memmove(line, rest, strlen(rest) + 1);
}

int tui_capture_poll(tui_capture_t *c, tui_capture_line_fn fn, void *ctx) {
    if (!c->active || c->read_fd < 0) return 0;
    fflush(stderr);
    int lines = 0;
    char buf[4096];
    ssize_t n;
    while ((n = read(c->read_fd, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            char ch = buf[i];
            if (ch == '\n') {
                c->partial[c->plen] = '\0';
                if (c->plen > 0 && c->partial[c->plen - 1] == '\r') c->partial[--c->plen] = '\0';
                tui_capture_strip_nslog(c->partial);
                if (c->partial[0] && fn) fn(c->partial, ctx);
                lines++;
                c->plen = 0;
            } else if (c->plen + 1 < sizeof(c->partial)) {
                c->partial[c->plen++] = ch;
            }
        }
    }
    return lines;
}

void tui_capture_end(tui_capture_t *c) {
    if (!c->active) return;
    fflush(stderr);
    dup2(c->saved_fd, STDERR_FILENO);
    close(c->saved_fd);
    if (c->read_fd >= 0) close(c->read_fd);
    if (c->temp) unlink(c->path);
    c->active = false;
    c->saved_fd = -1;
    c->read_fd = -1;
}
