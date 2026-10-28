/**
 * @file cli_util.h
 * @brief Small CLI helpers shared by the tools (Pure C)
 */

#pragma once

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Log to stderr as "[INFO] <message>\n". */
void cli_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Write s as a JSON string literal (quotes and escapes included). */
void json_print_string(FILE *f, const char *s);

#ifdef __cplusplus
}
#endif
