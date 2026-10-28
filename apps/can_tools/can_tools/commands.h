/**
 * @file commands.h
 * @brief Entry points of the can_tools subcommands (C-compatible)
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* main.m: can_reader's command line (subcommands, or the menu without arguments) */
int can_reader_main(int argc, const char *argv[]);

int interactive_main(void);                                          /* interactive.m */
int cli_scan(void);                                                  /* cmd_scan.m */
int cli_query(const char *device, int pid_argc, const char *pid_argv[]); /* cmd_query.m */
int cli_supported(const char *device);
int cli_raw(const char *device, const char *command);
int cli_monitor(const char *device, int seconds,                     /* cmd_monitor.m */
                const char *filter_argv[], int filter_argc);
int cli_detect(const char *device, bool allow_probe);                /* cmd_detect.m */
int cli_discover(const char *device, const char *dump_dir_override); /* cmd_discover.m */
int cmd_discover_main(int argc, const char *argv[]);                 /* can_discover's arguments */
int cmd_probe_main(int argc, const char *argv[]);                    /* cmd_probe.m: vsp_probe's arguments */
int cmd_replay_main(int argc, char *argv[]);                         /* cmd_replay.c: discover_replay's arguments */
int cli_p992(int argc, const char *argv[]);                          /* cmd_p992.m */
int cmd_tui_main(int argc, const char *argv[]);                      /* tui/tui_main.m */

#ifdef __cplusplus
}
#endif
