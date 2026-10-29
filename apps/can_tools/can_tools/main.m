/**
 * @file main.m
 * @brief can_tools: one binary for the CAN tools
 *
 *   can_tools <command> [args...]     see can_tools help
 *   can_tools                         interactive menu
 *
 * Old command lines keep working through argv[0] aliases (symlinks made by
 * the build): can_reader, can_discover, vsp_probe, discover_replay. Under an
 * alias the tool's own arguments and help text are used unchanged.
 */

#import <Foundation/Foundation.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cli_util.h"
#include "cli_pcan.h"
#include "frame_source.h"
#import "obd_adapter.h"
#import "cli_common.h"
#include "commands.h"

/*******************************************************************************
 * can_reader command line (alias can_reader; also can_tools scan/detect/...)
 ******************************************************************************/

static void cli_print_usage(void) {
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  macos_obd_reader                                     Interactive mode\n");
    fprintf(stderr, "  macos_obd_reader scan                                Scan for BLE OBD devices (+ PCAN-USB)\n");
    fprintf(stderr, "  macos_obd_reader query <device> <PIDs..>             Query OBD-II PIDs\n");
    fprintf(stderr, "  macos_obd_reader supported <device>                  List supported PIDs\n");
    fprintf(stderr, "  macos_obd_reader monitor <device> [secs] [--out FILE] [--bitrate R] [CAN_IDs]  CAN passive monitor\n");
    fprintf(stderr, "  macos_obd_reader raw <device> <command>              Send raw AT command\n");
    fprintf(stderr, "  macos_obd_reader detect <device> [--probe]           Detect bus type (receive-only; --probe sends 0100)\n");
    fprintf(stderr, "  macos_obd_reader discover <device> [--dump-dir DIR] [--bitrate R]  Auto-discover PT-CAN signals\n");
    fprintf(stderr, "  macos_obd_reader p992 list                           List 992 XML channels + mappings\n");
    fprintf(stderr, "  macos_obd_reader p992 read <device> [channel...]     Read 992 profile channels once\n");
    fprintf(stderr, "  macos_obd_reader p992 watch <device> [...]           Stream 992 profile channels as NDJSON\n");
    fprintf(stderr, "  macos_obd_reader p992 fastlog <device> [...]         Periodic-message fast logger\n");
    fprintf(stderr, "  macos_obd_reader p992 debug <device>                 Probe 992 routes and raw responses\n");
    fprintf(stderr, "  macos_obd_reader p992 bin-probe <device>             Probe exact AiM BIN-tail request containers\n");
    fprintf(stderr, "  macos_obd_reader p992 scan-did <device> [DID] [...]  Scan 0x700..0x7EF tx IDs for a DID\n");
    fprintf(stderr, "  macos_obd_reader p992 scan-did-range <device> [...]  Scan DID window on known responding ECU IDs\n");
    fprintf(stderr, "  macos_obd_reader p992 uds-read <device> <DID> [...]  Read one UDS DID with optional route/session\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "  <device>:   BLE name prefix match (e.g., OBDLink, vLinker)\n");
    fprintf(stderr, "              pcan / pcan1..pcan8: PEAK PCAN-USB, listen-only (monitor, detect, discover)\n");
    fprintf(stderr, "  <CAN_IDs>:  Optional hex filter IDs (e.g., 0A5 1A0 316)\n");
    fprintf(stderr, "  --bitrate:  PCAN only: auto or %s (default 500k)\n", pcan_bitrate_list());
    fprintf(stderr, "\n");
    fprintf(stderr, "  detect:    Listen 3s for broadcast traffic; --probe also sends 0100 (OBD port only)\n");
    fprintf(stderr, "  discover:  Interactive guided signal discovery + default capture dump\n");
    fprintf(stderr, "             Default dump dir: captures/YYYYMMDD_HHMMSS\n");
    fprintf(stderr, "  monitor:   Silent mode (no ACK) — safe for direct CAN bus tap\n");
    fprintf(stderr, "  query/supported/raw: Active mode (sends OBD-II requests)\n");
    fprintf(stderr, "  p992:      AiM 992 XML-backed reader. Custom DID channels expose raw + heuristic decode.\n");
}

/** CLI mode entry point */
static int cli_main(int argc, const char *argv[]) {
    g_cli_mode = true;
    const char *subcmd = argv[1];

    signal(SIGINT, signal_handler);

    /* PCAN is listen-only: a command that would transmit refuses it here,
     * before any adapter object exists or the PCAN library is loaded. */
    const char *pcan_tx = NULL;
    if (argc >= 3 && pcan_is_device(argv[2]) &&
        (strcmp(subcmd, "query") == 0 || strcmp(subcmd, "supported") == 0 || strcmp(subcmd, "raw") == 0)) {
        pcan_tx = subcmd;
    } else if (strcmp(subcmd, "p992") == 0 && argc >= 4 && pcan_is_device(argv[3]) &&
               strcmp(argv[2], "list") != 0 && strcmp(argv[2], "help") != 0) {
        pcan_tx = "p992";
    } else if (strcmp(subcmd, "detect") == 0 && argc >= 3 && pcan_is_device(argv[2])) {
        for (int i = 3; i < argc; i++) {
            if (strcmp(argv[i], "--probe") == 0) pcan_tx = "detect --probe";
        }
    }
    if (pcan_tx) return cli_pcan_refuse(pcan_tx);
    /* monitor / detect / discover on PCAN: no Bluetooth object at all */
    bool pcan_device = argc >= 3 && pcan_is_device(argv[2]) &&
                       (strcmp(subcmd, "monitor") == 0 || strcmp(subcmd, "detect") == 0 ||
                        strcmp(subcmd, "discover") == 0);

    bool needs_ble = false;
    if (strcmp(subcmd, "scan") == 0 ||
        strcmp(subcmd, "query") == 0 ||
        strcmp(subcmd, "supported") == 0 ||
        strcmp(subcmd, "monitor") == 0 ||
        strcmp(subcmd, "raw") == 0 ||
        strcmp(subcmd, "detect") == 0 ||
        strcmp(subcmd, "discover") == 0) {
        needs_ble = true;
    } else if (strcmp(subcmd, "p992") == 0) {
        if (argc >= 3 &&
            strcmp(argv[2], "list") != 0 &&
            strcmp(argv[2], "help") != 0 &&
            strcmp(argv[2], "--help") != 0) {
            needs_ble = true;
        }
    }

    if (needs_ble && !pcan_device) {
        g_ble = [[OBDAdapter alloc] initWithNameHints:[OBDAdapter defaultNameHints]
                                                quiet:NO];
        g_ble.logCommandErrors = YES;
    }

    int ret = 0;

    if (strcmp(subcmd, "scan") == 0) {
        ret = cli_scan();
    } else if (strcmp(subcmd, "p992") == 0) {
        ret = cli_p992(argc, argv);
    } else if (strcmp(subcmd, "query") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: query <device> <pid_hex> [pid_hex...]\n");
            ret = 1;
        } else {
            ret = cli_query(argv[2], argc - 3, &argv[3]);
        }
    } else if (strcmp(subcmd, "supported") == 0) {
        if (argc < 3) {
            fprintf(stderr, "[ERROR] Usage: supported <device>\n");
            ret = 1;
        } else {
            ret = cli_supported(argv[2]);
        }
    } else if (strcmp(subcmd, "monitor") == 0) {
        if (argc < 3) {
            fprintf(stderr, "[ERROR] Usage: monitor <device> [seconds] [--out FILE] [--bitrate RATE|auto] [CAN_ID_hex...]\n");
            ret = 1;
        } else {
            int secs = 5;
            int filter_start = 3;  /* argv index where filter IDs begin */
            const char *filter_args[64];
            int filter_argc = 0;
            const char *bitrate_arg = NULL;
            g_cli_monitor_out_path = NULL;
            /* If argv[3] looks like a number (seconds), consume it */
            if (argc >= 4) {
                char *endp;
                long val = strtol(argv[3], &endp, 10);
                if (*endp == '\0' && val > 0) {
                    secs = (int)val;
                    filter_start = 4;
                }
            }
            for (int i = filter_start; i < argc; i++) {
                if (strcmp(argv[i], "--out") == 0) {
                    if (i + 1 >= argc) {
                        fprintf(stderr, "[ERROR] --out requires a file path\n");
                        ret = 1;
                        break;
                    }
                    g_cli_monitor_out_path = argv[++i];
                } else if (strncmp(argv[i], "--out=", 6) == 0) {
                    g_cli_monitor_out_path = argv[i] + 6;
                } else if (strcmp(argv[i], "--bitrate") == 0) {
                    if (i + 1 >= argc) {
                        fprintf(stderr, "[ERROR] --bitrate requires a value\n");
                        ret = 1;
                        break;
                    }
                    bitrate_arg = argv[++i];
                } else if (strncmp(argv[i], "--bitrate=", 10) == 0) {
                    bitrate_arg = argv[i] + 10;
                } else if (filter_argc < (int)(sizeof(filter_args) / sizeof(filter_args[0]))) {
                    filter_args[filter_argc++] = argv[i];
                } else {
                    fprintf(stderr, "[ERROR] Too many monitor filter arguments\n");
                    ret = 1;
                    break;
                }
            }
            if (ret == 0 && pcan_device) {
                pcan_bitrate_t br;
                ret = cli_pcan_parse_bitrate(bitrate_arg, &br)
                      ? cli_monitor_pcan(argv[2], secs, filter_args, filter_argc, &br) : 1;
            } else if (ret == 0 && bitrate_arg) {
                fprintf(stderr, "[ERROR] --bitrate applies to PCAN devices only (pcan, pcan1..pcan8)\n");
                ret = 1;
            } else if (ret == 0) {
                ret = cli_monitor(argv[2], secs,
                                  filter_argc > 0 ? filter_args : NULL,
                                  filter_argc);
            }
        }
    } else if (strcmp(subcmd, "raw") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: raw <device> <command>\n");
            ret = 1;
        } else {
            ret = cli_raw(argv[2], argv[3]);
        }
    } else if (strcmp(subcmd, "detect") == 0) {
        if (argc < 3) {
            fprintf(stderr, "[ERROR] Usage: detect <device> [--probe]\n");
            ret = 1;
        } else if (pcan_device) {
            const char *bitrate_arg = NULL;
            for (int i = 3; i < argc && ret == 0; i++) {
                if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) bitrate_arg = argv[++i];
                else if (strncmp(argv[i], "--bitrate=", 10) == 0) bitrate_arg = argv[i] + 10;
                else {
                    fprintf(stderr, "[ERROR] Usage: detect <pcan device> [--bitrate RATE|auto]\n");
                    ret = 1;
                }
            }
            pcan_bitrate_t br;
            if (ret == 0) ret = cli_pcan_parse_bitrate(bitrate_arg, &br) ? cli_detect_pcan(argv[2], &br) : 1;
        } else {
            bool allow_probe = argc > 3 && strcmp(argv[3], "--probe") == 0;
            ret = cli_detect(argv[2], allow_probe);
        }
    } else if (strcmp(subcmd, "discover") == 0) {
        if (argc < 3) {
            fprintf(stderr, "[ERROR] Usage: discover <device> [--dump-dir DIR] [--bitrate RATE|auto]\n");
            ret = 1;
        } else {
            const char *dump_dir_override = NULL;
            const char *bitrate_arg = NULL;
            for (int i = 3; i < argc; i++) {
                if (strcmp(argv[i], "--dump-dir") == 0) {
                    if (i + 1 >= argc) {
                        fprintf(stderr, "[ERROR] --dump-dir requires a directory path\n");
                        return 1;
                    }
                    dump_dir_override = argv[++i];
                } else if (strncmp(argv[i], "--dump-dir=", 11) == 0) {
                    dump_dir_override = argv[i] + 11;
                } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
                    bitrate_arg = argv[++i];
                } else if (strncmp(argv[i], "--bitrate=", 10) == 0) {
                    bitrate_arg = argv[i] + 10;
                } else {
                    fprintf(stderr, "[ERROR] Unknown discover option: %s\n", argv[i]);
                    fprintf(stderr, "[ERROR] Usage: discover <device> [--dump-dir DIR] [--bitrate RATE|auto]\n");
                    return 1;
                }
            }
            if (pcan_device) {
                pcan_bitrate_t br;
                ret = cli_pcan_parse_bitrate(bitrate_arg, &br) ? cli_discover_pcan(argv[2], dump_dir_override, &br) : 1;
            } else if (bitrate_arg) {
                fprintf(stderr, "[ERROR] --bitrate applies to PCAN devices only (pcan, pcan1..pcan8)\n");
                ret = 1;
            } else {
                ret = cli_discover(argv[2], dump_dir_override);
            }
        }
    } else if (strcmp(subcmd, "-h") == 0 || strcmp(subcmd, "--help") == 0 ||
               strcmp(subcmd, "help") == 0) {
        cli_print_usage();
        ret = 0;
    } else {
        fprintf(stderr, "[ERROR] Unknown command: %s\n", subcmd);
        cli_print_usage();
        ret = 1;
    }

    return ret;
}

/** can_reader: subcommands as before, or the interactive menu without arguments. */
int can_reader_main(int argc, const char *argv[]) {
    if (argc >= 2) {
        int ret = cli_main(argc, argv);
        frame_buffer_free(&g_cli_capture);
        cli_release_adapter();
        return ret;
    }
    return interactive_main();
}

/*******************************************************************************
 * can_tools command line
 ******************************************************************************/

static void can_tools_usage(void) {
    fprintf(stderr,
            "Usage: can_tools <command> [args...]\n"
            "       can_tools                                    Live dashboard (TUI) in a terminal\n"
            "       can_tools menu                               The old interactive menu\n"
            "\n"
            "Receive-only (nothing is transmitted on the CAN bus):\n"
            "  scan                                         List BLE/Classic OBD adapters and PCAN-USB channels (JSON)\n"
            "  monitor <device> [secs] [--out FILE] [IDs]   Passive CAN monitor, silent mode (no ACK)\n"
            "  detect <device>                              Listen 3 s for broadcast traffic\n"
            "  discover [device] [--dump-dir DIR]           Guided PT-CAN signal discovery\n"
            "  probe <profile.vsp> --mode can [...]         VSP signal rate check from CAN frames\n"
            "  probe <profile.vsp> --plan                   Selection plan only, no adapter\n"
            "  replay <session_dir> [--out-dir DIR]         Re-run discovery on a saved capture (offline)\n"
            "  tui --replay PATH [--profile VSP]            Dashboard over a saved capture (offline)\n"
            "  tui [--device NAME] [--profile VSP]          Live dashboard: connect screen, silent mode;\n"
            "                                               TX mode (x) only on a GATEWAY/SILENT bus\n"
            "\n"
            "Transmit (OBD port only; never on a direct bus tap such as PT-CAN):\n"
            "  detect <device> --probe                      Also sends 0100 to check for an ECU\n"
            "  probe <profile.vsp> --mode obd [...]         VSP diagnostic queries (OBD/UDS)\n"
            "  probe <profile.vsp> [--mode auto] [...]      CAN phase, then queries only if no\n"
            "                                               broadcast traffic was heard\n"
            "  query <device> <PIDs..>                      Query OBD-II PIDs\n"
            "  supported <device>                           List supported PIDs\n"
            "  raw <device> <command>                       Send a raw AT/OBD command\n"
            "\n"
            "Legacy:\n"
            "  p992 <subcommand> [...]                      Porsche 992 AiM debug commands (transmit;\n"
            "                                               can_tools p992 help)\n"
            "\n"
            "  <device>: adapter name prefix, case-insensitive (e.g. OBDLink, vLinker),\n"
            "            or pcan / pcan1..pcan8: a PEAK PCAN-USB through the PCBUSB library,\n"
            "            listen-only (whole bus, hardware µs timestamps); monitor, detect,\n"
            "            discover, probe --mode can and tui take it with [--bitrate RATE|auto]\n"
            "            (1M 800k 500k 250k 125k 100k 95k 83k 50k 47k 33k 20k 10k 5k; default 500k).\n"
            "            The transmit commands refuse it: PCAN is listen-only.\n"
            "  Help per command: can_tools probe --help, discover --help, replay --help\n"
            "  Aliases with the old command lines: can_reader, can_discover, vsp_probe, discover_replay\n"
            "  JSON on stdout, logs on stderr.\n");
}

static int can_tools_main(int argc, const char *argv[]) {
    if (argc < 2) {
        /* The TUI needs a terminal; scripts get the usage. */
        if (isatty(STDIN_FILENO) && isatty(STDOUT_FILENO)) {
            const char *tui_argv[] = { "tui" };
            return cmd_tui_main(1, tui_argv);
        }
        can_tools_usage();
        return 2;
    }

    const char *cmd = argv[1];
    if (strcmp(cmd, "menu") == 0) return interactive_main();
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "-h") == 0 || strcmp(cmd, "--help") == 0) {
        can_tools_usage();
        return 0;
    }

    static const char *kReaderCommands[] = {
        "scan", "detect", "monitor", "query", "supported", "raw", "p992",
    };
    for (size_t i = 0; i < sizeof(kReaderCommands) / sizeof(kReaderCommands[0]); i++) {
        if (strcmp(cmd, kReaderCommands[i]) == 0) return can_reader_main(argc, argv);
    }
    if (strcmp(cmd, "discover") == 0) return cmd_discover_main(argc - 1, argv + 1);
    if (strcmp(cmd, "probe") == 0) return cmd_probe_main(argc - 1, argv + 1);
    if (strcmp(cmd, "replay") == 0) return cmd_replay_main(argc - 1, (char **)(argv + 1));
    if (strcmp(cmd, "tui") == 0) return cmd_tui_main(argc - 1, argv + 1);

    fprintf(stderr, "[ERROR] Unknown command: %s\n", cmd);
    can_tools_usage();
    return 1;
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        const char *slash = strrchr(argv[0], '/');
        const char *name = slash ? slash + 1 : argv[0];

        if (strcmp(name, "can_reader") == 0) return can_reader_main(argc, argv);
        if (strcmp(name, "can_discover") == 0) return cmd_discover_main(argc, argv);
        if (strcmp(name, "vsp_probe") == 0) return cmd_probe_main(argc, argv);
        if (strcmp(name, "discover_replay") == 0) return cmd_replay_main(argc, (char **)argv);
        return can_tools_main(argc, argv);
    }
}
