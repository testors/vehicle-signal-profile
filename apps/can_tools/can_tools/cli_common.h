/**
 * @file cli_common.h
 * @brief State and helpers shared by the can_tools commands (Objective-C)
 *
 * One adapter (g_ble) per process. SIGINT during a monitor/capture sets
 * g_interrupt (ends that capture); otherwise it clears g_running.
 */

#pragma once

#import <Foundation/Foundation.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>

#include "elm327.h"
#include "stn2255.h"
#include "frame_source.h"
#import "obd_adapter.h"

/** true while a can_reader-style subcommand runs (monitor frames are stored, not printed). */
extern bool g_cli_mode;
/** Monitor capture buffer for monitor, detect and p992 fastlog. */
extern frame_buffer_t g_cli_capture;
/** monitor --out FILE */
extern const char *g_cli_monitor_out_path;
extern uint32_t g_sample_count;
extern uint32_t g_can_frame_count;

extern OBDAdapter *g_ble;
extern volatile sig_atomic_t g_running;
extern volatile sig_atomic_t g_interrupt;
extern stn2255_info_t g_stn_info;
extern bool g_stn_detected;
/** PCAN source of the running command (monitor, detect, discover), or NULL.
 *  While it is started, SIGINT ends the capture like a running BT monitor. */
extern frame_source_t *volatile g_cli_pcan_source;

void signal_handler(int sig);
/** OBDAdapter monitor callback: CLI mode stores the frame, interactive mode prints it. */
void monitor_frame_callback(const can_frame_t *frame, void *context);
int64_t get_timestamp_ms(void);
void run_loop_for(double seconds);

/** g_ble = adapter with the default OBD name hints and per-command errors on stderr. */
void cli_create_adapter(void);
/** Release g_ble (nil is fine). */
void cli_release_adapter(void);

/* Connection (common/obd_connect). Each returns the matched device index or exits(1). */
int cli_ble_connect(const char *device_prefix);                 /* scan + connect, no init */
int cli_auto_connect(const char *device_prefix);                /* + ACTIVE init (transmits) */
int cli_ptcan_connect(const char *device_prefix,                /* + PASSIVE init (receive-only) */
                      const uint32_t *filter_ids, int filter_count);
