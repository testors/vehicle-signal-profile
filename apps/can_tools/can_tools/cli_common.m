/**
 * @file cli_common.m
 * @brief State and helpers shared by the can_tools commands
 *
 * Moved from can_reader/main.m (globals, SIGINT handler, monitor callback,
 * run loop helper). The connect wrappers keep can_reader's contract (print,
 * then exit(1) on failure) on top of common/obd_connect.
 */

#import "cli_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#import "obd_connect.h"

/*******************************************************************************
 * CLI Mode State
 ******************************************************************************/

bool g_cli_mode = false;

/** Monitor capture buffer for monitor, detect and p992 fastlog (CLI mode);
 *  grows to FRAME_BUFFER_MAX frames, then counts drops. */
frame_buffer_t g_cli_capture = {0};
const char *g_cli_monitor_out_path = NULL;

/** Local counters (replaces obd_provider) */
uint32_t g_sample_count = 0;
uint32_t g_can_frame_count = 0;

/*******************************************************************************
 * Global State
 ******************************************************************************/

OBDAdapter *g_ble = nil;
volatile sig_atomic_t g_running = 1;
volatile sig_atomic_t g_interrupt = 0;

/* STN2255 state */
stn2255_info_t g_stn_info = {0};
bool g_stn_detected = false;

void signal_handler(int sig) {
    (void)sig;
    if (g_ble.canMonitorActive) {
        g_interrupt = YES;
    } else {
        g_running = NO;
    }
}

/** OBDAdapter monitor callback (monitor, detect, p992 fastlog, menu 7):
 *  CLI mode stores the frame for the JSON output, interactive mode prints it. */
void monitor_frame_callback(const can_frame_t *frame, void *context) {
    (void)context;
    g_can_frame_count++;
    if (g_cli_mode) {
        /* CLI mode: store frames for JSON output */
        frame_buffer_append(&g_cli_capture, frame);
    } else {
        printf("  %03X [%d] ", frame->can_id, frame->dlc);
        for (int j = 0; j < frame->dlc; j++) {
            printf("%02X ", frame->data[j]);
        }
        printf("\n");
    }
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

int64_t get_timestamp_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/** Run the event loop for a given duration */
void run_loop_for(double seconds) {
    NSDate *until = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while ([[NSDate date] compare:until] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }
}

/*******************************************************************************
 * Adapter and connection (common/obd_connect)
 ******************************************************************************/

void cli_create_adapter(void) {
    g_ble = [[OBDAdapter alloc] initWithNameHints:[OBDAdapter defaultNameHints]
                                            quiet:NO];
    g_ble.logCommandErrors = YES;
}

void cli_release_adapter(void) {
    [g_ble release];
    g_ble = nil;
}

static void cli_apply_link(const obd_link_t *link) {
    g_stn_info = link->stn_info;
    if (link->stn_detected) g_stn_detected = true;
}

/**
 * BLE scan + connect (shared by all CLI subcommands).
 * Returns matched device index, or exits with code 1 on failure.
 */
int cli_ble_connect(const char *device_prefix) {
    obd_link_t link;
    obd_link_init(&link);
    if (!obd_connect_device(g_ble, device_prefix, &link)) exit(1);
    return link.device_index;
}

/**
 * Standard OBD-II auto-connect: connect + ELM327 init + STN detection (transmits).
 * Returns matched device index, or exits with code 1 on failure.
 */
int cli_auto_connect(const char *device_prefix) {
    obd_link_t link;
    obd_link_init(&link);
    obd_init_config_t cfg = { .profile = OBD_INIT_ACTIVE };
    if (!obd_open(g_ble, device_prefix, &cfg, &link)) exit(1);
    cli_apply_link(&link);
    return link.device_index;
}

/**
 * PT-CAN passive connect: connect + receive-only init (silent mode, no ACK).
 * filter_ids pass exactly (mask 7FF); on a plain ELM327 the ATCF/ATCM pair
 * holds one filter, so the last ID wins. Returns matched device index, or
 * exits with code 1 on failure.
 */
int cli_ptcan_connect(const char *device_prefix, const uint32_t *filter_ids,
                      int filter_count) {
    obd_can_filter_t filters[16];
    if (filter_count > 16) filter_count = 16;
    for (int i = 0; i < filter_count; i++) {
        filters[i].id = filter_ids[i];
        filters[i].mask = 0x7FF;
    }
    obd_init_config_t cfg = {
        .profile = OBD_INIT_PASSIVE,
        .filters = filter_count > 0 ? filters : NULL,
        .filter_count = filter_count,
        .apply_elm_filters = true,
    };
    obd_link_t link;
    obd_link_init(&link);
    if (!obd_open(g_ble, device_prefix, &cfg, &link)) exit(1);
    cli_apply_link(&link);
    return link.device_index;
}
