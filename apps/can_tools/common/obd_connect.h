/**
 * @file obd_connect.h
 * @brief The one scan / connect / adapter-init path every command uses
 *
 * Connect policy (can_reader's): wait up to 10 s for BLE; scan BLE + Classic
 * for 8 s, once more if nothing was found; paired Classic devices are listed
 * even when BLE is off. The first device whose name contains device_prefix
 * (case-insensitive) is used; a NULL or empty prefix takes the first device.
 *
 * Init profiles (obd_init_config_t.profile):
 *
 *   OBD_INIT_PASSIVE  receive-only CAN monitor, nothing is sent on the bus:
 *                     ATZ (+0.5 s), ATE0, ATL0, STI, ATSP6 (required),
 *                     ATCSM1 (warns on failure), ATS1, ATCAF0, ATH1, filters.
 *   OBD_INIT_ACTIVE   OBD-II queries (transmits): elm327_build_init_cmds
 *                     (ATZ..ATSP0..ATDPN), STI, and on STN STDI +
 *                     stn2255_build_init_cmds.
 *   OBD_INIT_BASIC    reset and STN detection only: ATZ (+0.5 s), ATE0, ATL0,
 *                     ATS1, STI. The caller configures the protocol itself
 *                     (vsp_probe's OBD schedulers).
 *   OBD_INIT_NONE     no commands.
 *
 * Messages go to stderr ("[INFO] ...", "[WARN] ...", "[ERROR] ...").
 * Functions return false on failure; they never exit.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#import "obd_adapter.h"
#include "stn2255.h"

typedef enum {
    OBD_INIT_NONE = 0,
    OBD_INIT_BASIC,
    OBD_INIT_PASSIVE,
    OBD_INIT_ACTIVE,
} obd_init_profile_t;

/** Hardware receive filter: pass frames with (can_id & mask) == (id & mask). */
typedef struct {
    uint32_t id;
    uint32_t mask;
} obd_can_filter_t;

typedef struct {
    obd_init_profile_t      profile;
    /* PASSIVE only. filters == NULL: leave the adapter's filters alone.
     * filters != NULL (filter_count may be 0): on STN send STFAC, then
     * "STFAP id,mask" per entry. On a plain ELM327 send "ATCF id" + "ATCM mask"
     * per entry, but only when apply_elm_filters is set: an ELM327 holds a
     * single filter, so with several entries only the last one passes. */
    const obd_can_filter_t *filters;
    int                     filter_count;
    bool                    apply_elm_filters;
} obd_init_config_t;

typedef struct {
    int            device_index;    /**< index in adapter.discoveredNames, -1 if none */
    char           device_name[128];
    bool           stn_detected;    /**< set by the last init that sent STI */
    stn2255_info_t stn_info;
} obd_link_t;

/** Reset link to "not connected". */
void obd_link_init(obd_link_t *link);

/** Scan BLE + Classic for `seconds` (adapter.discoveredNames holds the result). */
void obd_scan(OBDAdapter *adapter, double seconds);

/** Scan and connect with the policy above; waits for service discovery (10 s). */
bool obd_connect_device(OBDAdapter *adapter, const char *device_prefix, obd_link_t *link);

/** Run an init profile on a connected adapter. Updates link->stn_*. */
bool obd_init_adapter(OBDAdapter *adapter, const obd_init_config_t *cfg, obd_link_t *link);

/** obd_connect_device() then obd_init_adapter(). */
bool obd_open(OBDAdapter *adapter, const char *device_prefix,
              const obd_init_config_t *cfg, obd_link_t *link);

/** Monitor command for the detected chip: "STMA" on STN, else "ATMA". */
const char *obd_monitor_cmd(const obd_link_t *link);
