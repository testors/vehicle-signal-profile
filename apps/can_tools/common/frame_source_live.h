/**
 * @file frame_source_live.h
 * @brief Live frame source over an OBDAdapter monitor (Objective-C)
 *
 * start: startMonitorWithCallback + sendRawString("<monitor_cmd>\r")
 * pump:  runLoopFor(seconds)
 * open:  adapter.isConnected
 * stop:  stopMonitor + sendRawString("\r") + runLoopFor(0.2)
 *
 * The adapter must already be connected and configured for passive
 * monitoring (ATSP6, ATCSM1, ATCAF0, ATH1, ...); this source only issues the
 * monitor command the caller chose (ATMA / STMA / STM) and the CR that ends it.
 */

#pragma once

#import "obd_adapter.h"
#include "frame_source.h"

/** monitor_cmd is copied (e.g. "STMA"). The adapter is retained. */
frame_source_t *frame_source_live_create(OBDAdapter *adapter, const char *monitor_cmd);
