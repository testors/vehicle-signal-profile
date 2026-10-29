/*
 * Fake OBDAdapter for tests: linked into can_tools_fake (all can_tools
 * sources + this file), this category replaces every Bluetooth edge of
 * OBDAdapter. No CoreBluetooth or IOBluetooth object is ever created, so a
 * test run cannot reach a real adapter. Every command and raw write is
 * logged to cmds.txt in the current directory.
 *
 * Environment:
 *   FAKE_NAME        comma-separated names a scan finds (default "vLinker FAKE"; "" = none)
 *   FAKE_BLE_OFF=1   BLE never ready      FAKE_CLASSIC=1  devices are Classic
 *   FAKE_STN=1       STI answers like an STN chip
 *   FAKE_RESP=probe  answer hex requests with header lines from the last ATCRA id
 *   FAKE_STREAM      direct | gateway | silent (default direct): frames on the
 *                    wall clock while the monitor command runs
 *   FAKE_SLOW_AFTER_MS        from this long after the first monitor start every
 *                             stream runs at <= 1 Hz (a DIRECT bus that later looks slow)
 *   FAKE_DISCONNECT_AFTER_MS  drop the connection this long after the first monitor start
 */

/* The category replaces OBDAdapter methods on purpose. */
#pragma clang diagnostic ignored "-Wobjc-protocol-method-implementation"

#import <Foundation/Foundation.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#import "obd_adapter.h"
#include "elm327.h"

static FILE *fake_log(void) {
    static FILE *f = NULL;
    if (!f) {
        f = fopen("cmds.txt", "w");
        if (f) setvbuf(f, NULL, _IOLBF, 0);
    }
    return f ? f : stderr;
}

static obd_adapter_frame_callback_t g_cb = NULL;
static void *g_ctx = NULL;
static int g_connected_idx = 0;
static char g_rx[8] = "7E8";
static bool g_streaming = false;
static int64_t g_first_monitor_ms = 0;

typedef struct {
    uint32_t id;
    int      period_ms;
    uint8_t  dlc;
    int64_t  next_ms;
    uint32_t n;
} fake_stream_t;

static fake_stream_t g_streams[8];
static int g_nstreams = 0;

static int64_t wall_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static NSArray<NSString *> *fake_names(void) {
    const char *v = getenv("FAKE_NAME");
    NSString *s = [NSString stringWithUTF8String:v ? v : "vLinker FAKE"];
    return [s componentsSeparatedByString:@","];
}

static void setup_streams(void) {
    const char *mode = getenv("FAKE_STREAM");
    if (!mode) mode = "direct";
    g_nstreams = 0;
    if (strcmp(mode, "direct") == 0) {
        g_streams[g_nstreams++] = (fake_stream_t){ 0x0A5, 10, 8, 0, 0 };
        g_streams[g_nstreams++] = (fake_stream_t){ 0x0F3, 10, 8, 0, 0 };
        g_streams[g_nstreams++] = (fake_stream_t){ 0x1A0, 20, 8, 0, 0 };
        g_streams[g_nstreams++] = (fake_stream_t){ 0x0D9, 50, 6, 0, 0 };
    } else if (strcmp(mode, "gateway") == 0) {
        g_streams[g_nstreams++] = (fake_stream_t){ 0x0F3, 200, 8, 0, 0 };
        g_streams[g_nstreams++] = (fake_stream_t){ 0x0A5, 500, 8, 0, 0 };
        g_streams[g_nstreams++] = (fake_stream_t){ 0x3B0, 1000, 8, 0, 0 };
    }
    int64_t now = wall_ms();
    for (int i = 0; i < g_nstreams; i++) g_streams[i].next_ms = now + i;
}

static void emit_due(void) {
    if (!g_streaming || !g_cb) return;
    int64_t now = wall_ms();
    for (int i = 0; i < g_nstreams; i++) {
        fake_stream_t *s = &g_streams[i];
        while (s->next_ms <= now && g_streaming && g_cb) {
            can_frame_t f;
            memset(&f, 0, sizeof(f));
            f.timestamp_ms = s->next_ms;
            f.can_id = s->id;
            f.dlc = s->dlc;
            f.data[0] = (uint8_t)(s->n & 0x0F);
            f.data[2] = (uint8_t)((s->n * 7) & 0xFF);
            f.data[4] = (uint8_t)(0x40 + (s->n % 16));
            f.data[5] = (uint8_t)(s->n & 0xFF);
            s->n++;
            int period = s->period_ms;
            const char *slow = getenv("FAKE_SLOW_AFTER_MS");
            if (slow && g_first_monitor_ms > 0 && s->next_ms - g_first_monitor_ms >= atoll(slow) && period < 1000) {
                period = 1000;
            }
            s->next_ms += period;
            g_cb(&f, g_ctx);
        }
    }
}

@implementation OBDAdapter (Fake)

- (instancetype)initWithNameHints:(NSArray<NSString *> *)nameHints quiet:(BOOL)quiet {
    self = [super init];
    if (self) {
        self.quiet = quiet;
        self.nameHints = nameHints ?: @[];
        self.discoveredDevices = [NSMutableArray array];
        self.discoveredNames = [NSMutableArray array];
        self.discoveredRSSIs = [NSMutableArray array];
        self.classicDevices = [NSMutableArray array];
        self.discoveredTransports = [NSMutableArray array];
        self.discoveredSourceIndices = [NSMutableArray array];
    }
    return self;
}

- (BOOL)isBleReady { return getenv("FAKE_BLE_OFF") == NULL; }

- (BOOL)waitForBleReady:(double)t {
    (void)t;
    fprintf(fake_log(), "WAIT_BLE\n");
    return getenv("FAKE_BLE_OFF") == NULL;
}

- (void)runLoopFor:(double)seconds {
    int64_t until = wall_ms() + (int64_t)(seconds * 1000.0);
    do {
        emit_due();
        const char *after = getenv("FAKE_DISCONNECT_AFTER_MS");
        if (after && g_first_monitor_ms > 0 && self.isConnected &&
            wall_ms() - g_first_monitor_ms >= atoll(after)) {
            fprintf(fake_log(), "LINK_LOST\n");
            self.isConnected = NO;
            self.serviceDiscoveryDone = NO;
            g_streaming = false;
        }
        int64_t left = until - wall_ms();
        if (left > 0) usleep((useconds_t)(left < 2 ? left : 2) * 1000);
    } while (wall_ms() < until);
}

- (void)startScan {
    fprintf(fake_log(), "SCAN\n");
    [self.discoveredNames removeAllObjects];
    [self.discoveredRSSIs removeAllObjects];
    [self.discoveredTransports removeAllObjects];
    [self.discoveredSourceIndices removeAllObjects];
    int i = 0;
    bool ble_off = getenv("FAKE_BLE_OFF") != NULL;
    for (NSString *name in fake_names()) {
        if (name.length == 0) continue;
        bool classic = getenv("FAKE_CLASSIC") != NULL;
        if (ble_off && !classic) continue;
        [self.discoveredNames addObject:name];
        [self.discoveredRSSIs addObject:@(-40 - i)];
        [self.discoveredTransports addObject:@(classic ? OBD_ADAPTER_TRANSPORT_CLASSIC : OBD_ADAPTER_TRANSPORT_BLE)];
        [self.discoveredSourceIndices addObject:@(i)];
        i++;
    }
}

- (void)stopScan {}

- (BOOL)connectToDevice:(int)index {
    fprintf(fake_log(), "CONNECT %d %s\n", index,
            index < (int)self.discoveredNames.count ? self.discoveredNames[(NSUInteger)index].UTF8String : "?");
    g_connected_idx = index;
    self.isConnected = YES;
    self.serviceDiscoveryDone = YES;
    return YES;
}

- (NSString *)connectedDeviceName {
    NSArray *names = fake_names();
    return g_connected_idx < (int)names.count ? names[(NSUInteger)g_connected_idx] : names[0];
}

- (void)disconnect {
    fprintf(fake_log(), "DISCONNECT\n");
    self.isConnected = NO;
    self.serviceDiscoveryDone = NO;
    self.canMonitorActive = NO;
    g_streaming = false;
}

- (BOOL)sendCommand:(const char *)cmd response:(char *)resp maxLen:(int)maxLen timeout:(double)t {
    (void)t;
    fprintf(fake_log(), "CMD %s\n", cmd);
    if (!self.isConnected) return NO;
    if (strcmp(cmd, "STI") == 0) {
        if (!getenv("FAKE_STN")) return NO;
        snprintf(resp, (size_t)maxLen, "STN2255 v5.6.19");
        return YES;
    }
    if (strcmp(cmd, "ATI") == 0) { snprintf(resp, (size_t)maxLen, "ELM327 v1.5"); return YES; }
    if (strcmp(cmd, "ATCSM1") == 0) {
        /* FAKE_CSM_REPLY: the reply ("timeout" = none) from the FAKE_CSM_FROM-th
           ATCSM1 on (default 1): the PASSIVE init must refuse to go on */
        static int csm_count = 0;
        const char *reply = getenv("FAKE_CSM_REPLY");
        const char *from = getenv("FAKE_CSM_FROM");
        if (reply && ++csm_count >= (from ? atoi(from) : 1)) {
            if (strcmp(reply, "timeout") == 0) return NO;
            snprintf(resp, (size_t)maxLen, "%s", reply);
            return YES;
        }
    }
    if (strncmp(cmd, "ATCRA", 5) == 0) snprintf(g_rx, sizeof(g_rx), "%s", cmd + 5);

    size_t n = strlen(cmd);
    bool hex = n >= 4 && strspn(cmd, "0123456789ABCDEF ") == n;
    const char *mode = getenv("FAKE_RESP");
    if (mode && strcmp(mode, "probe") == 0 && hex) {
        if (cmd[0] == '0' && cmd[1] == '1') {
            snprintf(resp, (size_t)maxLen, "%s 03 41 %c%c 80 55 55 55 55", g_rx, cmd[2], cmd[3]);
            return YES;
        }
        if (cmd[0] == '2' && cmd[1] == '2' && n >= 6) {
            snprintf(resp, (size_t)maxLen, "%s 05 62 %c%c %c%c 0A 0B 55 55", g_rx, cmd[2], cmd[3], cmd[4], cmd[5]);
            return YES;
        }
    }
    snprintf(resp, (size_t)maxLen, "OK");
    return YES;
}

- (BOOL)sendCommand:(const char *)cmd response:(char *)resp maxLen:(int)maxLen {
    return [self sendCommand:cmd response:resp maxLen:maxLen timeout:3.0];
}

- (void)startMonitorWithCallback:(obd_adapter_frame_callback_t)callback context:(void *)context {
    fprintf(fake_log(), "MONITOR_ON\n");
    g_cb = callback;
    g_ctx = context;
    self.canMonitorActive = YES;
}

- (void)stopMonitor {
    fprintf(fake_log(), "MONITOR_OFF\n");
    self.canMonitorActive = NO;
    g_streaming = false;
    g_cb = NULL;
    g_ctx = NULL;
}

- (void)sendRawString:(const char *)str {
    char shown[128];
    size_t k = 0;
    for (const char *p = str; *p && k + 5 < sizeof(shown); p++) {
        if (*p == '\r') { memcpy(shown + k, "<CR>", 4); k += 4; }
        else shown[k++] = *p;
    }
    shown[k] = '\0';
    fprintf(fake_log(), "RAW %s\n", shown);
    if (!self.isConnected) return;
    bool monitorCmd = strcmp(str, "ATMA\r") == 0 || strcmp(str, "STMA\r") == 0 || strcmp(str, "STM\r") == 0;
    if (self.canMonitorActive && monitorCmd) {
        setup_streams();
        g_streaming = true;
        if (g_first_monitor_ms == 0) g_first_monitor_ms = wall_ms();
    } else if (strcmp(str, "\r") == 0) {
        g_streaming = false;
    }
}

@end
