/**
 * @file obd_adapter_lifecycle_test.m
 * @brief OBDAdapter create/use/free across short-lived autorelease pools (no hardware)
 *
 * The adapter is compiled without ARC. Each round creates it inside one pool,
 * drains that pool, then uses and releases it inside another: anything the
 * adapter failed to retain (device lists, name hints, the Classic inquiry) is
 * freed by then and touching it crashes (deterministically with
 * NSZombieEnabled=YES). After release the run loop is pumped so a late
 * CoreBluetooth / IOBluetooth callback to a freed delegate would crash too.
 * No scan or connection is started; only the managers are created.
 */

#import <Foundation/Foundation.h>
#include <stdio.h>

#import "obd_adapter.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);            \
            fprintf(stderr, __VA_ARGS__);                                   \
            fprintf(stderr, "\n");                                          \
        }                                                                   \
    } while (0)

static void pump(double seconds) {
    NSDate *until = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while ([[NSDate date] compare:until] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.001]];
        }
    }
}

int main(void) {
    @autoreleasepool {
        NSArray *hints = [[NSArray alloc] initWithObjects:@"OBDLink", @"vLinker", nil];
        NSUInteger hintsRetain = [hints retainCount];

        for (int round = 0; round < 100; round++) {
            OBDAdapter *adapter = nil;
            @autoreleasepool {
                adapter = [[OBDAdapter alloc] initWithNameHints:hints quiet:YES];
            }

            @autoreleasepool {
                [adapter.discoveredDevices removeAllObjects];
                [adapter.discoveredNames addObject:@"vLinker TEST"];
                [adapter.discoveredRSSIs addObject:@(-40)];
                [adapter.discoveredTransports addObject:@(OBD_ADAPTER_TRANSPORT_BLE)];
                [adapter.discoveredSourceIndices addObject:@0];
                [adapter.classicDevices removeAllObjects];
                CHECK(adapter.discoveredNames.count == 1, "round %d: names", round);
                CHECK(adapter.nameHints.count == 2, "round %d: hints", round);
                CHECK(adapter.classicInquiry.inquiryLength == 4, "round %d: inquiry", round);
                CHECK(!adapter.isConnected && adapter.connectedDeviceName == nil,
                      "round %d: idle state", round);
                [adapter runLoopFor:0.002];
                [adapter disconnect];
                [adapter release];
            }
            pump(0.002);
        }

        CHECK([hints retainCount] == hintsRetain, "name hints released by dealloc (%lu vs %lu)",
              (unsigned long)[hints retainCount], (unsigned long)hintsRetain);
        [hints release];
    }

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
