/**
 * @file main.m
 * @brief VSP signal acquisition-rate validator.
 *
 * Validates whether selected vehicle-signal-profile signals can be observed
 * from passive CAN frames or diagnostic queries at a target rate.
 */

#import <Foundation/Foundation.h>
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include "elm327.h"
#include "stn2255.h"
#import "obd_adapter.h"

typedef NS_ENUM(NSInteger, VSPProbeMode) {
    VSPProbeModeAuto = 0,
    VSPProbeModeCAN,
    VSPProbeModeOBD,
};

typedef NS_ENUM(NSInteger, VSPProbeOBDStrategy) {
    VSPProbeOBDStrategyAuto = 0,
    VSPProbeOBDStrategyPoll,
    VSPProbeOBDStrategyPeriodic,
};

typedef struct {
    const char *profile_path;
    const char *device;
    const char *signals;
    const char *out_path;
    VSPProbeMode mode;
    VSPProbeOBDStrategy obd_strategy;
    bool plan_only;
    double target_hz;
    double duration_s;
    int timeout_ms;
} vsp_probe_options_t;

@interface VSPProbeItem : NSObject
@property (nonatomic, strong) NSDictionary *signal;
@property (nonatomic, strong) NSDictionary *query;
@property (nonatomic, copy) NSString *name;
@property (nonatomic, copy) NSString *canonicalId;
@property (nonatomic, copy) NSString *group;
@property (nonatomic, copy) NSString *acquisitionType;
@property (nonatomic, copy) NSString *queryKey;
@property (nonatomic, copy) NSString *requestHex;
@property (nonatomic, copy) NSString *requestBareHex;
@property (nonatomic, copy) NSString *positiveMarkerHex;
@property (nonatomic, assign) uint32_t frameId;
@property (nonatomic, assign) uint32_t frameIdMask;      /* can_frames[].frame_id_mask, 0 = full id */
@property (nonatomic, assign) uint32_t txId;
@property (nonatomic, assign) uint32_t rxId;
@property (nonatomic, assign) int attempts;
@property (nonatomic, assign) int samples;
@property (nonatomic, assign) int errors;
@property (nonatomic, assign) double firstSampleMs;
@property (nonatomic, assign) double lastSampleMs;
@property (nonatomic, assign) double testDurationS;
@property (nonatomic, assign) double lastValue;
@property (nonatomic, assign) BOOL hasLastValue;
@property (nonatomic, assign) BOOL lastValueNull;      /* value table state without a number */
@property (nonatomic, copy) NSString *lastLabel;        /* value table label, if any */
@property (nonatomic, copy) NSString *lastPayloadHex;
@end

@implementation VSPProbeItem
@end

typedef struct {
    __unsafe_unretained NSArray<VSPProbeItem *> *items;
    bool diagnostic;
} vsp_monitor_context_t;

static volatile sig_atomic_t g_interrupted = 0;

static void signal_handler(int sig) {
    (void)sig;
    g_interrupted = 1;
}

static int64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void usage(void) {
    fprintf(stderr,
            "Usage: vsp_probe <profile.vsp> [--device NAME] [--hz N] [--seconds N]\n"
            "                 [--mode auto|can|obd] [--obd poll|periodic|auto]\n"
            "                 [--signals LIST] [--timeout-ms N] [--plan] [--out FILE]\n\n"
            "Default signal groups: rpm, gear, steering, throttle, brake.\n"
            "--signals accepts group names, canonical ids, or VSP signal names.\n");
}

static bool parse_double_arg(const char *s, double *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    double v = strtod(s, &end);
    if (!end || *end != '\0') return false;
    *out = v;
    return true;
}

static bool parse_int_arg(const char *s, int *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_options(int argc, const char *argv[], vsp_probe_options_t *opts) {
    memset(opts, 0, sizeof(*opts));
    opts->mode = VSPProbeModeAuto;
    opts->obd_strategy = VSPProbeOBDStrategyAuto;
    opts->target_hz = 5.0;
    opts->duration_s = 10.0;
    opts->timeout_ms = 800;

    if (argc < 2) return false;
    opts->profile_path = argv[1];

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            opts->device = argv[++i];
        } else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
            if (!parse_double_arg(argv[++i], &opts->target_hz)) return false;
        } else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            if (!parse_double_arg(argv[++i], &opts->duration_s)) return false;
        } else if (strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
            if (!parse_int_arg(argv[++i], &opts->timeout_ms)) return false;
        } else if (strcmp(argv[i], "--signals") == 0 && i + 1 < argc) {
            opts->signals = argv[++i];
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            opts->out_path = argv[++i];
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            const char *mode = argv[++i];
            if (strcasecmp(mode, "auto") == 0) opts->mode = VSPProbeModeAuto;
            else if (strcasecmp(mode, "can") == 0) opts->mode = VSPProbeModeCAN;
            else if (strcasecmp(mode, "obd") == 0) opts->mode = VSPProbeModeOBD;
            else return false;
        } else if (strcmp(argv[i], "--obd") == 0 && i + 1 < argc) {
            const char *strategy = argv[++i];
            if (strcasecmp(strategy, "auto") == 0) opts->obd_strategy = VSPProbeOBDStrategyAuto;
            else if (strcasecmp(strategy, "poll") == 0) opts->obd_strategy = VSPProbeOBDStrategyPoll;
            else if (strcasecmp(strategy, "periodic") == 0) opts->obd_strategy = VSPProbeOBDStrategyPeriodic;
            else return false;
        } else if (strcmp(argv[i], "--plan") == 0) {
            opts->plan_only = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            return false;
        } else {
            fprintf(stderr, "[ERROR] Unexpected argument: %s\n", argv[i]);
            return false;
        }
    }

    if (opts->target_hz <= 0.0 || opts->duration_s <= 0.0 || opts->timeout_ms <= 0) {
        return false;
    }
    if (!opts->plan_only && (!opts->device || !*opts->device)) {
        fprintf(stderr, "[ERROR] --device is required unless --plan is used\n");
        return false;
    }
    return true;
}

static NSString *lower_string(NSString *s) {
    return s ? [[s lowercaseString] stringByTrimmingCharactersInSet:
                [NSCharacterSet whitespaceAndNewlineCharacterSet]] : @"";
}

static NSString *strip_hex(NSString *s) {
    if (!s) return @"";
    NSMutableString *out = [NSMutableString string];
    const char *c = s.UTF8String;
    for (size_t i = 0; c[i]; i++) {
        if (isxdigit((unsigned char)c[i])) {
            [out appendFormat:@"%c", toupper((unsigned char)c[i])];
        }
    }
    return out;
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool parse_hex_bytes(NSString *hexString, uint8_t *out, int maxLen, int *lenOut) {
    NSString *hex = strip_hex(hexString);
    const char *c = hex.UTF8String;
    int n = 0;
    for (int i = 0; c[i] && c[i + 1] && n < maxLen; i += 2) {
        int hi = hex_value(c[i]);
        int lo = hex_value(c[i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[n++] = (uint8_t)((hi << 4) | lo);
    }
    if (lenOut) *lenOut = n;
    return n > 0;
}

static NSString *format_bytes_hex(const uint8_t *bytes, int len) {
    NSMutableString *s = [NSMutableString string];
    for (int i = 0; i < len; i++) {
        [s appendFormat:@"%02X", bytes[i]];
    }
    return s;
}

static NSString *positive_marker_for_request(NSString *requestHex) {
    uint8_t req[16];
    int reqLen = 0;
    if (!parse_hex_bytes(requestHex, req, sizeof(req), &reqLen)) return nil;

    if (reqLen >= 2 && req[0] == 0x01) {
        return [NSString stringWithFormat:@"41%02X", req[1]];
    }
    if (reqLen >= 3 && req[0] == 0x22) {
        return [NSString stringWithFormat:@"62%02X%02X", req[1], req[2]];
    }
    if (reqLen >= 2 && req[0] == 0x21) {
        return [NSString stringWithFormat:@"61%02X", req[1]];
    }
    return nil;
}

static NSString *single_frame_payload_for_request(NSString *requestHex, uint8_t filler) {
    uint8_t req[7];
    int reqLen = 0;
    if (!parse_hex_bytes(requestHex, req, sizeof(req), &reqLen) || reqLen <= 0 || reqLen > 7) {
        return nil;
    }

    uint8_t payload[8];
    payload[0] = (uint8_t)reqLen;
    memcpy(payload + 1, req, (size_t)reqLen);
    for (int i = 1 + reqLen; i < 8; i++) payload[i] = filler;
    return format_bytes_hex(payload, 8);
}

static NSDictionary *load_profile(const char *path) {
    NSString *p = [NSString stringWithUTF8String:path];
    NSData *data = [NSData dataWithContentsOfFile:p];
    if (!data) {
        fprintf(stderr, "[ERROR] Failed to read profile: %s\n", path);
        return nil;
    }

    NSError *err = nil;
    id obj = [NSJSONSerialization JSONObjectWithData:data options:0 error:&err];
    if (!obj || ![obj isKindOfClass:[NSDictionary class]]) {
        fprintf(stderr, "[ERROR] Invalid VSP JSON: %s\n",
                err.localizedDescription.UTF8String ?: "unknown");
        return nil;
    }
    return obj;
}

static NSDictionary<NSString *, NSDictionary *> *query_map(NSDictionary *profile) {
    NSMutableDictionary *map = [NSMutableDictionary dictionary];
    NSArray *queries = profile[@"diagnostic_queries"];
    if (![queries isKindOfClass:[NSArray class]]) return map;

    for (NSDictionary *q in queries) {
        if (![q isKindOfClass:[NSDictionary class]]) continue;
        NSString *qid = q[@"id"];
        if ([qid isKindOfClass:[NSString class]]) map[qid] = q;
    }
    return map;
}

static NSString *canonical_id_for_signal(NSDictionary *signal) {
    id canonical = signal[@"canonical"];
    if ([canonical isKindOfClass:[NSDictionary class]]) {
        id cid = canonical[@"id"];
        if ([cid isKindOfClass:[NSString class]]) return cid;
    }
    return nil;
}

static NSString *group_for_canonical(NSString *canonical) {
    NSString *cid = lower_string(canonical);
    if ([cid isEqualToString:@"engine.speed"]) return @"rpm";
    if ([cid isEqualToString:@"transmission.gear.selected"]) return @"gear";
    if ([cid isEqualToString:@"steering.wheel.angle"]) return @"steering";
    if ([cid isEqualToString:@"throttle.position"] ||
        [cid isEqualToString:@"accelerator.pedal.position"]) return @"throttle";
    if ([cid isEqualToString:@"brake.pedal.position"] ||
        [cid isEqualToString:@"brake.pressure"]) return @"brake";
    return @"custom";
}

static void add_group_targets(NSMutableSet<NSString *> *canonicalTargets,
                              NSMutableSet<NSString *> *nameTargets,
                              NSString *token) {
    NSString *t = lower_string(token);
    if (t.length == 0) return;

    if ([t isEqualToString:@"rpm"] || [t isEqualToString:@"engine_speed"]) {
        [canonicalTargets addObject:@"engine.speed"];
    } else if ([t isEqualToString:@"gear"]) {
        [canonicalTargets addObject:@"transmission.gear.selected"];
    } else if ([t isEqualToString:@"steering"] || [t isEqualToString:@"steer"] ||
               [t isEqualToString:@"조향"]) {
        [canonicalTargets addObject:@"steering.wheel.angle"];
    } else if ([t isEqualToString:@"throttle"] || [t isEqualToString:@"스로틀"] ||
               [t isEqualToString:@"accelerator"] || [t isEqualToString:@"accel"]) {
        [canonicalTargets addObject:@"throttle.position"];
        [canonicalTargets addObject:@"accelerator.pedal.position"];
    } else if ([t isEqualToString:@"brake"] || [t isEqualToString:@"브레이크"]) {
        [canonicalTargets addObject:@"brake.pedal.position"];
        [canonicalTargets addObject:@"brake.pressure"];
    } else if ([t containsString:@"."]) {
        [canonicalTargets addObject:t];
    } else {
        [nameTargets addObject:t];
    }
}

static void build_targets(const char *signals,
                          NSMutableSet<NSString *> *canonicalTargets,
                          NSMutableSet<NSString *> *nameTargets) {
    if (!signals || !*signals) {
        [canonicalTargets addObject:@"engine.speed"];
        [canonicalTargets addObject:@"transmission.gear.selected"];
        [canonicalTargets addObject:@"steering.wheel.angle"];
        [canonicalTargets addObject:@"throttle.position"];
        [canonicalTargets addObject:@"accelerator.pedal.position"];
        [canonicalTargets addObject:@"brake.pedal.position"];
        [canonicalTargets addObject:@"brake.pressure"];
        return;
    }

    NSString *s = [NSString stringWithUTF8String:signals];
    for (NSString *part in [s componentsSeparatedByString:@","]) {
        add_group_targets(canonicalTargets, nameTargets, part);
    }
}

static bool acquisition_allowed(NSString *type, VSPProbeMode mode) {
    if (mode == VSPProbeModeCAN) return [type isEqualToString:@"can_frame"];
    if (mode == VSPProbeModeOBD) return [type isEqualToString:@"diagnostic_query"];
    return [type isEqualToString:@"can_frame"] || [type isEqualToString:@"diagnostic_query"];
}

static NSNumber *num_or_nil(id value) {
    if ([value isKindOfClass:[NSNumber class]]) return value;
    return nil;
}

static NSDictionary *fallback_query_from_signal(NSDictionary *signal) {
    NSDictionary *acq = signal[@"acquisition"];
    if (![acq isKindOfClass:[NSDictionary class]]) return nil;
    NSString *requestHex = acq[@"request_hex"];
    NSNumber *txId = num_or_nil(acq[@"tx_id"]);
    NSNumber *rxId = num_or_nil(acq[@"rx_id"]) ?: num_or_nil(acq[@"response_frame_id"]);
    if (!requestHex || !txId) return nil;

    NSMutableDictionary *q = [NSMutableDictionary dictionary];
    q[@"id"] = acq[@"query_id"] ?: [NSString stringWithFormat:@"inline_%@_%@",
                                    txId, strip_hex(requestHex)];
    q[@"kind"] = acq[@"request_kind"] ?: @"diagnostic_query";
    q[@"role"] = @"channel_query";
    q[@"tx_id"] = txId;
    if (rxId) q[@"rx_id"] = rxId;
    q[@"request_hex"] = requestHex;
    if (acq[@"payload_hex"]) q[@"payload_hex"] = acq[@"payload_hex"];
    return q;
}

static uint32_t frame_id_mask_for(NSDictionary *profile, uint32_t frameId) {
    NSArray *frames = profile[@"can_frames"];
    if (![frames isKindOfClass:[NSArray class]]) return 0;
    for (NSDictionary *frame in frames) {
        if (![frame isKindOfClass:[NSDictionary class]]) continue;
        if ([num_or_nil(frame[@"frame_id"]) unsignedIntValue] != frameId) continue;
        return [num_or_nil(frame[@"frame_id_mask"]) unsignedIntValue];
    }
    return 0;
}

static NSArray<VSPProbeItem *> *select_items(NSDictionary *profile,
                                             const vsp_probe_options_t *opts) {
    NSMutableSet<NSString *> *canonicalTargets = [NSMutableSet set];
    NSMutableSet<NSString *> *nameTargets = [NSMutableSet set];
    build_targets(opts->signals, canonicalTargets, nameTargets);

    NSDictionary *queries = query_map(profile);
    NSMutableArray<VSPProbeItem *> *items = [NSMutableArray array];
    NSArray *signals = profile[@"signals"];
    if (![signals isKindOfClass:[NSArray class]]) return items;

    for (NSDictionary *signal in signals) {
        if (![signal isKindOfClass:[NSDictionary class]]) continue;
        NSDictionary *acq = signal[@"acquisition"];
        if (![acq isKindOfClass:[NSDictionary class]]) continue;
        NSString *type = acq[@"type"];
        if (![type isKindOfClass:[NSString class]] || !acquisition_allowed(type, opts->mode)) {
            continue;
        }

        NSString *name = signal[@"name"];
        NSString *canonical = canonical_id_for_signal(signal);
        NSString *nameLower = lower_string(name);
        NSString *canonicalLower = lower_string(canonical);
        bool selected = (canonicalLower.length > 0 && [canonicalTargets containsObject:canonicalLower]) ||
                        (nameLower.length > 0 && [nameTargets containsObject:nameLower]);
        if (!selected) continue;

        VSPProbeItem *item = [[VSPProbeItem alloc] init];
        item.signal = signal;
        item.name = name ?: @"";
        item.canonicalId = canonical ?: @"";
        item.group = group_for_canonical(canonical);
        item.acquisitionType = type;

        if ([type isEqualToString:@"can_frame"]) {
            NSNumber *frameId = num_or_nil(acq[@"frame_id"]);
            if (!frameId) continue;
            item.frameId = frameId.unsignedIntValue;
            item.frameIdMask = frame_id_mask_for(profile, item.frameId);
        } else {
            NSString *qid = acq[@"query_id"];
            NSDictionary *q = ([qid isKindOfClass:[NSString class]]) ? queries[qid] : nil;
            if (!q) q = fallback_query_from_signal(signal);
            if (!q) continue;

            item.query = q;
            item.requestHex = q[@"request_hex"] ?: acq[@"request_hex"] ?: @"";
            item.requestBareHex = strip_hex(item.requestHex);
            item.positiveMarkerHex = positive_marker_for_request(item.requestHex);
            item.txId = [num_or_nil(q[@"tx_id"]) unsignedIntValue];
            NSNumber *rx = num_or_nil(q[@"rx_id"]) ?: num_or_nil(acq[@"rx_id"]) ?:
                           num_or_nil(acq[@"response_frame_id"]);
            item.rxId = rx ? rx.unsignedIntValue : 0;
            item.queryKey = [NSString stringWithFormat:@"%03X:%03X:%@",
                             item.txId, item.rxId, item.requestBareHex ?: @""];
        }
        [items addObject:item];
    }

    [items sortUsingComparator:^NSComparisonResult(VSPProbeItem *a, VSPProbeItem *b) {
        NSComparisonResult c = [a.group compare:b.group];
        if (c != NSOrderedSame) return c;
        c = [a.canonicalId compare:b.canonicalId];
        if (c != NSOrderedSame) return c;
        c = [a.acquisitionType compare:b.acquisitionType];
        if (c != NSOrderedSame) return c;
        return [a.name compare:b.name];
    }];
    return items;
}

static uint64_t extract_bits_little(const uint8_t *data, int len, int startBit, int bitLen) {
    uint64_t raw = 0;
    for (int i = 0; i < bitLen; i++) {
        int bit = startBit + i;
        int byteIdx = bit / 8;
        int bitIdx = bit % 8;
        if (byteIdx < 0 || byteIdx >= len) break;
        if ((data[byteIdx] >> bitIdx) & 1U) raw |= (1ULL << i);
    }
    return raw;
}

static uint64_t extract_bits_big_dbc(const uint8_t *data, int len, int startBit, int bitLen) {
    uint64_t raw = 0;
    int bit = startBit;
    for (int i = 0; i < bitLen; i++) {
        int byteIdx = bit / 8;
        int bitIdx = bit % 8;
        raw <<= 1;
        if (byteIdx >= 0 && byteIdx < len && ((data[byteIdx] >> bitIdx) & 1U)) {
            raw |= 1U;
        }
        bit = (bitIdx == 0) ? (bit + 15) : (bit - 1);
    }
    return raw;
}

static bool decode_signal_value(NSDictionary *signal,
                                const uint8_t *data,
                                int dataLen,
                                double *valueOut,
                                BOOL *hasNumberOut,
                                NSString **labelOut) {
    NSDictionary *decode = signal[@"decode"];
    if (![decode isKindOfClass:[NSDictionary class]]) return false;

    NSNumber *start = num_or_nil(decode[@"start_bit"]);
    NSNumber *bits = num_or_nil(decode[@"bit_length"]);
    if (!start || !bits || bits.intValue <= 0 || bits.intValue > 63) return false;

    NSString *order = decode[@"byte_order"] ?: @"little_endian";
    uint64_t raw = 0;
    if ([order isEqualToString:@"big_endian"]) {
        raw = extract_bits_big_dbc(data, dataLen, start.intValue, bits.intValue);
    } else {
        raw = extract_bits_little(data, dataLen, start.intValue, bits.intValue);
    }

    BOOL isSigned = [decode[@"signed"] respondsToSelector:@selector(boolValue)] ?
                    [decode[@"signed"] boolValue] : NO;
    double signedRaw = (double)raw;
    if (isSigned && bits.intValue > 0) {
        uint64_t signBit = 1ULL << (bits.intValue - 1);
        if (raw & signBit) {
            uint64_t mask = (bits.intValue == 64) ? UINT64_MAX : ((1ULL << bits.intValue) - 1);
            int64_t signedVal = -((int64_t)((~raw + 1) & mask));
            signedRaw = (double)signedVal;
        }
    }

    /* Sign-magnitude: raw is the magnitude, sign_bit=1 negates it. */
    NSNumber *signBit = num_or_nil(decode[@"sign_bit"]);
    if (signBit && !isSigned) {
        int sb = signBit.intValue;
        if (sb >= 0 && sb / 8 < dataLen && ((data[sb / 8] >> (sb % 8)) & 1)) {
            signedRaw = -signedRaw;
        }
    }

    double factor = [decode[@"factor"] respondsToSelector:@selector(doubleValue)] ?
                    [decode[@"factor"] doubleValue] : 1.0;
    double offset = [decode[@"offset"] respondsToSelector:@selector(doubleValue)] ?
                    [decode[@"offset"] doubleValue] : 0.0;
    double value = signedRaw * factor + offset;
    BOOL hasNumber = YES;
    NSString *label = nil;

    /* Value table: label for the raw code, optional numeric override (null = none). */
    NSArray *values = decode[@"values"];
    if ([values isKindOfClass:[NSArray class]]) {
        long long rawCode = llround(signedRaw);
        for (NSDictionary *entry in values) {
            if (![entry isKindOfClass:[NSDictionary class]]) continue;
            NSNumber *entryRaw = num_or_nil(entry[@"raw"]);
            if (!entryRaw || entryRaw.longLongValue != rawCode) continue;
            if ([entry[@"label"] isKindOfClass:[NSString class]]) label = entry[@"label"];
            id override = entry[@"value"];
            if (override == [NSNull null]) hasNumber = NO;
            else if ([override isKindOfClass:[NSNumber class]]) value = [override doubleValue];
            break;
        }
    }

    if (valueOut) *valueOut = value;
    if (hasNumberOut) *hasNumberOut = hasNumber;
    if (labelOut) *labelOut = label;
    return true;
}

/* The signal's mux field, decoded like the signal itself. False when there is none
 * or it lacks a position (DBC style {"value": n}), so callers fall back to markers. */
static bool signal_mux(NSDictionary *signal, uint64_t *valueOut, int *startOut, int *bitsOut, bool *bigOut) {
    NSDictionary *decode = signal[@"decode"];
    if (![decode isKindOfClass:[NSDictionary class]]) return false;
    NSDictionary *mux = decode[@"mux"];
    if (![mux isKindOfClass:[NSDictionary class]]) return false;
    NSNumber *value = num_or_nil(mux[@"value"]);
    NSNumber *start = num_or_nil(mux[@"start_bit"]);
    NSNumber *bits = num_or_nil(mux[@"bit_length"]);
    if (!value || !start || !bits || bits.intValue <= 0 || bits.intValue > 63) return false;
    if (valueOut) *valueOut = value.unsignedLongLongValue;
    if (startOut) *startOut = start.intValue;
    if (bitsOut) *bitsOut = bits.intValue;
    if (bigOut) *bigOut = [mux[@"byte_order"] isEqual:@"big_endian"];
    return true;
}

static bool signal_mux_matches(NSDictionary *signal, const uint8_t *data, int len) {
    uint64_t value = 0;
    int start = 0, bits = 0;
    bool big = false;
    if (!signal_mux(signal, &value, &start, &bits, &big)) return true;
    uint64_t raw = big ? extract_bits_big_dbc(data, len, start, bits)
                       : extract_bits_little(data, len, start, bits);
    return raw == value;
}

static int extract_line_hex(const char *line, char *hexOut, size_t hexOutLen) {
    size_t j = 0;
    if (!line || !hexOut || hexOutLen == 0) return 0;
    for (size_t i = 0; line[i] && j + 1 < hexOutLen; i++) {
        if (hex_value(line[i]) >= 0) hexOut[j++] = (char)toupper((unsigned char)line[i]);
    }
    hexOut[j] = '\0';
    return (int)j;
}

static bool parse_hex_byte_c(const char *hex, uint8_t *out) {
    int hi = hex_value(hex[0]);
    int lo = hex_value(hex[1]);
    if (hi < 0 || lo < 0) return false;
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool frame_initial_isotp_payload(const can_frame_t *frame,
                                        uint8_t *payloadOut,
                                        int maxPayload,
                                        int *payloadLenOut) {
    if (payloadLenOut) *payloadLenOut = 0;
    if (!frame || frame->dlc == 0 || !payloadOut || maxPayload <= 0) return false;

    uint8_t pci = frame->data[0];
    uint8_t type = (uint8_t)(pci >> 4);
    int len = 0;
    const uint8_t *src = NULL;

    if (type == 0x0) {
        len = pci & 0x0F;
        if (len > frame->dlc - 1) len = frame->dlc - 1;
        src = frame->data + 1;
    } else if (type == 0x1 && frame->dlc >= 2) {
        len = frame->dlc - 2;
        src = frame->data + 2;
    } else {
        return false;
    }

    if (len <= 0) return false;
    if (len > maxPayload) len = maxPayload;
    memcpy(payloadOut, src, (size_t)len);
    if (payloadLenOut) *payloadLenOut = len;
    return true;
}

static bool payload_has_marker(const uint8_t *payload,
                               int payloadLen,
                               NSString *markerHex) {
    if (!markerHex || markerHex.length == 0) return true;
    uint8_t marker[16];
    int markerLen = 0;
    if (!parse_hex_bytes(markerHex, marker, sizeof(marker), &markerLen)) return true;
    if (payloadLen < markerLen) return false;
    return memcmp(payload, marker, (size_t)markerLen) == 0;
}

static void mark_sample(VSPProbeItem *item,
                        double timestampMs,
                        const uint8_t *payload,
                        int payloadLen,
                        const uint8_t *decodeBytes,
                        int decodeLen) {
    item.samples += 1;
    if (item.firstSampleMs <= 0.0) item.firstSampleMs = timestampMs;
    item.lastSampleMs = timestampMs;
    if (payload && payloadLen > 0) item.lastPayloadHex = format_bytes_hex(payload, payloadLen);

    double value = 0.0;
    BOOL hasNumber = YES;
    NSString *label = nil;
    if (decodeBytes && decodeLen > 0 &&
        decode_signal_value(item.signal, decodeBytes, decodeLen, &value, &hasNumber, &label)) {
        item.lastValue = value;
        item.hasLastValue = hasNumber;
        item.lastValueNull = !hasNumber;
        item.lastLabel = label;
    }
}

/* Diagnostic layouts are per CAN frame, PCI byte(s) included (VSP_RUNTIME_SPEC.md,
 * "Diagnostic Response Frames"). A signal with a mux is matched by it, which also
 * selects consecutive frames; one without falls back to the positive-response marker
 * at the start of a single or first frame. */
static void match_diagnostic_frame(NSArray<VSPProbeItem *> *items,
                                   NSString *queryKey,
                                   const can_frame_t *frame,
                                   double ts) {
    uint8_t payload[64];
    int payloadLen = 0;
    bool initial = frame_initial_isotp_payload(frame, payload, sizeof(payload), &payloadLen);

    for (VSPProbeItem *item in items) {
        if (queryKey && ![item.queryKey isEqualToString:queryKey]) continue;
        if (item.rxId != 0 && item.rxId != frame->can_id) continue;
        if (signal_mux(item.signal, NULL, NULL, NULL, NULL)) {
            if (!signal_mux_matches(item.signal, frame->data, frame->dlc)) continue;
        } else if (!initial || !payload_has_marker(payload, payloadLen, item.positiveMarkerHex)) {
            continue;
        }
        mark_sample(item, ts, frame->data, frame->dlc, frame->data, frame->dlc);
    }
}

static void monitor_callback(const can_frame_t *frame, void *context) {
    vsp_monitor_context_t *ctx = (vsp_monitor_context_t *)context;
    if (!ctx || !frame) return;

    double ts = frame->timestamp_ms > 0 ? (double)frame->timestamp_ms : (double)now_ms();

    if (!ctx->diagnostic) {
        for (VSPProbeItem *item in ctx->items) {
            uint32_t mask = item.frameIdMask ? item.frameIdMask : UINT32_MAX;
            if ((item.frameId & mask) != (frame->can_id & mask)) continue;
            if (!signal_mux_matches(item.signal, frame->data, frame->dlc)) continue;
            mark_sample(item, ts, frame->data, frame->dlc, frame->data, frame->dlc);
        }
        return;
    }

    match_diagnostic_frame(ctx->items, nil, frame, ts);
}

static void send_best_effort(OBDAdapter *adapter, const char *cmd) {
    char resp[512] = "";
    [adapter sendCommand:cmd response:resp maxLen:sizeof(resp)];
}

static bool send_required(OBDAdapter *adapter, const char *cmd) {
    char resp[512] = "";
    if (![adapter sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
        elm327_is_error_response(resp)) {
        fprintf(stderr, "[ERROR] Command failed: %s", cmd);
        if (resp[0]) fprintf(stderr, " => %s", resp);
        fprintf(stderr, "\n");
        return false;
    }
    return true;
}

static bool init_basic_adapter(OBDAdapter *adapter, stn2255_info_t *stnInfo, bool *stnDetected) {
    char resp[512] = "";
    const char *init[] = { "ATZ", "ATE0", "ATL0", "ATS1" };
    for (int i = 0; i < 4; i++) {
        [adapter sendCommand:init[i] response:resp maxLen:sizeof(resp) timeout:3.0];
        if (strcmp(init[i], "ATZ") == 0) [adapter runLoopFor:0.5];
    }

    *stnDetected = false;
    char sti[256] = "";
    if ([adapter sendCommand:"STI" response:sti maxLen:sizeof(sti)] &&
        stn2255_detect(resp, sti, stnInfo)) {
        *stnDetected = true;
    }
    return true;
}

static NSMutableArray<NSNumber *> *unique_can_ids_for_items(NSArray<VSPProbeItem *> *items,
                                                            bool diagnostic) {
    NSMutableSet<NSNumber *> *seen = [NSMutableSet set];
    NSMutableArray<NSNumber *> *ids = [NSMutableArray array];
    for (VSPProbeItem *item in items) {
        uint32_t vals[2] = { diagnostic ? item.txId : item.frameId,
                             diagnostic ? item.rxId : 0 };
        for (int i = 0; i < 2; i++) {
            if (vals[i] == 0) continue;
            NSNumber *n = @(vals[i]);
            if (![seen containsObject:n]) {
                [seen addObject:n];
                [ids addObject:n];
            }
        }
    }
    return ids;
}

static void cleanup_periodic_obd(OBDAdapter *adapter, NSString *strategyName);

static bool setup_passive_can(OBDAdapter *adapter,
                              NSArray<VSPProbeItem *> *items,
                              bool *stnDetectedOut) {
    stn2255_info_t stn = {0};
    bool stnDetected = false;
    init_basic_adapter(adapter, &stn, &stnDetected);
    if (stnDetectedOut) *stnDetectedOut = stnDetected;

    if (!send_required(adapter, "ATSP6")) return false;
    send_best_effort(adapter, "ATCSM1");
    send_best_effort(adapter, "ATS1");
    send_best_effort(adapter, "ATCAF0");
    send_best_effort(adapter, "ATH1");

    if (stnDetected) {
        send_best_effort(adapter, "STFAC");
        for (NSNumber *n in unique_can_ids_for_items(items, false)) {
            uint32_t mask = 0x7FF;
            for (VSPProbeItem *item in items) {
                if (item.frameId == n.unsignedIntValue && item.frameIdMask) mask = item.frameIdMask & 0x7FF;
            }
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "STFAP %03X,%03X", n.unsignedIntValue & mask, mask);
            send_best_effort(adapter, cmd);
        }
    }
    return true;
}

static void run_monitor_phase(OBDAdapter *adapter,
                              const char *monitorCmd,
                              NSArray<VSPProbeItem *> *items,
                              bool diagnostic,
                              double seconds) {
    vsp_monitor_context_t ctx = { .items = items, .diagnostic = diagnostic };
    [adapter startMonitorWithCallback:monitor_callback context:&ctx];
    [adapter sendRawString:[[NSString stringWithFormat:@"%s\r", monitorCmd] UTF8String]];

    NSDate *end = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while (!g_interrupted && adapter.isConnected &&
           [[NSDate date] compare:end] == NSOrderedAscending) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }

    [adapter stopMonitor];
    [adapter sendRawString:"\r"];
    [adapter runLoopFor:0.2];
}

static bool run_can_phase(OBDAdapter *adapter,
                          NSArray<VSPProbeItem *> *items,
                          double seconds,
                          NSMutableDictionary *phaseOut) {
    if (items.count == 0) return true;

    bool stnDetected = false;
    if (!setup_passive_can(adapter, items, &stnDetected)) return false;

    NSString *monitorCmd = stnDetected ? @"STMA" : @"ATMA";
    phaseOut[@"name"] = @"can";
    phaseOut[@"strategy"] = @"passive_can_monitor";
    phaseOut[@"monitor_command"] = monitorCmd;
    phaseOut[@"stn_detected"] = @(stnDetected);

    int64_t startMs = now_ms();
    run_monitor_phase(adapter, monitorCmd.UTF8String, items, false, seconds);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    for (VSPProbeItem *item in items) item.testDurationS = phaseDuration;
    return true;
}

static NSArray<VSPProbeItem *> *unique_query_items(NSArray<VSPProbeItem *> *items) {
    NSMutableSet<NSString *> *seen = [NSMutableSet set];
    NSMutableArray<VSPProbeItem *> *unique = [NSMutableArray array];
    for (VSPProbeItem *item in items) {
        if (!item.queryKey || [seen containsObject:item.queryKey]) continue;
        [seen addObject:item.queryKey];
        [unique addObject:item];
    }
    return unique;
}

static NSArray<NSDictionary *> *flow_control_queries_for_items(NSDictionary *profile,
                                                               NSArray<VSPProbeItem *> *queryItems) {
    NSArray *queries = profile[@"diagnostic_queries"];
    if (![queries isKindOfClass:[NSArray class]]) return @[];

    NSMutableArray *out = [NSMutableArray array];
    for (NSDictionary *q in queries) {
        if (![q isKindOfClass:[NSDictionary class]]) continue;
        NSString *role = q[@"role"];
        if (![role isKindOfClass:[NSString class]] || ![role isEqualToString:@"flow_control"]) {
            continue;
        }

        uint32_t tx = [num_or_nil(q[@"tx_id"]) unsignedIntValue];
        uint32_t rx = [num_or_nil(q[@"rx_id"]) unsignedIntValue];
        for (VSPProbeItem *item in queryItems) {
            if (item.txId == tx && (rx == 0 || item.rxId == rx)) {
                [out addObject:q];
                break;
            }
        }
    }
    return out;
}

static bool setup_periodic_obd(OBDAdapter *adapter,
                               NSDictionary *profile,
                               NSArray<VSPProbeItem *> *items,
                               double targetHz,
                               VSPProbeOBDStrategy strategy,
                               NSString **strategyNameOut) {
    stn2255_info_t stn = {0};
    bool stnDetected = false;
    init_basic_adapter(adapter, &stn, &stnDetected);

    NSString *deviceName = lower_string(adapter.connectedDeviceName);
    bool useMic = [deviceName containsString:@"vlinker"];
    /* vLinker monitoring mode is receive-only; the manufacturer confirmed
       ATMA/STM/STMA keep the bus silent and do not transmit PM messages. */
    if (useMic) {
        if (strategy == VSPProbeOBDStrategyPeriodic) {
            fprintf(stderr, "[ERROR] vLinker periodic OBD is unsupported: "
                    "monitoring mode disables PM transmissions\n");
        }
        return false;
    }
    if (strategy == VSPProbeOBDStrategyPeriodic && !stnDetected) {
        fprintf(stderr, "[ERROR] Periodic OBD requires STN/OBDLink periodic-message support\n");
        return false;
    }
    if (strategy == VSPProbeOBDStrategyAuto && !stnDetected) {
        return false;
    }

    int periodMs = (int)llround(1000.0 / targetHz);
    if (periodMs < 20) periodMs = 20;

    NSArray<VSPProbeItem *> *queries = unique_query_items(items);
    NSArray<NSDictionary *> *flowControls = flow_control_queries_for_items(profile, queries);
    NSMutableArray<NSDictionary *> *rawSchedules = [NSMutableArray array];
    for (VSPProbeItem *item in queries) {
        NSString *payload = item.query[@"payload_hex"];
        if (![payload isKindOfClass:[NSString class]] || payload.length == 0) {
            payload = single_frame_payload_for_request(item.requestHex, 0x55);
        }
        if (!payload) return false;
        [rawSchedules addObject:@{@"tx": @(item.txId), @"payload": strip_hex(payload)}];
    }
    for (NSDictionary *q in flowControls) {
        NSString *payload = q[@"payload_hex"];
        if (![payload isKindOfClass:[NSString class]] || payload.length == 0) {
            payload = single_frame_payload_for_request(q[@"request_hex"], 0xAA);
        }
        if (payload && q[@"tx_id"]) {
            [rawSchedules addObject:@{@"tx": q[@"tx_id"], @"payload": strip_hex(payload)}];
        }
    }
    send_best_effort(adapter, "ATH1");
    send_best_effort(adapter, "ATS1");

    if (!useMic && stnDetected) {
        *strategyNameOut = @"periodic_stn_raw";
        if (!send_required(adapter, "STPPMC") ||
            !send_required(adapter, "STCFCPC") ||
            !send_required(adapter, "STP 31") ||
            !send_required(adapter, "STPO") ||
            !send_required(adapter, "ATCSM0") ||
            !send_required(adapter, "ATH1") ||
            !send_required(adapter, "ATCAF0") ||
            !send_required(adapter, "STCMM 1") ||
            !send_required(adapter, "STCSEGR 0") ||
            !send_required(adapter, "STCSEGT 0") ||
            !send_required(adapter, "STFAC")) {
            cleanup_periodic_obd(adapter, *strategyNameOut);
            return false;
        } else {
            for (NSNumber *n in unique_can_ids_for_items(items, true)) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "STFAP %03X,7FF", n.unsignedIntValue);
                send_best_effort(adapter, cmd);
            }

            bool scheduled = true;
            for (NSDictionary *sched in rawSchedules) {
                char cmd[128];
                snprintf(cmd, sizeof(cmd), "STPPMA %d, %03X, %s",
                         periodMs,
                         [sched[@"tx"] unsignedIntValue],
                         [sched[@"payload"] UTF8String]);
                if (!send_required(adapter, cmd)) {
                    scheduled = false;
                    break;
                }
                [adapter runLoopFor:(double)periodMs / 1000.0 / MAX((int)rawSchedules.count, 1)];
            }
            if (scheduled) return true;
            cleanup_periodic_obd(adapter, *strategyNameOut);
            return false;
        }
    }

    return false;
}

static void cleanup_periodic_obd(OBDAdapter *adapter, NSString *strategyName) {
    if ([strategyName isEqualToString:@"periodic_stn_raw"]) {
        send_best_effort(adapter, "STPPMC");
    }
    send_best_effort(adapter, "STCMM 0");
}

static bool setup_poll_obd(OBDAdapter *adapter) {
    stn2255_info_t stn = {0};
    bool stnDetected = false;
    init_basic_adapter(adapter, &stn, &stnDetected);
    (void)stnDetected;

    send_best_effort(adapter, "ATSP6");
    send_best_effort(adapter, "ATCSM0");
    send_best_effort(adapter, "ATAT0");
    send_best_effort(adapter, "ATST 10");
    send_best_effort(adapter, "ATAL");
    send_best_effort(adapter, "ATS1");
    send_best_effort(adapter, "ATH1");
    send_best_effort(adapter, "ATCAF1");
    send_best_effort(adapter, "ATCFC1");
    send_best_effort(adapter, "ATFCSD300001");
    send_best_effort(adapter, "ATFCSM1");
    return true;
}

/* One ELM327 response line with headers on: 3 hex digits of 11-bit ID, then bytes. */
static bool parse_response_frame_line(const char *line, can_frame_t *frame) {
    char hex[256];
    int hexLen = extract_line_hex(line, hex, sizeof(hex));
    if (hexLen < 5) return false;
    unsigned int idVal = 0;
    if (sscanf(hex, "%3x", &idVal) != 1) return false;
    frame->can_id = idVal;
    frame->dlc = 0;
    for (int i = 3; i + 1 < hexLen && frame->dlc < 8; i += 2) {
        if (!parse_hex_byte_c(hex + i, &frame->data[frame->dlc])) break;
        frame->dlc++;
    }
    return frame->dlc > 0;
}

static bool poll_query_once(OBDAdapter *adapter,
                            VSPProbeItem *queryItem,
                            NSArray<VSPProbeItem *> *allItems,
                            double timeoutSeconds) {
    char cmd[64];
    char resp[4096] = "";

    snprintf(cmd, sizeof(cmd), "ATSH%03X", queryItem.txId);
    send_best_effort(adapter, cmd);
    snprintf(cmd, sizeof(cmd), "ATFCSH%03X", queryItem.txId);
    send_best_effort(adapter, cmd);
    if (queryItem.rxId != 0) {
        snprintf(cmd, sizeof(cmd), "ATCRA%03X", queryItem.rxId);
        send_best_effort(adapter, cmd);
    }

    for (VSPProbeItem *item in allItems) {
        if ([item.queryKey isEqualToString:queryItem.queryKey]) item.attempts += 1;
    }

    BOOL ok = [adapter sendCommand:queryItem.requestBareHex.UTF8String
                          response:resp
                            maxLen:sizeof(resp)
                           timeout:timeoutSeconds];
    if (!ok || elm327_is_error_response(resp)) {
        for (VSPProbeItem *item in allItems) {
            if ([item.queryKey isEqualToString:queryItem.queryKey]) item.errors += 1;
        }
        return false;
    }

    /* With ATH1 every response line is one CAN frame "ID PCI DATA..."; decode each
     * frame like the monitor path does instead of the reassembled payload. */
    double ts = (double)now_ms();
    int frames = 0;
    char buf[sizeof(resp)];
    snprintf(buf, sizeof(buf), "%s", resp);
    for (char *line = strtok(buf, "\r\n"); line; line = strtok(NULL, "\r\n")) {
        can_frame_t frame = {0};
        if (!parse_response_frame_line(line, &frame)) continue;
        frames += 1;
        match_diagnostic_frame(allItems, queryItem.queryKey, &frame, ts);
    }
    return frames > 0;
}

static bool run_poll_obd_phase(OBDAdapter *adapter,
                               NSArray<VSPProbeItem *> *items,
                               double targetHz,
                               double seconds,
                               int timeoutMs,
                               NSMutableDictionary *phaseOut) {
    if (!setup_poll_obd(adapter)) return false;

    NSArray<VSPProbeItem *> *queries = unique_query_items(items);
    phaseOut[@"name"] = @"obd";
    phaseOut[@"strategy"] = @"poll";
    phaseOut[@"query_count"] = @(queries.count);

    double interval = 1.0 / targetHz;
    double timeout = (double)timeoutMs / 1000.0;
    int64_t startMs = now_ms();
    NSDate *end = [NSDate dateWithTimeIntervalSinceNow:seconds];

    while (!g_interrupted && adapter.isConnected &&
           [[NSDate date] compare:end] == NSOrderedAscending) {
        int64_t cycleStart = now_ms();
        for (VSPProbeItem *q in queries) {
            if (g_interrupted) break;
            poll_query_once(adapter, q, items, timeout);
        }
        double elapsed = (double)(now_ms() - cycleStart) / 1000.0;
        double sleepS = interval - elapsed;
        if (sleepS > 0.0) [adapter runLoopFor:sleepS];
    }
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    for (VSPProbeItem *item in items) item.testDurationS = phaseDuration;
    return true;
}

static bool run_periodic_obd_phase(OBDAdapter *adapter,
                                   NSDictionary *profile,
                                   NSArray<VSPProbeItem *> *items,
                                   double targetHz,
                                   double seconds,
                                   VSPProbeOBDStrategy strategy,
                                   NSMutableDictionary *phaseOut) {
    NSString *strategyName = nil;
    if (!setup_periodic_obd(adapter, profile, items, targetHz, strategy, &strategyName)) {
        return false;
    }

    phaseOut[@"name"] = @"obd";
    phaseOut[@"strategy"] = strategyName ?: @"periodic";
    phaseOut[@"query_count"] = @(unique_query_items(items).count);
    phaseOut[@"monitor_command"] = @"STM";

    int64_t startMs = now_ms();
    run_monitor_phase(adapter, "STM", items, true, seconds);
    double phaseDuration = (double)(now_ms() - startMs) / 1000.0;
    phaseOut[@"actual_duration_s"] = @(((double)llround(phaseDuration * 1000.0)) / 1000.0);
    int scheduledAttempts = (int)floor(targetHz * phaseDuration);
    for (VSPProbeItem *item in items) {
        item.testDurationS = phaseDuration;
        item.attempts = scheduledAttempts;
    }
    cleanup_periodic_obd(adapter, strategyName);
    return true;
}

static bool run_obd_phase(OBDAdapter *adapter,
                          NSDictionary *profile,
                          NSArray<VSPProbeItem *> *items,
                          const vsp_probe_options_t *opts,
                          NSMutableDictionary *phaseOut) {
    if (items.count == 0) return true;

    if (opts->obd_strategy != VSPProbeOBDStrategyPoll) {
        if (run_periodic_obd_phase(adapter, profile, items,
                                   opts->target_hz, opts->duration_s,
                                   opts->obd_strategy, phaseOut)) {
            return true;
        }
        if (opts->obd_strategy == VSPProbeOBDStrategyPeriodic) return false;
        fprintf(stderr, "[WARN] Periodic OBD unavailable; falling back to poll mode\n");
    }
    return run_poll_obd_phase(adapter, items, opts->target_hz, opts->duration_s,
                              opts->timeout_ms, phaseOut);
}

static NSDictionary *plan_for_item(VSPProbeItem *item) {
    NSMutableDictionary *d = [NSMutableDictionary dictionary];
    d[@"name"] = item.name ?: @"";
    d[@"canonical_id"] = item.canonicalId ?: @"";
    d[@"group"] = item.group ?: @"";
    d[@"acquisition_type"] = item.acquisitionType ?: @"";

    NSDictionary *decode = item.signal[@"decode"];
    if ([decode isKindOfClass:[NSDictionary class]]) {
        d[@"unit"] = decode[@"unit"] ?: @"";
    }

    if ([item.acquisitionType isEqualToString:@"can_frame"]) {
        d[@"frame_id"] = [NSString stringWithFormat:@"0x%03X", item.frameId];
    } else {
        d[@"query_id"] = item.query[@"id"] ?: @"";
        d[@"tx_id"] = [NSString stringWithFormat:@"0x%03X", item.txId];
        if (item.rxId) d[@"rx_id"] = [NSString stringWithFormat:@"0x%03X", item.rxId];
        d[@"request_hex"] = item.requestHex ?: @"";
        if (item.positiveMarkerHex) d[@"positive_marker_hex"] = item.positiveMarkerHex;
    }
    return d;
}

static NSDictionary *result_for_item(VSPProbeItem *item,
                                     double targetHz,
                                     double durationS) {
    NSMutableDictionary *d = [[plan_for_item(item) mutableCopy] autorelease];
    double itemDuration = item.testDurationS > 0.0 ? item.testDurationS : durationS;
    double observed = itemDuration > 0.0 ? (double)item.samples / itemDuration : 0.0;
    bool met = observed + (targetHz * 0.05) >= targetHz;

    d[@"target_hz"] = @(targetHz);
    d[@"observed_hz"] = @(((double)llround(observed * 1000.0)) / 1000.0);
    d[@"measured_duration_s"] = @(((double)llround(itemDuration * 1000.0)) / 1000.0);
    d[@"samples"] = @(item.samples);
    d[@"attempts"] = @(item.attempts);
    d[@"errors"] = @(item.errors);
    d[@"target_met"] = @(met);
    d[@"status"] = item.samples > 0 ? (met ? @"pass" : @"under_target") : @"no_samples";
    if (item.lastPayloadHex) d[@"last_payload_hex"] = item.lastPayloadHex;
    if (item.hasLastValue) d[@"last_value"] = @(item.lastValue);
    else if (item.lastValueNull) d[@"last_value"] = [NSNull null];
    if (item.lastLabel) d[@"last_label"] = item.lastLabel;
    if (item.firstSampleMs > 0.0) d[@"first_sample_ms"] = @((long long)item.firstSampleMs);
    if (item.lastSampleMs > 0.0) d[@"last_sample_ms"] = @((long long)item.lastSampleMs);
    return d;
}

static NSString *mode_name(VSPProbeMode mode) {
    switch (mode) {
        case VSPProbeModeCAN: return @"can";
        case VSPProbeModeOBD: return @"obd";
        default: return @"auto";
    }
}

static NSDictionary *build_output(NSDictionary *profile,
                                  const char *profilePath,
                                  const vsp_probe_options_t *opts,
                                  NSArray<VSPProbeItem *> *items,
                                  NSArray<NSDictionary *> *phases,
                                  bool planOnly,
                                  double actualDuration) {
    NSMutableDictionary *out = [NSMutableDictionary dictionary];
    out[@"tool"] = @"vsp_probe";
    out[@"profile_path"] = [NSString stringWithUTF8String:profilePath];
    out[@"schema"] = profile[@"schema"] ?: @"";
    out[@"vehicle"] = profile[@"vehicle"] ?: @{};
    out[@"mode"] = mode_name(opts->mode);
    out[@"target_hz"] = @(opts->target_hz);
    out[@"duration_s"] = @(opts->duration_s);
    out[@"plan_only"] = @(planOnly);
    if (opts->device) out[@"device"] = [NSString stringWithUTF8String:opts->device];
    if (actualDuration > 0.0) out[@"actual_duration_s"] = @(((double)llround(actualDuration * 1000.0)) / 1000.0);
    if (phases) out[@"phases"] = phases;

    NSMutableArray *signalOut = [NSMutableArray array];
    int pass = 0;
    int under = 0;
    int none = 0;
    for (VSPProbeItem *item in items) {
        NSDictionary *d = planOnly ? plan_for_item(item) :
                            result_for_item(item, opts->target_hz, actualDuration > 0.0 ? actualDuration : opts->duration_s);
        [signalOut addObject:d];
        if (!planOnly) {
            NSString *status = d[@"status"];
            if ([status isEqualToString:@"pass"]) pass++;
            else if ([status isEqualToString:@"under_target"]) under++;
            else none++;
        }
    }
    out[@"signals"] = signalOut;
    out[@"selected_count"] = @(signalOut.count);
    if (!planOnly) {
        out[@"summary"] = @{@"pass": @(pass), @"under_target": @(under), @"no_samples": @(none)};
    }
    return out;
}

static bool write_json(id obj, const char *path) {
    NSError *err = nil;
    NSData *data = [NSJSONSerialization dataWithJSONObject:obj
                                                   options:NSJSONWritingPrettyPrinted
                                                     error:&err];
    if (!data) {
        fprintf(stderr, "[ERROR] Failed to encode JSON: %s\n",
                err.localizedDescription.UTF8String ?: "unknown");
        return false;
    }
    if (path && *path) {
        NSString *p = [NSString stringWithUTF8String:path];
        if (![data writeToFile:p atomically:YES]) {
            fprintf(stderr, "[ERROR] Failed to write output: %s\n", path);
            return false;
        }
    } else {
        fwrite(data.bytes, 1, data.length, stdout);
        fputc('\n', stdout);
    }
    return true;
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        vsp_probe_options_t opts;
        if (!parse_options(argc, argv, &opts)) {
            usage();
            return 1;
        }

        NSDictionary *profile = load_profile(opts.profile_path);
        if (!profile) return 1;

        NSArray<VSPProbeItem *> *items = select_items(profile, &opts);
        if (items.count == 0) {
            fprintf(stderr, "[ERROR] No VSP signals selected for requested mode/filter\n");
            return 1;
        }

        if (opts.plan_only) {
            NSDictionary *out = build_output(profile, opts.profile_path, &opts, items, nil, true, 0.0);
            return write_json(out, opts.out_path) ? 0 : 1;
        }

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        OBDAdapter *adapter = [[OBDAdapter alloc] initWithNameHints:@[@"OBDLink", @"vLinker", @"ELM327", @"OBD", @"iOBD"]
                                                              quiet:NO];
        NSString *device = [NSString stringWithUTF8String:opts.device];
        if (![adapter scanAndConnectMatching:device scanSeconds:8.0]) {
            [adapter release];
            return 1;
        }

        NSMutableArray<VSPProbeItem *> *canItems = [NSMutableArray array];
        NSMutableArray<VSPProbeItem *> *obdItems = [NSMutableArray array];
        for (VSPProbeItem *item in items) {
            if ([item.acquisitionType isEqualToString:@"can_frame"]) [canItems addObject:item];
            else if ([item.acquisitionType isEqualToString:@"diagnostic_query"]) [obdItems addObject:item];
        }

        NSMutableArray<NSDictionary *> *phases = [NSMutableArray array];
        int64_t startMs = now_ms();
        bool ok = true;

        if (canItems.count > 0 && (opts.mode == VSPProbeModeAuto || opts.mode == VSPProbeModeCAN)) {
            NSMutableDictionary *phase = [NSMutableDictionary dictionary];
            ok = run_can_phase(adapter, canItems, opts.duration_s, phase);
            [phases addObject:phase];
        }
        if (ok && !g_interrupted &&
            obdItems.count > 0 && (opts.mode == VSPProbeModeAuto || opts.mode == VSPProbeModeOBD)) {
            NSMutableDictionary *phase = [NSMutableDictionary dictionary];
            ok = run_obd_phase(adapter, profile, obdItems, &opts, phase);
            [phases addObject:phase];
        }

        int64_t endMs = now_ms();
        [adapter disconnect];

        double actualDuration = (double)(endMs - startMs) / 1000.0;
        NSDictionary *out = build_output(profile, opts.profile_path, &opts,
                                         items, phases, false, actualDuration);
        if (!write_json(out, opts.out_path)) return 1;
        return ok && !g_interrupted ? 0 : 1;
    }
}
