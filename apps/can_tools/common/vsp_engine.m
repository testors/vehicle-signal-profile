/**
 * @file vsp_engine.m
 * @brief VSP profile loading, signal selection and CAN frame decoding
 *
 * Moved from vsp_probe/main.m. Selection runs on the NSJSONSerialization
 * tree (same helpers as before); the per-frame rules (match, mux, marker,
 * decode) are precomputed into C structs so feeding frames needs no
 * Objective-C. Compiled without ARC like the rest of the project.
 */

#import <Foundation/Foundation.h>

#include "vsp_engine.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * String pool (C strings owned by a profile / decoder)
 ******************************************************************************/

typedef struct {
    char **items;
    int    count;
    int    capacity;
} str_pool_t;

static const char *pool_add(str_pool_t *pool, const char *s) {
    if (!s) return NULL;
    if (pool->count >= pool->capacity) {
        int cap = pool->capacity > 0 ? pool->capacity * 2 : 64;
        char **grown = (char **)realloc(pool->items, (size_t)cap * sizeof(*grown));
        if (!grown) return NULL;
        pool->items = grown;
        pool->capacity = cap;
    }
    char *copy = strdup(s);
    if (!copy) return NULL;
    pool->items[pool->count++] = copy;
    return copy;
}

/** NSString -> pooled UTF-8 copy; NULL for nil / non-strings. */
static const char *pool_ns(str_pool_t *pool, id obj) {
    if (![obj isKindOfClass:[NSString class]]) return NULL;
    return pool_add(pool, [(NSString *)obj UTF8String]);
}

/** Like pool_ns but "" instead of NULL. */
static const char *pool_ns_or_empty(str_pool_t *pool, id obj) {
    const char *s = pool_ns(pool, obj);
    return s ? s : "";
}

static void pool_free(str_pool_t *pool) {
    for (int i = 0; i < pool->count; i++) free(pool->items[i]);
    free(pool->items);
    memset(pool, 0, sizeof(*pool));
}

static void set_err(char *err, size_t err_len, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
static void set_err(char *err, size_t err_len, const char *fmt, ...) {
    if (!err || err_len == 0) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, err_len, fmt, ap);
    va_end(ap);
}

/*******************************************************************************
 * JSON / hex helpers (from vsp_probe)
 ******************************************************************************/

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

static NSNumber *num_or_nil(id value) {
    if ([value isKindOfClass:[NSNumber class]]) return value;
    return nil;
}

/*******************************************************************************
 * Profile
 ******************************************************************************/

struct vsp_profile {
    NSDictionary       *json;       /* retained */
    str_pool_t          pool;
    vsp_profile_info_t  info;
    vsp_bus_info_t     *buses;
    vsp_frame_info_t   *frames;
    vsp_query_info_t   *queries;
};

static NSArray *array_or_empty(id value) {
    return [value isKindOfClass:[NSArray class]] ? value : @[];
}

static int count_dicts(NSArray *array) {
    int n = 0;
    for (id entry in array) {
        if ([entry isKindOfClass:[NSDictionary class]]) n++;
    }
    return n;
}

static int int_or_zero(id value) {
    NSNumber *n = num_or_nil(value);
    return n ? n.intValue : 0;
}

static bool profile_index(vsp_profile_t *p, const char *path) {
    NSDictionary *json = p->json;
    str_pool_t *pool = &p->pool;
    vsp_profile_info_t *info = &p->info;

    info->path = pool_add(pool, path);
    info->schema = pool_ns(pool, json[@"schema"]);
    NSDictionary *vehicle = json[@"vehicle"];
    if ([vehicle isKindOfClass:[NSDictionary class]]) {
        info->manufacturer = pool_ns(pool, vehicle[@"manufacturer"]);
        info->model = pool_ns(pool, vehicle[@"model"]);
    }
    NSDictionary *app = json[@"applicability"];
    if ([app isKindOfClass:[NSDictionary class]]) {
        info->generation = pool_ns(pool, app[@"generation"]);
        info->platform = pool_ns(pool, app[@"platform"]);
        info->market = pool_ns(pool, app[@"market"]);
        info->trim = pool_ns(pool, app[@"trim"]);
        info->powertrain = pool_ns(pool, app[@"powertrain"]);
        info->year_start = int_or_zero(app[@"year_start"]);
        info->year_end = int_or_zero(app[@"year_end"]);
    }

    NSArray *buses = array_or_empty(json[@"buses"]);
    NSArray *frames = array_or_empty(json[@"can_frames"]);
    NSArray *signals = array_or_empty(json[@"signals"]);
    NSArray *queries = array_or_empty(json[@"diagnostic_queries"]);
    info->bus_count = count_dicts(buses);
    info->frame_count = count_dicts(frames);
    info->signal_count = count_dicts(signals);
    info->query_count = count_dicts(queries);

    p->buses = (vsp_bus_info_t *)calloc((size_t)info->bus_count + 1, sizeof(*p->buses));
    p->frames = (vsp_frame_info_t *)calloc((size_t)info->frame_count + 1, sizeof(*p->frames));
    p->queries = (vsp_query_info_t *)calloc((size_t)info->query_count + 1, sizeof(*p->queries));
    if (!p->buses || !p->frames || !p->queries) return false;

    int n = 0;
    for (NSDictionary *bus in buses) {
        if (![bus isKindOfClass:[NSDictionary class]]) continue;
        vsp_bus_info_t *b = &p->buses[n++];
        b->id = pool_ns(pool, bus[@"id"]);
        b->type = pool_ns(pool, bus[@"type"]);
        b->bitrate = int_or_zero(bus[@"bitrate"]);
    }
    n = 0;
    for (NSDictionary *frame in frames) {
        if (![frame isKindOfClass:[NSDictionary class]]) continue;
        vsp_frame_info_t *f = &p->frames[n++];
        f->frame_id = [num_or_nil(frame[@"frame_id"]) unsignedIntValue];
        f->frame_id_mask = [num_or_nil(frame[@"frame_id_mask"]) unsignedIntValue];
        f->name = pool_ns(pool, frame[@"name"]);
        f->bus = pool_ns(pool, frame[@"bus"]);
        NSNumber *dlc = num_or_nil(frame[@"dlc"]);
        f->dlc = dlc ? dlc.intValue : -1;
        double cycle = [num_or_nil(frame[@"cycle_ms"]) doubleValue];
        f->cycle_ms = cycle > 0.0 ? cycle : 0.0;
        f->extended = [frame[@"id_format"] isEqual:@"extended"];
        f->reassembled = [frame[@"reassembled"] isKindOfClass:[NSNumber class]] &&
                         [frame[@"reassembled"] boolValue];
    }
    n = 0;
    for (NSDictionary *q in queries) {
        if (![q isKindOfClass:[NSDictionary class]]) continue;
        vsp_query_info_t *qi = &p->queries[n++];
        qi->id = pool_ns(pool, q[@"id"]);
        qi->kind = pool_ns(pool, q[@"kind"]);
        qi->role = pool_ns(pool, q[@"role"]);
        NSNumber *tx = num_or_nil(q[@"tx_id"]);
        qi->has_tx_id = tx != nil;
        qi->tx_id = tx.unsignedIntValue;
        qi->rx_id = [num_or_nil(q[@"rx_id"]) unsignedIntValue];
        qi->request_hex = pool_ns(pool, q[@"request_hex"]);
        qi->payload_hex = pool_ns(pool, q[@"payload_hex"]);
    }
    return true;
}

static vsp_profile_t *profile_from_data(NSData *data, const char *path,
                                        char *err, size_t err_len) {
    NSError *jsonErr = nil;
    id obj = [NSJSONSerialization JSONObjectWithData:data options:0 error:&jsonErr];
    if (!obj || ![obj isKindOfClass:[NSDictionary class]]) {
        set_err(err, err_len, "Invalid VSP JSON: %s",
                jsonErr.localizedDescription.UTF8String ?: "unknown");
        return NULL;
    }

    vsp_profile_t *p = (vsp_profile_t *)calloc(1, sizeof(*p));
    if (!p) {
        set_err(err, err_len, "Out of memory");
        return NULL;
    }
    p->json = [obj retain];
    if (!profile_index(p, path)) {
        vsp_profile_free(p);
        set_err(err, err_len, "Out of memory");
        return NULL;
    }
    return p;
}

vsp_profile_t *vsp_profile_load(const char *path, char *err, size_t err_len) {
    @autoreleasepool {
        NSString *ns = path ? [NSString stringWithUTF8String:path] : nil;
        NSData *data = ns ? [NSData dataWithContentsOfFile:ns] : nil;
        if (!data) {
            set_err(err, err_len, "Failed to read profile: %s", path ? path : "(null)");
            return NULL;
        }
        return profile_from_data(data, path, err, err_len);
    }
}

vsp_profile_t *vsp_profile_load_json(const char *json, size_t len, char *err, size_t err_len) {
    @autoreleasepool {
        NSData *data = json ? [NSData dataWithBytes:json length:len] : nil;
        if (!data) {
            set_err(err, err_len, "Invalid VSP JSON: no data");
            return NULL;
        }
        return profile_from_data(data, NULL, err, err_len);
    }
}

void vsp_profile_free(vsp_profile_t *p) {
    if (!p) return;
    [p->json release];
    free(p->buses);
    free(p->frames);
    free(p->queries);
    pool_free(&p->pool);
    free(p);
}

void vsp_profile_get_info(const vsp_profile_t *p, vsp_profile_info_t *out) {
    if (!out) return;
    if (!p) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = p->info;
}

bool vsp_profile_bus(const vsp_profile_t *p, int idx, vsp_bus_info_t *out) {
    if (!p || !out || idx < 0 || idx >= p->info.bus_count) return false;
    *out = p->buses[idx];
    return true;
}

bool vsp_profile_frame(const vsp_profile_t *p, int idx, vsp_frame_info_t *out) {
    if (!p || !out || idx < 0 || idx >= p->info.frame_count) return false;
    *out = p->frames[idx];
    return true;
}

bool vsp_profile_query(const vsp_profile_t *p, int idx, vsp_query_info_t *out) {
    if (!p || !out || idx < 0 || idx >= p->info.query_count) return false;
    *out = p->queries[idx];
    return true;
}

NSDictionary *vsp_profile_json(const vsp_profile_t *p) {
    return p ? p->json : nil;
}

/*******************************************************************************
 * Signal selection (from vsp_probe)
 ******************************************************************************/

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

static bool acquisition_allowed(NSString *type, vsp_acq_filter_t filter) {
    if (filter == VSP_ACQ_CAN) return [type isEqualToString:@"can_frame"];
    if (filter == VSP_ACQ_DIAGNOSTIC) return [type isEqualToString:@"diagnostic_query"];
    return [type isEqualToString:@"can_frame"] || [type isEqualToString:@"diagnostic_query"];
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

/* A reassembled frame is a vendor stream buffer, not one CAN frame (VSP_RUNTIME_SPEC.md,
 * "Reassembled Frames"); its signals cannot be read from single frames. */
static bool frame_is_reassembled(NSDictionary *profile, uint32_t frameId) {
    NSArray *frames = profile[@"can_frames"];
    if (![frames isKindOfClass:[NSArray class]]) return false;
    for (NSDictionary *frame in frames) {
        if (![frame isKindOfClass:[NSDictionary class]]) continue;
        if ([num_or_nil(frame[@"frame_id"]) unsignedIntValue] != frameId) continue;
        return [frame[@"reassembled"] isKindOfClass:[NSNumber class]] && [frame[@"reassembled"] boolValue];
    }
    return false;
}

static double frame_cycle_ms_for(NSDictionary *profile, uint32_t frameId) {
    NSArray *frames = profile[@"can_frames"];
    if (![frames isKindOfClass:[NSArray class]]) return 0.0;
    for (NSDictionary *frame in frames) {
        if (![frame isKindOfClass:[NSDictionary class]]) continue;
        if ([num_or_nil(frame[@"frame_id"]) unsignedIntValue] != frameId) continue;
        double cycle = [num_or_nil(frame[@"cycle_ms"]) doubleValue];
        return cycle > 0.0 ? cycle : 0.0;
    }
    return 0.0;
}

/** A selected signal while the selection is built and sorted. */
@interface VSPEngineItem : NSObject
@property (nonatomic, retain) NSDictionary *signal;
@property (nonatomic, retain) NSDictionary *query;
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
@property (nonatomic, assign) double frameCycleMs;       /* can_frames[].cycle_ms, 0 = not known */
@property (nonatomic, assign) uint32_t txId;
@property (nonatomic, assign) uint32_t rxId;
@end

@implementation VSPEngineItem
- (void)dealloc {
    [_signal release];
    [_query release];
    [_name release];
    [_canonicalId release];
    [_group release];
    [_acquisitionType release];
    [_queryKey release];
    [_requestHex release];
    [_requestBareHex release];
    [_positiveMarkerHex release];
    [super dealloc];
}
@end

/* skipped receives @[name, frame_id] for signals on reassembled frames. */
static NSArray<VSPEngineItem *> *select_items(NSDictionary *profile,
                                              const vsp_select_t *select,
                                              NSMutableArray *skipped) {
    NSMutableSet<NSString *> *canonicalTargets = [NSMutableSet set];
    NSMutableSet<NSString *> *nameTargets = [NSMutableSet set];
    if (!select->all_signals) build_targets(select->signals, canonicalTargets, nameTargets);

    NSDictionary *queries = query_map(profile);
    NSMutableArray<VSPEngineItem *> *items = [NSMutableArray array];
    NSArray *signals = profile[@"signals"];
    if (![signals isKindOfClass:[NSArray class]]) return items;

    for (NSDictionary *signal in signals) {
        if (![signal isKindOfClass:[NSDictionary class]]) continue;
        NSDictionary *acq = signal[@"acquisition"];
        if (![acq isKindOfClass:[NSDictionary class]]) continue;
        NSString *type = acq[@"type"];
        if (![type isKindOfClass:[NSString class]] || !acquisition_allowed(type, select->acquisition)) {
            continue;
        }

        NSString *name = signal[@"name"];
        NSString *canonical = canonical_id_for_signal(signal);
        NSString *nameLower = lower_string(name);
        NSString *canonicalLower = lower_string(canonical);
        bool selected = select->all_signals ||
                        (canonicalLower.length > 0 && [canonicalTargets containsObject:canonicalLower]) ||
                        (nameLower.length > 0 && [nameTargets containsObject:nameLower]);
        if (!selected) continue;

        VSPEngineItem *item = [[[VSPEngineItem alloc] init] autorelease];
        item.signal = signal;
        item.name = name ?: @"";
        item.canonicalId = canonical ?: @"";
        item.group = group_for_canonical(canonical);
        item.acquisitionType = type;

        if ([type isEqualToString:@"can_frame"]) {
            NSNumber *frameId = num_or_nil(acq[@"frame_id"]);
            if (!frameId) continue;
            item.frameId = frameId.unsignedIntValue;
            if (frame_is_reassembled(profile, item.frameId)) {
                [skipped addObject:@[ name ?: @"", @(item.frameId) ]];
                continue;
            }
            item.frameIdMask = frame_id_mask_for(profile, item.frameId);
            item.frameCycleMs = frame_cycle_ms_for(profile, item.frameId);
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

    [items sortUsingComparator:^NSComparisonResult(VSPEngineItem *a, VSPEngineItem *b) {
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

/*******************************************************************************
 * Frame matching and decoding (C)
 ******************************************************************************/

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

typedef struct {
    long long   raw;
    const char *label;          /* NULL: entry has no string label */
    int         kind;           /* 0 = no override, 1 = null, 2 = number */
    double      value;
} value_entry_t;

/** decode block precomputed (decode_signal_value in vsp_probe). */
typedef struct {
    bool           valid;       /* false: decode never yields a value */
    bool           is_float;
    int            start_bit;
    int            bit_length;
    bool           big_endian;
    double         factor;
    double         offset;
    bool           is_signed;
    bool           has_sign_bit;
    int            sign_bit;
    int            poly_count;
    double        *poly;
    int            value_count;
    value_entry_t *values;
} decode_spec_t;

/** Positional decode.mux (signal_mux in vsp_probe). */
typedef struct {
    bool     valid;
    uint64_t value;
    int      start_bit;
    int      bit_length;
    bool     big_endian;
} mux_spec_t;

typedef struct {
    vsp_signal_info_t  info;
    vsp_signal_state_t state;
    decode_spec_t      decode;
    mux_spec_t         mux;
    bool               has_marker;
    uint8_t            marker[16];
    int                marker_len;
} engine_signal_t;

struct vsp_decoder {
    engine_signal_t      *signals;
    int                   count;
    vsp_skipped_signal_t *skipped;
    int                   skipped_count;
    str_pool_t            pool;
    NSMutableArray       *signal_json;  /* retained; signals[] entries */
    NSMutableArray       *query_json;   /* retained; query dict or NSNull */
};

static bool build_decode_spec(NSDictionary *signal, str_pool_t *pool, decode_spec_t *d) {
    memset(d, 0, sizeof(*d));
    NSDictionary *decode = signal[@"decode"];
    if (![decode isKindOfClass:[NSDictionary class]]) return true;

    NSNumber *start = num_or_nil(decode[@"start_bit"]);
    NSNumber *bits = num_or_nil(decode[@"bit_length"]);
    NSString *valueType = [decode[@"value_type"] isKindOfClass:[NSString class]] ? decode[@"value_type"] : @"integer";
    bool isFloat = [valueType isEqualToString:@"float32"] || [valueType isEqualToString:@"float64"];
    if (!isFloat && ![valueType isEqualToString:@"integer"]) return true;   /* unknown type: skip */
    int maxBits = isFloat ? 64 : 63;
    if (!start || !bits || bits.intValue <= 0 || bits.intValue > maxBits) return true;

    id order = decode[@"byte_order"] ?: @"little_endian";
    d->big_endian = [order isKindOfClass:[NSString class]] && [order isEqualToString:@"big_endian"];
    d->start_bit = start.intValue;
    d->bit_length = bits.intValue;

    d->factor = [decode[@"factor"] respondsToSelector:@selector(doubleValue)] ?
                [decode[@"factor"] doubleValue] : 1.0;
    d->offset = [decode[@"offset"] respondsToSelector:@selector(doubleValue)] ?
                [decode[@"offset"] doubleValue] : 0.0;

    /* IEEE 754 field (VSP decode.value_type): the bits are the float itself. */
    if (isFloat) {
        if (d->bit_length != 32 && d->bit_length != 64) return true;
        d->is_float = true;
        d->valid = true;
        return true;
    }

    d->is_signed = [decode[@"signed"] respondsToSelector:@selector(boolValue)] ?
                   [decode[@"signed"] boolValue] : NO;

    /* Sign-magnitude: raw is the magnitude, sign_bit=1 negates it. */
    NSNumber *signBit = num_or_nil(decode[@"sign_bit"]);
    if (signBit) {
        d->has_sign_bit = true;
        d->sign_bit = signBit.intValue;
    }

    /* Non-linear conversion (VSP decode.polynomial): c0 + c1 * raw + c2 * raw^2 ... */
    NSArray *poly = decode[@"polynomial"];
    if ([poly isKindOfClass:[NSArray class]] && poly.count > 0) {
        d->poly = (double *)calloc(poly.count, sizeof(double));
        if (!d->poly) return false;
        for (NSUInteger i = 0; i < poly.count; i++) {
            NSNumber *c = num_or_nil(poly[i]);
            if (!c) return true;   /* a non-number coefficient: never decodes */
            d->poly[i] = c.doubleValue;
        }
        d->poly_count = (int)poly.count;
    }

    /* Value table: label for the raw code, optional numeric override (null = none). */
    NSArray *values = decode[@"values"];
    if ([values isKindOfClass:[NSArray class]] && values.count > 0) {
        d->values = (value_entry_t *)calloc(values.count, sizeof(value_entry_t));
        if (!d->values) return false;
        for (NSDictionary *entry in values) {
            if (![entry isKindOfClass:[NSDictionary class]]) continue;
            NSNumber *entryRaw = num_or_nil(entry[@"raw"]);
            if (!entryRaw) continue;
            value_entry_t *e = &d->values[d->value_count++];
            e->raw = entryRaw.longLongValue;
            e->label = pool_ns(pool, entry[@"label"]);
            id override = entry[@"value"];
            if (override == [NSNull null]) {
                e->kind = 1;
            } else if ([override isKindOfClass:[NSNumber class]]) {
                e->kind = 2;
                e->value = [override doubleValue];
            }
        }
    }

    d->valid = true;
    return true;
}

static void build_mux_spec(NSDictionary *signal, mux_spec_t *m) {
    memset(m, 0, sizeof(*m));
    NSDictionary *decode = signal[@"decode"];
    if (![decode isKindOfClass:[NSDictionary class]]) return;
    NSDictionary *mux = decode[@"mux"];
    if (![mux isKindOfClass:[NSDictionary class]]) return;
    NSNumber *value = num_or_nil(mux[@"value"]);
    NSNumber *start = num_or_nil(mux[@"start_bit"]);
    NSNumber *bits = num_or_nil(mux[@"bit_length"]);
    if (!value || !start || !bits || bits.intValue <= 0 || bits.intValue > 63) return;
    m->valid = true;
    m->value = value.unsignedLongLongValue;
    m->start_bit = start.intValue;
    m->bit_length = bits.intValue;
    m->big_endian = [mux[@"byte_order"] isEqual:@"big_endian"];
}

static void free_decode_spec(decode_spec_t *d) {
    free(d->poly);
    free(d->values);
    memset(d, 0, sizeof(*d));
}

static bool decode_value(const decode_spec_t *d,
                         const uint8_t *data,
                         int dataLen,
                         double *valueOut,
                         bool *hasNumberOut,
                         const char **labelOut) {
    if (!d->valid) return false;

    uint64_t raw = d->big_endian
        ? extract_bits_big_dbc(data, dataLen, d->start_bit, d->bit_length)
        : extract_bits_little(data, dataLen, d->start_bit, d->bit_length);

    if (d->is_float) {
        double fv;
        if (d->bit_length == 32) {
            uint32_t b32 = (uint32_t)raw;
            float f32;
            memcpy(&f32, &b32, sizeof(f32));
            fv = f32;
        } else {
            memcpy(&fv, &raw, sizeof(fv));
        }
        *valueOut = fv * d->factor + d->offset;
        *hasNumberOut = true;
        *labelOut = NULL;
        return true;
    }

    double signedRaw = (double)raw;
    if (d->is_signed && d->bit_length > 0) {
        uint64_t signBit = 1ULL << (d->bit_length - 1);
        if (raw & signBit) {
            uint64_t mask = (d->bit_length == 64) ? UINT64_MAX : ((1ULL << d->bit_length) - 1);
            int64_t signedVal = -((int64_t)((~raw + 1) & mask));
            signedRaw = (double)signedVal;
        }
    }

    if (d->has_sign_bit && !d->is_signed) {
        int sb = d->sign_bit;
        if (sb >= 0 && sb / 8 < dataLen && ((data[sb / 8] >> (sb % 8)) & 1)) {
            signedRaw = -signedRaw;
        }
    }

    double value = signedRaw * d->factor + d->offset;
    if (d->poly_count > 0) {
        value = 0.0;
        for (int i = d->poly_count - 1; i >= 0; i--) {
            value = value * signedRaw + d->poly[i];   /* Horner */
        }
    }
    bool hasNumber = true;
    const char *label = NULL;

    if (d->value_count > 0) {
        long long rawCode = llround(signedRaw);
        for (int i = 0; i < d->value_count; i++) {
            const value_entry_t *e = &d->values[i];
            if (e->raw != rawCode) continue;
            if (e->label) label = e->label;
            if (e->kind == 1) hasNumber = false;
            else if (e->kind == 2) value = e->value;
            break;
        }
    }

    *valueOut = value;
    *hasNumberOut = hasNumber;
    *labelOut = label;
    return true;
}

static bool mux_matches(const mux_spec_t *m, const uint8_t *data, int len) {
    if (!m->valid) return true;
    uint64_t raw = m->big_endian ? extract_bits_big_dbc(data, len, m->start_bit, m->bit_length)
                                 : extract_bits_little(data, len, m->start_bit, m->bit_length);
    return raw == m->value;
}

static bool payload_has_marker(const engine_signal_t *sig, const uint8_t *payload, int payloadLen) {
    if (!sig->has_marker) return true;
    if (payloadLen < sig->marker_len) return false;
    return memcmp(payload, sig->marker, (size_t)sig->marker_len) == 0;
}

static void mark_sample(engine_signal_t *sig,
                        double timestampMs,
                        const uint8_t *payload,
                        int payloadLen,
                        const uint8_t *decodeBytes,
                        int decodeLen) {
    vsp_signal_state_t *st = &sig->state;
    st->samples += 1;
    if (st->first_sample_ms <= 0.0) st->first_sample_ms = timestampMs;
    st->last_sample_ms = timestampMs;
    if (payload && payloadLen > 0) {
        int n = payloadLen < (int)sizeof(st->last_payload) ? payloadLen : (int)sizeof(st->last_payload);
        memcpy(st->last_payload, payload, (size_t)n);
        st->last_payload_len = n;
    }

    double value = 0.0;
    bool hasNumber = true;
    const char *label = NULL;
    if (decodeBytes && decodeLen > 0 &&
        decode_value(&sig->decode, decodeBytes, decodeLen, &value, &hasNumber, &label)) {
        st->value = value;
        st->has_value = hasNumber;
        st->value_null = !hasNumber;
        st->label = label;
    }
}

static void note_updated(int idx, int *updated, int max_updated, int n) {
    if (updated && n < max_updated) updated[n] = idx;
}

static int feed_can(vsp_decoder_t *dec, const can_frame_t *frame, double ts,
                    int *updated, int max_updated) {
    if (!dec || !frame) return 0;
    int n = 0;
    for (int i = 0; i < dec->count; i++) {
        engine_signal_t *sig = &dec->signals[i];
        if (sig->info.kind != VSP_SIGNAL_CAN_FRAME) continue;
        uint32_t mask = sig->info.frame_id_mask ? sig->info.frame_id_mask : UINT32_MAX;
        if ((sig->info.frame_id & mask) != (frame->can_id & mask)) continue;
        if (!mux_matches(&sig->mux, frame->data, frame->dlc)) continue;
        mark_sample(sig, ts, frame->data, frame->dlc, frame->data, frame->dlc);
        note_updated(i, updated, max_updated, n);
        n++;
    }
    return n;
}

int vsp_decoder_feed_can(vsp_decoder_t *dec, const can_frame_t *frame, double ts) {
    return feed_can(dec, frame, ts, NULL, 0);
}

/* Diagnostic layouts are per CAN frame, PCI byte(s) included (VSP_RUNTIME_SPEC.md,
 * "Diagnostic Response Frames"). A signal with a mux is matched by it, which also
 * selects consecutive frames; one without falls back to the positive-response marker
 * at the start of a single or first frame. */
static int feed_diagnostic(vsp_decoder_t *dec, const can_frame_t *frame, double ts,
                           const char *queryKey, int *updated, int max_updated) {
    if (!dec || !frame) return 0;
    uint8_t payload[64];
    int payloadLen = 0;
    bool initial = frame_initial_isotp_payload(frame, payload, sizeof(payload), &payloadLen);

    int n = 0;
    for (int i = 0; i < dec->count; i++) {
        engine_signal_t *sig = &dec->signals[i];
        if (sig->info.kind != VSP_SIGNAL_DIAGNOSTIC_QUERY) continue;
        if (queryKey && strcmp(sig->info.query_key, queryKey) != 0) continue;
        if (sig->info.rx_id != 0 && sig->info.rx_id != frame->can_id) continue;
        if (sig->mux.valid) {
            if (!mux_matches(&sig->mux, frame->data, frame->dlc)) continue;
        } else if (!initial || !payload_has_marker(sig, payload, payloadLen)) {
            continue;
        }
        mark_sample(sig, ts, frame->data, frame->dlc, frame->data, frame->dlc);
        note_updated(i, updated, max_updated, n);
        n++;
    }
    return n;
}

int vsp_decoder_feed_diagnostic(vsp_decoder_t *dec, const can_frame_t *frame,
                                double ts, const char *queryKey) {
    return feed_diagnostic(dec, frame, ts, queryKey, NULL, 0);
}

int vsp_decoder_feed_diagnostic_collect(vsp_decoder_t *dec, const can_frame_t *frame, double ts,
                                        const char *queryKey, int *updated, int max_updated) {
    return feed_diagnostic(dec, frame, ts, queryKey, updated, max_updated);
}

int vsp_decoder_feed(vsp_decoder_t *dec, const can_frame_t *frame, double ts) {
    return vsp_decoder_feed_can(dec, frame, ts) +
           vsp_decoder_feed_diagnostic(dec, frame, ts, NULL);
}

int vsp_decoder_feed_collect(vsp_decoder_t *dec, const can_frame_t *frame, double ts,
                             int *updated, int max_updated) {
    int n = feed_can(dec, frame, ts, updated, max_updated);
    int filled = n < max_updated ? n : max_updated;
    n += feed_diagnostic(dec, frame, ts, NULL, updated ? updated + filled : NULL,
                         max_updated - filled);
    return n;
}

/*******************************************************************************
 * Decoder
 ******************************************************************************/

static bool fill_signal(vsp_decoder_t *dec, engine_signal_t *sig, VSPEngineItem *item) {
    str_pool_t *pool = &dec->pool;
    vsp_signal_info_t *info = &sig->info;

    info->name = pool_ns_or_empty(pool, item.name);
    info->canonical_id = pool_ns_or_empty(pool, item.canonicalId);
    info->group = pool_ns_or_empty(pool, item.group);
    info->acquisition_type = pool_ns_or_empty(pool, item.acquisitionType);
    info->kind = [item.acquisitionType isEqualToString:@"can_frame"]
                 ? VSP_SIGNAL_CAN_FRAME : VSP_SIGNAL_DIAGNOSTIC_QUERY;
    NSDictionary *decode = item.signal[@"decode"];
    if ([decode isKindOfClass:[NSDictionary class]]) info->unit = pool_ns(pool, decode[@"unit"]);

    info->frame_id = item.frameId;
    info->frame_id_mask = item.frameIdMask;
    info->cycle_ms = item.frameCycleMs;
    info->tx_id = item.txId;
    info->rx_id = item.rxId;
    if (item.query) {
        info->query_id = pool_ns(pool, item.query[@"id"]);
        info->query_payload_hex = pool_ns(pool, item.query[@"payload_hex"]);
    }
    info->request_hex = pool_ns_or_empty(pool, item.requestHex);
    info->request_bare_hex = pool_ns_or_empty(pool, item.requestBareHex);
    info->positive_marker_hex = pool_ns(pool, item.positiveMarkerHex);
    info->query_key = pool_ns_or_empty(pool, item.queryKey);

    if (item.positiveMarkerHex.length > 0) {
        int len = 0;
        if (parse_hex_bytes(item.positiveMarkerHex, sig->marker, sizeof(sig->marker), &len)) {
            sig->has_marker = true;
            sig->marker_len = len;
        }
    }

    build_mux_spec(item.signal, &sig->mux);
    info->has_mux = sig->mux.valid;
    bool ok = build_decode_spec(item.signal, pool, &sig->decode);
    info->factor = 1.0;
    if ([decode isKindOfClass:[NSDictionary class]] &&
        [decode[@"factor"] respondsToSelector:@selector(doubleValue)]) {
        info->factor = [decode[@"factor"] doubleValue];
    }
    /* One raw step is |factor| only for raw * factor + offset of an integer field;
       under 6 bits (< 64 steps: flags, states, nibbles) every step matters: none. */
    const decode_spec_t *d = &sig->decode;
    bool linear = d->valid && !d->is_float && d->poly_count == 0 && d->bit_length >= 6;
    info->lsb = linear ? fabs(d->factor) : 0.0;
    for (int i = 0; i < d->value_count; i++) {
        if (d->values[i].kind == 2) info->lsb = 0.0;   /* numeric override */
    }
    info->has_value_table = sig->decode.value_count > 0;
    return ok;
}

vsp_decoder_t *vsp_decoder_create(const vsp_profile_t *profile, const vsp_select_t *select) {
    if (!profile) return NULL;
    vsp_select_t defaults = { .signals = NULL, .all_signals = false, .acquisition = VSP_ACQ_ANY };
    if (!select) select = &defaults;

    @autoreleasepool {
        vsp_decoder_t *dec = (vsp_decoder_t *)calloc(1, sizeof(*dec));
        if (!dec) return NULL;

        NSMutableArray *skipped = [NSMutableArray array];
        NSArray<VSPEngineItem *> *items = select_items(profile->json, select, skipped);

        dec->signal_json = [[NSMutableArray alloc] initWithCapacity:items.count];
        dec->query_json = [[NSMutableArray alloc] initWithCapacity:items.count];
        dec->signals = (engine_signal_t *)calloc(items.count + 1, sizeof(*dec->signals));
        dec->skipped = (vsp_skipped_signal_t *)calloc(skipped.count + 1, sizeof(*dec->skipped));
        if (!dec->signals || !dec->skipped) {
            vsp_decoder_free(dec);
            return NULL;
        }

        for (VSPEngineItem *item in items) {
            engine_signal_t *sig = &dec->signals[dec->count++];
            if (!fill_signal(dec, sig, item)) {
                vsp_decoder_free(dec);
                return NULL;
            }
            [dec->signal_json addObject:item.signal];
            [dec->query_json addObject:item.query ?: (id)[NSNull null]];
        }
        for (NSArray *entry in skipped) {
            vsp_skipped_signal_t *sk = &dec->skipped[dec->skipped_count++];
            sk->name = pool_ns_or_empty(&dec->pool, entry[0]);
            sk->frame_id = [entry[1] unsignedIntValue];
        }
        return dec;
    }
}

void vsp_decoder_free(vsp_decoder_t *dec) {
    if (!dec) return;
    for (int i = 0; i < dec->count; i++) free_decode_spec(&dec->signals[i].decode);
    free(dec->signals);
    free(dec->skipped);
    [dec->signal_json release];
    [dec->query_json release];
    pool_free(&dec->pool);
    free(dec);
}

int vsp_decoder_signal_count(const vsp_decoder_t *dec) {
    return dec ? dec->count : 0;
}

const vsp_signal_info_t *vsp_decoder_signal_info(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->count) return NULL;
    return &dec->signals[idx].info;
}

int vsp_decoder_value_count(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->count) return 0;
    return dec->signals[idx].decode.value_count;
}

bool vsp_decoder_value_entry(const vsp_decoder_t *dec, int idx, int i, long long *raw,
                             const char **label, bool *has_number, double *value) {
    if (!dec || idx < 0 || idx >= dec->count) return false;
    const decode_spec_t *d = &dec->signals[idx].decode;
    if (i < 0 || i >= d->value_count) return false;
    const value_entry_t *e = &d->values[i];
    if (raw) *raw = e->raw;
    if (label) *label = e->label;
    if (has_number) *has_number = e->kind != 1;
    if (value) *value = e->kind == 2 ? e->value : (double)e->raw * d->factor + d->offset;
    return true;
}

const vsp_signal_state_t *vsp_decoder_signal_state(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->count) return NULL;
    return &dec->signals[idx].state;
}

int vsp_decoder_skipped_count(const vsp_decoder_t *dec) {
    return dec ? dec->skipped_count : 0;
}

const vsp_skipped_signal_t *vsp_decoder_skipped(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->skipped_count) return NULL;
    return &dec->skipped[idx];
}

void vsp_decoder_reset(vsp_decoder_t *dec) {
    if (!dec) return;
    for (int i = 0; i < dec->count; i++) {
        memset(&dec->signals[i].state, 0, sizeof(dec->signals[i].state));
    }
}

NSDictionary *vsp_decoder_signal_json(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->count) return nil;
    return dec->signal_json[(NSUInteger)idx];
}

NSDictionary *vsp_decoder_query_json(const vsp_decoder_t *dec, int idx) {
    if (!dec || idx < 0 || idx >= dec->count) return nil;
    id q = dec->query_json[(NSUInteger)idx];
    return q == [NSNull null] ? nil : q;
}

/*******************************************************************************
 * Hex helpers (C API)
 ******************************************************************************/

void vsp_hex_strip(const char *in, char *out, size_t out_len) {
    if (!out || out_len == 0) return;
    size_t j = 0;
    for (size_t i = 0; in && in[i] && j + 1 < out_len; i++) {
        if (isxdigit((unsigned char)in[i])) out[j++] = (char)toupper((unsigned char)in[i]);
    }
    out[j] = '\0';
}

void vsp_hex_format(const uint8_t *bytes, int len, char *out, size_t out_len) {
    if (!out || out_len == 0) return;
    size_t j = 0;
    out[0] = '\0';
    for (int i = 0; bytes && i < len && j + 2 < out_len; i++) {
        snprintf(out + j, out_len - j, "%02X", bytes[i]);
        j += 2;
    }
}

bool vsp_single_frame_payload_hex(const char *request_hex, uint8_t filler,
                                  char *out, size_t out_len) {
    @autoreleasepool {
        NSString *req = request_hex ? [NSString stringWithUTF8String:request_hex] : nil;
        NSString *payload = single_frame_payload_for_request(req, filler);
        if (!payload) return false;
        if (out && out_len > 0) snprintf(out, out_len, "%s", payload.UTF8String);
        return true;
    }
}
