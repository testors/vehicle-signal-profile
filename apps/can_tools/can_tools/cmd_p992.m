/**
 * @file cmd_p992.m
 * @brief p992: Porsche 992 AiM legacy/debug commands
 *
 * Moved unchanged from can_reader/main.m (only cli_p992 is no longer static).
 * Isolated legacy code: its own route/session/fastlog command sequences.
 */

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <IOBluetooth/IOBluetooth.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <limits.h>
#include <ctype.h>

#include "elm327.h"
#include "stn2255.h"
#include "cli_util.h"
#include "frame_source.h"
#import "obd_adapter.h"
#import "cli_common.h"
#include "commands.h"

/*******************************************************************************
 * Porsche 992 AiM Profile
 ******************************************************************************/

#define AIM992_MAX_CHANNELS      16
#define AIM992_MAX_RESPONSE_HEX  256
#define AIM992_MAX_PAYLOAD_BYTES 64

typedef enum {
    AIM992_QUERY_PID = 0,
    AIM992_QUERY_UDS
} aim992_query_kind_t;

typedef enum {
    AIM992_DECODE_STD = 0,
    AIM992_DECODE_PPS_AIM_PERCENT,
    AIM992_DECODE_MAP_MBAR,
    AIM992_DECODE_STEER_HEURISTIC,
    AIM992_DECODE_BRAKE_HEURISTIC
} aim992_decode_kind_t;

typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
} aim992_route_t;

typedef struct {
    char   long_name[48];
    char   short_name[16];
    char   unit[16];
    char   sens_name[48];
    int    idfis;
    int    fid;
    int    max_freq_ius;
    double lo;
    double hi;
    double conv1;
    bool   have_lo;
    bool   have_hi;
    bool   have_conv1;
} aim992_xml_channel_t;

typedef struct {
    const char            *long_name;
    const char            *short_name;
    aim992_query_kind_t    query_kind;
    aim992_decode_kind_t   decode_kind;
    uint8_t                pid;
    uint16_t               did;
    aim992_route_t         routes[4];
    int                    route_count;
    const char            *request_label;
    const char            *note;
} aim992_channel_spec_t;

typedef struct {
    bool     prepared;
    uint32_t tx_id;
    uint32_t rx_id;
    bool     flow_control_ready;
    uint32_t flow_control_tx_id;
} aim992_query_ctx_t;

typedef struct {
    const aim992_xml_channel_t  *xml;
    const aim992_channel_spec_t *spec;
    bool                         has_value;
    bool                         raw_available;
    bool                         heuristic;
    bool                         used_fallback;
    double                       value;
    uint32_t                     tx_id;
    uint32_t                     rx_id;
    char                         transport[24];
    char                         request[24];
    char                         unit[16];
    char                         raw_hex[AIM992_MAX_RESPONSE_HEX];
    char                         status[24];
    char                         note[160];
} aim992_result_t;

static const aim992_channel_spec_t s_aim992_specs[] = {
    { "RPM",            "RPM",  AIM992_QUERY_PID, AIM992_DECODE_STD,               0x0C, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 0C", "standard OBD-II PID; AiM companion uses functional 0x7DF" },
    { "THROTTLE_POS",   "THRT", AIM992_QUERY_PID, AIM992_DECODE_STD,               0x11, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 11", "standard OBD-II PID; AiM TPS channel, observed range about 12.55..85.10%" },
    { "ACCEL_POS",      "APS",  AIM992_QUERY_PID, AIM992_DECODE_PPS_AIM_PERCENT,   0x49, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 49", "AiM PPS channel; Mode 01 PID 49 raw with live-calibrated AiM scale 0x26->1.09%, 0xE5->104.89%" },
    { "COOLANT_TEMP",   "ECT",  AIM992_QUERY_PID, AIM992_DECODE_STD,               0x05, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 05", "standard OBD-II PID; AiM companion uses functional 0x7DF" },
    { "INTK_AIR_TEMP",  "IAT",  AIM992_QUERY_PID, AIM992_DECODE_STD,               0x0F, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 0F", "standard OBD-II PID; AiM companion uses functional 0x7DF" },
    { "MANIFOLD_PRESS", "MAP",  AIM992_QUERY_PID, AIM992_DECODE_MAP_MBAR,          0x0B, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 0B", "standard PID value converted from kPa to mbar to match XML unit" },
    { "FUEL_LEVEL",     "FLI",  AIM992_QUERY_PID, AIM992_DECODE_STD,               0x2F, 0x0000,
      { { 0x7E0, 0x7E8 }, { 0x7DF, 0x7E8 } }, 2, "01 2F", "standard PID inferred from XC1 mux 0x412F; AiM companion omits explicit 01 2F record" },
    { "STEER_ANGLE",    "STAN", AIM992_QUERY_UDS, AIM992_DECODE_STEER_HEURISTIC,   0x00, 0x5075,
      { { 0x70C, 0x776 }, { 0, 0 } }, 1, "22 50 75", "primary grouped chassis DID from passive AiM sniff; 22 51 01 kept as fallback until bytecode/live evidence disproves it" },
    { "BRAKE_PRESS",    "BRKP", AIM992_QUERY_UDS, AIM992_DECODE_BRAKE_HEURISTIC,   0x00, 0x2B21,
      { { 0x713, 0x77D }, { 0, 0 } }, 1, "22 2B 21", "confirmed by passive AiM sniff and GT3 XC1 companion BIN; scale from GT3 XC1 CANP" },
};

static const int s_aim992_spec_count =
    sizeof(s_aim992_specs) / sizeof(s_aim992_specs[0]);

static int aim992_hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool aim992_parse_hex_byte(const char *hex, uint8_t *out) {
    int hi = aim992_hex_value(hex[0]);
    int lo = aim992_hex_value(hex[1]);
    if (hi < 0 || lo < 0) return false;
    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static void aim992_copy_cstr(char *dst, size_t dst_len, const char *src) {
    if (!dst || dst_len == 0) return;
    snprintf(dst, dst_len, "%s", src ? src : "");
}

static void aim992_copy_nsstring(char *dst, size_t dst_len, NSString *src) {
    aim992_copy_cstr(dst, dst_len, src ? src.UTF8String : "");
}

static NSString *aim992_xpath_string(NSXMLNode *node, NSString *xpath) {
    NSError *err = nil;
    NSArray *matches = [node nodesForXPath:xpath error:&err];
    if (!matches || matches.count == 0) return nil;

    NSXMLNode *match = matches[0];
    NSString *value = match.stringValue;
    if (!value) return nil;
    return [value stringByTrimmingCharactersInSet:
            [NSCharacterSet whitespaceAndNewlineCharacterSet]];
}

static const aim992_channel_spec_t *aim992_find_spec_by_name(const char *name) {
    if (!name || !name[0]) return NULL;

    for (int i = 0; i < s_aim992_spec_count; i++) {
        const aim992_channel_spec_t *spec = &s_aim992_specs[i];
        if (strcasecmp(spec->long_name, name) == 0 ||
            strcasecmp(spec->short_name, name) == 0) {
            return spec;
        }
    }
    return NULL;
}

static int aim992_find_xml_channel_index(const aim992_xml_channel_t channels[],
                                         int channel_count,
                                         const char *name) {
    if (!name || !name[0]) return -1;

    for (int i = 0; i < channel_count; i++) {
        if (strcasecmp(channels[i].long_name, name) == 0 ||
            strcasecmp(channels[i].short_name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static NSString *aim992_resolve_xml_path(void) {
    const char *env_path = getenv("CAN_TOOLS_992_XML");
    NSFileManager *fm = [NSFileManager defaultManager];

    if (env_path && env_path[0]) {
        NSString *candidate = [[NSString stringWithUTF8String:env_path]
                               stringByStandardizingPath];
        BOOL is_dir = NO;
        if ([fm fileExistsAtPath:candidate isDirectory:&is_dir] && !is_dir) {
            return candidate;
        }
    }

    NSMutableArray<NSString *> *bases = [NSMutableArray array];

    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        [bases addObject:[NSString stringWithUTF8String:cwd]];
    }

    NSString *exe_path = [NSBundle mainBundle].executablePath;
    if (exe_path.length > 0) {
        [bases addObject:[exe_path stringByDeletingLastPathComponent]];
    }

    NSArray<NSString *> *suffixes = @[
        @"ecus/PORSCHE_992_OBDII_65_20.xml",
        @"../ecus/PORSCHE_992_OBDII_65_20.xml",
        @"../../ecus/PORSCHE_992_OBDII_65_20.xml",
        @"992/PORSCHE_992_OBDII_65_20.xml",
        @"../992/PORSCHE_992_OBDII_65_20.xml",
        @"../../992/PORSCHE_992_OBDII_65_20.xml",
    ];

    for (NSString *base in bases) {
        for (NSString *suffix in suffixes) {
            NSString *candidate =
                [[base stringByAppendingPathComponent:suffix] stringByStandardizingPath];
            BOOL is_dir = NO;
            if ([fm fileExistsAtPath:candidate isDirectory:&is_dir] && !is_dir) {
                return candidate;
            }
        }
    }

    return nil;
}

static bool aim992_load_xml_channels(aim992_xml_channel_t channels[],
                                     int max_channels,
                                     int *channel_count_out,
                                     char *xml_path_out,
                                     size_t xml_path_out_len) {
    if (channel_count_out) *channel_count_out = 0;

    NSString *xml_path = aim992_resolve_xml_path();
    if (!xml_path) return false;

    NSError *read_err = nil;
    NSData *xml_data = [NSData dataWithContentsOfFile:xml_path options:0 error:&read_err];
    if (!xml_data) return false;

    NSError *parse_err = nil;
    NSXMLDocument *doc = [[NSXMLDocument alloc] initWithData:xml_data options:0 error:&parse_err];
    if (!doc) return false;

    NSError *xpath_err = nil;
    NSArray *nodes = [doc nodesForXPath:@"//e[@c='Canale']" error:&xpath_err];
    if (!nodes || nodes.count == 0) return false;

    int count = 0;
    for (NSXMLNode *node in nodes) {
        if (count >= max_channels) break;

        NSString *long_name = aim992_xpath_string(node, @"./p[@n='LongName']");
        if (long_name.length == 0) continue;

        aim992_xml_channel_t *ch = &channels[count];
        memset(ch, 0, sizeof(*ch));

        aim992_copy_nsstring(ch->long_name, sizeof(ch->long_name), long_name);
        aim992_copy_nsstring(ch->short_name, sizeof(ch->short_name),
                             aim992_xpath_string(node, @"./p[@n='ShortName']"));
        aim992_copy_nsstring(ch->unit, sizeof(ch->unit),
                             aim992_xpath_string(node, @"./e[@c='SENS']/p[@n='UIB']"));
        aim992_copy_nsstring(ch->sens_name, sizeof(ch->sens_name),
                             aim992_xpath_string(node, @"./e[@c='SENS']/@i"));

        NSString *idfis_str = aim992_xpath_string(node, @"./p[@n='IDFis']");
        NSString *fid_str = aim992_xpath_string(node, @"./p[@n='FID']");
        NSString *max_freq_str = aim992_xpath_string(node, @"./p[@n='MaxFreqIus']");
        NSString *lo_str = aim992_xpath_string(node, @"./e[@c='SENS']/p[@n='Lo']");
        NSString *hi_str = aim992_xpath_string(node, @"./e[@c='SENS']/p[@n='Hi']");
        NSString *conv1_str = aim992_xpath_string(node,
                                                  @"./e[@c='SENS']/e[@c='ConvFunc']/p[@n='1']");

        ch->idfis = idfis_str.length > 0 ? idfis_str.intValue : -1;
        ch->fid = fid_str.length > 0 ? fid_str.intValue : -1;
        ch->max_freq_ius = max_freq_str.length > 0 ? max_freq_str.intValue : 0;
        if (lo_str.length > 0) {
            ch->lo = lo_str.doubleValue;
            ch->have_lo = true;
        }
        if (hi_str.length > 0) {
            ch->hi = hi_str.doubleValue;
            ch->have_hi = true;
        }
        if (conv1_str.length > 0) {
            ch->conv1 = conv1_str.doubleValue;
            ch->have_conv1 = true;
        }

        count++;
    }

    if (channel_count_out) *channel_count_out = count;
    if (xml_path_out && xml_path_out_len > 0) {
        aim992_copy_nsstring(xml_path_out, xml_path_out_len, xml_path);
    }
    return count > 0;
}

static int aim992_select_channels(const aim992_xml_channel_t channels[],
                                  int channel_count,
                                  int name_argc,
                                  const char *name_argv[],
                                  int selected_indices[],
                                  int max_selected) {
    int selected_count = 0;

    if (name_argc == 0) {
        for (int i = 0; i < channel_count && selected_count < max_selected; i++) {
            selected_indices[selected_count++] = i;
        }
        return selected_count;
    }

    for (int i = 0; i < name_argc && selected_count < max_selected; i++) {
        int idx = aim992_find_xml_channel_index(channels, channel_count, name_argv[i]);
        if (idx < 0) return -1;

        bool duplicate = false;
        for (int j = 0; j < selected_count; j++) {
            if (selected_indices[j] == idx) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            selected_indices[selected_count++] = idx;
        }
    }

    return selected_count;
}

static bool aim992_result_value_in_range(const aim992_xml_channel_t *xml, double value) {
    if (!xml) return true;
    if (xml->have_lo && value < xml->lo - 1e-6) return false;
    if (xml->have_hi && value > xml->hi + 1e-6) return false;
    return true;
}

static void aim992_result_init(aim992_result_t *result,
                               const aim992_xml_channel_t *xml,
                               const aim992_channel_spec_t *spec) {
    memset(result, 0, sizeof(*result));
    result->xml = xml;
    result->spec = spec;
    if (xml) {
        aim992_copy_cstr(result->unit, sizeof(result->unit), xml->unit);
    }
    if (spec) {
        aim992_copy_cstr(result->request, sizeof(result->request), spec->request_label);
        aim992_copy_cstr(result->note, sizeof(result->note), spec->note);
        result->tx_id = spec->route_count > 0 ? spec->routes[0].tx_id : 0;
        result->rx_id = spec->route_count > 0 ? spec->routes[0].rx_id : 0;
    }
    aim992_copy_cstr(result->status, sizeof(result->status), "pending");
}

static bool aim992_send_command_ok(const char *cmd, char *resp, size_t resp_len) {
    if (!cmd || !resp || resp_len == 0) return false;
    if (![g_ble sendCommand:cmd response:resp maxLen:(int)resp_len]) return false;
    return !elm327_is_error_response(resp);
}

static bool aim992_prepare_query_context(aim992_query_ctx_t *ctx) {
    if (!ctx) return false;
    if (ctx->prepared) return true;

    char resp[256];
    if (!aim992_send_command_ok("ATSP6", resp, sizeof(resp))) return false;

    if (![g_ble sendCommand:"ATCSM0" response:resp maxLen:sizeof(resp)]) {
        /* best effort */
    }
    [g_ble sendCommand:"ATAT2" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATST 0A" response:resp maxLen:sizeof(resp)];
    if (g_stn_detected) {
        [g_ble sendCommand:"STPTO 0A" response:resp maxLen:sizeof(resp)];
    }
    [g_ble sendCommand:"ATS0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAL" response:resp maxLen:sizeof(resp)];

    ctx->prepared = true;
    ctx->tx_id = UINT32_MAX;
    ctx->rx_id = UINT32_MAX;
    ctx->flow_control_ready = false;
    ctx->flow_control_tx_id = UINT32_MAX;
    return true;
}

static bool aim992_set_route(aim992_query_ctx_t *ctx, aim992_route_t route) {
    if (!ctx) return false;
    if (!aim992_prepare_query_context(ctx)) return false;

    if (ctx->tx_id == route.tx_id && ctx->rx_id == route.rx_id) {
        return true;
    }

    char cmd[32];
    char resp[256];

    snprintf(cmd, sizeof(cmd), "ATSH %03X", route.tx_id);
    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;

    snprintf(cmd, sizeof(cmd), "ATCRA %03X", route.rx_id);
    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;

    ctx->tx_id = route.tx_id;
    ctx->rx_id = route.rx_id;
    return true;
}

static void aim992_configure_flow_control(aim992_route_t route) {
    char cmd[32];
    char resp[256];

    /* AiM uses 30 00 01 on the same tester CAN ID for 992 multi-frame DIDs. */
    [g_ble sendCommand:"ATCFC1" response:resp maxLen:sizeof(resp)];
    snprintf(cmd, sizeof(cmd), "ATFCSH%03X", route.tx_id);
    [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATFCSD300001" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATFCSM1" response:resp maxLen:sizeof(resp)];
}

static bool aim992_ensure_flow_control(aim992_query_ctx_t *ctx, aim992_route_t route) {
    if (!ctx) return false;

    char cmd[32];
    char resp[256];

    if (!ctx->flow_control_ready) {
        if (!aim992_send_command_ok("ATCFC1", resp, sizeof(resp))) return false;
        snprintf(cmd, sizeof(cmd), "ATFCSH%03X", route.tx_id);
        if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;
        if (!aim992_send_command_ok("ATFCSD300001", resp, sizeof(resp))) return false;
        if (!aim992_send_command_ok("ATFCSM1", resp, sizeof(resp))) return false;

        ctx->flow_control_ready = true;
        ctx->flow_control_tx_id = route.tx_id;
        return true;
    }

    if (ctx->flow_control_tx_id == route.tx_id) {
        return true;
    }

    snprintf(cmd, sizeof(cmd), "ATFCSH%03X", route.tx_id);
    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;

    ctx->flow_control_tx_id = route.tx_id;
    return true;
}

static bool aim992_extract_after_marker(const char *response,
                                        const char *marker_hex,
                                        uint8_t *payload_out,
                                        int max_payload,
                                        int *payload_len_out,
                                        char *raw_hex_out,
                                        size_t raw_hex_out_len) {
    if (payload_len_out) *payload_len_out = 0;

    char hex[AIM992_MAX_RESPONSE_HEX];
    int hex_len = elm327_extract_hex(response, hex, sizeof(hex));
    if (raw_hex_out && raw_hex_out_len > 0) {
        aim992_copy_cstr(raw_hex_out, raw_hex_out_len, hex);
    }
    if (hex_len <= 0) return false;

    const char *marker = strstr(hex, marker_hex);
    if (!marker) return false;

    int offset = (int)(marker - hex) + (int)strlen(marker_hex);
    int payload_len = 0;
    while (offset + 1 < hex_len && payload_len < max_payload) {
        if (!aim992_parse_hex_byte(hex + offset, &payload_out[payload_len])) return false;
        payload_len++;
        offset += 2;
    }

    if (payload_len_out) *payload_len_out = payload_len;
    return true;
}

static const char *aim992_uds_nrc_name(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFunctionNotSupported";
        case 0x13: return "incorrectMessageLengthOrInvalidFormat";
        case 0x22: return "conditionsNotCorrect";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x78: return "responsePending";
        default:   return "unknown";
    }
}

static bool aim992_find_uds_negative_response(const char *hex,
                                              uint8_t service_id,
                                              uint8_t *nrc_out) {
    if (!hex || !hex[0]) return false;

    char marker[8];
    snprintf(marker, sizeof(marker), "7F%02X", service_id);

    const char *p = strstr(hex, marker);
    if (!p) return false;

    int offset = (int)(p - hex) + 4;
    if (offset + 1 >= (int)strlen(hex)) return false;

    uint8_t nrc = 0;
    if (!aim992_parse_hex_byte(hex + offset, &nrc)) return false;
    if (nrc_out) *nrc_out = nrc;
    return true;
}

static bool aim992_find_uds_positive_did_response(const char *hex, uint16_t did) {
    if (!hex || !hex[0]) return false;

    char marker[12];
    snprintf(marker, sizeof(marker), "62%02X%02X", did >> 8, did & 0xFF);
    return strstr(hex, marker) != NULL;
}

static bool aim992_find_uds_positive_session_response(const char *hex, uint8_t session) {
    if (!hex || !hex[0]) return false;

    char marker[8];
    snprintf(marker, sizeof(marker), "50%02X", session);
    return strstr(hex, marker) != NULL;
}

static int aim992_extract_line_hex(const char *line, char *hex_out, size_t hex_out_len) {
    size_t j = 0;
    if (!line || !hex_out || hex_out_len == 0) return 0;

    for (size_t i = 0; line[i] != '\0' && j + 1 < hex_out_len; i++) {
        if (aim992_hex_value(line[i]) >= 0) {
            hex_out[j++] = (char)toupper((unsigned char)line[i]);
        }
    }
    hex_out[j] = '\0';
    return (int)j;
}

static bool aim992_parse_isotp_payload(const char *response,
                                       uint8_t *payload_out,
                                       int max_payload,
                                       int *payload_len_out,
                                       uint32_t *resp_id_out) {
    if (payload_len_out) *payload_len_out = 0;
    if (!response || !payload_out || max_payload <= 0) return false;

    char buf[1024];
    aim992_copy_cstr(buf, sizeof(buf), response);

    int payload_len = 0;
    int total_len = -1;
    bool started = false;
    uint32_t resp_id = UINT32_MAX;

    for (char *line = strtok(buf, "\r\n"); line; line = strtok(NULL, "\r\n")) {
        char hex[128];
        int hex_len = aim992_extract_line_hex(line, hex, sizeof(hex));
        if (hex_len < 5) continue;

        unsigned int id_val = 0;
        if (sscanf(hex, "%3x", &id_val) != 1) continue;

        const char *data_hex = hex + 3;
        int data_hex_len = hex_len - 3;
        if (data_hex_len < 2) continue;

        uint8_t data[32];
        int data_len = 0;
        for (int i = 0; i + 1 < data_hex_len && data_len < (int)sizeof(data); i += 2) {
            if (!aim992_parse_hex_byte(data_hex + i, &data[data_len])) break;
            data_len++;
        }
        if (data_len <= 0) continue;

        uint8_t pci = data[0];
        uint8_t frame_type = (uint8_t)(pci >> 4);
        if (frame_type == 0x0) {
            int len = pci & 0x0F;
            if (len > data_len - 1) len = data_len - 1;
            if (len <= 0) continue;
            if (len > max_payload) len = max_payload;
            memcpy(payload_out, data + 1, (size_t)len);
            payload_len = len;
            total_len = len;
            resp_id = id_val;
            started = true;
            break;
        }

        if (frame_type == 0x1) {
            total_len = ((pci & 0x0F) << 8) | data[1];
            int copy_len = data_len - 2;
            if (copy_len < 0) copy_len = 0;
            if (copy_len > max_payload) copy_len = max_payload;
            memcpy(payload_out, data + 2, (size_t)copy_len);
            payload_len = copy_len;
            resp_id = id_val;
            started = true;
            if (payload_len >= total_len) break;
            continue;
        }

        if (frame_type == 0x2 && started) {
            int copy_len = data_len - 1;
            if (copy_len <= 0) continue;
            if (payload_len + copy_len > max_payload) {
                copy_len = max_payload - payload_len;
            }
            if (copy_len <= 0) break;
            memcpy(payload_out + payload_len, data + 1, (size_t)copy_len);
            payload_len += copy_len;
            if (total_len >= 0 && payload_len >= total_len) break;
        }
    }

    if (!started || payload_len <= 0) return false;
    if (total_len >= 0 && payload_len > total_len) payload_len = total_len;
    if (payload_len_out) *payload_len_out = payload_len;
    if (resp_id_out) *resp_id_out = resp_id;
    return true;
}

static void aim992_format_payload_hex(const uint8_t *payload,
                                      int payload_len,
                                      char *hex_out,
                                      size_t hex_out_len) {
    if (!hex_out || hex_out_len == 0) return;
    hex_out[0] = '\0';
    if (!payload || payload_len <= 0) return;

    size_t pos = 0;
    for (int i = 0; i < payload_len && pos + 3 < hex_out_len; i++) {
        pos += (size_t)snprintf(hex_out + pos, hex_out_len - pos, "%02X", payload[i]);
    }
}

static bool aim992_payload_ascii(const uint8_t *payload,
                                 int payload_len,
                                 int offset,
                                 char *text_out,
                                 size_t text_out_len) {
    if (!text_out || text_out_len == 0) return false;
    text_out[0] = '\0';
    if (!payload || payload_len <= offset) return false;

    size_t pos = 0;
    for (int i = offset; i < payload_len && pos + 1 < text_out_len; i++) {
        uint8_t b = payload[i];
        if (b < 0x20 || b > 0x7E) return false;
        text_out[pos++] = (char)b;
    }
    if (pos == 0) return false;
    text_out[pos] = '\0';
    return true;
}

static void aim992_fill_pid_result(aim992_result_t *result,
                                   const uint8_t *data,
                                   int data_len,
                                   const char *raw_hex);

static bool aim992_query_pid(aim992_query_ctx_t *ctx,
                             uint8_t pid,
                             aim992_route_t route,
                             uint8_t *data_out,
                             int *data_len_out,
                             char *raw_hex_out,
                             size_t raw_hex_out_len) {
    if (!ctx || !data_out || !data_len_out) return false;
    if (!aim992_set_route(ctx, route)) return false;

    char cmd[16];
    char resp[512];
    char marker[8];

    elm327_build_pid_query(pid, cmd, sizeof(cmd));
    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;

    snprintf(marker, sizeof(marker), "41%02X", pid);

    uint8_t payload[ELM327_MAX_DATA_BYTES];
    int payload_len = 0;
    if (!aim992_extract_after_marker(resp, marker, payload, sizeof(payload),
                                     &payload_len, raw_hex_out, raw_hex_out_len)) {
        return false;
    }

    const elm327_pid_info_t *info = elm327_get_pid_info(pid);
    int expected_len = info ? info->data_bytes : payload_len;
    if (expected_len > payload_len) return false;

    memcpy(data_out, payload, (size_t)expected_len);
    *data_len_out = expected_len;
    return true;
}

static int aim992_mode01_response_bytes(uint8_t pid) {
    const elm327_pid_info_t *info = elm327_get_pid_info(pid);
    return 1 + (info ? info->data_bytes : 1);
}

static bool aim992_find_mode01_batch_data(const uint8_t *payload,
                                          int payload_len,
                                          uint8_t pid,
                                          uint8_t *data_out,
                                          int *data_len_out) {
    if (!payload || payload_len < 2 || !data_out || !data_len_out) return false;
    if (payload[0] != 0x41) return false;

    int offset = 1;
    while (offset < payload_len) {
        uint8_t candidate_pid = payload[offset++];
        const elm327_pid_info_t *info = elm327_get_pid_info(candidate_pid);
        int data_len = info ? info->data_bytes : 1;
        if (data_len <= 0 || offset + data_len > payload_len) return false;

        if (candidate_pid == pid) {
            memcpy(data_out, payload + offset, (size_t)data_len);
            *data_len_out = data_len;
            return true;
        }

        offset += data_len;
    }

    return false;
}

static bool aim992_query_pid_batch(aim992_query_ctx_t *ctx,
                                   const aim992_channel_spec_t *specs[],
                                   const int result_indices[],
                                   int pid_count,
                                   aim992_route_t route,
                                   aim992_result_t results[]) {
    if (!ctx || !specs || !result_indices || !results ||
        pid_count <= 1 || pid_count > STN2255_MAX_BATCH_PIDS) {
        return false;
    }
    if (!aim992_set_route(ctx, route)) return false;

    uint8_t pids[STN2255_MAX_BATCH_PIDS];
    int expected_response_payload_len = 1;
    for (int i = 0; i < pid_count; i++) {
        pids[i] = specs[i]->pid;
        expected_response_payload_len += aim992_mode01_response_bytes(specs[i]->pid);
    }

    if (expected_response_payload_len > 7 && route.tx_id != 0x7DF) {
        if (!aim992_ensure_flow_control(ctx, route)) return false;
    }

    char cmd[STN2255_CMD_MAX_LEN];
    if (stn2255_build_batch_query(pids, pid_count, cmd, sizeof(cmd)) != 0) {
        return false;
    }

    char resp[1024];
    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) return false;

    char raw_hex[AIM992_MAX_RESPONSE_HEX] = "";
    elm327_extract_hex(resp, raw_hex, sizeof(raw_hex));

    uint8_t isotp_payload[AIM992_MAX_PAYLOAD_BYTES];
    int isotp_payload_len = 0;
    uint32_t resp_id = UINT32_MAX;
    if (aim992_parse_isotp_payload(resp, isotp_payload, sizeof(isotp_payload),
                                   &isotp_payload_len, &resp_id)) {
        bool any = false;
        for (int i = 0; i < pid_count; i++) {
            uint8_t data[ELM327_MAX_DATA_BYTES];
            int data_len = 0;
            if (!aim992_find_mode01_batch_data(isotp_payload, isotp_payload_len,
                                               specs[i]->pid, data, &data_len)) {
                continue;
            }

            aim992_result_t *res = &results[result_indices[i]];
            if (res->has_value) continue;

            res->tx_id = route.tx_id;
            res->rx_id = route.rx_id;
            aim992_fill_pid_result(res, data, data_len, raw_hex);
            aim992_copy_cstr(res->transport, sizeof(res->transport), "mode01_batch");
            any = true;
        }
        if (any) return true;
    }

    stn2255_batch_result_t batch;
    if (stn2255_parse_batch_response(resp, &batch) <= 0) return false;

    bool any = false;
    for (int i = 0; i < pid_count; i++) {
        aim992_result_t *res = &results[result_indices[i]];
        if (res->has_value) continue;

        for (int j = 0; j < batch.count; j++) {
            if (!batch.results[j].valid || batch.results[j].pid != specs[i]->pid) continue;

            res->tx_id = route.tx_id;
            res->rx_id = route.rx_id;
            aim992_fill_pid_result(res, batch.results[j].data,
                                   batch.results[j].data_len, raw_hex);
            aim992_copy_cstr(res->transport, sizeof(res->transport), "mode01_batch");
            any = true;
            break;
        }
    }

    return any;
}

static bool aim992_query_did(aim992_query_ctx_t *ctx,
                             uint16_t did,
                             const aim992_route_t routes[],
                             int route_count,
                             uint8_t *payload_out,
                             int *payload_len_out,
                             char *raw_hex_out,
                             size_t raw_hex_out_len,
                             aim992_route_t *matched_route_out,
                             char *failure_note_out,
                             size_t failure_note_out_len) {
    if (!ctx || !routes || route_count <= 0 || !payload_out || !payload_len_out) return false;
    if (failure_note_out && failure_note_out_len > 0) failure_note_out[0] = '\0';

    char cmd[16];
    char resp[512];
    char marker[12];

    snprintf(cmd, sizeof(cmd), "22%02X%02X", did >> 8, did & 0xFF);
    snprintf(marker, sizeof(marker), "62%02X%02X", did >> 8, did & 0xFF);

	for (int i = 0; i < route_count; i++) {
	    if (!aim992_set_route(ctx, routes[i])) continue;
	    if (!aim992_ensure_flow_control(ctx, routes[i])) continue;
	    if (!aim992_send_command_ok(cmd, resp, sizeof(resp))) continue;

	    uint8_t isotp_payload[AIM992_MAX_PAYLOAD_BYTES];
	    int isotp_payload_len = 0;
	    uint32_t resp_id = UINT32_MAX;
	    if (aim992_parse_isotp_payload(resp, isotp_payload, sizeof(isotp_payload),
	                                   &isotp_payload_len, &resp_id)) {
	        if (isotp_payload_len >= 3 &&
	            isotp_payload[0] == 0x62 &&
	            isotp_payload[1] == (uint8_t)(did >> 8) &&
	            isotp_payload[2] == (uint8_t)(did & 0xFF)) {
	            int payload_len = isotp_payload_len - 3;
	            if (payload_len > AIM992_MAX_PAYLOAD_BYTES) payload_len = AIM992_MAX_PAYLOAD_BYTES;
	            memcpy(payload_out, isotp_payload + 3, (size_t)payload_len);
	            *payload_len_out = payload_len;
	            if (raw_hex_out && raw_hex_out_len > 0) {
	                aim992_format_payload_hex(isotp_payload, isotp_payload_len,
	                                          raw_hex_out, raw_hex_out_len);
	            }
	            if (matched_route_out) *matched_route_out = routes[i];
	            return true;
	        }

	        if (isotp_payload_len >= 3 &&
	            isotp_payload[0] == 0x7F &&
	            isotp_payload[1] == 0x22 &&
	            failure_note_out && failure_note_out_len > 0) {
	            if (matched_route_out) *matched_route_out = routes[i];
	            snprintf(failure_note_out, failure_note_out_len,
	                     "No positive response to 22 %02X %02X; last negative response was %03X->%03X NRC 0x%02X (%s)",
	                     did >> 8, did & 0xFF,
	                     routes[i].tx_id, routes[i].rx_id,
	                     isotp_payload[2], aim992_uds_nrc_name(isotp_payload[2]));
	        }
	        continue;
	    }

	    int payload_len = 0;
	    if (!aim992_extract_after_marker(resp, marker, payload_out, AIM992_MAX_PAYLOAD_BYTES,
                                         &payload_len, raw_hex_out, raw_hex_out_len)) {
            char hex[AIM992_MAX_RESPONSE_HEX] = "";
            uint8_t nrc = 0;
            elm327_extract_hex(resp, hex, sizeof(hex));
            if (aim992_find_uds_negative_response(hex, 0x22, &nrc) &&
                failure_note_out && failure_note_out_len > 0) {
                if (matched_route_out) *matched_route_out = routes[i];
                snprintf(failure_note_out, failure_note_out_len,
                         "No positive response to 22 %02X %02X; last negative response was %03X->%03X NRC 0x%02X (%s)",
                         did >> 8, did & 0xFF,
                         routes[i].tx_id, routes[i].rx_id,
                         nrc, aim992_uds_nrc_name(nrc));
            }
            continue;
        }

        *payload_len_out = payload_len;
        if (matched_route_out) *matched_route_out = routes[i];
        return true;
    }

    return false;
}

static int16_t aim992_read_s16_be(const uint8_t *p) {
    return (int16_t)((p[0] << 8) | p[1]);
}

static int16_t aim992_read_s16_le(const uint8_t *p) {
    return (int16_t)((p[1] << 8) | p[0]);
}

static uint16_t aim992_read_u16_be(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}

static bool aim992_decode_steer_guess(const aim992_xml_channel_t *xml,
                                      const uint8_t *payload,
                                      int payload_len,
                                      double *value_out,
                                      char *note_out,
                                      size_t note_out_len) {
    double scale = (xml && xml->have_conv1) ? xml->conv1 : 0.1;
    const struct { int offset; bool little; } candidates[] = {
        { 0, false }, { 2, false }, { 0, true }, { 2, true },
    };

    for (int i = 0; i < 4; i++) {
        int offset = candidates[i].offset;
        if (offset + 1 >= payload_len) continue;

        int16_t raw = candidates[i].little
            ? aim992_read_s16_le(payload + offset)
            : aim992_read_s16_be(payload + offset);
        double value = raw * scale;
        if (!aim992_result_value_in_range(xml, value)) continue;

        if (value_out) *value_out = value;
        if (note_out && note_out_len > 0) {
            snprintf(note_out, note_out_len,
                     "heuristic DID 0x5101 decode: %s-endian signed16 payload[%d..%d] x %.3g",
                     candidates[i].little ? "little" : "big",
                     offset, offset + 1, scale);
        }
        return true;
    }

    return false;
}

static bool aim992_decode_steer_5075_guess(const aim992_xml_channel_t *xml,
                                           const uint8_t *payload,
                                           int payload_len,
                                           double *value_out,
                                           char *note_out,
                                           size_t note_out_len) {
    if (!payload || payload_len <= 9) return false;

    const int lane_a = payload[7];
    const int lane_b = payload[9];
    const bool complement_pair = (lane_a + lane_b) == 200;

    const double candidate_a = ((double)lane_a - 128.0) / 10.0;
    const double candidate_b = ((double)lane_b - 100.0) / 10.0;

    if (aim992_result_value_in_range(xml, candidate_a)) {
        if (value_out) *value_out = candidate_a;
        if (note_out && note_out_len > 0) {
            snprintf(note_out, note_out_len,
                     "heuristic DID 0x5075 decode: payload[7] -> (byte-128)/10 = %.3f deg%s; mirror lane payload[9] -> (byte-100)/10 = %.3f deg",
                     candidate_a,
                     complement_pair ? ", payload[7]+payload[9]=200" : "",
                     candidate_b);
        }
        return true;
    }

    if (aim992_result_value_in_range(xml, candidate_b)) {
        if (value_out) *value_out = candidate_b;
        if (note_out && note_out_len > 0) {
            snprintf(note_out, note_out_len,
                     "heuristic DID 0x5075 decode: payload[9] -> (byte-100)/10 = %.3f deg%s; alternate lane payload[7] -> (byte-128)/10 = %.3f deg",
                     candidate_b,
                     complement_pair ? ", payload[7]+payload[9]=200" : "",
                     candidate_a);
        }
        return true;
    }

    return false;
}

static bool aim992_decode_brake_2b21_guess(const aim992_xml_channel_t *xml,
                                           const uint8_t *payload,
                                           int payload_len,
                                           double *value_out,
                                           char *note_out,
                                           size_t note_out_len) {
    if (!payload || payload_len < 2) return false;

    const uint16_t raw = aim992_read_u16_be(payload);
    const double value_unclamped = ((double)raw * 0.016) - 44.451;
    const double value = value_unclamped > 0.0 ? value_unclamped : 0.0;

    if (!aim992_result_value_in_range(xml, value)) return false;

    if (value_out) *value_out = value;
    if (note_out && note_out_len > 0) {
        snprintf(note_out, note_out_len,
                 "GT3 XC1 DID 0x2B21 decode: big-endian raw payload[0..1]=0x%04X, raw*0.016-44.451 = %.3f bar",
                 raw, value);
    }
    return true;
}

static void aim992_fill_pid_result(aim992_result_t *result,
                                   const uint8_t *data,
                                   int data_len,
                                   const char *raw_hex) {
    if (!result || !result->spec || !result->xml) return;

	    double value = elm327_convert_pid_value(result->spec->pid, data, data_len);
	    switch (result->spec->decode_kind) {
	        case AIM992_DECODE_PPS_AIM_PERCENT:
	            if (data_len >= 1) {
	                value = 1.09 + (((double)data[0] - 38.0) * (103.80 / 191.0));
	                if (value < 0.0) value = 0.0;
	            }
	            break;
	        case AIM992_DECODE_MAP_MBAR:
	            value *= 10.0;
	            aim992_copy_cstr(result->unit, sizeof(result->unit), "mbar");
            break;
        case AIM992_DECODE_STD:
        default:
            break;
    }

    result->has_value = true;
    result->raw_available = (raw_hex && raw_hex[0]);
    if (raw_hex) {
        aim992_copy_cstr(result->raw_hex, sizeof(result->raw_hex), raw_hex);
    }
    aim992_copy_cstr(result->transport, sizeof(result->transport), "mode01");
    aim992_copy_cstr(result->status, sizeof(result->status), "ok");
    result->value = value;
}

static void aim992_print_list_item(FILE *f,
                                   const aim992_xml_channel_t *xml,
                                   const aim992_channel_spec_t *spec) {
    char tx_id[8] = "";

    if (spec && spec->route_count > 0) {
        snprintf(tx_id, sizeof(tx_id), "%03X", spec->routes[0].tx_id);
    }

    fprintf(f, "  {\"long_name\":");
    json_print_string(f, xml->long_name);
    fprintf(f, ",\"short_name\":");
    json_print_string(f, xml->short_name);
    fprintf(f, ",\"idfis\":%d,\"fid\":%d", xml->idfis, xml->fid);
    fprintf(f, ",\"unit\":");
    json_print_string(f, xml->unit);
    fprintf(f, ",\"sens\":");
    json_print_string(f, xml->sens_name);
    if (xml->have_lo) fprintf(f, ",\"lo\":%.6g", xml->lo);
    if (xml->have_hi) fprintf(f, ",\"hi\":%.6g", xml->hi);
    if (xml->have_conv1) fprintf(f, ",\"conv1\":%.6g", xml->conv1);
    if (xml->max_freq_ius > 0) {
        fprintf(f, ",\"max_freq_hz\":%.3f", 1000000.0 / (double)xml->max_freq_ius);
    }
    if (spec) {
        fprintf(f, ",\"transport\":");
        json_print_string(f, spec->query_kind == AIM992_QUERY_PID ? "mode01" : "uds");
        fprintf(f, ",\"request\":");
        json_print_string(f, spec->request_label);
        fprintf(f, ",\"tx_id\":");
        json_print_string(f, tx_id);
        fprintf(f, ",\"rx_ids\":[");
        bool first_rx = true;
        for (int i = 0; i < spec->route_count; i++) {
            char rx_id[8];
            bool duplicate = false;
            for (int j = 0; j < i; j++) {
                if (spec->routes[j].rx_id == spec->routes[i].rx_id) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            snprintf(rx_id, sizeof(rx_id), "%03X", spec->routes[i].rx_id);
            if (!first_rx) fprintf(f, ",");
            json_print_string(f, rx_id);
            first_rx = false;
        }
        fprintf(f, "]");
        fprintf(f, ",\"routes\":[");
        for (int i = 0; i < spec->route_count; i++) {
            char route_tx[8];
            char route_rx[8];
            snprintf(route_tx, sizeof(route_tx), "%03X", spec->routes[i].tx_id);
            snprintf(route_rx, sizeof(route_rx), "%03X", spec->routes[i].rx_id);
            if (i > 0) fprintf(f, ",");
            fprintf(f, "{\"tx_id\":");
            json_print_string(f, route_tx);
            fprintf(f, ",\"rx_id\":");
            json_print_string(f, route_rx);
            fprintf(f, "}");
        }
        fprintf(f, "]");
        fprintf(f, ",\"decode\":");
        switch (spec->decode_kind) {
            case AIM992_DECODE_STEER_HEURISTIC:
            case AIM992_DECODE_BRAKE_HEURISTIC:
                json_print_string(f, "heuristic");
                break;
            default:
                json_print_string(f, "documented");
                break;
        }
        fprintf(f, ",\"note\":");
        json_print_string(f, spec->note);
    } else {
        fprintf(f, ",\"transport\":\"unmapped\"");
    }
    fprintf(f, "}");
}

static void aim992_print_result_item(FILE *f, const aim992_result_t *result) {
    char tx_id[8] = "";
    char rx_id[8] = "";

    if (result->tx_id <= 0x7FF) snprintf(tx_id, sizeof(tx_id), "%03X", result->tx_id);
    if (result->rx_id <= 0x7FF) snprintf(rx_id, sizeof(rx_id), "%03X", result->rx_id);

    fprintf(f, "  {\"channel\":");
    json_print_string(f, result->xml->long_name);
    fprintf(f, ",\"short_name\":");
    json_print_string(f, result->xml->short_name);
    fprintf(f, ",\"status\":");
    json_print_string(f, result->status);
    fprintf(f, ",\"value\":");
    if (result->has_value) fprintf(f, "%.6f", result->value);
    else fprintf(f, "null");
    fprintf(f, ",\"unit\":");
    json_print_string(f, result->unit);
    fprintf(f, ",\"transport\":");
    json_print_string(f, result->transport);
    fprintf(f, ",\"request\":");
    json_print_string(f, result->request);
    fprintf(f, ",\"tx_id\":");
    json_print_string(f, tx_id);
    fprintf(f, ",\"rx_id\":");
    json_print_string(f, rx_id);
    fprintf(f, ",\"raw\":");
    if (result->raw_available) json_print_string(f, result->raw_hex);
    else fprintf(f, "null");
    fprintf(f, ",\"heuristic\":%s", result->heuristic ? "true" : "false");
    fprintf(f, ",\"fallback\":%s", result->used_fallback ? "true" : "false");
    fprintf(f, ",\"note\":");
    json_print_string(f, result->note);
    fprintf(f, "}");
}

static bool aim992_collect_snapshot(aim992_query_ctx_t *ctx,
                                    const aim992_xml_channel_t channels[],
                                    const int selected_indices[],
                                    int selected_count,
                                    aim992_result_t results[]) {
    int fuel_result_idx = -1;
    int steer_result_idx = -1;
    int brake_result_idx = -1;
    const aim992_channel_spec_t *pid_specs[AIM992_MAX_CHANNELS];
    int pid_result_indices[AIM992_MAX_CHANNELS];
    int pid_count = 0;

    for (int i = 0; i < selected_count; i++) {
        int ch_idx = selected_indices[i];
        const aim992_xml_channel_t *xml = &channels[ch_idx];
        const aim992_channel_spec_t *spec = aim992_find_spec_by_name(xml->long_name);

        aim992_result_init(&results[i], xml, spec);

        if (!spec) {
            aim992_copy_cstr(results[i].status, sizeof(results[i].status), "unsupported");
            aim992_copy_cstr(results[i].transport, sizeof(results[i].transport), "unmapped");
            aim992_copy_cstr(results[i].note, sizeof(results[i].note),
                             "XML channel exists but no request mapping is implemented");
            continue;
        }

        if (spec->query_kind == AIM992_QUERY_UDS) {
            aim992_copy_cstr(results[i].transport, sizeof(results[i].transport), "uds");
            if (strcasecmp(xml->long_name, "STEER_ANGLE") == 0) {
                steer_result_idx = i;
            } else if (strcasecmp(xml->long_name, "BRAKE_PRESS") == 0) {
                brake_result_idx = i;
            }
            continue;
        }

        pid_specs[pid_count] = spec;
        pid_result_indices[pid_count] = i;
        pid_count++;
    }

    bool pid_grouped[AIM992_MAX_CHANNELS] = { false };
    for (int i = 0; i < pid_count; i++) {
        if (pid_grouped[i]) continue;
        if (pid_specs[i]->route_count <= 0) continue;

        aim992_route_t route = pid_specs[i]->routes[0];
        if (route.tx_id == 0 || route.rx_id == 0) continue;

        const aim992_channel_spec_t *batch_specs[STN2255_MAX_BATCH_PIDS];
        int batch_result_indices[STN2255_MAX_BATCH_PIDS];
        int batch_count = 0;
        int response_payload_bytes = 1; /* service 0x41 */
        int max_response_payload_bytes = route.tx_id == 0x7E0 ? AIM992_MAX_PAYLOAD_BYTES : 7;

        for (int j = i; j < pid_count && batch_count < STN2255_MAX_BATCH_PIDS; j++) {
            if (pid_grouped[j] || pid_specs[j]->route_count <= 0) continue;
            aim992_route_t candidate = pid_specs[j]->routes[0];
            if (candidate.tx_id != route.tx_id || candidate.rx_id != route.rx_id) continue;

            int add_bytes = aim992_mode01_response_bytes(pid_specs[j]->pid);
            if (response_payload_bytes + add_bytes > max_response_payload_bytes) continue;

            batch_specs[batch_count] = pid_specs[j];
            batch_result_indices[batch_count] = pid_result_indices[j];
            pid_grouped[j] = true;
            batch_count++;
            response_payload_bytes += add_bytes;
        }

        if (batch_count > 1) {
            aim992_query_pid_batch(ctx, batch_specs, batch_result_indices,
                                   batch_count, route, results);
        }
    }

    for (int i = 0; i < pid_count; i++) {
        int result_idx = pid_result_indices[i];
        const aim992_channel_spec_t *spec = pid_specs[i];
        const aim992_xml_channel_t *xml = results[result_idx].xml;

        if (results[result_idx].has_value) {
            continue;
        }

        uint8_t data[ELM327_MAX_DATA_BYTES];
        int data_len = 0;
        char raw_hex[AIM992_MAX_RESPONSE_HEX] = "";

        bool pid_ok = false;
        for (int route_idx = 0; route_idx < spec->route_count; route_idx++) {
            aim992_route_t route = spec->routes[route_idx];
            if (route.tx_id == 0 || route.rx_id == 0) continue;
            if (!aim992_query_pid(ctx, spec->pid, route,
                                  data, &data_len, raw_hex, sizeof(raw_hex))) {
                continue;
            }
            results[result_idx].tx_id = route.tx_id;
            results[result_idx].rx_id = route.rx_id;
            results[result_idx].used_fallback = route_idx > 0;
            aim992_fill_pid_result(&results[result_idx], data, data_len, raw_hex);
            if (route_idx > 0) {
                aim992_copy_cstr(results[result_idx].note, sizeof(results[result_idx].note),
                                 "primary route returned no data; used fallback route");
            }
            pid_ok = true;
            break;
        }

        if (pid_ok) {
            continue;
        }

        if (strcasecmp(xml->long_name, "FUEL_LEVEL") == 0) {
            fuel_result_idx = result_idx;
            aim992_copy_cstr(results[result_idx].status, sizeof(results[result_idx].status), "no_data");
            aim992_copy_cstr(results[result_idx].transport, sizeof(results[result_idx].transport), "mode01");
            aim992_copy_cstr(results[result_idx].note, sizeof(results[result_idx].note),
                             "01 2F returned no data; probing 22 50 75 fallback");
        } else {
            aim992_copy_cstr(results[result_idx].status, sizeof(results[result_idx].status), "no_data");
            aim992_copy_cstr(results[result_idx].transport, sizeof(results[result_idx].transport), "mode01");
        }
    }

    if (steer_result_idx >= 0) {
        static const aim992_route_t grouped_chassis_routes[] = {
            { 0x70C, 0x776 },
        };
        static const aim992_route_t legacy_chassis_routes[] = {
            { 0x70C, 0x73B },
            { 0x7E4, 0x7EC },
            { 0x7E0, 0x7E8 },
            { 0x7E1, 0x7E9 },
        };

        uint8_t payload[AIM992_MAX_PAYLOAD_BYTES];
        int payload_len = 0;
        char raw_hex[AIM992_MAX_RESPONSE_HEX] = "";
        aim992_route_t matched_route = {0};
        char failure_note[160] = "";

        if (aim992_query_did(ctx, 0x5075, grouped_chassis_routes, 1,
                             payload, &payload_len, raw_hex, sizeof(raw_hex), &matched_route,
                             failure_note, sizeof(failure_note))) {
            if (steer_result_idx >= 0) {
                aim992_result_t *res = &results[steer_result_idx];
                res->tx_id = matched_route.tx_id;
                res->rx_id = matched_route.rx_id;
                res->raw_available = true;
                aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
                aim992_copy_cstr(res->transport, sizeof(res->transport), "uds");
                aim992_copy_cstr(res->request, sizeof(res->request), "22 50 75");
                res->heuristic = true;
                if (aim992_decode_steer_5075_guess(res->xml, payload, payload_len,
                                              &res->value, res->note, sizeof(res->note))) {
                    res->has_value = true;
                    aim992_copy_cstr(res->status, sizeof(res->status), "ok");
                } else {
                    aim992_copy_cstr(res->status, sizeof(res->status), "raw_only");
	                    aim992_copy_cstr(res->note, sizeof(res->note),
	                                     "DID 0x5075 responded, but no plausible steering-angle field was found");
	                }
	            }
	        } else if (aim992_query_did(ctx, 0x5101, legacy_chassis_routes, 4,
	                                    payload, &payload_len, raw_hex, sizeof(raw_hex), &matched_route,
	                                    failure_note, sizeof(failure_note))) {
            if (steer_result_idx >= 0) {
                aim992_result_t *res = &results[steer_result_idx];
                res->tx_id = matched_route.tx_id;
                res->rx_id = matched_route.rx_id;
                res->raw_available = true;
                aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
                aim992_copy_cstr(res->transport, sizeof(res->transport), "uds_fallback");
                aim992_copy_cstr(res->request, sizeof(res->request), "22 51 01");
                res->heuristic = true;
                if (aim992_decode_steer_guess(res->xml, payload, payload_len,
                                              &res->value, res->note, sizeof(res->note))) {
                    res->has_value = true;
                    aim992_copy_cstr(res->status, sizeof(res->status), "ok");
                } else {
                    aim992_copy_cstr(res->status, sizeof(res->status), "raw_only");
	                    aim992_copy_cstr(res->note, sizeof(res->note),
	                                     "Fallback DID 0x5101 responded, but no plausible steering-angle field was found");
	                }
	            }
	        } else {
	            const bool negative = failure_note[0] != '\0';
	            const char *note = negative
                ? failure_note
                : "No positive response to grouped DID 0x5075 on 70C->776, and no fallback response to 22 51 01 on legacy routes";
            if (steer_result_idx >= 0) {
                aim992_result_t *res = &results[steer_result_idx];
                if (negative) {
                    res->tx_id = matched_route.tx_id;
                    res->rx_id = matched_route.rx_id;
                    res->raw_available = raw_hex[0] != '\0';
                    aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
                }
                aim992_copy_cstr(results[steer_result_idx].status,
                                 sizeof(results[steer_result_idx].status),
                                 negative ? "negative_response" : "no_data");
                aim992_copy_cstr(results[steer_result_idx].note,
	                                 sizeof(results[steer_result_idx].note),
	                                 note);
	            }
	        }
	    }

    if (brake_result_idx >= 0) {
        static const aim992_route_t brake_routes[] = {
            { 0x713, 0x77D },
        };

        uint8_t payload[AIM992_MAX_PAYLOAD_BYTES];
        int payload_len = 0;
        char raw_hex[AIM992_MAX_RESPONSE_HEX] = "";
        aim992_route_t matched_route = {0};
        char failure_note[160] = "";

        if (aim992_query_did(ctx, 0x2B21, brake_routes, 1,
                             payload, &payload_len, raw_hex, sizeof(raw_hex), &matched_route,
                             failure_note, sizeof(failure_note))) {
            aim992_result_t *res = &results[brake_result_idx];
            res->tx_id = matched_route.tx_id;
            res->rx_id = matched_route.rx_id;
            res->raw_available = true;
            aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
            aim992_copy_cstr(res->transport, sizeof(res->transport), "uds");
            aim992_copy_cstr(res->request, sizeof(res->request), "22 2B 21");
            res->heuristic = true;
            if (aim992_decode_brake_2b21_guess(res->xml, payload, payload_len,
                                               &res->value, res->note, sizeof(res->note))) {
                res->has_value = true;
                aim992_copy_cstr(res->status, sizeof(res->status), "ok");
            } else {
                aim992_copy_cstr(res->status, sizeof(res->status), "raw_only");
                aim992_copy_cstr(res->note, sizeof(res->note),
                                 "DID 0x2B21 responded, but no plausible brake-pressure field was found");
            }
        } else {
            const bool negative = failure_note[0] != '\0';
            aim992_result_t *res = &results[brake_result_idx];
            if (negative) {
                res->tx_id = matched_route.tx_id;
                res->rx_id = matched_route.rx_id;
                res->raw_available = raw_hex[0] != '\0';
                aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
            }
            aim992_copy_cstr(res->status, sizeof(res->status),
                             negative ? "negative_response" : "no_data");
            aim992_copy_cstr(res->note, sizeof(res->note),
                             negative ? failure_note : "No positive response to DID 0x2B21 on 713->77D");
        }
    }

    if (fuel_result_idx >= 0 && !results[fuel_result_idx].has_value) {
        static const aim992_route_t fuel_fallback_routes[] = {
            { 0x7E0, 0x7E8 },
            { 0x7E1, 0x7E9 },
            { 0x7E4, 0x7EC },
        };

        uint8_t payload[AIM992_MAX_PAYLOAD_BYTES];
        int payload_len = 0;
        char raw_hex[AIM992_MAX_RESPONSE_HEX] = "";
        aim992_route_t matched_route = {0};
        char failure_note[160] = "";

        if (aim992_query_did(ctx, 0x5075, fuel_fallback_routes, 3,
                             payload, &payload_len, raw_hex, sizeof(raw_hex), &matched_route,
                             failure_note, sizeof(failure_note))) {
            aim992_result_t *res = &results[fuel_result_idx];
            res->tx_id = matched_route.tx_id;
            res->rx_id = matched_route.rx_id;
            res->raw_available = true;
            res->used_fallback = true;
            aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
            aim992_copy_cstr(res->transport, sizeof(res->transport), "uds_fallback");
            aim992_copy_cstr(res->request, sizeof(res->request), "22 50 75");
            aim992_copy_cstr(res->status, sizeof(res->status), "raw_only");
            aim992_copy_cstr(res->note, sizeof(res->note),
                             "01 2F returned no data; DID 0x5075 responded but scaling is not confirmed yet");
        } else if (failure_note[0]) {
            aim992_result_t *res = &results[fuel_result_idx];
            res->tx_id = matched_route.tx_id;
            res->rx_id = matched_route.rx_id;
            res->raw_available = raw_hex[0] != '\0';
            res->used_fallback = true;
            aim992_copy_cstr(res->raw_hex, sizeof(res->raw_hex), raw_hex);
            aim992_copy_cstr(res->transport, sizeof(res->transport), "uds_fallback");
            aim992_copy_cstr(res->request, sizeof(res->request), "22 50 75");
            aim992_copy_cstr(res->status, sizeof(res->status), "negative_response");
            aim992_copy_cstr(res->note, sizeof(res->note), failure_note);
        }
    }

    return true;
}

static int cli_p992_list(void) {
    aim992_xml_channel_t channels[AIM992_MAX_CHANNELS];
    int channel_count = 0;
    char xml_path[PATH_MAX];

    if (!aim992_load_xml_channels(channels, AIM992_MAX_CHANNELS, &channel_count,
                                  xml_path, sizeof(xml_path))) {
        fprintf(stderr, "[ERROR] Failed to load 992 XML profile\n");
        return 1;
    }

    printf("{\"profile\":\"p992\",\"xml_path\":");
    json_print_string(stdout, xml_path);
    printf(",\"channels\":[\n");
    for (int i = 0; i < channel_count; i++) {
        if (i > 0) printf(",\n");
        aim992_print_list_item(stdout, &channels[i],
                               aim992_find_spec_by_name(channels[i].long_name));
    }
    if (channel_count > 0) printf("\n");
    printf("]}\n");
    return 0;
}

static int cli_p992_read(const char *device, int name_argc, const char *name_argv[]) {
    aim992_xml_channel_t channels[AIM992_MAX_CHANNELS];
    int selected_indices[AIM992_MAX_CHANNELS];
    aim992_result_t results[AIM992_MAX_CHANNELS];
    int channel_count = 0;
    char xml_path[PATH_MAX];

    if (!aim992_load_xml_channels(channels, AIM992_MAX_CHANNELS, &channel_count,
                                  xml_path, sizeof(xml_path))) {
        fprintf(stderr, "[ERROR] Failed to load 992 XML profile\n");
        return 1;
    }

    int selected_count = aim992_select_channels(channels, channel_count,
                                                name_argc, name_argv,
                                                selected_indices, AIM992_MAX_CHANNELS);
    if (selected_count < 0) {
        fprintf(stderr, "[ERROR] Unknown 992 channel name\n");
        return 1;
    }

    int matchIdx = cli_auto_connect(device);
    aim992_query_ctx_t ctx = {0};

    if (!aim992_collect_snapshot(&ctx, channels, selected_indices, selected_count, results)) {
        fprintf(stderr, "[ERROR] Failed to collect 992 snapshot\n");
        [g_ble disconnect];
        return 1;
    }

    printf("{\"profile\":\"p992\",\"xml_path\":");
    json_print_string(stdout, xml_path);
    printf(",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"results\":[\n");
    for (int i = 0; i < selected_count; i++) {
        if (i > 0) printf(",\n");
        aim992_print_result_item(stdout, &results[i]);
    }
    if (selected_count > 0) printf("\n");
    printf("]}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_watch(const char *device,
                          int interval_ms,
                          int count,
                          int name_argc,
                          const char *name_argv[]) {
    aim992_xml_channel_t channels[AIM992_MAX_CHANNELS];
    int selected_indices[AIM992_MAX_CHANNELS];
    aim992_result_t results[AIM992_MAX_CHANNELS];
    int channel_count = 0;
    char xml_path[PATH_MAX];

    if (!aim992_load_xml_channels(channels, AIM992_MAX_CHANNELS, &channel_count,
                                  xml_path, sizeof(xml_path))) {
        fprintf(stderr, "[ERROR] Failed to load 992 XML profile\n");
        return 1;
    }

    int selected_count = aim992_select_channels(channels, channel_count,
                                                name_argc, name_argv,
                                                selected_indices, AIM992_MAX_CHANNELS);
    if (selected_count < 0) {
        fprintf(stderr, "[ERROR] Unknown 992 channel name\n");
        return 1;
    }

    int matchIdx = cli_auto_connect(device);
    aim992_query_ctx_t ctx = {0};
    int seq = 0;
    long long next_start_ms = get_timestamp_ms();

    while (g_ble.isConnected && !g_interrupt && (count <= 0 || seq < count)) {
        if (!aim992_collect_snapshot(&ctx, channels, selected_indices, selected_count, results)) {
            fprintf(stderr, "[ERROR] Failed to collect 992 snapshot\n");
            [g_ble disconnect];
            return 1;
        }

        printf("{\"profile\":\"p992\",\"xml_path\":");
        json_print_string(stdout, xml_path);
        printf(",\"device\":");
        json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
        printf(",\"sequence\":%d,\"timestamp_ms\":%lld,\"results\":[",
               seq + 1, (long long)get_timestamp_ms());
        for (int i = 0; i < selected_count; i++) {
            if (i > 0) printf(",");
            aim992_print_result_item(stdout, &results[i]);
        }
        printf("]}\n");
        fflush(stdout);

        seq++;
        if (count > 0 && seq >= count) break;

        next_start_ms += interval_ms;
        long long now_ms = get_timestamp_ms();
        if (interval_ms > 0 && next_start_ms > now_ms) {
            run_loop_for((next_start_ms - now_ms) / 1000.0);
        } else {
            next_start_ms = now_ms;
        }
    }

    [g_ble disconnect];
    g_interrupt = NO;
    return 0;
}

static void aim992_debug_command(bool *first, const char *stage, const char *cmd) {
    char resp[1024] = "";
    char hex[AIM992_MAX_RESPONSE_HEX] = "";
    bool received = [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];
    bool error = received && elm327_is_error_response(resp);

    if (received && !error) {
        elm327_extract_hex(resp, hex, sizeof(hex));
    }

    if (!*first) printf(",\n");
    *first = false;

    printf("  {\"stage\":");
    json_print_string(stdout, stage);
    printf(",\"cmd\":");
    json_print_string(stdout, cmd);
    printf(",\"received\":%s,\"ok\":%s,\"error\":%s",
           received ? "true" : "false",
           (received && !error) ? "true" : "false",
           error ? "true" : "false");
    printf(",\"resp\":");
    if (received) json_print_string(stdout, resp);
    else printf("null");
    printf(",\"hex\":");
    if (hex[0]) json_print_string(stdout, hex);
    else printf("null");
    printf("}");
}

static int cli_p992_debug(const char *device) {
    int matchIdx = cli_auto_connect(device);
    bool first = true;

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"steps\":[\n");

    aim992_debug_command(&first, "setup", "ATSP6");
    aim992_debug_command(&first, "setup", "ATAT1");
    aim992_debug_command(&first, "setup", "ATST FF");
    aim992_debug_command(&first, "setup", "ATAL");
    aim992_debug_command(&first, "setup", "ATS1");
    aim992_debug_command(&first, "setup", "ATH1");
    aim992_debug_command(&first, "setup", "ATCAF1");

    aim992_debug_command(&first, "mode01_functional_route", "ATSH 7DF");
    aim992_debug_command(&first, "mode01_functional_route", "ATCRA 7E8");
    aim992_debug_command(&first, "mode01_functional_route", "0100");
    aim992_debug_command(&first, "mode01_functional_route", "010C");
    aim992_debug_command(&first, "mode01_functional_route", "0111");
    aim992_debug_command(&first, "mode01_functional_route", "0149");
    aim992_debug_command(&first, "mode01_functional_route", "012F");

    aim992_debug_command(&first, "mode01_physical_route", "ATSH 7E0");
    aim992_debug_command(&first, "mode01_physical_route", "ATCRA 7E8");
    aim992_debug_command(&first, "mode01_physical_route", "0100");
    aim992_debug_command(&first, "mode01_physical_route", "010C");
    aim992_debug_command(&first, "mode01_physical_route", "0111");
    aim992_debug_command(&first, "mode01_physical_route", "0149");
    aim992_debug_command(&first, "mode01_physical_route", "012F");
    aim992_debug_command(&first, "uds_7e0_7e8", "225075");
    aim992_debug_command(&first, "uds_7e0_7e8", "225101");

    aim992_debug_command(&first, "uds_7e0_extended_session", "1003");
    aim992_debug_command(&first, "uds_7e0_extended_session", "3E00");
    aim992_debug_command(&first, "uds_7e0_extended_session", "225075");
    aim992_debug_command(&first, "uds_7e0_extended_session", "225101");

    aim992_debug_command(&first, "uds_70c_714", "ATSH 70C");
    aim992_debug_command(&first, "uds_70c_714", "ATCRA 714");
    aim992_debug_command(&first, "uds_70c_714", "225101");

    aim992_debug_command(&first, "uds_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "uds_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "uds_70c_73b", "225101");

    aim992_debug_command(&first, "uds_70c_auto_receive", "ATAR");
    aim992_debug_command(&first, "uds_70c_auto_receive", "ATSH 70C");
    aim992_debug_command(&first, "uds_70c_auto_receive", "225101");

    aim992_debug_command(&first, "uds_70c_extended_session", "1003");
    aim992_debug_command(&first, "uds_70c_extended_session", "3E00");
    aim992_debug_command(&first, "uds_70c_extended_session", "225101");

    aim992_debug_command(&first, "uds_73b_70c", "ATSH 73B");
    aim992_debug_command(&first, "uds_73b_70c", "ATCRA 70C");
    aim992_debug_command(&first, "uds_73b_70c", "225101");

    aim992_debug_command(&first, "uds_73b_auto_receive", "ATAR");
    aim992_debug_command(&first, "uds_73b_auto_receive", "ATSH 73B");
    aim992_debug_command(&first, "uds_73b_auto_receive", "225101");

    aim992_debug_command(&first, "bin_raw_container_setup", "ATCAF0");
    aim992_debug_command(&first, "bin_raw_container_7e0_7e8", "ATSH 7E0");
    aim992_debug_command(&first, "bin_raw_container_7e0_7e8", "ATCRA 7E8");
    aim992_debug_command(&first, "bin_raw_container_7e0_7e8", "0322507500000000");
    aim992_debug_command(&first, "bin_raw_container_7e0_7e8", "0322510100000000");

    aim992_debug_command(&first, "bin_raw_container_70c_714", "ATSH 70C");
    aim992_debug_command(&first, "bin_raw_container_70c_714", "ATCRA 714");
    aim992_debug_command(&first, "bin_raw_container_70c_714", "0322510100000000");

    aim992_debug_command(&first, "bin_raw_container_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "bin_raw_container_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "bin_raw_container_70c_73b", "0322510100000000");

    aim992_debug_command(&first, "bin_raw_container_70c_auto_receive", "ATAR");
    aim992_debug_command(&first, "bin_raw_container_70c_auto_receive", "ATSH 70C");
    aim992_debug_command(&first, "bin_raw_container_70c_auto_receive", "0322510100000000");

    aim992_debug_command(&first, "bin_raw_container_73b_70c", "ATSH 73B");
    aim992_debug_command(&first, "bin_raw_container_73b_70c", "ATCRA 70C");
    aim992_debug_command(&first, "bin_raw_container_73b_70c", "0322510100000000");

    aim992_debug_command(&first, "bin_raw_container_73b_auto_receive", "ATAR");
    aim992_debug_command(&first, "bin_raw_container_73b_auto_receive", "ATSH 73B");
    aim992_debug_command(&first, "bin_raw_container_73b_auto_receive", "0322510100000000");

    printf("\n]}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_bin_probe(const char *device) {
    int matchIdx = cli_auto_connect(device);
    bool first = true;

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"probe\":\"bin_tail_exact\",\"steps\":[\n");

    aim992_debug_command(&first, "setup", "ATSP6");
    aim992_debug_command(&first, "setup", "ATAT0");
    aim992_debug_command(&first, "setup", "ATST FF");
    aim992_debug_command(&first, "setup", "ATAL");
    aim992_debug_command(&first, "setup", "ATS1");
    aim992_debug_command(&first, "setup", "ATH1");

    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATCAF1");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATFCSH70C");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATFCSD300002");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATFCSM1");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "caf1_flow_control_70c_73b", "225101");

    aim992_debug_command(&first, "caf1_auto_receive_70c", "ATAR");
    aim992_debug_command(&first, "caf1_auto_receive_70c", "ATSH 70C");
    aim992_debug_command(&first, "caf1_auto_receive_70c", "225101");

    aim992_debug_command(&first, "caf1_extended_addr_70c_73b", "ATCEA 00");
    aim992_debug_command(&first, "caf1_extended_addr_70c_73b", "ATCAF1");
    aim992_debug_command(&first, "caf1_extended_addr_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "caf1_extended_addr_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "caf1_extended_addr_70c_73b", "225101");
    aim992_debug_command(&first, "caf1_extended_addr_70c_auto", "ATAR");
    aim992_debug_command(&first, "caf1_extended_addr_70c_auto", "ATSH 70C");
    aim992_debug_command(&first, "caf1_extended_addr_70c_auto", "225101");
    aim992_debug_command(&first, "caf1_extended_addr_off", "ATCEA");

    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "ATCAF0");
    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "03225101AAAAAAAA");
    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "0003225101AAAAAA");
    aim992_debug_command(&first, "caf0_bin_padding_70c_73b", "0322510100000000");

    aim992_debug_command(&first, "caf0_bin_padding_70c_auto", "ATAR");
    aim992_debug_command(&first, "caf0_bin_padding_70c_auto", "ATSH 70C");
    aim992_debug_command(&first, "caf0_bin_padding_70c_auto", "03225101AAAAAAAA");
    aim992_debug_command(&first, "caf0_bin_padding_70c_auto", "0003225101AAAAAA");

    aim992_debug_command(&first, "caf0_bin_padding_7e0_7e8", "ATSH 7E0");
    aim992_debug_command(&first, "caf0_bin_padding_7e0_7e8", "ATCRA 7E8");
    aim992_debug_command(&first, "caf0_bin_padding_7e0_7e8", "03225075AAAAAAAA");
    aim992_debug_command(&first, "caf0_bin_padding_7e0_7e8", "03225101AAAAAAAA");

    aim992_debug_command(&first, "hybrid_route_7e0_73b", "ATCAF1");
    aim992_debug_command(&first, "hybrid_route_7e0_73b", "ATSH 7E0");
    aim992_debug_command(&first, "hybrid_route_7e0_73b", "ATCRA 73B");
    aim992_debug_command(&first, "hybrid_route_7e0_73b", "225075");
    aim992_debug_command(&first, "hybrid_route_7e0_73b", "225101");

    aim992_debug_command(&first, "hybrid_route_70c_73b", "ATCAF1");
    aim992_debug_command(&first, "hybrid_route_70c_73b", "ATSH 70C");
    aim992_debug_command(&first, "hybrid_route_70c_73b", "ATCRA 73B");
    aim992_debug_command(&first, "hybrid_route_70c_73b", "225075");
    aim992_debug_command(&first, "hybrid_route_70c_73b", "225101");

    aim992_debug_command(&first, "caf0_hybrid_route_7e0_73b", "ATCAF0");
    aim992_debug_command(&first, "caf0_hybrid_route_7e0_73b", "ATSH 7E0");
    aim992_debug_command(&first, "caf0_hybrid_route_7e0_73b", "ATCRA 73B");
    aim992_debug_command(&first, "caf0_hybrid_route_7e0_73b", "03225075AAAAAAAA");
    aim992_debug_command(&first, "caf0_hybrid_route_7e0_73b", "03225101AAAAAAAA");

    printf("\n]}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_scan_did(const char *device, uint16_t did, int session) {
    int matchIdx = cli_auto_connect(device);
    char resp[512];
    bool first = true;

    [g_ble sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAT0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATST 10" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAL" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAR" response:resp maxLen:sizeof(resp)];

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"did\":\"%04X\"", did);
    if (session >= 0) printf(",\"session\":\"%02X\"", session & 0xFF);
    printf(",\"responders\":[\n");

    for (uint32_t tx = 0x700; tx <= 0x7EF; tx++) {
        char cmd[32];
        char query[16];
        char hex[AIM992_MAX_RESPONSE_HEX] = "";
        char session_resp[512] = "";
        char session_hex[AIM992_MAX_RESPONSE_HEX] = "";
        bool session_received = false;
        bool session_error = false;
        bool session_positive = false;
        bool session_negative = false;
        uint8_t session_nrc = 0;

        snprintf(cmd, sizeof(cmd), "ATSH %03X", tx);
        if (![g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
            elm327_is_error_response(resp)) {
            continue;
        }

        if (session >= 0) {
            char session_cmd[16];
            snprintf(session_cmd, sizeof(session_cmd), "10%02X", session & 0xFF);
            session_received = [g_ble sendCommand:session_cmd
                                         response:session_resp
                                           maxLen:sizeof(session_resp)];
            session_error = session_received && elm327_is_error_response(session_resp);
            if (session_received && !session_error) {
                elm327_extract_hex(session_resp, session_hex, sizeof(session_hex));
                session_positive =
                    aim992_find_uds_positive_session_response(session_hex, (uint8_t)session);
                session_negative =
                    aim992_find_uds_negative_response(session_hex, 0x10, &session_nrc);
            }
        }

        snprintf(query, sizeof(query), "22%02X%02X", did >> 8, did & 0xFF);
        if (![g_ble sendCommand:query response:resp maxLen:sizeof(resp)] ||
            elm327_is_error_response(resp)) {
            continue;
        }

        elm327_extract_hex(resp, hex, sizeof(hex));
        bool positive = aim992_find_uds_positive_did_response(hex, did);
        uint8_t nrc = 0;
        bool negative = aim992_find_uds_negative_response(hex, 0x22, &nrc);
        if (!first) printf(",\n");
        first = false;

        printf("  {\"tx_id\":\"%03X\",\"resp\":", tx);
        json_print_string(stdout, resp);
        printf(",\"hex\":");
        if (hex[0]) json_print_string(stdout, hex);
        else printf("null");
        if (session >= 0) {
            printf(",\"session_resp\":");
            if (session_received) json_print_string(stdout, session_resp);
            else printf("null");
            printf(",\"session_hex\":");
            if (session_hex[0]) json_print_string(stdout, session_hex);
            else printf("null");
            printf(",\"session_status\":");
            json_print_string(stdout,
                              session_positive ? "positive" :
                              (session_negative ? "negative" :
                               (session_error ? "adapter_error" : "unknown")));
            if (session_negative) {
                printf(",\"session_nrc\":\"%02X\",\"session_nrc_name\":", session_nrc);
                json_print_string(stdout, aim992_uds_nrc_name(session_nrc));
            }
        }
        printf(",\"uds_status\":");
        json_print_string(stdout, positive ? "positive" : (negative ? "negative" : "unknown"));
        printf(",\"positive\":%s", positive ? "true" : "false");
        if (negative) {
            printf(",\"nrc\":\"%02X\",\"nrc_name\":", nrc);
            json_print_string(stdout, aim992_uds_nrc_name(nrc));
        }
        printf("}");
        fflush(stdout);
    }

    if (!first) printf("\n");
    printf("]}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_scan_did_range(const char *device, uint16_t start_did, uint16_t end_did) {
    static const aim992_route_t routes[] = {
        { 0x7E0, 0x7E8 },
        { 0x7E1, 0x7E9 },
        { 0x7E4, 0x7EC },
    };

    if (end_did < start_did) {
        uint16_t tmp = start_did;
        start_did = end_did;
        end_did = tmp;
    }

    int matchIdx = cli_auto_connect(device);
    char resp[512];
    bool first = true;

    [g_ble sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAT0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATST 10" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAL" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF1" response:resp maxLen:sizeof(resp)];

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"start_did\":\"%04X\",\"end_did\":\"%04X\",\"responses\":[\n",
           start_did, end_did);

    for (uint32_t did = start_did; did <= end_did; did++) {
        for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
            char cmd[32];
            char query[16];
            char hex[AIM992_MAX_RESPONSE_HEX] = "";

            snprintf(cmd, sizeof(cmd), "ATSH %03X", routes[i].tx_id);
            if (![g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
                elm327_is_error_response(resp)) {
                continue;
            }

            snprintf(cmd, sizeof(cmd), "ATCRA %03X", routes[i].rx_id);
            if (![g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
                elm327_is_error_response(resp)) {
                continue;
            }

            snprintf(query, sizeof(query), "22%02X%02X", did >> 8, did & 0xFF);
            if (![g_ble sendCommand:query response:resp maxLen:sizeof(resp)] ||
                elm327_is_error_response(resp)) {
                continue;
            }

            elm327_extract_hex(resp, hex, sizeof(hex));
            bool positive = aim992_find_uds_positive_did_response(hex, (uint16_t)did);
            uint8_t nrc = 0;
            bool negative = aim992_find_uds_negative_response(hex, 0x22, &nrc);

            if (!positive && !negative) continue;

            if (!first) printf(",\n");
            first = false;

            printf("  {\"did\":\"%04X\",\"tx_id\":\"%03X\",\"rx_id\":\"%03X\",\"resp\":",
                   (unsigned)did, routes[i].tx_id, routes[i].rx_id);
            json_print_string(stdout, resp);
            printf(",\"hex\":");
            if (hex[0]) json_print_string(stdout, hex);
            else printf("null");
            printf(",\"uds_status\":");
            json_print_string(stdout, positive ? "positive" : "negative");
            printf(",\"positive\":%s", positive ? "true" : "false");
            if (negative) {
                printf(",\"nrc\":\"%02X\",\"nrc_name\":", nrc);
                json_print_string(stdout, aim992_uds_nrc_name(nrc));
            }
            printf("}");
            fflush(stdout);
        }

        if (did == 0xFFFF) break;
    }

    if (!first) printf("\n");
    printf("]}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_uds_read(const char *device,
                             uint16_t did,
                             uint32_t tx_id,
                             int rx_id,
                             int session,
                             const char *pre_request_hex) {
    int matchIdx = cli_auto_connect(device);
    char resp[512];
    char cmd[32];

    [g_ble sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAT0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATST FF" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAL" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF1" response:resp maxLen:sizeof(resp)];

    snprintf(cmd, sizeof(cmd), "ATSH %03X", tx_id);
    [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];

    if (rx_id >= 0) {
        snprintf(cmd, sizeof(cmd), "ATCRA %03X", rx_id);
        [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];
    } else {
        [g_ble sendCommand:"ATAR" response:resp maxLen:sizeof(resp)];
    }

    aim992_route_t uds_route = { tx_id, rx_id >= 0 ? (uint32_t)rx_id : UINT32_MAX };
    aim992_configure_flow_control(uds_route);

    char session_resp[512] = "";
    char session_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char session_payload_hex[AIM992_MAX_RESPONSE_HEX] = "";
    bool session_received = false;
    bool session_error = false;
    bool session_positive = false;
    bool session_negative = false;
    uint8_t session_nrc = 0;
    uint8_t session_payload[AIM992_MAX_PAYLOAD_BYTES];
    int session_payload_len = 0;
    uint32_t session_resp_id = UINT32_MAX;

    if (session >= 0) {
        snprintf(cmd, sizeof(cmd), "10%02X", session & 0xFF);
        session_received = [g_ble sendCommand:cmd
                                     response:session_resp
                                       maxLen:sizeof(session_resp)];
        session_error = session_received && elm327_is_error_response(session_resp);
        if (session_received && !session_error) {
            elm327_extract_hex(session_resp, session_hex, sizeof(session_hex));
            if (aim992_parse_isotp_payload(session_resp, session_payload,
                                           AIM992_MAX_PAYLOAD_BYTES,
                                           &session_payload_len,
                                           &session_resp_id)) {
                aim992_format_payload_hex(session_payload, session_payload_len,
                                          session_payload_hex, sizeof(session_payload_hex));
                session_positive =
                    session_payload_len >= 2 &&
                    session_payload[0] == 0x50 &&
                    session_payload[1] == (uint8_t)session;
                session_negative =
                    session_payload_len >= 3 &&
                    session_payload[0] == 0x7F &&
                    session_payload[1] == 0x10;
                if (session_negative) session_nrc = session_payload[2];
            } else {
                session_positive =
                    aim992_find_uds_positive_session_response(session_hex, (uint8_t)session);
                session_negative =
                    aim992_find_uds_negative_response(session_hex, 0x10, &session_nrc);
            }
        }
    }

    char query_resp[1024] = "";
    char query_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char query_payload_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char text[128] = "";
    char pre_resp[512] = "";
    char pre_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char pre_payload_hex[AIM992_MAX_RESPONSE_HEX] = "";
    bool query_received = false;
    bool query_error = false;
    bool positive = false;
    bool negative = false;
    bool pre_received = false;
    bool pre_error = false;
    bool pre_positive = false;
    bool pre_negative = false;
    uint8_t nrc = 0;
    uint8_t pre_nrc = 0;
    uint8_t query_payload[128];
    uint8_t pre_payload[128];
    int query_payload_len = 0;
    int pre_payload_len = 0;
    uint32_t query_resp_id = UINT32_MAX;
    uint32_t pre_resp_id = UINT32_MAX;

    if (pre_request_hex && pre_request_hex[0]) {
        uint8_t pre_service_id = 0;
        if (strlen(pre_request_hex) >= 2 &&
            (strlen(pre_request_hex) % 2) == 0 &&
            aim992_parse_hex_byte(pre_request_hex, &pre_service_id)) {
            snprintf(cmd, sizeof(cmd), "%s", pre_request_hex);
            pre_received = [g_ble sendCommand:cmd response:pre_resp maxLen:sizeof(pre_resp)];
            pre_error = pre_received && elm327_is_error_response(pre_resp);
            if (pre_received && !pre_error) {
                elm327_extract_hex(pre_resp, pre_hex, sizeof(pre_hex));
                if (aim992_parse_isotp_payload(pre_resp, pre_payload, sizeof(pre_payload),
                                               &pre_payload_len, &pre_resp_id)) {
                    aim992_format_payload_hex(pre_payload, pre_payload_len,
                                              pre_payload_hex, sizeof(pre_payload_hex));
                    pre_positive =
                        pre_payload_len >= 1 &&
                        pre_payload[0] == (uint8_t)(pre_service_id + 0x40);
                    pre_negative =
                        pre_payload_len >= 3 &&
                        pre_payload[0] == 0x7F &&
                        pre_payload[1] == pre_service_id;
                    if (pre_negative) pre_nrc = pre_payload[2];
                } else {
                    char positive_marker[8];
                    snprintf(positive_marker, sizeof(positive_marker), "%02X",
                             pre_service_id + 0x40);
                    pre_positive = strstr(pre_hex, positive_marker) != NULL;
                    pre_negative =
                        aim992_find_uds_negative_response(pre_hex, pre_service_id, &pre_nrc);
                }
            }
        }
    }

    snprintf(cmd, sizeof(cmd), "22%02X%02X", did >> 8, did & 0xFF);
    query_received = [g_ble sendCommand:cmd response:query_resp maxLen:sizeof(query_resp)];
    query_error = query_received && elm327_is_error_response(query_resp);
    if (query_received && !query_error) {
        elm327_extract_hex(query_resp, query_hex, sizeof(query_hex));
        if (aim992_parse_isotp_payload(query_resp, query_payload, sizeof(query_payload),
                                       &query_payload_len, &query_resp_id)) {
            aim992_format_payload_hex(query_payload, query_payload_len,
                                      query_payload_hex, sizeof(query_payload_hex));
            positive =
                query_payload_len >= 3 &&
                query_payload[0] == 0x62 &&
                query_payload[1] == (uint8_t)(did >> 8) &&
                query_payload[2] == (uint8_t)(did & 0xFF);
            negative =
                query_payload_len >= 3 &&
                query_payload[0] == 0x7F &&
                query_payload[1] == 0x22;
            if (negative) nrc = query_payload[2];
            if (positive) {
                aim992_payload_ascii(query_payload, query_payload_len, 3,
                                     text, sizeof(text));
            }
        } else {
            positive = aim992_find_uds_positive_did_response(query_hex, did);
            negative = aim992_find_uds_negative_response(query_hex, 0x22, &nrc);
        }
    }

    const char *status = "unknown";
    if (!query_received) status = "timeout";
    else if (query_error) status = strstr(query_resp, "NO DATA") ? "no_data" : "adapter_error";
    else if (positive) status = "positive";
    else if (negative) status = "negative";

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"did\":\"%04X\",\"tx_id\":\"%03X\",\"rx_id\":", did, tx_id);
    if (rx_id >= 0) {
        char rx_id_str[8];
        snprintf(rx_id_str, sizeof(rx_id_str), "%03X", rx_id);
        json_print_string(stdout, rx_id_str);
    } else {
        printf("null");
    }
    if (session >= 0) printf(",\"session\":\"%02X\"", session & 0xFF);
    printf(",\"status\":");
    json_print_string(stdout, status);
    printf(",\"resp\":");
    if (query_received) json_print_string(stdout, query_resp);
    else printf("null");
    printf(",\"hex\":");
    if (query_hex[0]) json_print_string(stdout, query_hex);
    else printf("null");
    printf(",\"resp_id\":");
    if (query_resp_id <= 0x7FF) {
        char resp_id_str[8];
        snprintf(resp_id_str, sizeof(resp_id_str), "%03X", query_resp_id);
        json_print_string(stdout, resp_id_str);
    } else {
        printf("null");
    }
    printf(",\"payload_hex\":");
    if (query_payload_hex[0]) json_print_string(stdout, query_payload_hex);
    else printf("null");
    printf(",\"positive\":%s", positive ? "true" : "false");
    if (negative) {
        printf(",\"nrc\":\"%02X\",\"nrc_name\":", nrc);
        json_print_string(stdout, aim992_uds_nrc_name(nrc));
    }
    printf(",\"text\":");
    if (text[0]) json_print_string(stdout, text);
    else printf("null");
    if (session >= 0) {
        printf(",\"session_resp\":");
        if (session_received) json_print_string(stdout, session_resp);
        else printf("null");
        printf(",\"session_hex\":");
        if (session_hex[0]) json_print_string(stdout, session_hex);
        else printf("null");
        printf(",\"session_resp_id\":");
        if (session_resp_id <= 0x7FF) {
            char session_resp_id_str[8];
            snprintf(session_resp_id_str, sizeof(session_resp_id_str), "%03X", session_resp_id);
            json_print_string(stdout, session_resp_id_str);
        } else {
            printf("null");
        }
        printf(",\"session_payload_hex\":");
        if (session_payload_hex[0]) json_print_string(stdout, session_payload_hex);
        else printf("null");
        printf(",\"session_status\":");
        if (!session_received) json_print_string(stdout, "timeout");
        else if (session_error) json_print_string(stdout, strstr(session_resp, "NO DATA") ? "no_data" : "adapter_error");
        else if (session_positive) json_print_string(stdout, "positive");
        else if (session_negative) json_print_string(stdout, "negative");
        else json_print_string(stdout, "unknown");
        if (session_negative) {
            printf(",\"session_nrc\":\"%02X\",\"session_nrc_name\":", session_nrc);
            json_print_string(stdout, aim992_uds_nrc_name(session_nrc));
        }
    }
    if (pre_request_hex && pre_request_hex[0]) {
        printf(",\"pre_request\":");
        json_print_string(stdout, pre_request_hex);
        printf(",\"pre_resp\":");
        if (pre_received) json_print_string(stdout, pre_resp);
        else printf("null");
        printf(",\"pre_hex\":");
        if (pre_hex[0]) json_print_string(stdout, pre_hex);
        else printf("null");
        printf(",\"pre_resp_id\":");
        if (pre_resp_id <= 0x7FF) {
            char pre_resp_id_str[8];
            snprintf(pre_resp_id_str, sizeof(pre_resp_id_str), "%03X", pre_resp_id);
            json_print_string(stdout, pre_resp_id_str);
        } else {
            printf("null");
        }
        printf(",\"pre_payload_hex\":");
        if (pre_payload_hex[0]) json_print_string(stdout, pre_payload_hex);
        else printf("null");
        printf(",\"pre_status\":");
        if (!pre_received) json_print_string(stdout, "timeout");
        else if (pre_error) json_print_string(stdout, strstr(pre_resp, "NO DATA") ? "no_data" : "adapter_error");
        else if (pre_positive) json_print_string(stdout, "positive");
        else if (pre_negative) json_print_string(stdout, "negative");
        else json_print_string(stdout, "unknown");
        if (pre_negative) {
            printf(",\"pre_nrc\":\"%02X\",\"pre_nrc_name\":", pre_nrc);
            json_print_string(stdout, aim992_uds_nrc_name(pre_nrc));
        }
    }
    printf("}\n");

    [g_ble disconnect];
    return 0;
}

static int cli_p992_uds_raw(const char *device,
                            const char *request_hex,
                            uint32_t tx_id,
                            int rx_id,
                            int session) {
    uint8_t service_id = 0;
    if (!request_hex || strlen(request_hex) < 2 ||
        (strlen(request_hex) % 2) != 0 ||
        !aim992_parse_hex_byte(request_hex, &service_id)) {
        fprintf(stderr, "[ERROR] Invalid UDS request hex: %s\n",
                request_hex ? request_hex : "(null)");
        return 1;
    }

    int matchIdx = cli_auto_connect(device);
    char resp[512];
    char cmd[64];

    [g_ble sendCommand:"ATSP6" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAT0" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATST FF" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATAL" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATS1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATH1" response:resp maxLen:sizeof(resp)];
    [g_ble sendCommand:"ATCAF1" response:resp maxLen:sizeof(resp)];

    snprintf(cmd, sizeof(cmd), "ATSH %03X", tx_id);
    [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];

    if (rx_id >= 0) {
        snprintf(cmd, sizeof(cmd), "ATCRA %03X", rx_id);
        [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];
    } else {
        [g_ble sendCommand:"ATAR" response:resp maxLen:sizeof(resp)];
    }

    char session_resp[512] = "";
    char session_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char session_payload_hex[AIM992_MAX_RESPONSE_HEX] = "";
    bool session_received = false;
    bool session_error = false;
    bool session_positive = false;
    bool session_negative = false;
    uint8_t session_nrc = 0;
    uint8_t session_payload[AIM992_MAX_PAYLOAD_BYTES];
    int session_payload_len = 0;
    uint32_t session_resp_id = UINT32_MAX;

    if (session >= 0) {
        snprintf(cmd, sizeof(cmd), "10%02X", session & 0xFF);
        session_received = [g_ble sendCommand:cmd
                                     response:session_resp
                                       maxLen:sizeof(session_resp)];
        session_error = session_received && elm327_is_error_response(session_resp);
        if (session_received && !session_error) {
            elm327_extract_hex(session_resp, session_hex, sizeof(session_hex));
            if (aim992_parse_isotp_payload(session_resp, session_payload,
                                           AIM992_MAX_PAYLOAD_BYTES,
                                           &session_payload_len,
                                           &session_resp_id)) {
                aim992_format_payload_hex(session_payload, session_payload_len,
                                          session_payload_hex, sizeof(session_payload_hex));
                session_positive =
                    session_payload_len >= 2 &&
                    session_payload[0] == 0x50 &&
                    session_payload[1] == (uint8_t)session;
                session_negative =
                    session_payload_len >= 3 &&
                    session_payload[0] == 0x7F &&
                    session_payload[1] == 0x10;
                if (session_negative) session_nrc = session_payload[2];
            } else {
                session_positive =
                    aim992_find_uds_positive_session_response(session_hex, (uint8_t)session);
                session_negative =
                    aim992_find_uds_negative_response(session_hex, 0x10, &session_nrc);
            }
        }
    }

    char query_resp[1024] = "";
    char query_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char query_payload_hex[AIM992_MAX_RESPONSE_HEX] = "";
    char text[128] = "";
    bool query_received = false;
    bool query_error = false;
    bool positive = false;
    bool negative = false;
    uint8_t nrc = 0;
    uint8_t query_payload[128];
    int query_payload_len = 0;
    uint32_t query_resp_id = UINT32_MAX;

    aim992_copy_cstr(cmd, sizeof(cmd), request_hex);
    query_received = [g_ble sendCommand:cmd response:query_resp maxLen:sizeof(query_resp)];
    query_error = query_received && elm327_is_error_response(query_resp);
    if (query_received && !query_error) {
        elm327_extract_hex(query_resp, query_hex, sizeof(query_hex));
        if (aim992_parse_isotp_payload(query_resp, query_payload, sizeof(query_payload),
                                       &query_payload_len, &query_resp_id)) {
            aim992_format_payload_hex(query_payload, query_payload_len,
                                      query_payload_hex, sizeof(query_payload_hex));
            positive =
                query_payload_len >= 1 &&
                query_payload[0] == (uint8_t)(service_id + 0x40);
            negative =
                query_payload_len >= 3 &&
                query_payload[0] == 0x7F &&
                query_payload[1] == service_id;
            if (negative) nrc = query_payload[2];
            if (positive) {
                aim992_payload_ascii(query_payload, query_payload_len, 1,
                                     text, sizeof(text));
            }
        } else {
            char positive_marker[8];
            snprintf(positive_marker, sizeof(positive_marker), "%02X", service_id + 0x40);
            positive = strstr(query_hex, positive_marker) != NULL;
            negative = aim992_find_uds_negative_response(query_hex, service_id, &nrc);
        }
    }

    const char *status = "unknown";
    if (!query_received) status = "timeout";
    else if (query_error) status = strstr(query_resp, "NO DATA") ? "no_data" : "adapter_error";
    else if (positive) status = "positive";
    else if (negative) status = "negative";

    printf("{\"profile\":\"p992\",\"device\":");
    json_print_string(stdout, g_ble.discoveredNames[matchIdx].UTF8String);
    printf(",\"request\":");
    json_print_string(stdout, request_hex);
    printf(",\"service_id\":\"%02X\",\"tx_id\":\"%03X\",\"rx_id\":", service_id, tx_id);
    if (rx_id >= 0) {
        char rx_id_str[8];
        snprintf(rx_id_str, sizeof(rx_id_str), "%03X", rx_id);
        json_print_string(stdout, rx_id_str);
    } else {
        printf("null");
    }
    if (session >= 0) printf(",\"session\":\"%02X\"", session & 0xFF);
    printf(",\"status\":");
    json_print_string(stdout, status);
    printf(",\"resp\":");
    if (query_received) json_print_string(stdout, query_resp);
    else printf("null");
    printf(",\"hex\":");
    if (query_hex[0]) json_print_string(stdout, query_hex);
    else printf("null");
    printf(",\"resp_id\":");
    if (query_resp_id <= 0x7FF) {
        char resp_id_str[8];
        snprintf(resp_id_str, sizeof(resp_id_str), "%03X", query_resp_id);
        json_print_string(stdout, resp_id_str);
    } else {
        printf("null");
    }
    printf(",\"payload_hex\":");
    if (query_payload_hex[0]) json_print_string(stdout, query_payload_hex);
    else printf("null");
    printf(",\"positive\":%s", positive ? "true" : "false");
    if (negative) {
        printf(",\"nrc\":\"%02X\",\"nrc_name\":", nrc);
        json_print_string(stdout, aim992_uds_nrc_name(nrc));
    }
    printf(",\"text\":");
    if (text[0]) json_print_string(stdout, text);
    else printf("null");
    if (session >= 0) {
        printf(",\"session_resp\":");
        if (session_received) json_print_string(stdout, session_resp);
        else printf("null");
        printf(",\"session_hex\":");
        if (session_hex[0]) json_print_string(stdout, session_hex);
        else printf("null");
        printf(",\"session_resp_id\":");
        if (session_resp_id <= 0x7FF) {
            char session_resp_id_str[8];
            snprintf(session_resp_id_str, sizeof(session_resp_id_str), "%03X", session_resp_id);
            json_print_string(stdout, session_resp_id_str);
        } else {
            printf("null");
        }
        printf(",\"session_payload_hex\":");
        if (session_payload_hex[0]) json_print_string(stdout, session_payload_hex);
        else printf("null");
        printf(",\"session_status\":");
        if (!session_received) json_print_string(stdout, "timeout");
        else if (session_error) json_print_string(stdout, strstr(session_resp, "NO DATA") ? "no_data" : "adapter_error");
        else if (session_positive) json_print_string(stdout, "positive");
        else if (session_negative) json_print_string(stdout, "negative");
        else json_print_string(stdout, "unknown");
        if (session_negative) {
            printf(",\"session_nrc\":\"%02X\",\"session_nrc_name\":", session_nrc);
            json_print_string(stdout, aim992_uds_nrc_name(session_nrc));
        }
    }
    printf("}\n");

    [g_ble disconnect];
    return 0;
}

typedef struct {
    const char *name;
    uint32_t    tx_id;
    const char *payload_hex;
} aim992_fastlog_request_t;

static const aim992_fastlog_request_t s_aim992_fastlog_raw_requests[] = {
    { "RPM",          0x7DF, "02010C5555555555" },
    { "THROTTLE_POS", 0x7DF, "0201115555555555" },
    { "ACCEL_POS",    0x7DF, "0201495555555555" },
    { "STEER_ANGLE",  0x70C, "03225075AAAAAAAA" },
    { "STEER_FC",     0x70C, "300001AAAAAAAAAA" },
    { "BRAKE_PRESS",  0x713, "03222B21AAAAAAAA" },
};

static const int s_aim992_fastlog_raw_request_count =
    sizeof(s_aim992_fastlog_raw_requests) / sizeof(s_aim992_fastlog_raw_requests[0]);

static bool cli_contains_case_insensitive(const char *s, const char *needle) {
    if (!s || !needle || !*needle) return false;
    size_t needle_len = strlen(needle);
    for (const char *p = s; *p; p++) {
        if (strncasecmp(p, needle, needle_len) == 0) return true;
    }
    return false;
}

static bool cli_send_required_command(const char *cmd) {
    char resp[512] = "";
    if (![g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)] ||
        elm327_is_error_response(resp)) {
        fprintf(stderr, "[ERROR] Command failed: %s", cmd);
        if (resp[0]) fprintf(stderr, " => %s", resp);
        fprintf(stderr, "\n");
        return false;
    }
    return true;
}

static void cli_send_best_effort_command(const char *cmd) {
    char resp[512] = "";
    [g_ble sendCommand:cmd response:resp maxLen:sizeof(resp)];
}

static void cli_p992_fastlog_cleanup(void) {
    cli_send_best_effort_command("STPPMC");
    cli_send_best_effort_command("STCMM 0");
}

static int cli_p992_fastlog_write_json(const char *out_path,
                                       const char *device_name,
                                       int seconds,
                                       int period_ms,
                                       const char *adapter_mode,
                                       const char *monitor_cmd,
                                       const aim992_fastlog_request_t requests[],
                                       int request_count) {
    FILE *json_out = stdout;
    if (out_path && *out_path) {
        json_out = fopen(out_path, "w");
        if (!json_out) {
            fprintf(stderr, "[ERROR] Failed to open fastlog output: %s\n", out_path);
            return 1;
        }
    }

    fprintf(json_out, "{\"profile\":\"p992\",\"mode\":\"fastlog_periodic\",\"adapter_mode\":");
    json_print_string(json_out, adapter_mode);
    fprintf(json_out, ",\"device\":");
    json_print_string(json_out, device_name);
    fprintf(json_out, ",\"duration_s\":%d,\"period_ms\":%d", seconds, period_ms);
    fprintf(json_out, ",\"monitor_command\":");
    json_print_string(json_out, monitor_cmd);
    fprintf(json_out, ",\"schedule\":[");
    for (int i = 0; i < request_count; i++) {
        const aim992_fastlog_request_t *req = &requests[i];
        if (i > 0) fprintf(json_out, ",");
        fprintf(json_out, "{\"name\":");
        json_print_string(json_out, req->name);
        fprintf(json_out, ",\"tx_id\":\"%03X\",\"payload\":", req->tx_id);
        json_print_string(json_out, req->payload_hex);
        fprintf(json_out, "}");
    }
    fprintf(json_out, "],\"frames\":[\n");

    for (int i = 0; i < g_cli_capture.count; i++) {
        if (i > 0) fprintf(json_out, ",\n");
        fprintf(json_out,
                "  {\"timestamp_ms\":%lld,\"id\":\"%03X\",\"is_extended\":%s,\"dlc\":%d,\"data\":\"",
                (long long)g_cli_capture.frames[i].timestamp_ms,
                g_cli_capture.frames[i].can_id,
                g_cli_capture.frames[i].is_extended ? "true" : "false",
                g_cli_capture.frames[i].dlc);
        for (int j = 0; j < g_cli_capture.frames[i].dlc; j++) {
            if (j > 0) fprintf(json_out, " ");
            fprintf(json_out, "%02X", g_cli_capture.frames[i].data[j]);
        }
        fprintf(json_out, "\"}");
    }

    if (g_cli_capture.count > 0) fprintf(json_out, "\n");
    fprintf(json_out, "]}\n");

    if (json_out != stdout) {
        fclose(json_out);
        cli_log("Wrote fastlog JSON: %s", out_path);
    }
    return 0;
}

static int cli_p992_fastlog(const char *device,
                            int seconds,
                            int period_ms,
                            const char *monitor_cmd,
                            const char *out_path) {
    if (seconds <= 0) seconds = 10;
    if (period_ms < 20) period_ms = 20;

    int matchIdx = cli_ble_connect(device);
    const char *connected_name = g_ble.discoveredNames[matchIdx].UTF8String;
    bool use_vlinker = cli_contains_case_insensitive(device, "vlinker") ||
                       cli_contains_case_insensitive(connected_name, "vlinker");
    if (use_vlinker) {
        fprintf(stderr, "[ERROR] vLinker periodic fastlog is unsupported: "
                "ATMA/STM/STMA monitoring mode is receive-only and disables PM transmissions\n");
        [g_ble disconnect];
        return 1;
    }
    const char *adapter_mode = "stn_raw_periodic";
    const aim992_fastlog_request_t *requests = s_aim992_fastlog_raw_requests;
    int request_count = s_aim992_fastlog_raw_request_count;
    char resp[512] = "";

    const char *init_cmds[] = {
        "ATZ", "ATE0", "ATL0", "ATS1", "ATH1", "ATCAF0",
    };
    for (int i = 0; i < (int)(sizeof(init_cmds) / sizeof(init_cmds[0])); i++) {
        [g_ble sendCommand:init_cmds[i] response:resp maxLen:sizeof(resp)];
        if (strcmp(init_cmds[i], "ATZ") == 0) run_loop_for(0.5);
    }

    char sti_resp[256] = "";
    g_stn_detected = false;
    if ([g_ble sendCommand:"STI" response:sti_resp maxLen:sizeof(sti_resp)] &&
        stn2255_detect(resp, sti_resp, &g_stn_info)) {
        g_stn_detected = true;
        cli_log("STN detected: %s", g_stn_info.device_id);
    }
    if (!g_stn_detected) {
        fprintf(stderr, "[ERROR] fastlog requires STN/OBDLink periodic-message support\n");
        [g_ble disconnect];
        return 1;
    }

    cli_log("Initializing %s CAN scheduler...", adapter_mode);
    /* Use STN raw CAN 11-bit 500k and schedule full 8-byte CAN frames,
       matching AiM's observed bus traffic. */
    if (!cli_send_required_command("STPPMC") ||
        !cli_send_required_command("STCFCPC") ||
        !cli_send_required_command("STP 31") ||
        !cli_send_required_command("STPO") ||
        !cli_send_required_command("ATCSM0") ||
        !cli_send_required_command("STCMM 1") ||
        !cli_send_required_command("STCSEGR 0") ||
        !cli_send_required_command("STCSEGT 0") ||
        !cli_send_required_command("STFAC")) {
        cli_p992_fastlog_cleanup();
        [g_ble disconnect];
        return 1;
    }

    const uint32_t pass_ids[] = { 0x7DF, 0x7E8, 0x7E9, 0x70C, 0x776, 0x713, 0x77D };
    for (int i = 0; i < (int)(sizeof(pass_ids) / sizeof(pass_ids[0])); i++) {
        char filter_cmd[32];
        snprintf(filter_cmd, sizeof(filter_cmd), "STFAP %03X,7FF", pass_ids[i]);
        if (!cli_send_required_command(filter_cmd)) {
            cli_p992_fastlog_cleanup();
            [g_ble disconnect];
            return 1;
        }
    }

    for (int i = 0; i < request_count; i++) {
        const aim992_fastlog_request_t *req = &requests[i];
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "STPPMA %d, %03X, %s",
                 period_ms, req->tx_id, req->payload_hex);
        if (!cli_send_required_command(cmd)) {
            cli_p992_fastlog_cleanup();
            [g_ble disconnect];
            return 1;
        }
        cli_log("Periodic %s: %s", req->name, cmd);
        run_loop_for((double)period_ms / 1000.0 /
                     (double)request_count);
    }

    cli_log("Starting raw monitor (%s) for %d seconds...", monitor_cmd, seconds);
    frame_buffer_reset(&g_cli_capture);
    [g_ble startMonitorWithCallback:monitor_frame_callback context:NULL];

    char monitor_with_cr[16];
    snprintf(monitor_with_cr, sizeof(monitor_with_cr), "%s\r", monitor_cmd);
    [g_ble sendRawString:monitor_with_cr];

    NSDate *endTime = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while ([[NSDate date] compare:endTime] == NSOrderedAscending &&
           g_ble.isConnected && !g_interrupt) {
        @autoreleasepool {
            [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                     beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
        }
    }

    [g_ble stopMonitor];
    [g_ble sendRawString:"\r"];
    run_loop_for(0.2);

    cli_log("Fastlog stopped. Frames: %d", g_cli_capture.count);
    if (g_cli_capture.dropped > 0) {
        fprintf(stderr, "[WARN] Capture buffer full: %d frames dropped\n",
                g_cli_capture.dropped);
    }
    cli_p992_fastlog_cleanup();

    int ret = cli_p992_fastlog_write_json(out_path,
                                          connected_name,
                                          seconds,
                                          period_ms,
                                          adapter_mode,
                                          monitor_cmd,
                                          requests,
                                          request_count);
    [g_ble disconnect];
    g_interrupt = NO;
    return ret;
}

int cli_p992(int argc, const char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "[ERROR] Usage: p992 <list|read|watch|debug> ...\n");
        return 1;
    }

    const char *subcmd = argv[2];
    if (strcmp(subcmd, "list") == 0) {
        return cli_p992_list();
    }

    if (strcmp(subcmd, "read") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 read <device> [channel...]\n");
            return 1;
        }
        return cli_p992_read(argv[3], argc - 4, &argv[4]);
    }

    if (strcmp(subcmd, "watch") == 0) {
        if (argc < 4) {
            fprintf(stderr,
                    "[ERROR] Usage: p992 watch <device> [--interval-ms N] [--count N] [channel...]\n");
            return 1;
        }

        int interval_ms = 1000;
        int count = 0;
        const char *channel_names[AIM992_MAX_CHANNELS];
        int channel_name_count = 0;

        for (int i = 4; i < argc; i++) {
            if (strcmp(argv[i], "--interval-ms") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --interval-ms requires a value\n");
                    return 1;
                }
                interval_ms = atoi(argv[++i]);
            } else if (strncmp(argv[i], "--interval-ms=", 14) == 0) {
                interval_ms = atoi(argv[i] + 14);
            } else if (strcmp(argv[i], "--count") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --count requires a value\n");
                    return 1;
                }
                count = atoi(argv[++i]);
            } else if (strncmp(argv[i], "--count=", 8) == 0) {
                count = atoi(argv[i] + 8);
            } else if (channel_name_count < AIM992_MAX_CHANNELS) {
                channel_names[channel_name_count++] = argv[i];
            } else {
                fprintf(stderr, "[ERROR] Too many channel names\n");
                return 1;
            }
        }

        if (interval_ms < 100) interval_ms = 100;
        return cli_p992_watch(argv[3], interval_ms, count,
                              channel_name_count, channel_names);
    }

    if (strcmp(subcmd, "fastlog") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 fastlog <device> [seconds] [--period-ms N] [--monitor-cmd STM|STMA|ATMA] [--out FILE]\n");
            return 1;
        }

        int seconds = 10;
        int period_ms = 166;
        const char *monitor_cmd = "STM";
        const char *out_path = NULL;
        int arg_start = 4;

        if (argc >= 5) {
            char *endp = NULL;
            long val = strtol(argv[4], &endp, 10);
            if (endp && *endp == '\0' && val > 0) {
                seconds = (int)val;
                arg_start = 5;
            }
        }

        for (int i = arg_start; i < argc; i++) {
            if (strcmp(argv[i], "--period-ms") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --period-ms requires a value\n");
                    return 1;
                }
                period_ms = atoi(argv[++i]);
            } else if (strncmp(argv[i], "--period-ms=", 12) == 0) {
                period_ms = atoi(argv[i] + 12);
            } else if (strcmp(argv[i], "--monitor-cmd") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --monitor-cmd requires a value\n");
                    return 1;
                }
                monitor_cmd = argv[++i];
            } else if (strncmp(argv[i], "--monitor-cmd=", 14) == 0) {
                monitor_cmd = argv[i] + 14;
            } else if (strcmp(argv[i], "--out") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --out requires a file path\n");
                    return 1;
                }
                out_path = argv[++i];
            } else if (strncmp(argv[i], "--out=", 6) == 0) {
                out_path = argv[i] + 6;
            } else {
                fprintf(stderr, "[ERROR] Unexpected fastlog argument: %s\n", argv[i]);
                return 1;
            }
        }

        if (strcasecmp(monitor_cmd, "STM") != 0 &&
            strcasecmp(monitor_cmd, "STMA") != 0 &&
            strcasecmp(monitor_cmd, "ATMA") != 0) {
            fprintf(stderr, "[ERROR] --monitor-cmd must be STM, STMA, or ATMA\n");
            return 1;
        }

        return cli_p992_fastlog(argv[3], seconds, period_ms, monitor_cmd, out_path);
    }

    if (strcmp(subcmd, "debug") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 debug <device>\n");
            return 1;
        }
        return cli_p992_debug(argv[3]);
    }

    if (strcmp(subcmd, "bin-probe") == 0) {
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 bin-probe <device>\n");
            return 1;
        }
        return cli_p992_bin_probe(argv[3]);
    }

    if (strcmp(subcmd, "scan-did") == 0) {
        uint16_t did = 0x5101;
        int session = -1;
        bool did_set = false;
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 scan-did <device> [DID_hex] [--session XX]\n");
            return 1;
        }

        for (int i = 4; i < argc; i++) {
            if (strcmp(argv[i], "--session") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --session requires a hex byte\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i]);
                    return 1;
                }
                session = (int)val;
            } else if (strncmp(argv[i], "--session=", 10) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 10, "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i] + 10);
                    return 1;
                }
                session = (int)val;
            } else if (!did_set) {
                unsigned int val = 0;
                if (sscanf(argv[i], "%x", &val) != 1 || val > 0xFFFF) {
                    fprintf(stderr, "[ERROR] Invalid DID hex: %s\n", argv[i]);
                    return 1;
                }
                did = (uint16_t)val;
                did_set = true;
            } else {
                fprintf(stderr, "[ERROR] Unexpected scan-did argument: %s\n", argv[i]);
                return 1;
            }
        }
        return cli_p992_scan_did(argv[3], did, session);
    }

    if (strcmp(subcmd, "scan-did-range") == 0) {
        uint16_t start_did = 0x50F0;
        uint16_t end_did = 0x5110;
        if (argc < 4) {
            fprintf(stderr, "[ERROR] Usage: p992 scan-did-range <device> [START_DID_hex END_DID_hex]\n");
            return 1;
        }
        if (argc >= 6) {
            unsigned int start_val = 0;
            unsigned int end_val = 0;
            if (sscanf(argv[4], "%x", &start_val) != 1 || start_val > 0xFFFF ||
                sscanf(argv[5], "%x", &end_val) != 1 || end_val > 0xFFFF) {
                fprintf(stderr, "[ERROR] Invalid DID range: %s %s\n", argv[4], argv[5]);
                return 1;
            }
            start_did = (uint16_t)start_val;
            end_did = (uint16_t)end_val;
        } else if (argc != 4) {
            fprintf(stderr, "[ERROR] Usage: p992 scan-did-range <device> [START_DID_hex END_DID_hex]\n");
            return 1;
        }
        return cli_p992_scan_did_range(argv[3], start_did, end_did);
    }

    if (strcmp(subcmd, "uds-read") == 0) {
        uint16_t did = 0;
        uint32_t tx_id = 0x7E0;
        int rx_id = -1;
        int session = -1;
        const char *pre_request_hex = NULL;
        bool did_set = false;

        if (argc < 5) {
            fprintf(stderr, "[ERROR] Usage: p992 uds-read <device> <DID_hex> [--tx-id XXX] [--rx-id XXX] [--session XX] [--pre HEX]\n");
            return 1;
        }

        for (int i = 4; i < argc; i++) {
            if (strcmp(argv[i], "--tx-id") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --tx-id requires a hex CAN ID\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid tx CAN ID: %s\n", argv[i]);
                    return 1;
                }
                tx_id = val;
            } else if (strncmp(argv[i], "--tx-id=", 8) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 8, "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid tx CAN ID: %s\n", argv[i] + 8);
                    return 1;
                }
                tx_id = val;
            } else if (strcmp(argv[i], "--rx-id") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --rx-id requires a hex CAN ID\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid rx CAN ID: %s\n", argv[i]);
                    return 1;
                }
                rx_id = (int)val;
            } else if (strncmp(argv[i], "--rx-id=", 8) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 8, "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid rx CAN ID: %s\n", argv[i] + 8);
                    return 1;
                }
                rx_id = (int)val;
            } else if (strcmp(argv[i], "--session") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --session requires a hex byte\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i]);
                    return 1;
                }
                session = (int)val;
            } else if (strncmp(argv[i], "--session=", 10) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 10, "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i] + 10);
                    return 1;
                }
                session = (int)val;
            } else if (strcmp(argv[i], "--pre") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --pre requires a hex request\n");
                    return 1;
                }
                pre_request_hex = argv[++i];
            } else if (strncmp(argv[i], "--pre=", 6) == 0) {
                pre_request_hex = argv[i] + 6;
            } else if (!did_set) {
                unsigned int val = 0;
                if (sscanf(argv[i], "%x", &val) != 1 || val > 0xFFFF) {
                    fprintf(stderr, "[ERROR] Invalid DID hex: %s\n", argv[i]);
                    return 1;
                }
                did = (uint16_t)val;
                did_set = true;
            } else {
                fprintf(stderr, "[ERROR] Unexpected uds-read argument: %s\n", argv[i]);
                return 1;
            }
        }

        if (!did_set) {
            fprintf(stderr, "[ERROR] uds-read requires a DID hex value\n");
            return 1;
        }
        return cli_p992_uds_read(argv[3], did, tx_id, rx_id, session, pre_request_hex);
    }

    if (strcmp(subcmd, "uds-raw") == 0) {
        const char *request_hex = NULL;
        uint32_t tx_id = 0x7E0;
        int rx_id = -1;
        int session = -1;

        if (argc < 5) {
            fprintf(stderr, "[ERROR] Usage: p992 uds-raw <device> <REQ_hex> [--tx-id XXX] [--rx-id XXX] [--session XX]\n");
            return 1;
        }

        for (int i = 4; i < argc; i++) {
            if (strcmp(argv[i], "--tx-id") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --tx-id requires a hex CAN ID\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid tx CAN ID: %s\n", argv[i]);
                    return 1;
                }
                tx_id = val;
            } else if (strncmp(argv[i], "--tx-id=", 8) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 8, "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid tx CAN ID: %s\n", argv[i] + 8);
                    return 1;
                }
                tx_id = val;
            } else if (strcmp(argv[i], "--rx-id") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --rx-id requires a hex CAN ID\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid rx CAN ID: %s\n", argv[i]);
                    return 1;
                }
                rx_id = (int)val;
            } else if (strncmp(argv[i], "--rx-id=", 8) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 8, "%x", &val) != 1 || val > 0x7FF) {
                    fprintf(stderr, "[ERROR] Invalid rx CAN ID: %s\n", argv[i] + 8);
                    return 1;
                }
                rx_id = (int)val;
            } else if (strcmp(argv[i], "--session") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "[ERROR] --session requires a hex byte\n");
                    return 1;
                }
                unsigned int val = 0;
                if (sscanf(argv[++i], "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i]);
                    return 1;
                }
                session = (int)val;
            } else if (strncmp(argv[i], "--session=", 10) == 0) {
                unsigned int val = 0;
                if (sscanf(argv[i] + 10, "%x", &val) != 1 || val > 0xFF) {
                    fprintf(stderr, "[ERROR] Invalid session hex: %s\n", argv[i] + 10);
                    return 1;
                }
                session = (int)val;
            } else if (!request_hex) {
                request_hex = argv[i];
            } else {
                fprintf(stderr, "[ERROR] Unexpected uds-raw argument: %s\n", argv[i]);
                return 1;
            }
        }

        if (!request_hex) {
            fprintf(stderr, "[ERROR] uds-raw requires a request hex value\n");
            return 1;
        }
        return cli_p992_uds_raw(argv[3], request_hex, tx_id, rx_id, session);
    }

    if (strcmp(subcmd, "help") == 0 || strcmp(subcmd, "--help") == 0) {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "  macos_obd_reader p992 list\n");
        fprintf(stderr, "  macos_obd_reader p992 read <device> [channel...]\n");
        fprintf(stderr, "  macos_obd_reader p992 watch <device> [--interval-ms N] [--count N] [channel...]\n");
        fprintf(stderr, "  macos_obd_reader p992 fastlog <device> [secs] [--period-ms N] [--out FILE]\n");
        fprintf(stderr, "  macos_obd_reader p992 debug <device>\n");
        fprintf(stderr, "  macos_obd_reader p992 bin-probe <device>\n");
        fprintf(stderr, "  macos_obd_reader p992 scan-did <device> [DID_hex] [--session XX]\n");
        fprintf(stderr, "  macos_obd_reader p992 scan-did-range <device> [START END]\n");
        fprintf(stderr, "  macos_obd_reader p992 uds-read <device> <DID> [...]\n");
        fprintf(stderr, "  macos_obd_reader p992 uds-raw <device> <REQ> [...]\n");
        return 0;
    }

    fprintf(stderr, "[ERROR] Unknown p992 command: %s\n", subcmd);
    return 1;
}

