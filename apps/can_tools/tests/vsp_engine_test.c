/**
 * @file vsp_engine_test.c
 * @brief vsp_engine selection + decode rules on an embedded VSP (no hardware)
 *
 * Expected values follow the rules vsp_probe applied before the engine was
 * extracted (see the comments next to each frame).
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "vsp_engine.h"

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

#define CHECK_STR(a, b) CHECK((a) && (b) && strcmp((a), (b)) == 0, "\"%s\" != \"%s\"", \
                              (a) ? (a) : "(null)", (b) ? (b) : "(null)")
#define CHECK_NEAR(a, b) CHECK(fabs((a) - (b)) < 1e-9, "%.12g != %.12g", (double)(a), (double)(b))

static const char *kProfile =
"{"
" \"schema\": \"vehicle_data_profile.v1\","
" \"vehicle\": {\"manufacturer\": \"Test\", \"model\": \"Bench\"},"
" \"applicability\": {\"generation\": \"T1\", \"platform\": null, \"year_start\": 2020, \"year_end\": null},"
" \"buses\": [{\"id\": \"can0\", \"type\": \"can\", \"bitrate\": 500000}],"
" \"can_frames\": ["
"  {\"frame_id\": 256, \"name\": \"ENGINE\", \"dlc\": 8, \"cycle_ms\": 10},"
"  {\"frame_id\": 257, \"name\": \"CHASSIS\", \"dlc\": 8},"
"  {\"frame_id\": 258, \"name\": \"MUXED\", \"dlc\": 8},"
"  {\"frame_id\": 259, \"name\": \"TRANS\", \"dlc\": 8},"
"  {\"frame_id\": 260, \"name\": \"MISC\", \"dlc\": 8},"
"  {\"frame_id\": 512, \"name\": \"MASKED\", \"dlc\": 8, \"frame_id_mask\": 2032},"
"  {\"frame_id\": 2047, \"name\": \"STREAM\", \"dlc\": 8, \"reassembled\": true}"
" ],"
" \"diagnostic_queries\": ["
"  {\"id\": \"diag_7DF_010C\", \"kind\": \"obd_mode_01\", \"role\": \"channel_query\","
"   \"tx_id\": 2015, \"rx_id\": 2024, \"request_hex\": \"01 0C\", \"payload_hex\": \"02 01 0C 55 55 55 55 55\"},"
"  {\"id\": \"fc_7DF\", \"kind\": \"isotp_flow_control\", \"role\": \"flow_control\","
"   \"tx_id\": 2015, \"rx_id\": 2024, \"request_hex\": \"30 00 01\"}"
" ],"
" \"signals\": ["
"  {\"name\": \"RPM_LE\", \"canonical\": {\"id\": \"engine.speed\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
"   \"decode\": {\"start_bit\": 8, \"bit_length\": 16, \"byte_order\": \"little_endian\", \"signed\": false,"
"              \"factor\": 0.25, \"offset\": 0, \"unit\": \"rpm\", \"mux\": null}},"
"  {\"name\": \"STEER\", \"canonical\": {\"id\": \"steering.wheel.angle\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 257},"
"   \"decode\": {\"start_bit\": 7, \"bit_length\": 12, \"byte_order\": \"big_endian\", \"signed\": true,"
"              \"factor\": 0.5, \"offset\": 0, \"unit\": \"deg\"}},"
"  {\"name\": \"THR_BE\", \"canonical\": {\"id\": \"throttle.position\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 257},"
"   \"decode\": {\"start_bit\": 23, \"bit_length\": 8, \"byte_order\": \"big_endian\", \"signed\": false,"
"              \"factor\": 0.4, \"offset\": 0, \"unit\": \"%\"}},"
"  {\"name\": \"MUX_A\","
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 258},"
"   \"decode\": {\"start_bit\": 8, \"bit_length\": 8, \"byte_order\": \"little_endian\", \"factor\": 1, \"offset\": -40,"
"              \"unit\": \"C\", \"mux\": {\"value\": 2, \"start_bit\": 0, \"bit_length\": 8, \"byte_order\": \"little_endian\"}}},"
"  {\"name\": \"GEAR\", \"canonical\": {\"id\": \"transmission.gear.selected\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 259},"
"   \"decode\": {\"start_bit\": 0, \"bit_length\": 4, \"byte_order\": \"little_endian\", \"factor\": 1, \"offset\": 0,"
"              \"values\": [{\"raw\": 0, \"label\": \"P\", \"value\": null}, {\"raw\": 1, \"label\": \"R\", \"value\": -1},"
"                         {\"raw\": 2, \"label\": \"N\"}, {\"raw\": 5, \"label\": \"D\", \"value\": 1}]}},"
"  {\"name\": \"BRAKE_SM\", \"canonical\": {\"id\": \"brake.pressure\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 260},"
"   \"decode\": {\"start_bit\": 0, \"bit_length\": 12, \"byte_order\": \"little_endian\", \"signed\": false,"
"              \"sign_bit\": 15, \"factor\": 0.1, \"offset\": 0, \"unit\": \"bar\"}},"
"  {\"name\": \"FLOAT_T\","
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 260},"
"   \"decode\": {\"start_bit\": 32, \"bit_length\": 32, \"byte_order\": \"little_endian\", \"value_type\": \"float32\","
"              \"factor\": 2, \"offset\": 0}},"
"  {\"name\": \"MASKED\","
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 512},"
"   \"decode\": {\"start_bit\": 0, \"bit_length\": 8, \"byte_order\": \"little_endian\", \"factor\": 1, \"offset\": 0}},"
"  {\"name\": \"STREAM_RPM\", \"canonical\": {\"id\": \"engine.speed\"},"
"   \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 2047},"
"   \"decode\": {\"start_bit\": 0, \"bit_length\": 16, \"byte_order\": \"little_endian\", \"factor\": 1, \"offset\": 0}},"
"  {\"name\": \"OBD_RPM\", \"canonical\": {\"id\": \"engine.speed\"},"
"   \"acquisition\": {\"type\": \"diagnostic_query\", \"query_id\": \"diag_7DF_010C\", \"tx_id\": 2015, \"rx_id\": 2024,"
"                   \"request_hex\": \"01 0C\"},"
"   \"decode\": {\"start_bit\": 31, \"bit_length\": 16, \"byte_order\": \"big_endian\", \"signed\": false,"
"              \"factor\": 0.25, \"offset\": 0, \"unit\": \"rpm\", \"mux\": null}},"
"  {\"name\": \"OBD_INLINE\","
"   \"acquisition\": {\"type\": \"diagnostic_query\", \"tx_id\": 2016, \"rx_id\": 2025, \"request_hex\": \"22 F1 90\"},"
"   \"decode\": {\"start_bit\": 39, \"bit_length\": 8, \"byte_order\": \"big_endian\", \"factor\": 1, \"offset\": 0}}"
" ]"
"}";

static int find_signal(const vsp_decoder_t *dec, const char *name) {
    for (int i = 0; i < vsp_decoder_signal_count(dec); i++) {
        if (strcmp(vsp_decoder_signal_info(dec, i)->name, name) == 0) return i;
    }
    return -1;
}

static const vsp_signal_state_t *state_of(const vsp_decoder_t *dec, const char *name) {
    int idx = find_signal(dec, name);
    CHECK(idx >= 0, "signal %s not selected", name);
    static const vsp_signal_state_t empty;
    return idx >= 0 ? vsp_decoder_signal_state(dec, idx) : &empty;
}

static can_frame_t make_frame(uint32_t id, const uint8_t *data, int dlc, int64_t ts) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.can_id = id;
    f.dlc = (uint8_t)dlc;
    memcpy(f.data, data, (size_t)dlc);
    f.timestamp_ms = ts;
    return f;
}

static int feed(vsp_decoder_t *dec, uint32_t id, const uint8_t *data, int dlc, double ts) {
    can_frame_t f = make_frame(id, data, dlc, (int64_t)ts);
    return vsp_decoder_feed(dec, &f, ts);
}

static void test_profile_info(const vsp_profile_t *p) {
    vsp_profile_info_t info;
    vsp_profile_get_info(p, &info);
    CHECK(info.path == NULL, "path should be NULL for in-memory JSON");
    CHECK_STR(info.schema, "vehicle_data_profile.v1");
    CHECK_STR(info.manufacturer, "Test");
    CHECK_STR(info.model, "Bench");
    CHECK_STR(info.generation, "T1");
    CHECK(info.platform == NULL, "platform null -> NULL");
    CHECK(info.year_start == 2020 && info.year_end == 0, "years %d %d", info.year_start, info.year_end);
    CHECK(info.bus_count == 1 && info.frame_count == 7 && info.signal_count == 11 && info.query_count == 2,
          "counts %d %d %d %d", info.bus_count, info.frame_count, info.signal_count, info.query_count);

    vsp_bus_info_t bus;
    CHECK(vsp_profile_bus(p, 0, &bus) && bus.bitrate == 500000, "bus bitrate");
    CHECK_STR(bus.id, "can0");

    vsp_frame_info_t f;
    CHECK(vsp_profile_frame(p, 0, &f) && f.frame_id == 0x100 && f.cycle_ms == 10.0 && f.dlc == 8,
          "frame 0x100 info");
    CHECK_STR(f.name, "ENGINE");
    CHECK(vsp_profile_frame(p, 5, &f) && f.frame_id == 0x200 && f.frame_id_mask == 0x7F0, "masked frame");
    CHECK(vsp_profile_frame(p, 6, &f) && f.reassembled && !vsp_profile_frame(p, 7, &f), "reassembled frame");

    vsp_query_info_t q;
    CHECK(vsp_profile_query(p, 1, &q) && q.has_tx_id && q.tx_id == 0x7DF && q.rx_id == 0x7E8, "fc query ids");
    CHECK_STR(q.role, "flow_control");
    CHECK(q.payload_hex == NULL, "fc payload absent");
}

static void test_selection(const vsp_profile_t *p) {
    /* Default groups: rpm, gear, steering, throttle, brake (canonical ids). */
    vsp_select_t sel = { .signals = NULL, .all_signals = false, .acquisition = VSP_ACQ_ANY };
    vsp_decoder_t *dec = vsp_decoder_create(p, &sel);
    CHECK(dec != NULL, "decoder");
    static const char *kDefaultOrder[] = { "BRAKE_SM", "GEAR", "RPM_LE", "OBD_RPM", "STEER", "THR_BE" };
    CHECK(vsp_decoder_signal_count(dec) == 6, "default count %d", vsp_decoder_signal_count(dec));
    for (int i = 0; i < 6 && i < vsp_decoder_signal_count(dec); i++) {
        CHECK_STR(vsp_decoder_signal_info(dec, i)->name, kDefaultOrder[i]);
    }
    CHECK(vsp_decoder_skipped_count(dec) == 1, "skipped %d", vsp_decoder_skipped_count(dec));
    if (vsp_decoder_skipped_count(dec) == 1) {
        CHECK_STR(vsp_decoder_skipped(dec, 0)->name, "STREAM_RPM");
        CHECK(vsp_decoder_skipped(dec, 0)->frame_id == 0x7FF, "skipped frame id");
    }
    vsp_decoder_free(dec);

    sel.acquisition = VSP_ACQ_CAN;
    dec = vsp_decoder_create(p, &sel);
    CHECK(vsp_decoder_signal_count(dec) == 5 && find_signal(dec, "OBD_RPM") < 0, "CAN filter");
    vsp_decoder_free(dec);

    sel.acquisition = VSP_ACQ_DIAGNOSTIC;
    dec = vsp_decoder_create(p, &sel);
    CHECK(vsp_decoder_signal_count(dec) == 1 && find_signal(dec, "OBD_RPM") == 0, "diagnostic filter");
    vsp_decoder_free(dec);

    /* Signal names, case-insensitive, and a group alias. */
    sel.acquisition = VSP_ACQ_ANY;
    sel.signals = "masked, MUX_a,steer";
    dec = vsp_decoder_create(p, &sel);
    CHECK(vsp_decoder_signal_count(dec) == 3 && find_signal(dec, "MASKED") >= 0 &&
          find_signal(dec, "MUX_A") >= 0 && find_signal(dec, "STEER") >= 0,
          "name/group selection count %d", vsp_decoder_signal_count(dec));
    vsp_decoder_free(dec);

    /* Canonical id */
    sel.signals = "brake.pressure";
    dec = vsp_decoder_create(p, &sel);
    CHECK(vsp_decoder_signal_count(dec) == 1 && find_signal(dec, "BRAKE_SM") == 0, "canonical selection");
    vsp_decoder_free(dec);

    /* Everything decodable, custom group sorted between brake and gear. */
    sel.signals = NULL;
    sel.all_signals = true;
    dec = vsp_decoder_create(p, &sel);
    static const char *kAllOrder[] = { "BRAKE_SM", "FLOAT_T", "MASKED", "MUX_A", "OBD_INLINE",
                                       "GEAR", "RPM_LE", "OBD_RPM", "STEER", "THR_BE" };
    CHECK(vsp_decoder_signal_count(dec) == 10, "all count %d", vsp_decoder_signal_count(dec));
    for (int i = 0; i < 10 && i < vsp_decoder_signal_count(dec); i++) {
        CHECK_STR(vsp_decoder_signal_info(dec, i)->name, kAllOrder[i]);
    }
    vsp_decoder_free(dec);
}

static void test_signal_info(const vsp_decoder_t *dec) {
    const vsp_signal_info_t *rpm = vsp_decoder_signal_info(dec, find_signal(dec, "RPM_LE"));
    CHECK(rpm->kind == VSP_SIGNAL_CAN_FRAME && rpm->frame_id == 0x100 && rpm->cycle_ms == 10.0 &&
          !rpm->has_mux && rpm->frame_id_mask == 0, "RPM_LE info");
    CHECK_STR(rpm->group, "rpm");
    CHECK_STR(rpm->unit, "rpm");
    CHECK_STR(rpm->acquisition_type, "can_frame");

    const vsp_signal_info_t *masked = vsp_decoder_signal_info(dec, find_signal(dec, "MASKED"));
    CHECK(masked->frame_id_mask == 0x7F0 && masked->unit == NULL, "MASKED info");
    CHECK_STR(masked->group, "custom");
    CHECK_STR(masked->canonical_id, "");

    const vsp_signal_info_t *mux = vsp_decoder_signal_info(dec, find_signal(dec, "MUX_A"));
    CHECK(mux->has_mux, "MUX_A has mux");

    const vsp_signal_info_t *obd = vsp_decoder_signal_info(dec, find_signal(dec, "OBD_RPM"));
    CHECK(obd->kind == VSP_SIGNAL_DIAGNOSTIC_QUERY && obd->tx_id == 0x7DF && obd->rx_id == 0x7E8,
          "OBD_RPM ids");
    CHECK_STR(obd->query_id, "diag_7DF_010C");
    CHECK_STR(obd->request_hex, "01 0C");
    CHECK_STR(obd->request_bare_hex, "010C");
    CHECK_STR(obd->positive_marker_hex, "410C");
    CHECK_STR(obd->query_key, "7DF:7E8:010C");
    CHECK_STR(obd->query_payload_hex, "02 01 0C 55 55 55 55 55");

    /* No diagnostic_queries[] entry: the query comes from the acquisition. */
    const vsp_signal_info_t *inl = vsp_decoder_signal_info(dec, find_signal(dec, "OBD_INLINE"));
    CHECK_STR(inl->query_id, "inline_2016_22F190");
    CHECK_STR(inl->positive_marker_hex, "62F190");
    CHECK_STR(inl->query_key, "7E0:7E9:22F190");
    CHECK(inl->query_payload_hex == NULL, "inline query has no payload_hex");
}

static void test_decode(vsp_decoder_t *dec) {
    /* Little endian 16 bit at bit 8: 0x1F40 = 8000 * 0.25 = 2000 rpm. */
    const uint8_t rpm[8] = { 0x00, 0x40, 0x1F, 0, 0, 0, 0, 0 };
    CHECK(feed(dec, 0x100, rpm, 8, 1000.0) == 1, "0x100 updates one signal");
    const vsp_signal_state_t *st = state_of(dec, "RPM_LE");
    CHECK(st->samples == 1 && st->has_value && !st->value_null && st->label == NULL, "RPM_LE state");
    CHECK_NEAR(st->value, 2000.0);
    CHECK(st->first_sample_ms == 1000.0 && st->last_sample_ms == 1000.0, "RPM_LE timestamps");
    CHECK(st->last_payload_len == 8 && st->last_payload[2] == 0x1F, "RPM_LE payload");

    /* DBC big endian 12 bit from bit 7: byte0 + high nibble of byte1 = 0xABC,
       two's complement -> -1348 * 0.5 = -674. Bit 23 / 8 bit = byte 2 = 250 * 0.4. */
    const uint8_t chassis[8] = { 0xAB, 0xC0, 0xFA, 0, 0, 0, 0, 0 };
    CHECK(feed(dec, 0x101, chassis, 8, 1010.0) == 2, "0x101 updates two signals");
    CHECK_NEAR(state_of(dec, "STEER")->value, -674.0);
    CHECK_NEAR(state_of(dec, "THR_BE")->value, 100.0);

    /* Mux byte0 == 2 selects MUX_A; offset -40. */
    const uint8_t mux1[8] = { 0x01, 0x64, 0, 0, 0, 0, 0, 0 };
    const uint8_t mux2[8] = { 0x02, 0x64, 0, 0, 0, 0, 0, 0 };
    CHECK(feed(dec, 0x102, mux1, 8, 1020.0) == 0, "mux 1 ignored");
    CHECK(state_of(dec, "MUX_A")->samples == 0, "MUX_A no sample for mux 1");
    CHECK(feed(dec, 0x102, mux2, 8, 1030.0) == 1, "mux 2 matches");
    CHECK_NEAR(state_of(dec, "MUX_A")->value, 60.0);

    /* Value table: label, null override, numeric override, plain value. */
    const uint8_t g0[1] = { 0x00 }, g1[1] = { 0x01 }, g2[1] = { 0x02 }, g5[1] = { 0x05 }, g7[1] = { 0x07 };
    feed(dec, 0x103, g0, 1, 1040.0);
    st = state_of(dec, "GEAR");
    CHECK(!st->has_value && st->value_null, "GEAR P has no number");
    CHECK_STR(st->label, "P");
    feed(dec, 0x103, g1, 1, 1041.0);
    CHECK(st->has_value && !st->value_null, "GEAR R has a number");
    CHECK_NEAR(st->value, -1.0);
    CHECK_STR(st->label, "R");
    feed(dec, 0x103, g2, 1, 1042.0);
    CHECK_NEAR(st->value, 2.0);
    CHECK_STR(st->label, "N");
    feed(dec, 0x103, g5, 1, 1043.0);
    CHECK_NEAR(st->value, 1.0);
    CHECK_STR(st->label, "D");
    feed(dec, 0x103, g7, 1, 1044.0);
    CHECK(st->has_value && st->label == NULL, "GEAR 7: no table entry");
    CHECK_NEAR(st->value, 7.0);
    CHECK(st->samples == 5 && st->first_sample_ms == 1040.0 && st->last_sample_ms == 1044.0, "GEAR samples");

    /* sign_bit 15 negates the 12-bit magnitude 0x3E8 (1000 * 0.1);
       float32 1.5 (0x3FC00000 little endian at bit 32) * 2. */
    const uint8_t misc_neg[8] = { 0xE8, 0x83, 0, 0, 0x00, 0x00, 0xC0, 0x3F };
    const uint8_t misc_pos[8] = { 0xE8, 0x03, 0, 0, 0x00, 0x00, 0xC0, 0x3F };
    CHECK(feed(dec, 0x104, misc_neg, 8, 1050.0) == 2, "0x104 updates two signals");
    CHECK_NEAR(state_of(dec, "BRAKE_SM")->value, -100.0);
    CHECK_NEAR(state_of(dec, "FLOAT_T")->value, 3.0);
    feed(dec, 0x104, misc_pos, 8, 1051.0);
    CHECK_NEAR(state_of(dec, "BRAKE_SM")->value, 100.0);

    /* frame_id_mask 0x7F0: 0x205 is the 0x200 frame, 0x210 is not. */
    const uint8_t m[8] = { 0x2A, 0, 0, 0, 0, 0, 0, 0 };
    CHECK(feed(dec, 0x205, m, 8, 1060.0) == 1, "0x205 matches mask");
    CHECK(feed(dec, 0x210, m, 8, 1061.0) == 0, "0x210 outside mask");
    CHECK(state_of(dec, "MASKED")->samples == 1, "MASKED one sample");
    CHECK_NEAR(state_of(dec, "MASKED")->value, 42.0);

    /* Reassembled stream frame: its signal was skipped. */
    CHECK(feed(dec, 0x7FF, m, 8, 1062.0) == 0, "0x7FF not decoded");

    /* Diagnostic, single frame: PCI 04, marker 41 0C, then bytes 3..4 big endian. */
    const uint8_t sf_rpm[8] = { 0x04, 0x41, 0x0C, 0x1F, 0x40, 0x55, 0x55, 0x55 };
    const uint8_t sf_speed[8] = { 0x03, 0x41, 0x0D, 0x32, 0x55, 0x55, 0x55, 0x55 };
    const uint8_t cf[8] = { 0x21, 0x41, 0x0C, 0x1F, 0x40, 0x55, 0x55, 0x55 };
    CHECK(feed(dec, 0x7E8, sf_speed, 8, 1100.0) == 0, "other PID ignored");
    CHECK(feed(dec, 0x7E9, sf_rpm, 8, 1101.0) == 0, "other response id ignored");
    CHECK(feed(dec, 0x7E8, cf, 8, 1102.0) == 0, "consecutive frame has no marker");
    CHECK(feed(dec, 0x7E8, sf_rpm, 8, 1103.0) == 1, "41 0C single frame matches");
    st = state_of(dec, "OBD_RPM");
    CHECK(st->samples == 1 && st->first_sample_ms == 1103.0, "OBD_RPM sample");
    CHECK_NEAR(st->value, 2000.0);

    /* Query key restricts the diagnostic match to one request. */
    can_frame_t f = make_frame(0x7E8, sf_rpm, 8, 0);
    CHECK(vsp_decoder_feed_diagnostic(dec, &f, 1104.0, "7DF:7E8:010D") == 0, "other query key");
    CHECK(vsp_decoder_feed_diagnostic(dec, &f, 1105.0, "7DF:7E8:010C") == 1, "own query key");
    CHECK(vsp_decoder_feed_can(dec, &f, 1106.0) == 0, "CAN rules do not see responses");
    CHECK(st->samples == 2 && st->last_sample_ms == 1105.0, "OBD_RPM after key filter");

    /* UDS 22 F1 90 from the acquisition: marker 62 F1 90, byte 4. */
    const uint8_t uds[8] = { 0x05, 0x62, 0xF1, 0x90, 0x2A, 0x00, 0x00, 0x00 };
    CHECK(feed(dec, 0x7E9, uds, 8, 1110.0) == 1, "62 F1 90 matches");
    CHECK_NEAR(state_of(dec, "OBD_INLINE")->value, 42.0);

    vsp_decoder_reset(dec);
    CHECK(state_of(dec, "RPM_LE")->samples == 0 && !state_of(dec, "RPM_LE")->has_value, "reset");
}

/* lsb: the value of one raw step, only for a linear integer field of 6+ bits. */
static void test_lsb(const vsp_decoder_t *dec) {
    CHECK_NEAR(vsp_decoder_signal_info(dec, find_signal(dec, "RPM_LE"))->lsb, 0.25);
    CHECK_NEAR(vsp_decoder_signal_info(dec, find_signal(dec, "STEER"))->lsb, 0.5);     /* signed */
    CHECK_NEAR(vsp_decoder_signal_info(dec, find_signal(dec, "BRAKE_SM"))->lsb, 0.1);  /* sign_bit */
    CHECK_NEAR(vsp_decoder_signal_info(dec, find_signal(dec, "MUX_A"))->lsb, 1.0);     /* offset -40 */
    CHECK(vsp_decoder_signal_info(dec, find_signal(dec, "FLOAT_T"))->lsb == 0, "float32: no lsb");
    CHECK(vsp_decoder_signal_info(dec, find_signal(dec, "GEAR"))->lsb == 0, "4 bit, numeric overrides: no lsb");

    static const char *kLsb =
    "{\"schema\": \"vehicle_data_profile.v1\", \"can_frames\": [{\"frame_id\": 256, \"dlc\": 8}], \"signals\": ["
    " {\"name\": \"POLY\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 8, \"factor\": 1, \"polynomial\": [0, 0.5, 0.01]}},"
    " {\"name\": \"F64\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 64, \"value_type\": \"float64\", \"factor\": 1}},"
    " {\"name\": \"BIT1\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 1, \"factor\": 1}},"
    " {\"name\": \"BITS5\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 5, \"factor\": 0.5}},"
    " {\"name\": \"BITS6\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 6, \"factor\": -0.5}},"
    " {\"name\": \"LABELS\", \"acquisition\": {\"type\": \"can_frame\", \"frame_id\": 256},"
    "  \"decode\": {\"start_bit\": 0, \"bit_length\": 8, \"factor\": 2, \"values\": [{\"raw\": 255, \"label\": \"SNA\"}]}}"
    "]}";
    char err[256] = "";
    vsp_profile_t *p = vsp_profile_load_json(kLsb, strlen(kLsb), err, sizeof(err));
    CHECK(p != NULL, "lsb profile: %s", err);
    if (!p) return;
    vsp_select_t all = { .signals = NULL, .all_signals = true, .acquisition = VSP_ACQ_ANY };
    vsp_decoder_t *d = vsp_decoder_create(p, &all);
    CHECK(d && vsp_decoder_signal_count(d) == 6, "lsb decoder");
    if (d && vsp_decoder_signal_count(d) == 6) {
        CHECK(vsp_decoder_signal_info(d, find_signal(d, "POLY"))->lsb == 0, "polynomial: no lsb");
        CHECK(vsp_decoder_signal_info(d, find_signal(d, "F64"))->lsb == 0, "float64: no lsb");
        CHECK(vsp_decoder_signal_info(d, find_signal(d, "BIT1"))->lsb == 0, "1 bit flag: no lsb");
        CHECK(vsp_decoder_signal_info(d, find_signal(d, "BITS5"))->lsb == 0, "5 bits: no lsb");
        CHECK_NEAR(vsp_decoder_signal_info(d, find_signal(d, "BITS6"))->lsb, 0.5);    /* |factor| */
        CHECK_NEAR(vsp_decoder_signal_info(d, find_signal(d, "LABELS"))->lsb, 2.0);   /* label only */
    }
    vsp_decoder_free(d);
    vsp_profile_free(p);
}

static void test_hex_and_errors(void) {
    char buf[64];
    CHECK(vsp_single_frame_payload_hex("01 0C", 0x55, buf, sizeof(buf)), "single frame");
    CHECK_STR(buf, "02010C5555555555");
    CHECK(vsp_single_frame_payload_hex("30 00 01", 0xAA, buf, sizeof(buf)), "fc single frame");
    CHECK_STR(buf, "03300001AAAAAAAA");
    CHECK(!vsp_single_frame_payload_hex("", 0x55, buf, sizeof(buf)), "empty request");
    CHECK(!vsp_single_frame_payload_hex(NULL, 0x55, buf, sizeof(buf)), "NULL request");
    vsp_hex_strip("03 22 50 75 aa", buf, sizeof(buf));
    CHECK_STR(buf, "03225075AA");
    const uint8_t b[3] = { 0x0A, 0xBC, 0xFF };
    vsp_hex_format(b, 3, buf, sizeof(buf));
    CHECK_STR(buf, "0ABCFF");

    char err[256] = "";
    CHECK(vsp_profile_load("/nonexistent/x.vsp", err, sizeof(err)) == NULL, "missing file");
    CHECK_STR(err, "Failed to read profile: /nonexistent/x.vsp");
    const char *arr = "[1, 2]";
    CHECK(vsp_profile_load_json(arr, strlen(arr), err, sizeof(err)) == NULL, "array is not a profile");
    CHECK_STR(err, "Invalid VSP JSON: unknown");
    const char *bad = "{\"signals\": [";
    CHECK(vsp_profile_load_json(bad, strlen(bad), err, sizeof(err)) == NULL, "truncated JSON");
    CHECK(strncmp(err, "Invalid VSP JSON: ", 18) == 0 && strlen(err) > 18, "JSON error text: %s", err);
}

int main(void) {
    char err[256] = "";
    vsp_profile_t *p = vsp_profile_load_json(kProfile, strlen(kProfile), err, sizeof(err));
    CHECK(p != NULL, "profile: %s", err);
    if (!p) return 1;

    test_profile_info(p);
    test_selection(p);

    vsp_select_t all = { .signals = NULL, .all_signals = true, .acquisition = VSP_ACQ_ANY };
    vsp_decoder_t *dec = vsp_decoder_create(p, &all);
    CHECK(dec != NULL, "decoder");
    if (dec) {
        test_signal_info(dec);
        test_lsb(dec);
        test_decode(dec);
        vsp_decoder_free(dec);
    }
    vsp_profile_free(p);
    test_hex_and_errors();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
