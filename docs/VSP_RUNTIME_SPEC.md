# VSP Runtime Spec

This document is the application-facing contract for Vehicle Signal Profile
(`.vsp`) files. A `.vsp` file is UTF-8 JSON with MIME type
`application/vnd.vehicle-signal-profile+json`.

The public runtime package contains ready-to-use profile files.

## Top Level

```json
{
  "schema": "vehicle_data_profile.v1",
  "vehicle": {},
  "applicability": {},
  "buses": [],
  "can_frames": [],
  "diagnostic_queries": [],
  "signals": []
}
```

Fields:

- `schema`: profile schema ID. Current value is `vehicle_data_profile.v1`.
- `vehicle`: manufacturer/model labels.
- `applicability`: generation, model year, market, trim, and powertrain scope.
- `buses`: physical buses used by the profile.
- `can_frames`: CAN frame layouts used by passive CAN signals and diagnostic
  response signals.
- `diagnostic_queries`: active OBD/UDS/local-ID requests.
- `signals`: user-facing channels. Runtime code should start here.

Applications should ignore unknown top-level fields for forward compatibility.

## Runtime Modes

Choose runtime channels from top-level `signals[]` by
`signals[].acquisition.type`.

CAN sniffing mode:

1. Select signals where `acquisition.type == "can_frame"`, except those on a
   reassembled frame (see [Reassembled Frames](#reassembled-frames)).
2. Listen on `acquisition.bus`.
3. Match `acquisition.frame_id`.
4. If `decode.mux` is present, decode and compare the mux value first.
5. Decode the signal bits and compute `raw * factor + offset`.

OBD/query mode:

1. Select signals where `acquisition.type == "diagnostic_query"`.
2. Find the query in `diagnostic_queries[]` by `acquisition.query_id`.
3. Send `request_hex` to `tx_id` on the query bus.
4. Wait for `acquisition.response_frame_id` or query `rx_id`.
5. If `decode.mux` is present, decode and compare the response mux value first.
6. Decode the signal bits and compute `raw * factor + offset`.

Do not treat every `can_frames[]` entry as passive broadcast data. Diagnostic
response layouts are also stored in `can_frames[]` so both modes can use the
same bit decoder.

## Acquisition Objects

Passive CAN signal:

```json
{
  "type": "can_frame",
  "bus": "can0",
  "frame_id": 165,
  "frame_id_hex": "0xA5",
  "frame_name": "DME_EngineData"
}
```

Diagnostic signal:

```json
{
  "type": "diagnostic_query",
  "bus": "can0",
  "query_id": "diag_7DF_010C",
  "tx_id": 2015,
  "tx_id_hex": "0x7DF",
  "rx_id": 2024,
  "rx_id_hex": "0x7E8",
  "request_hex": "01 0C",
  "request_kind": "obd_mode_01",
  "response_frame_id": 2024,
  "response_frame_id_hex": "0x7E8"
}
```

## Decode Objects

Each signal has a `decode` object:

```json
{
  "start_bit": 40,
  "bit_length": 16,
  "byte_order": "little_endian",
  "signed": false,
  "factor": 0.25,
  "offset": 0.0,
  "minimum": 0.0,
  "maximum": 16000.0,
  "unit": "rpm",
  "mux": null
}
```

Rules:

- `start_bit` and `bit_length` describe the raw bit field inside the CAN
  payload or diagnostic response frame.
- `byte_order` is `little_endian` or `big_endian`.
- `start_bit` uses DBC numbering (`byte_index * 8 + bit_in_byte`, bit 0 = LSB
  of byte 0). It names the field's LSB for `little_endian` and its MSB for
  `big_endian`; a big-endian field continues from bit 0 of a byte to bit 7 of
  the next byte. Every bit a signal reads (the field, `sign_bit` and the `mux`
  field) lies inside the frame's `dlc` bytes.
- `signed` controls signed integer interpretation before scaling.
- `sign_bit` is optional. When present, the field is sign-magnitude: the bit
  field is an unsigned magnitude, and if payload bit `sign_bit` is `1` the
  magnitude is negated before scaling. `sign_bit` uses the same bit numbering
  as `start_bit` (`byte_index * 8 + bit_in_byte`, bit 0 = LSB of byte 0), it
  must not overlap the magnitude field, and `signed` must be `false`.
- Runtime value is `raw * factor + offset`, where `raw` is the magnitude,
  negated when `sign_bit` is set.
- `unit` is the unit of the decoded runtime value, spelled as in
  [Units](#units).
- `values` is optional: a value table for enumerated raw codes (gear, mode,
  status), the VSP form of a DBC `VAL_` table. Each entry has an integer `raw`
  (after `signed` / `sign_bit` interpretation, before scaling), a display
  `label`, and an optional `value`:
  - `value` absent: the runtime value is `raw * factor + offset` as usual.
  - `value` a number: use it as the runtime value for this raw code.
  - `value` `null`: this raw code is a state with no numeric value; show the
    label only.
  Raw codes without an entry decode normally. `raw` values are unique and
  inside the field's raw range.
- `mux` is optional. When present, the signal is valid only when the mux field
  equals `mux.value`. See [Mux Objects](#mux-objects).
- `value_type` is optional: `integer` (the default when absent), `float32` or
  `float64`. A float field is the IEEE 754 binary32 (`bit_length` 32) or
  binary64 (`bit_length` 64) bit pattern, read with `byte_order` like any field
  and then reinterpreted as a float; `signed` is `false`, and `sign_bit` and
  `values` are absent. Runtime value is `float * factor + offset`.
- `polynomial` is optional: coefficients `[c0, c1, c2, ...]` (at least three)
  of a non-linear conversion, such as a thermistor curve. Runtime value is
  `c0 + c1 * raw + c2 * raw^2 + ...`, where `raw` is the integer after
  `signed` / `sign_bit` interpretation. `factor` is `1` and `offset` is `0`.
- An application that does not implement `value_type` or `polynomial` must
  skip the signals that use them, not decode them as plain integers.

Float and polynomial examples:

```json
"decode": {"start_bit": 0, "bit_length": 32, "byte_order": "little_endian", "signed": false,
           "value_type": "float32", "factor": 1, "offset": 0, "unit": "C", "mux": null}
"decode": {"start_bit": 16, "bit_length": 8, "byte_order": "little_endian", "signed": false,
           "factor": 1, "offset": 0, "polynomial": [163.4003, -2.240209, 0.0153881, -4.4224167e-05, 2.3601227e-08],
           "unit": "C", "mux": null}
```

Value table example (BMW F-series 0x3F9 gear nibble):

```json
"values": [
  {"raw": 1, "label": "N", "value": 0},
  {"raw": 2, "label": "R", "value": -1},
  {"raw": 3, "label": "P", "value": null},
  {"raw": 5, "label": "1", "value": 1}
]
```

Gear signals follow the numeric convention in
[CANONICAL_SIGNAL_IDS.md](CANONICAL_SIGNAL_IDS.md#gear-values).

## Mux Objects

```json
"mux": {"value": 16652, "start_bit": 15, "bit_length": 16, "byte_order": "big_endian"}
```

- The mux field is an unsigned bit field of the same payload as the signal.
  `start_bit`, `bit_length` and `byte_order` follow the `decode` rules exactly
  (`start_bit` is the LSB for `little_endian` and the MSB for `big_endian`).
- `byte_order` may be absent in older profiles; treat it as `little_endian`.
- Decode the mux field first; skip the frame for this signal unless it equals
  `value`.
- A profile converted from a DBC can instead carry `{"value": n}` alone. The
  mux field is then the signal of the same frame whose `decode.mux` is
  `{"role": "multiplexor"}`.

## Diagnostic Response Frames

Diagnostic response layouts are per CAN frame. Decode every received frame on
`response_frame_id` on its own, with byte 0 being the first byte of the CAN
payload: the ISO-TP PCI byte(s) are part of the layout and responses are not
reassembled first.

- Single frame: `04 41 0C 1A F8` has the Mode 01 mux `41 0C` in bytes 1-2
  (`start_bit` 15, 16 bits, big-endian) and the value in bytes 3-4.
- Multi-frame: each frame is matched by its own mux. A consecutive frame is
  usually selected by its sequence byte (`0x21`, `0x22`, ...) in byte 0; a first
  frame `10 09 62 2B 21 ..` by the response SID and DID in bytes 2-4.

Send the profile's `flow_control` query when a multi-frame response needs it.
Without flow control an ECU sends only the first frame; some profiles read
their value from that frame alone.

## CAN ID Masks

`can_frames[].frame_id_mask` is optional (`frame_id_mask_hex` is the same value
in hex). When present, a received frame belongs to this layout when
`(received_id & frame_id_mask) == (frame_id & frame_id_mask)`. Some devices
put a node or sequence number in the low ID bits. Signals reference the frame by
`frame_id`; look up the mask in `can_frames[]`. Without a mask, the full ID must
match.

## Reassembled Frames

`can_frames[].reassembled` is optional. `true` marks a layout that is not one
CAN frame but a buffer the source logger builds from several frames of a
vendor stream (AiM's drivers for MoTeC dash data, Sybele, Emerald and others).
VSP does not define how the buffer is built: `dlc` is the buffer length, and
`frame_id` can be the logger's placeholder rather than an ID on the bus. An
application that does not implement the vendor's framing skips the signals
of a reassembled frame. On a `can` bus every other frame has a `dlc` of 8 or
less.

## Frame Periods

`can_frames[].cycle_ms` is optional: the frame's nominal transmit period in
milliseconds (`100` = sent every 100 ms). `can_frames[].cycle_basis` says where
it comes from:

- `measured`: the mean interval over listen-only captures of that vehicle,
  steady in every capture.
- `reference`: a published database of the same message (for example a DBC
  `GenMsgCycleTime` in opendbc) whose field layout matches the profile's.
- absent: the profile's source file (a DBC `GenMsgCycleTime`).

A frame without `cycle_ms` may still be periodic; it may also be sent on events.
Use the period to tell a stale value from a steady one: when no frame has
arrived for about three periods, the signal's last value is old. A logger can
also sample a signal at its frame rate instead of a fixed default.

### CAN Bit Numbering

For passive CAN layouts, `start_bit` preserves the DBC bit numbering used by
the public conversion tools. It is not a byte offset. Bits inside each byte
are numbered 0 (least significant) through 7 (most significant).

- `little_endian`: `start_bit` is the least significant signal bit. Advance
  to `bit + 1` for each more significant bit.
- `big_endian`: `start_bit` is the most significant signal bit (DBC Motorola
  sawtooth numbering). Advance to `bit - 1`, except at bit 0 of a byte, where
  the next bit is `bit + 15`.

A decoder using contiguous network-MSB0 coordinates must convert a Motorola
start bit with `8 * (start_bit / 8) + 7 - (start_bit % 8)`, using integer
division. Copying `start_bit` unchanged into a network-MSB0 decoder is incorrect.
The bit length and the payload bytes do not change during this conversion.
Reject a layout if any addressed bit lies outside the declared frame payload.

Examples, before scaling or signed interpretation:

| Payload bytes | Byte order | Start bit | Bit length | Unsigned raw value |
| --- | --- | --- | --- | --- |
| `12 34` | `little_endian` | 4 | 12 | `0x341` |
| `12 34` | `big_endian` | 7 | 16 | `0x1234` |
| `01 80` | `big_endian` | 0 | 8 | `0xC0` |

For signed integers, apply two's-complement interpretation at `bit_length`
before applying factor and offset. For example, little-endian `9C FF`,
start bit 0, length 16, signed, represents -100 before scaling.

These examples specify decoding behavior, not evidence that any vehicle's
published signal layout or scale has been validated on hardware. Likewise,
canonical-name mapping confidence is not a vehicle validation status. Consumers
must not arbitrarily select the first of several channels with the same
canonical ID when their layouts differ.

## Diagnostic Queries

Each diagnostic query has a stable `id` referenced by diagnostic signals:

```json
{
  "id": "diag_7DF_010C",
  "bus": "can0",
  "kind": "obd_mode_01",
  "role": "channel_query",
  "tx_id": 2015,
  "tx_id_hex": "0x7DF",
  "rx_id": 2024,
  "rx_id_hex": "0x7E8",
  "request_hex": "01 0C",
  "payload_hex": "02 01 0C AA AA AA AA AA"
}
```

Roles:

- `channel_query`: directly feeds one or more diagnostic signals.
- `inferred_channel_query`: a channel query inferred from profile evidence and
  standard diagnostic semantics.
- `support_query`: capability discovery query. It may be useful before polling,
  but it does not directly feed a signal.
- `flow_control`: ISO-TP flow-control frame needed by some multi-frame
  responses.

Applications normally poll `channel_query` and `inferred_channel_query`
entries. ISO-TP implementations should also honor required `flow_control`
entries.

## Canonical IDs

`signals[].canonical.id` is the stable semantic name applications should use
for feature mapping. See [CANONICAL_SIGNAL_IDS.md](CANONICAL_SIGNAL_IDS.md)
for the registry.

Examples:

```text
engine.speed
steering.wheel.angle
accelerator.pedal.position
throttle.position
brake.pedal.position
brake.pressure
transmission.gear.selected
```

The same canonical ID may appear more than once when acquisition modes differ.
For example, a profile may contain both:

- `engine.speed` with `acquisition.type == "can_frame"`
- `engine.speed` with `acquisition.type == "diagnostic_query"`

This is intentional. CAN sniffing apps choose the CAN signal. OBD/query apps
choose the diagnostic signal.

## Units

`decode.unit` is one of these spellings (ASCII; `""` is a plain number, count or
code):

| Quantity | Units |
|---|---|
| none, switch, gear | `""`, `on/off`, `gear` |
| ratio | `ratio` (0-1), `%`, `%/s`, `lambda`, `AFR` |
| speed | `km/h`, `mph`, `m/s` |
| rotation, frequency | `rpm`, `krpm`, `rpm/s`, `Hz`, `bpm` |
| angle | `deg`, `rad`, `deg/s`, `rad/s` |
| acceleration | `g`, `m/s^2` |
| temperature | `C`, `F` |
| pressure | `bar`, `mbar`, `kPa`, `psi`, `kPa(g)`, `psi(g)` (gauge) |
| electric | `V`, `mV`, `A`, `W`, `kW` |
| time | `us`, `ms`, `s`, `min`, `h` |
| distance | `mm`, `m`, `km`, `mi` |
| volume, flow, consumption | `L`, `mm^3`, `L/s`, `L/h`, `L/100km`, `km/L`, `g/s`, `kg/s`, `kg/h` |
| force, torque, mass | `N`, `kgf`, `Nm`, `kg`, `kg/m^3` |

Absolute pressure has no suffix; `kPa(g)` and `psi(g)` are gauge pressure
(relative to the ambient pressure). A canonical ID's `preferred_unit` uses the
same spellings, plus `bool` (a two-state flag), `state` (an enumerated state)
and `count` (a whole number of things, such as a lap number), which name a
kind of value rather than a unit.

## Decode Basis

`signals[].decode_basis` is optional and says how the decode (bit layout,
sign, scale) was checked beyond its source file:

- `measured`: checked on a vehicle (a capture with a known action, such as
  turning the wheel left).
- `standard`: matches a published standard; a Mode 01 signal reproduces its
  SAE J1979 formula.
- `reference`: corrected or confirmed against a published decode of the same
  field, such as opendbc.
- absent: as the source file (an AiM driver, a VBOX database, a DBC) defines it,
  not checked independently.

## Signal Roles

`signals[].role` is optional and marks a field that is not a measurement to
plot:

- `counter`: a message counter (alive or rolling counter) that steps every
  frame.
- `checksum`: a checksum or CRC over the frame.
- `bitmask`: a word of independent flag bits. Show the bits, not the number. In
  its `values`, an entry whose `raw` is a power of two names that single bit.
- `identifier`: a constant code that names software or a calibration (such as a
  strategy CRC). Show it in hex.

## Profile Selection

Use `vehicle` and `applicability` to select a compatible profile for the target
vehicle. A `.vsp` file describes one compatible vehicle scope. If model year,
generation, market, trim, or powertrain changes the signal behavior, use a
separate profile.

Profile filenames and applicability are generation/scope-specific. Do not infer
that a base-model profile applies to another generation or trim unless the
selected `.vsp` already includes those channels.

The filename also shows the acquisition modes: `<name>__obd.vsp` has only
diagnostic-query signals, `<name>__can_obd.vsp` has both passive CAN and
diagnostic-query signals, and a name without either suffix is passive CAN only.

## Generated Profile Composition

Public profiles may be generated from multiple compatible source formats. A
single `.vsp` can contain both passive CAN signals and diagnostic-query signals
when build-time evidence shows they share a compatible vehicle scope.

Build-time composition follows these runtime-visible rules:

- Source labels may be corrected before publication when reviewed build
  evidence shows a supplier or protocol namespace was parsed as the vehicle
  manufacturer.
- Same-scope sources can be merged so both acquisition paths stay available.
- Reviewed alias/supplement evidence or strict compatibility checks may add
  base-model passive CAN definitions to a trim-specific OBD-focused profile,
  but the generated profile must still be selected by its own `vehicle` and
  `applicability` scope.
- Generation, platform, year, market, trim, and powertrain boundaries remain
  profile-selection boundaries unless the generated profile already represents
  a compatible merged scope. Revision or bitrate-like source suffixes may be
  collapsed by the generator when the signal layouts prove duplicate coverage.
- Conflict resolution must not collapse acquisition modes or distinct CAN
  layouts from different source files. If the same canonical ID appears in both
  CAN and diagnostic acquisition modes, or in multiple source-backed CAN frame
  layouts, each compatible channel can be present and runtime code should
  choose by `signals[].acquisition`.

Applications do not need source provenance. Select the profile by `vehicle` and
`applicability`, then choose channels by acquisition mode.

## Implementation Checklist

- Load the `.vsp` JSON.
- Verify `schema == "vehicle_data_profile.v1"`.
- Select runtime mode: CAN sniffing or OBD/query.
- Filter `signals[]` by `acquisition.type`.
- Map required features by `canonical.id`.
- Decode using `decode`; skip signals whose `value_type` or `polynomial` you
  do not implement.
- Skip signals on frames with `reassembled: true` unless you implement that
  vendor's stream.
- Optionally use `can_frames[].cycle_ms` to mark stale values, and
  `signals[].role` to keep counters, checksums and flag words out of gauges.
- Ignore unknown fields.
