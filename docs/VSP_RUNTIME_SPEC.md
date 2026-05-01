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

1. Select signals where `acquisition.type == "can_frame"`.
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
- `signed` controls signed integer interpretation before scaling.
- Runtime value is `raw * factor + offset`.
- `unit` is the unit of the decoded runtime value.
- `mux` is optional. When present, the signal is valid only when the mux field
  equals `mux.value`.

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

## Profile Selection

Use `vehicle` and `applicability` to select a compatible profile for the target
vehicle. A `.vsp` file describes one compatible vehicle scope. If model year,
generation, market, trim, or powertrain changes the signal behavior, use a
separate profile.

Profile filenames and applicability are generation/scope-specific. Do not infer
that a base-model profile applies to another generation or trim unless the
selected `.vsp` already includes those channels.

## Generated Profile Composition

Public profiles may be generated from multiple compatible source formats. A
single `.vsp` can contain both passive CAN signals and diagnostic-query signals
when build-time evidence shows they share a compatible vehicle scope.

Build-time composition follows these runtime-visible rules:

- Same-scope sources can be merged so both acquisition paths stay available.
- Reviewed alias/supplement evidence or strict compatibility checks may add
  base-model passive CAN definitions to a trim-specific OBD-focused profile,
  but the generated profile must still be selected by its own `vehicle` and
  `applicability` scope.
- Generation, platform, year, market, trim, and powertrain boundaries remain
  profile-selection boundaries unless the generated profile already represents
  a compatible merged scope. Revision or bitrate-like source suffixes may be
  collapsed by the generator when the signal layouts prove duplicate coverage.
- Conflict resolution must not collapse acquisition modes. If the same
  canonical ID appears in both CAN and diagnostic acquisition modes, both can
  be present and runtime code should choose by `signals[].acquisition.type`.

Applications do not need source provenance. Select the profile by `vehicle` and
`applicability`, then choose channels by acquisition mode.

## Implementation Checklist

- Load the `.vsp` JSON.
- Verify `schema == "vehicle_data_profile.v1"`.
- Select runtime mode: CAN sniffing or OBD/query.
- Filter `signals[]` by `acquisition.type`.
- Map required features by `canonical.id`.
- Decode using `decode`.
- Ignore unknown fields.
