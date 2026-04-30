# Vehicle Signal Profile

Vehicle Signal Profile (`.vsp`) is a JSON format for describing vehicle
telemetry channels that can be read either from passive CAN frames or active
diagnostic queries.

The public runtime package is:

- ready-to-use `.vsp` files under `profiles/`
- the runtime format and usage docs under `docs/`

Only the runtime files in this repository are part of the public contract.

## Layout

```text
vehicle-signal-profile/
├── profiles/                  # ready-to-use public .vsp files
├── docs/                      # public runtime docs
└── tools/                     # public VSP conversion and validation scripts
```

## Docs

- [docs/VSP_RUNTIME_SPEC.md](docs/VSP_RUNTIME_SPEC.md): app-facing `.vsp`
  runtime contract and usage guide.
- [docs/CANONICAL_SIGNAL_IDS.md](docs/CANONICAL_SIGNAL_IDS.md): stable
  semantic IDs used by applications.

## DBC Input

Generate a passive-CAN VSP from a DBC:

```bash
python3 tools/dbc2vsp.py path/to/vehicle.dbc \
  --manufacturer Example --model Vehicle \
  -o out/example_vehicle.vsp
```

DBC can represent passive CAN frame layouts. Profiles that also contain active
diagnostic queries are distributed as ready-to-use `.vsp` files under
`profiles/`.

Export passive CAN frame layouts from a VSP to DBC:

```bash
python3 tools/vsp2dbc.py profiles/<profile>.vsp -o out/<profile>.dbc
```

`vsp2dbc.py` is a lossy export for interoperability. DBC cannot represent active
diagnostic queries, so diagnostic response frame layouts are omitted unless
`--include-diagnostic-responses` is passed.

Compose multiple VSP files:

```bash
python3 tools/merge_vsp.py base.vsp overlay.vsp --prefer error -o out/combined.vsp
```

## Runtime Use

Runtime apps should start from top-level `signals[]` and choose channels by
`signals[].acquisition.type`:

- `can_frame`: listen for the referenced CAN frame and decode it passively.
- `diagnostic_query`: send the referenced query and decode the response frame.

Do not treat every `can_frames[]` entry as passive broadcast data. Diagnostic
response layouts are also stored there so both modes can use the same bit
decoder.

Validate a profile:

```bash
python3 tools/validate_vsp.py profiles/<profile>.vsp
```
