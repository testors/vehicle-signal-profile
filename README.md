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
    └── aim/                   # Race Studio 3 driver template for vsp2xc1
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

DBC also has no sign-magnitude type. A sign-magnitude VSP signal (`sign_bit`)
is exported as an unsigned magnitude signal `<Name>` plus a 1-bit `<Name>_Sign`
signal in the same message. `dbc2vsp.py` folds such a pair back into one
signal with `sign_bit`.

Float fields (`decode.value_type`) map to DBC `SIG_VALTYPE_` in both
directions. DBC has no polynomial conversion, so a `decode.polynomial` signal
is exported as its raw field (factor 1, offset 0) with the coefficients in a
`CM_ SG_ ... "VSP polynomial c0..cn: ..."` comment, which `dbc2vsp.py` reads
back.

Value tables (`decode.values`) map to DBC `VAL_` tables in both directions.
DBC keeps only the labels; the optional numeric `value` overrides are lost on
export.

Frame periods (`can_frames[].cycle_ms`) map to DBC `BA_ "GenMsgCycleTime"` in
both directions; `cycle_basis`, `decode_basis`, `role` and `reassembled` have no
DBC form and are lost on export.

Compose multiple VSP files:

```bash
python3 tools/merge_vsp.py base.vsp overlay.vsp --prefer error -o out/combined.vsp
```

## AiM Race Studio 3

`tools/vsp2xc1.py` writes a VSP's passive CAN signals (`acquisition.type`
`can_frame`) as a Race Studio 3 custom CAN driver (`.xc1`). Diagnostic queries
are left out: a custom CAN driver only listens. Each signal becomes one AiM
channel with its layout, sign, mux, scale or polynomial and value table.
Channels get AiM's unit, quantity and function id as AiM's own drivers write
them (`--aim-sensors off` keeps Race Studio's DBC-import "#" numbers), and a
frame with `cycle_ms` sets its channels' maximum frequency. Signals AiM cannot
hold are printed as notes and kept in the manifest.

```bash
python3 tools/vsp2xc1.py profiles/<profile>.vsp \
  --uid <id> --model "<model>" --index-fragments -o out/aim
```

This writes `<id>.xc1`, its XML, `<id>.manifest.json` and, with
`--index-fragments`, the two Race Studio list entries. The driver is
listen-only (`--silent on`) with the logger's 120 ohm terminator off
(`--termination off`) by default, for a tap on a bus that is already
terminated. The bit rate comes from the profile's bus (`--bitrate` overrides).

To install it, close Race Studio 3 and back up the two list files, then:

1. Copy `<id>.xc1` to `C:\AIM_SPORT\RaceStudio3\user\resources\can\custom\`.
2. Add `<id>.indiceCANCustom.entry.xml` to `indiceCANCustom.xml` in that
   folder, and `<id>.xmldb_unicode32_custecus.entry.xml` to
   `C:\AIM_SPORT\RaceStudio3\user\xmldb_unicode32_custecus.xml`, next to the
   existing entries.
3. Pick the manufacturer and model in the logger's ECU stream (CAN) setting.

Alternatively, export a DBC with `tools/vsp2dbc.py` and use Race Studio's DBC
import. Sign-magnitude signals then arrive as two channels, and the listen-only
and terminator settings should be checked after the import.

An `.xc1` file is the driver's XML encrypted with AES-128-ECB under Race
Studio's default no-password key (a 4-byte little-endian length, then the
zero-padded XML). `tools/aim_xc1.py` holds that key for interoperability and
needs the `openssl` command; `python3 tools/aim_xc1.py unpack <id>.xc1`
decrypts a driver for inspection and `pack` encrypts XML again. Drivers are
built on `tools/aim/rs3_can_template.xml`, a Race Studio 3 DBC-import driver
cut down to one placeholder channel; `--template` takes your own driver (XC1
or XML) instead.

## Runtime Use

Runtime apps should start from top-level `signals[]` and choose channels by
`signals[].acquisition.type`:

- `can_frame`: listen for the referenced CAN frame and decode it passively.
- `diagnostic_query`: send the referenced query and decode the response frame.

Profile file names show which modes a profile covers: `__obd.vsp` is
diagnostic queries only, `__can_obd.vsp` is both, and no suffix is passive CAN
only.

A distributed profile may be composed from compatible DBC, XC1, and diagnostic
evidence. Select the exact profile for the vehicle scope first; do not infer
that a sibling base-model or trim profile applies unless those channels are
already present in the selected `.vsp`.

Do not treat every `can_frames[]` entry as passive broadcast data. Diagnostic
response layouts are also stored there so both modes can use the same bit
decoder.

Validate a profile:

```bash
python3 tools/validate_vsp.py profiles/<profile>.vsp
```
