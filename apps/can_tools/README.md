# CAN Tools

CAN bus tools that work through BLE / Classic Bluetooth OBD adapters. They run on macOS and support ELM327 / STN2255
compatible adapters (OBDLink MX+, vLinker MC+ and others).
A PEAK PCAN-USB can also receive the whole bus, listen-only (see "PCAN-USB" below).

## Tools

There is one executable, `build/can_tools`; `can_tools <command> [args...]` picks the command.

| Command | Description | CAN transmit |
|------|------|----------|
| **scan** | Scan for BLE / Classic OBD adapters (JSON), plus PCAN-USB channels when PCBUSB is installed | none |
| **monitor** | Passive CAN monitor (silent mode, no ACK) | none |
| **detect** | Detect the bus type (3 s of receiving) | none (`--probe` sends `0100`) |
| **discover** | Guided PT-CAN signal discovery (interactive) | none |
| **probe** | Check that a VSP profile's CAN / OBD signals can be read at a target Hz | none for `--plan` and `--mode can`; `--mode obd` sends diagnostic queries; auto queries only when no periodic broadcast frames are heard |
| **replay** | Re-run discovery on a saved capture, without a vehicle | none (no adapter needed) |
| **query** / **supported** / **raw** | OBD-II PID query, supported PID list, raw AT command | transmits (OBD port only) |
| **p992** | Porsche 992 legacy/debug command set | transmits |
| **tui** | btop-style live dashboard: connect screen, signals, Raw CAN, log, discover, recording, replay | none (diagnostic queries only while TX mode is on) |
| (no arguments) | In a terminal, `tui` (connect screen); otherwise prints usage and exits with code 2 | none |
| **menu** | The old interactive menu | depends on the menu item |

`can_tools help` lists every command and whether it transmits. The build also creates symbolic links with the old tool
names in `build/`: `can_reader`, `can_discover`, `vsp_probe` and `discover_replay`. Run under one of those names, the
executable takes the old tool's arguments and help (for example `./build/can_reader monitor vLinker 10`,
`./build/vsp_probe <profile> --plan`). `can_tools discover` may omit the device, as `can_discover` does;
`can_reader discover` still needs one, as before.

`discover` (`can_reader discover`, `can_discover`) always runs the same discovery logic (`discover_session.m`,
`can_discover.c`). Every command that connects to an adapter scans, connects and initializes through the same path
(`common/obd_connect`). It waits up to 10 s for BLE, scans for 8 s and scans once more if nothing was found; paired
Classic adapters stay in the list even with BLE off. The device name matches case-insensitively as a substring; when
`discover` gets none it uses the first adapter. Receive-only commands (monitor, detect, discover, the CAN phase of
probe) use the PASSIVE init (`ATZ`, `ATE0`, `ATL0`, `STI`, `ATSP6`, `ATCSM1`, `ATS1`, `ATCAF0`, `ATH1`); query commands
(query, supported, raw) use the ACTIVE init (the ELM327 init commands plus STN options). The PASSIVE init succeeds only
when the `ATCSM1` reply contains `OK`: no reply, or `?` (unsupported), leaves it unconfirmed that the adapter sends no
ACK, so it logs `[ERROR] silent mode not confirmed: ATCSM1 …` and does not start the monitor (the TUI connect screen
shows the reason as `adapter init failed: …`).

## Build

Build in the `apps/can_tools` folder of the vehicle-signal-profile repository (every path in this document is
relative to that folder). python3 is required: the build generates the prior table from the repository's `profiles/`
(`../../profiles`; see "Generating the prior table").

```bash
./build.sh
```

or:

```bash
mkdir -p build && cd build && cmake .. && make
```

The build produces `build/can_tools` and four alias symlinks (`can_reader`, `can_discover`, `vsp_probe`,
`discover_replay`). `build/can_tools_fake` is a test-only build (a fake adapter instead of Bluetooth; it never touches
a real device). The TUI uses ncurses (part of the macOS SDK).

The build also produces `build/libcan_discover_core.a`. Another app that only wants the discovery engine can include
`common/can_discover.h` and call `disc_analyze_with_raw()`, `disc_render_draft_dbc()` and `disc_render_draft_vsp()`
directly.

## PCAN-USB (PEAK, receive-only)

Bluetooth adapters deliver frames as text and the host stamps their arrival time (measured: ~20 ms latency as a rule,
50–130 ms batches, ms resolution). A PEAK PCAN-USB receives the whole bus over USB with µs timestamps from the CAN
controller. In this tool PCAN is **receive-only**: it sends no ACK, no error frame, no frame of any kind.

**Install.** PEAK has no macOS driver; use UV Software's PCBUSB library (PCAN-Basic compatible, free binary):
<https://github.com/mac-can/PCBUSB-Library>. The package's `install.sh` installs `/usr/local/lib/libPCBUSB.dylib`
(plus versioned files) and `/usr/local/include/PCBUSB.h` (`sudo` may be needed without write access to `/usr/local`).
The build does not need it: at run time `can_tools` `dlopen`s only the two absolute paths
`/usr/local/lib/libPCBUSB.dylib` and `/opt/homebrew/lib/libPCBUSB.dylib` (never the current folder or `DYLD_*`
paths); without the library only the PCAN commands fail, with the install link, and everything else works.
**Minimum version 0.13.** After loading the library, and before touching a channel, it reads `PCAN_CHANNEL_VERSION`
and accepts only 0.x from 0.13 up (0.13 is the version tested). An older version, another major version or an
unreadable version is refused with the string found, and no channel is initialized. The version in use goes to the
connect log, the monitor/detect JSON (`library_version`) and the TUI's `session.json`.
Devices PCBUSB 0.13 supports: PCAN-USB (IPEH-002021/002022), PCAN-USB FD (IPEH-004022), PCAN-USB Pro FD (CAN1 only).
It is a universal binary, so it runs on Apple silicon too.

**Device names.** In place of `<device>` use `pcan` (the first available channel) or `pcan1`..`pcan8` (any case).
`--bitrate RATE|auto`: `1M 800k 500k 250k 125k 100k 95k 83k 50k 47k 33k 20k 10k 5k`, default 500k. `auto` opens
500k, 250k, 1M, 125k, 100k, 83k, 50k and 33k listen-only in turn, listens up to 0.5 s on each, and picks the first
rate on which 20 data frames from at least 2 ids are heard (the rates tried and the outcome go to the log).

```bash
./build/can_tools scan                                   # BT adapters + PCAN channels (transport "usb")
./build/can_tools monitor pcan 10 --out mon.json         # whole bus, 10 s; ids given = software filter
./build/can_tools monitor pcan1 10 --bitrate auto 0A5 1A0
./build/can_tools detect pcan                            # 3 s of receiving, bus classification (no ATRV, no --probe)
./build/can_tools discover pcan                          # guided discovery, phase dumps are 6-column TSV
./build/can_tools probe <profile.vsp> --device pcan --mode can --seconds 10
./build/can_tools tui --device pcan --bitrate 500k       # the connect screen lists PCAN channels as "USB" too
```

The monitor JSON carries `"interface":"pcan"`, `"bitrate"`, `"mode":"listen-only"` and a `"timestamp_us"` (epoch µs)
per frame. The TUI header shows `pcan1 PCAN-USB · … · listen-only 500k`, and the log notes that timestamps are
hardware µs. The connect screen re-reads the channel states about once a second, so plugging and unplugging shows,
and the `last:` line remembers PCAN channels too. Unplugging the USB while in use turns the dashboard DISCONNECTED.

**Listen-only guarantee and implementation.**
- The library function table (`pcan_api_t` in `common/pcan.h`) holds receive functions only: `CAN_Initialize`,
  `CAN_Uninitialize`, `CAN_GetStatus`, `CAN_Read`, `CAN_GetValue`, `CAN_SetValue`, `CAN_GetErrorText`.
  `CAN_Write`/`CAN_WriteFD` are neither declared nor `dlsym`ed, and `CAN_Reset` is not used either (a test checks the
  strings in the binaries).
- `pcan_open()` is the one place that opens a channel: `PCAN_LISTEN_ONLY = ON` on the uninitialized channel → read it
  back, still before init, and check it is ON (a failed read or anything but ON fails without initializing) →
  `CAN_Initialize` → read `PCAN_LISTEN_ONLY` again and, if it is not ON, `CAN_Uninitialize` at once and fail. A failed
  set never initializes. `CAN_SetValue` is called only here, and only with `PCAN_LISTEN_ONLY = ON`. Bitrate auto goes
  through the same steps for every rate it tries, and stops instead of trying another rate when listen-only cannot be
  confirmed.
- Commands that transmit refuse PCAN (before creating the adapter object or loading the library, exit code 1):
  `query`, `supported`, `raw`, `p992`, `detect --probe`, `probe --mode obd`, `probe --mode auto` (it may move on to
  queries) → `PCAN is listen-only: <command> would transmit`. Use `probe --mode can` with PCAN. The TUI's TX mode (`x`)
  is refused at once with `TX unavailable: PCAN is listen-only`, without stopping the monitor, and a PCAN connection
  has no transmit callback at all.
- PCAN commands (monitor, detect, discover, probe) create no Bluetooth object.

**First use: check on a bench before plugging into a car.** Put the PCAN-USB (listen-only under can_tools) and one
other CAN node alone on a bench bus, properly terminated with 120 Ω at both ends, and let that node send frames. With
nobody to ACK, the sender reports ACK errors (`CAN ERROR` on an STN/ELM adapter). Were the PCAN to ACK, the send would
succeed, and an ELM/STN would answer an OBD request with `NO DATA` instead of `CAN ERROR`. Meanwhile can_tools should
keep receiving the same retransmitted frame. The sender reporting ACK errors while can_tools receives the frames means
listen-only works.

**Timestamps and the 6-column TSV.** Controller time (µs) is moved onto the wall clock with
`min(host receive time − controller time)` over the first received batch as the reference, which then stays fixed (a
monotonic mapping, so each id's period stays exact). The drift from the host clock is logged when the channel closes.
Frames with hardware timestamps get a 6th column `timestamp_us` (epoch µs) in `*.frames.tsv` (TUI recordings,
discover phase dumps), named in the header too. Column 1, `timestamp_ms`, is `floor(timestamp_us / 1000)`, so tools
that read 5 columns see the same capture. The reader accepts 5 and 6 columns and rejects a row whose column 6 is not in
the same ms as column 1. Existing 5-column captures read as before. RTR, status and error frames are counted, not
passed on; receive queue overruns and read errors are counted and logged too.

**Limits.** Classic CAN only (`CAN_Initialize`/`CAN_Read`): CAN FD frames are not received. PCAN-USB Pro FD works on
CAN1 only. macOS only. Per the PCBUSB documentation, a device unplugged while in use may not answer again in the same
process after replugging: restart can_tools then. Not yet tested with a real device (the tests run against a fake
library, `tests/fake/fake_pcan.c`).

## probe (formerly vsp_probe)

Selects a CAN frame or a diagnostic query for each signal from the VSP's `signals[].canonical.id` and
`signals[].acquisition`, and reports as JSON whether the given target Hz is met.

By default it checks the rpm/gear/steering/throttle/brake groups:

```bash
# selection plan only, no hardware
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp --plan

# CAN acquisition only, 5 Hz target, 10 s
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp \
  --device vLinker --mode can --hz 5 --seconds 10

# OBD/UDS acquisition only, 5 Hz target
./build/can_tools probe ../../profiles/porsche__911_gt3__992__can_obd.vsp \
  --device vLinker --mode obd --hz 5 --seconds 10
```

The arguments are those of the old `vsp_probe` (the `./build/vsp_probe ...` alias works too).

When the profile frame has a period (`can_frames[].cycle_ms`), the plan and the results carry `frame_cycle_ms` and
`frame_hz`, and the results the observed / frame rate ratio `frame_rate_ratio`. When the target Hz is faster than the
frame and the frame arrived without gaps, the status is `frame_limited` instead of `under_target`; an observed rate
below half the frame rate is flagged `frame_rate_low` (lost frames, a gateway filter, or another period on this car).
Mux signals are not compared.

Diagnostic responses are decoded one CAN frame at a time as the VSP rules say (ISO-TP PCI included, no reassembly).
With `decode.mux` the mux picks the frame (the CF sequence number of a multi-frame response, `62 DID` in the first
frame, and so on); without it, the positive-response marker before a single or first frame (`41 PID`, `62 DID`,
`61 LID`) does. Passive CAN applies the mux and `can_frames[].frame_id_mask` as well. A mux with only `{"value": n}`
(converted from a DBC) is compared against the field of the `{"role": "multiplexor"}` signal in the layout of the
frame with the same bus and frame_id (`can_frames[].signals[]`, else the top-level `signals[]` of that frame); without
such a signal the signal is not decoded. A received frame shorter than the bytes the signal reads (field, `sign_bit`,
mux field) neither updates the value nor counts as a sample (missing bits are not read as 0; CAN counts them as
`short_frames`). 29-bit CAN ids are not supported yet.

`--mode auto` runs a CAN phase, then an OBD phase. When periodic broadcast frames are heard on the bus (a possible
direct PT-CAN tap), however, auto skips the OBD phase and transmits nothing. To query at the OBD port, say `--mode obd`.
Tapped directly into a vehicle bus, use `--mode can` only. `--signals` takes groups such as
`rpm,gear,steering,throttle,brake`, canonical ids, or VSP signal names. STN/OBDLink adapters use the raw periodic
scheduler. vLinker adapters, as the maker confirmed, keep the bus silent in `ATMA`/`STM`/`STMA` monitor mode and send
no periodic messages (PM) either, so auto falls back to polling.

## scan / detect / monitor / query / supported / raw / p992 (formerly can_reader)

Supports both active queries on D-CAN (OBD port) and passive monitoring of PT-CAN. The arguments are those of the old
`can_reader`, and the `./build/can_reader <command> ...` alias still works.

### CLI mode

```bash
# BLE scan
./build/can_tools scan

# bus type detection (receive only; are there periodic broadcast frames)
./build/can_tools detect OBDLink
# OBD port only: also sends an 01 00 query to check for D-CAN (never on a direct bus tap)
./build/can_tools detect OBDLink --probe

# supported PID list (D-CAN)
./build/can_tools supported OBDLink

# PID values (D-CAN)
./build/can_tools query OBDLink 0C 0D 11

# Porsche 992 legacy/debug profile commands
./build/can_tools p992 list

# Porsche 992 channels, read once
./build/can_tools p992 read OBDLink RPM THROTTLE_POS STEER_ANGLE BRAKE_PRESS

# Porsche 992 channel streaming (NDJSON, every 500 ms)
./build/can_tools p992 watch OBDLink --interval-ms 500 RPM STEER_ANGLE BRAKE_PRESS

# Porsche 992 route / raw response diagnostics
./build/can_tools p992 debug OBDLink
./build/can_tools p992 bin-probe OBDLink

# Porsche 992 ECU scan for a DID
./build/can_tools p992 scan-did OBDLink 5101
./build/can_tools p992 scan-did OBDLink 5101 --session 03

# Porsche 992 low-level UDS DID read
./build/can_tools p992 uds-read OBDLink F190 --tx-id 7E4 --rx-id 7EC
./build/can_tools p992 uds-raw OBDLink 3E00 --tx-id 7E4 --rx-id 7EC

# passive CAN monitor (PT-CAN, 10 s)
./build/can_tools monitor vLinker 10

# CAN monitor with an id filter
./build/can_tools monitor vLinker 10 0A5 1A0 316

# guided PT-CAN signal discovery
./build/can_tools discover vLinker

# raw AT command
./build/can_tools raw OBDLink "ATRV"
```

- JSON goes to stdout, logs to stderr: `2>/dev/null` leaves JSON alone to parse.
- `<device>`: BLE name prefix match (case-insensitive)
- `p992 watch` prints one JSON object per snapshot per line on stdout (NDJSON).
- `p992` is a legacy/debug command set made while validating on a real 992 GT3.

### Interactive mode

`menu` opens the old interactive menu (as does `./build/can_reader` without arguments).
`can_tools` without arguments now opens the TUI (see `tui` below).

```bash
./build/can_tools menu
```

## tui (live dashboard)

Like btop, one terminal handles connecting to the adapter, choosing a profile, live signals, Raw CAN, the log and
discover.

```bash
./build/can_tools                         # connect screen (= can_tools tui)
./build/can_tools tui --device OBDLink    # connect right away to the first adapter whose name contains OBDLink
./build/can_tools tui --device vLinker --profile ../../profiles/porsche__911_gt3__992__can_obd.vsp
./build/can_tools tui --replay captures/20261028_101500          # replay a saved session, no car needed
./build/can_tools tui --replay x.frames.tsv --snapshot-at 5 --size 120x40   # print the screen as text (no terminal needed)
```

Options: `--profile VSP|none` (without it, the profile picker after connecting; replay starts from session.json's
profile, as in "Session folder" below), `--profiles DIR` (the profile folder; by default `VSP_PROFILES_DIR`,
`./profiles`, the repository's `profiles/` (recorded as an absolute path at build time), then `profiles/` in the
executable's folder and its parents), `--no-tx`, `--no-rec`, `--captures DIR` (where session folders go, default
`captures/`), `--mono` (no color; `NO_COLOR` does the same), `--ascii` (terminals without UTF-8), and for replay only
`--speed X` (0.25–16). `--device ""` is refused.

**Connect screen**: the adapters a scan finds (RSSI for BLE, paired for Classic) keep updating, with a new scan every
8 s. ↑↓ selects and ⏎ connects. It connects to an adapter only on ⏎ or `--device`. `R` picks a saved capture from a
list and opens it as a replay (while the list is open nothing connects; an adapter that was waiting connects after
`esc` closes the list), `s` clears the list and scans again, `o` opens a typed capture path as a replay, `q` quits
(recording, `r`, is on the dashboard after connecting). The adapter and profile used last show on a `last:` line
(selected from the start), and ⏎ twice reaches the dashboard: the adapter of that name connects once a scan sees it,
and the profile picker opens on the last profile.
Connecting runs the PASSIVE init (`ATZ` … `ATCSM1` … `ATH1`, no filters), then starts the monitor with `STMA` (STN) or
`ATMA`.

**Dashboard**: on top Signals (value, unit, Hz, age, a 10 s sparkline or, when narrow, min/max, CAN id), below Raw
CAN (id, Hz, dlc, data; bytes that changed within 1 s are bright) and Log. Below 80x24 the panels show one at a time as
tabs. Without a profile (or with no signal to show), a 4-line box in place of Gauges and Signals says to press `p` to
choose a vehicle profile; on the tabs screen the same note sits at the right of the tab line.
While no profile is loaded, the frames on the bus suggest the vehicle (see "Vehicle guess" below). The header shows
the adapter, the bus verdict (`detecting…` → `PT-CAN DIRECT` / `GATEWAY` / `SILENT (no frames)`), `silent` or a red
`TX ON`, the profile, fr/s and elapsed time, and `● REC`. A signal that has not arrived for 3 times its period (at
least 300 ms) turns grey with Hz `0`, and after 10 s its value becomes `—`.
The 10 s graph follows the same rule: samples closer than that are joined (the left edge from the last sample before
the window), longer gaps are left blank.
A graph whose values in the window differ by at most 2 raw steps (LSB) draws as a flat line (linear integer fields of
6 bits or more only; float, polynomial and 5 bits or less draw as they are). The throttle, brake, RPM and speed gauges
show values below 0 as 0.
The bus verdict comes 3 s after the monitor starts and then every 5 s, over the last 3 s, with the discover rule (2 or
more ids at 20 Hz or more = DIRECT).

| Key | Action |
|----|------|
| `tab` | next panel (narrow screen: switch tabs) |
| `↑` `↓` `PgUp` `PgDn` | scroll the focused panel (a clipped panel shows `▼ n more`, `▲ n` on its bottom border) |
| `/` | filter: signal name, leading digits of a Raw CAN id (`esc` clears) |
| `s` | Raw CAN sort: hz → id → recent change |
| `d` | discover overlay |
| `p` | choose a profile (type to filter, ⏎ load, `Ctrl+N` no profile, `esc` keep) |
| `1`–`5` | load a suggested profile (no profile loaded) |
| `r` | recording on/off (live; in replay just a log line) |
| `R` | replay list (live, replay, connect screen) |
| `x` | TX mode on/off (see the safety rules below) |
| `space` `[` `]` | replay pause, slower, faster |
| `v` | last discover result |
| `?` | help |
| `q` | quit (asks while discover runs or TX mode is on); `Ctrl+C` quits at once |
| `c` | back to the connect screen after a disconnect |

**discover**: `d` opens the discovery window over the dashboard (the dashboard behind keeps updating). ⏎ starts the
next step (Baseline, Steering, Throttle, Brake, Gear, Wheel Speed), `s` skips one (not Baseline), `a` aborts (the steps
captured stay and the manifest says `interrupted`). The monitor never stops between steps. The `live` line shows the
values of the profile signals for that step, and the (id, byte) that was fixed during Baseline and now moves most, with
a sparkline. After the last step, ⏎ on the result screen loads the draft VSP straight into the dashboard (a car
without a profile → discover → check the values). `v` shows result.json. A DIRECT Baseline locks TX.

**Session folder** (made on connecting; a replay's discover uses `captures/YYYYMMDD_HHMMSS_replay/`):

```
captures/YYYYMMDD_HHMMSS/
├── live.frames.tsv          # recording (r again starts live_02.frames.tsv, ...); same format as the discover dumps
├── tui.log                  # stderr while the TUI owns the screen (also in the Log panel)
├── session.json             # adapter, transport, STN, monitor command, profile, recordings, bus verdict history
├── 00_baseline.frames.tsv … # per-step dumps when discover ran
├── manifest.json, result.json, ptcan_discover_draft.dbc, ptcan_discover_draft.vsp
```

A session folder replays as it is with `can_tools tui --replay <folder>` (the live*.frames.tsv files with frames,
else the NN_*.frames.tsv files with frames, joined, with gaps over 1 s shortened to 1 s) or `can_tools replay
<folder>`. The TUI's replay list (`R` on the connect screen, the live dashboard and in replay; `r` is recording) shows
the session folders with at least one frame and the `*.frames.tsv` files under the `--captures` folder, newest first,
up to 512 (recordings with only a header are left out), and ⏎ opens the chosen capture as a replay right away. A
capture deleted or emptied in the meantime leaves the list open with the reason, and while TX mode is on it first asks
whether to turn TX off and go on. The last adapter and profile are saved in `~/.can_tools/tui_last`. session.json and
tui_last write profile paths as absolute paths.

**The profile in replay**: `--profile` uses that profile (`none` = no profile). Without it, and for a capture chosen
from the list (`R`), the profile written in session.json of the capture folder (for a file, its folder) loads. When no
file is at that path (the repository moved, or it runs from another folder), the file of the same name in the profile
folder is used, and the Log notes `session profile moved: <old path> -> <new path>`. Without session.json, with a null
profile, or when the file is not found, the profile picker opens (on tui_last's last profile). `esc` closes it to
Raw CAN and Log alone, without a profile; `p` chooses one at any time.

**Vehicle guess**: without a profile (live or replay, even with `--profile none`) the profile folder is indexed, and
once 3 s of traffic are in, every 2 s the frames in the Raw CAN table (id, 29-bit or not, DLC) are matched against
each profile's passive CAN frames (`can_frames[]` on a CAN bus, not reassembled, not a diagnostic request or response
id). Every frame weighs log((N + 1) / profiles that list its id) (a common id many profiles share says less; the +1
keeps the weight above 0 in a folder of one profile, or of profiles with the same frames): a frame seen adds it (the
DLC must match when the profile gives one), one not seen costs 0.3 of it (it may be a frame of another bus, or an
OBD-port gateway may hide it). Candidates show only when at most 5 profiles score at least 60 % of the best with at
least half of their frames (and at least min(3, frames)) seen: more mean the traffic does not narrow it down. The best
counts the profiles that fall short too. When a profile with less than half of its frames seen leads (a profile of
several buses watched from one bus, for instance), it may be the car, so no list shows. Taking the best over the
qualifying profiles doubles the wrong lists when half of the frames are visible (simulation below).
The hint box becomes the candidate list: each row shows the vehicle, the frames seen, and the RPM, steering, throttle
and brake that profile reads from the live frames (of several signals of a kind, the first with a value in the last
2 s; `—` without a value, `n/a` when the profile has no such signal). Turn the wheel or press a pedal and pick the
candidate whose values move right. `1`–`5` load one; with an empty filter, the `p` picker puts the candidates on top
with their rank digits and preselects the first. When candidates appear or change while the picker is open it
re-sorts: an untouched selection follows the first candidate, one the user moved (moving, typing a filter) stays on
its profile. A change of candidates leaves a Log line.
A real 992 GT3 capture (direct PT-CAN tap) ranks `911 GT3 992` (12/12), `911 992` (10/11), `Audi A8 D5` (7/7). In a
simulation over the profile corpus (each of 400 profiles' frames plus 30 unrelated ids), with every frame visible a
list shows 76 % of the time and always holds the right car; with half of the frames visible it shows 58 % of the time
and holds the right car 85 % of the time. Without the car's profile in the folder (same-vehicle profiles removed for
the test), a list of other profiles showed 66 % of the time. Mostly the same make; about a quarter of the first
candidates have practically the same frames (90 % or more) and read correctly, the rest do not. So the list is a set
of candidates to check: pick by whether the live values are right.

**Safety rules (TX mode)**: by default the TUI sends nothing on the bus (ATCSM1 silent mode, not even ACKs). Only TX
mode, turned on with `x`, queries the profile's diagnostic signals (OBD/UDS), and it turns on only when all of these
hold: a live connection (not replay), a GATEWAY or SILENT bus verdict, no DIRECT verdict ever on this connection
(DIRECT locks until the connection drops, discover's Baseline included), no `--no-tx`, at least one diagnostic signal
in the profile, and `y` in the confirmation. On DIRECT, `x` only logs a line (`TX locked: DIRECT bus`).
Once on, it stops the monitor, sets up polling (`ATCSM0` and so on, the same code as `vsp_probe`'s poll mode) and
cycles through the distinct queries at no more than 10 per second. Turning it off (`x`, quitting, cleanup after a
disconnect) runs the PASSIVE init again (ATZ clears headers and periodic messages, ATCSM1 comes back) and restarts the
monitor or disconnects. When that re-init fails (ATCSM1 not `OK`), the adapter may still send ACKs, so the monitor
stays off, the header turns a red `NOT SILENT`, and the Log notes `[ERROR] TX off: silent mode (ATCSM1) not confirmed
…`. `c` back to the connect screen and connecting again redoes the PASSIVE init from the start. **Never turn TX mode
on while tapped directly into PT-CAN** — use it at the OBD port only.

## discover (formerly can_discover)

Guided PT-CAN signal discovery. The arguments are those of the old `can_discover` (the `./build/can_discover ...` alias
works too).

```bash
# a given adapter
./build/can_tools discover vLinker

# connect to the first adapter found
./build/can_tools discover
```

### Flow

Six guided capture phases identify the key motorsport signals:

```
Phase 1: Baseline     (5 s)  — normal traffic
Phase 2: Steering     (8 s)  — full left / right, repeatedly
Phase 3: Throttle     (8 s)  — accelerator to the floor and released, repeatedly
Phase 4: Brake        (8 s)  — brake fully pressed and released, repeatedly
Phase 5: Gear        (10 s)  — shift P → R → N → D [skippable]
Phase 6: Wheel Speed (15 s)  — drive slowly, 30+ km/h [skippable]
```

Nobody picks CAN candidates after each phase. Once the capture is done the tool infers them:

1. For every phase, each CAN id's raw frames are compared with the baseline, and counter / checksum / constant bytes
   are filtered out. A byte with 16 or more values in both phases counts as a checksum only when its mean change
   between frames (0–128, the shorter way round) is also 32 or more: a checksum jumps by ~64 on average, while a
   sensor byte whose noise at rest exceeds 16 values moves a little, and stays.
2. For each signal, the ids that changed most in its phase are chosen, and within each run of changing bytes (region)
   the actual bitfield (start bit, length, byte order, sign) is searched. Besides unsigned and two's complement,
   steering also tries sign-magnitude (a magnitude plus a separate sign bit, e.g. Porsche/Audi 0x0A0 `32|12` + sign
   bit 44). A Motorola field of 8 bits or less inside one byte has the same bits as the Intel field and is skipped;
   one that crosses a byte (`19|8@0`) is searched on its own.
3. Every candidate field is scored.
   - **evidence**: how much the field changed in its phase, against the baseline
   - **specificity**: whether it is quiet in phases where the signal should not move (e.g. wheel speed in a
     standstill phase). Phases where it moves along naturally (RPM while driving, brake while shifting) cost nothing.
   - **shape**: whether bit width, period, sign and byte order suit the kind of signal. For signals that may be signed
     (steering, brake) it also checks that the value stays continuous where the sign (top) bit flips. Reading the sign
     the wrong way (two's complement as sign-magnitude or the reverse) jumps by half the range around 0. Released
     pedals and a standstill have a single value, so for throttle and wheel speed the baseline's alive counter is not
     the signal: a field that includes counter bits loses points, a counter right next to a field does not mean the
     field is cut off, and the bit right above a counter is also tried as a field LSB. Counters are found per id in
     the baseline frames: they start at a bit that flips in nearly every frame (95 %), or at a bit whose 2-bit pair
     with the bit above changes by the same odd step in most frames (60 % or more) with values that hardly repeat
     (5 % or less). The second condition is for lost frames: a missed frame leaves the lowest bit unflipped and the
     counter two steps ahead, so 5 % loss drops the flip rate to ~95 %. Bits above join while the combined value keeps
     moving forward every frame (by less than half its range) or by a constant step (up to 8 bits). Checksum bits and
     jitter of the signal itself (e.g. the throttle plate angle at idle) do not qualify. Example: 992 GT3 `0x121` has a
     checksum in byte 0, a counter in bits 8–11, and the pedal from bit 12.
   - **prior**: whether the VSP corpus has a signal with **the same id and the same layout** (the same id alone
     earns almost nothing)
4. Candidates below the minimum change for their field (e.g. a byte that went 50→51 once) are dropped.
5. All signals are assigned together so that no two use the same bits. When one signal's best field suits another
   signal better, the first moves on to its next candidate (e.g. RPM and throttle moving in the same phase).

`confidence` comes from the strength of the evidence and two margins.
- **Assignment margin**: the drop in total score when the whole assignment is solved again with the signal barred
  from its location (every overlapping bit). The other signals may move too, so when two signals can swap at the same
  total (e.g. RPM/throttle on two fields that move alike) the margin is 0. HIGH needs 15 points or 15 % of the score.
- **Layout margin**: the score gap to the runner-up layout that reads the same bits differently (e.g. `8|12`, which
  includes a counter, against `12|8`). Layouts that differ only in width, scale or a constant offset (reading the same
  values 2^k times larger or shifted by a constant, e.g. a field with extra always-0 top bits) cannot be told apart by
  a capture and do not count as rivals. These are scores from the same field data, so HIGH needs 10 points or 10 %
  (values set with the discover_regression scenarios).

`ambiguity_margin` is the smaller of the two. HIGH means "there is evidence that tells this location and bit layout
apart from other readings", not that scale and offset are right as well. A lone candidate with strong evidence can be
HIGH, and similar candidates left over lower it to MEDIUM/LOW. The JSON `evidence` object and the VSP `discovery` block
show how the score is made up.

When done it writes a draft DBC and a draft VSP to the working folder, along with the JSON result.
The DBC is a compatible draft on raw values; the VSP is a starting profile for a vehicle with no information.
Both files come from the same final field list, so they agree, and the layouts reported in the JSON are in them as is.
A prior fills in name / scale / unit / canonical id only when the layout and sign match the observed bitfield.
(When a prior contains the observed field and its remaining bits never change during the capture, the prior layout is
adopted. When none fits, a last option is a prior with the same LSB that is wider than the observed field upward only,
again when those upper bits never change during the capture: the capture cannot tell the two widths apart, so the
corpus wins over the search width and the byte-boundary bonus. Of several, the one with more support; on a tie, the
wider.)
The scale is checked against the observations too: the observed raw range must lie within the prior's min/max, and
released brake / pedal and standstill (baseline) must sit in the lower quarter of the physical range. Idle is above 0
and small blips undershoot, so for RPM the lower half is enough. A prior that does not fit is not used.
The DLC is always the observed one. Wheel speed is split into 4 per-wheel signals where possible (the JSON result
shows one representative field). A wheel field without a prior has an unknown position (the profile corpus orders
FL FR RL RR in only 72 % of messages), so only the field reported in the JSON gets canonical id `wheel.speed` and the
others go out without a canonical id. A 29-bit id (above 0x7FF) is written with bit 31 set (`0x80000000 | id`) in the
DBC and as `id_format: "extended29"` in the VSP.
A discover run makes a `captures/YYYYMMDD_HHMMSS/` session folder by default and saves each phase's raw dump
(`*.frames.tsv`), `manifest.json`, `result.json`, `ptcan_discover_draft.dbc` and `ptcan_discover_draft.vsp` there.
`--dump-dir <DIR>` overrides the location.
When the capture buffer overflowed and frames were dropped, it warns on the console, records
`capture_dropped_frames` in the manifest, and computes Hz over the time the stored frames actually cover.
LOW-confidence results, checksums / counters and the other bit-packed fields still need a person to review and adjust
them.

### Re-analyzing saved captures (replay, formerly discover_replay)

Discovery can run again on a saved session folder without the vehicle. Use it to check engine changes against real
captures:

```bash
./build/can_tools replay captures/20260408_153012            # results go to <session>/replay/
./build/can_tools replay captures/20260408_153012 --out-dir /tmp/replay
```

Phase lengths come from the manifest's `analyzed_duration_s`, then `actual_duration_s`, then the frame timestamps.
Older sessions with a fixed-size capture buffer kept only the first frames, so when the frames cover less than 80 % of
the manifest time, Hz is computed over the time they cover, with a warning.

### Generating the prior table

The prior table (`can_discover_priors.h`) is generated at build time by `tools/gen_can_discover_priors.py` from the
repository's `profiles/*.vsp` (`../../profiles`) into `build/generated/`. The header is not committed; the next build
regenerates it when the profiles or the generator change. The profile folder is not needed at run time.
The top of the header records the commit that last changed `profiles/`, the file count and an input digest. Signals
with a DLC over 8, fields outside their frame and multiplexed signals are left out. VSP `decode.sign_bit`
(sign-magnitude) signals go in with sign `'S'` and the sign bit position, and a candidate reading the same bits with
another sign scheme adopts such a prior only when its sign transitions are smoother.

### Regression tests

Synthetic captures check the discovery engine (RPM/throttle separation, two signals in one message, a slow gateway,
a noise-only capture, skipped phases, batched timestamps, a pedal right next to a counter and small RPM blips (the
pattern of a real 992 GT3 capture), a Motorola pedal above a counter in the same byte, a throttle whose low bits
jitter at idle, DBC/VSP agreement, cycle times, a 512 KB thread stack, and more). The edge cases of the engine review
are in too: 3–10 % random frame loss (`_loss<P>_s<S>` in the name), an 8-bit Motorola field across a byte boundary, a
sensor byte with 16 values at rest, an RPM whose high byte does not move, and twin fields that cannot tell RPM from
throttle (never HIGH). There is a 29-bit id scenario, and every scenario checks that 29-bit ids carry bit 31 in the
draft DBC and that no canonical id is used by two signals. The ctest `discover_drafts` dumps each scenario's session,
builds the draft VSP with `can_tools replay`, and checks it with `tools/validate_vsp.py` (no errors, no warnings):

```bash
cd build && ctest --output-on-failure      # or ./build/discover_regression
```

Synthetic data guards against regressions and does not prove accuracy on real vehicles. Check real captures with
`discover_replay`.

### D-CAN / PT-CAN auto-detection

The per-id Hz of the Baseline capture decides how the adapter is attached to the bus:

| Mode | Rule | "Fast message" threshold | Path |
|------|----------|-------------------|------|
| **DIRECT** | 2 or more ids at 20 Hz or more | 75th percentile of id Hz × 0.8 (5–30 Hz) | adapter → CAN, direct |
| **GATEWAY** | otherwise | 1.0 Hz | adapter → OBD port → gateway (ZGW etc.) → PT-CAN |

Through a gateway only some messages are relayed, slowly, so even the fastest id is slow (0.1–5 Hz).
A direct bus may carry many slow body messages, but 20–100 Hz powertrain / chassis messages are always there, so the
rule asks whether fast ids exist rather than looking at the median (e.g. a direct 992 GT3 tap has a median of 4 Hz but
6 ids at 25–50 Hz).
The "fast message" threshold feeds the period bonus of RPM and wheel speed candidates. On a direct bus it follows the
fast end of that bus, so a typical PT-CAN gets 30 Hz and a bus whose engine message runs at 10 Hz (992 GT3) about 8 Hz.

```
[INFO] Bus mode: GATEWAY (D-CAN)    ← even the fastest id is ~5 Hz
[INFO] Bus mode: DIRECT (PT-CAN)    ← 2 or more ids at 20 Hz or more
```

The JSON output includes a `"bus_mode"` field as well.

### Signal characterization

Full-range inputs during the capture also reveal the data type:

- **Layout**: `start_bit` / `bit_length` / `byte_order` (0=big, 1=little), in DBC notation
- **byte / byte2**: the most significant byte the field spans and the next less significant one
- **Endianness**: big / little endian (signals spanning several bytes)
- **Signedness**: signed / unsigned / sign-magnitude. Sign-magnitude is reported as `sign_char` `'S'` with
  `sign_bit`, written as `"signed":false,"sign_bit":N` in the JSON, `"sign_bit"` in the draft VSP, and in the draft
  DBC as the magnitude signal plus a 1-bit `<Name>_Sign` signal (the convention of vehicle-signal-profile's
  `dbc2vsp`/`vsp2dbc`).
- **Raw Range**: observed minimum / maximum
- **Confidence**: HIGH/MEDIUM/LOW

### Example output

```json
{
  "bus_mode":"direct",
  "dbc_path":"ptcan_discover_draft_20260408_153012.dbc",
  "vsp_path":"ptcan_discover_draft_20260408_153012.vsp",
  "steering":{"can_id":"0x0A5","dlc":8,"hz":100.0,"byte":3,"byte2":2,"start_bit":16,"bit_length":16,"byte_order":1,"score":104.0,"confidence":"HIGH","evidence":{"phase":60.0,"specificity":20.0,"shape":24.0,"prior":0.0,"prior_support":0,"ambiguity_margin":104.0},"endian":"little","signed":true,"raw_min":-4500,"raw_max":4500},
  "rpm":{"can_id":"0x0F3","dlc":8,"hz":50.0,"byte":4,"byte2":5,"start_bit":39,"bit_length":16,"byte_order":0,"score":117.0,"confidence":"HIGH","evidence":{"phase":50.0,"specificity":20.0,"shape":47.0,"prior":0.0,"prior_support":0,"ambiguity_margin":27.0},"endian":"big","signed":false,"raw_min":2988,"raw_max":23800},
  "throttle":{"can_id":"0x0D9","dlc":6,"hz":20.0,"byte":3,"byte2":2,"start_bit":16,"bit_length":12,"byte_order":1,"score":123.0,"confidence":"HIGH","evidence":{"phase":56.0,"specificity":20.0,"shape":31.0,"prior":16.0,"prior_support":7,"ambiguity_margin":44.0},"endian":"little","signed":false,"raw_min":0,"raw_max":4000}
}
```

## Project layout

```
apps/can_tools/
├── can_tools/           # the single executable can_tools (one file per command)
│   ├── main.m           # command dispatch, help, argv[0] aliases (can_reader etc.)
│   ├── cli_common.m/h   # state shared by the commands, connect wrappers
│   ├── cli_pcan.c/h     # opening a PCAN device (library, channel, bitrate), refusal message for transmitting commands
│   ├── cmd_scan.m       # scan
│   ├── cmd_detect.m     # detect
│   ├── cmd_monitor.m    # monitor
│   ├── cmd_query.m      # query / supported / raw
│   ├── cmd_discover.m   # discover
│   ├── cmd_probe.m      # probe (formerly vsp_probe)
│   ├── cmd_replay.c     # replay (formerly discover_replay, plain C)
│   ├── cmd_p992.m       # p992 legacy/debug
│   ├── interactive.m    # the old interactive menu (can_tools menu)
│   ├── commands.h       # command entry points
│   └── tui/             # live dashboard
│       ├── tui_main.m   # tui options, replay loop, snapshot mode
│       ├── tui_live.m   # connect screen, PASSIVE connect, live loop, TX host
│       ├── tui_app.c    # app state, overlays (discover, result, profile, help, confirm), keys, recording, TX guard
│       ├── tui_model.c  # Raw CAN table, signal stats and history, bus verdict, log, number formats (plain C)
│       ├── tui_render.c / tui_screens.c / tui_grid.c  # drawing into a cell grid (plain C, no ncurses)
│       ├── tui_term.c   # ncurses backend
│       ├── tui_profiles.c, tui_replay.c, tui_recorder.c, tui_capture.c
├── common/              # shared libraries
│   ├── elm327.c/h       # ELM327 protocol handler
│   ├── stn2255.c/h      # STN2255 extensions (batch queries, STMA etc.)
│   ├── obd_adapter.m/h  # BLE / Classic BT OBD adapter connection
│   ├── obd_connect.m/h  # the shared scan / connect / init path (PASSIVE / ACTIVE profiles)
│   ├── obd_poll.m/h     # polled OBD queries (probe poll mode, TUI TX mode; transmits)
│   ├── frame_source.c/h # CAN frame source interface, replay source, capture buffer
│   ├── frame_source_live.m/h # live source: the adapter monitor as a frame source
│   ├── pcan.c/h         # PCAN-USB (PCBUSB, dlopen): receive-only function table, listen-only open, bitrate auto, time mapping
│   ├── frame_source_pcan.c/h # frame source over an open PCAN channel (hardware µs timestamps)
│   ├── frame_tsv.c/h    # capture dumps (*.frames.tsv), read and write (5 columns, 6 with hardware timestamps)
│   ├── discover_session.m/h # discovery session (phase capture → analysis → files, result JSON) + interactive CLI
│   ├── vsp_engine.m/h   # VSP profile loading, signal selection, frame decoding (C API)
│   ├── cli_util.c/h     # CLI logging, JSON string output
│   └── can_discover.c/h # PT-CAN signal discovery engine
├── tools/               # prior generator (runs at build time, build/generated/can_discover_priors.h), ref2dbc
├── tests/               # discovery engine regression, draft VSP validation, VSP decoder, replay, discovery session,
│   │                    # adapter lifecycle and TUI tests (no hardware needed)
│   ├── fake/            # fake adapter, fake PCBUSB (can_tools_fake)
│   ├── tui_fake/        # AT command logs of the TUI live path through the fake adapter
│   ├── pcan_fake/       # transmit command refusals, listen-only call order and the PCAN TUI over a fake PCBUSB
│   ├── pcan_test.c      # PCAN open order, bitrate auto, time mapping, 6-column TSV (unit test)
│   ├── golden/tui/      # TUI screen snapshot goldens (refresh with snapshots.sh update)
│   └── fixtures/tui/    # synthetic streams, test profiles, a discover session (made by gen_fixtures.py)
├── CMakeLists.txt
└── build.sh
```

## Notes

- macOS Bluetooth permission required (System Settings > Privacy & Security)
- PT-CAN signal discovery works both tapped directly into PT-CAN and through D-CAN (OBD port) — the bus mode is
  detected automatically
- In passive mode the adapter runs in silent mode (`ATCSM1`) — no CAN ACKs. Adapters that do not answer `ATCSM1` with
  `OK` (some cheap clones) cannot be used for the receive-only commands
- `ctest` (in the build folder) runs without hardware: the TUI tests use only replay and the fake adapter build
- Run with the ignition ON
