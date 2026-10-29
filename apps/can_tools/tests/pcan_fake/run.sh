#!/bin/sh
# PCAN-USB safety checks through can_tools_fake (all can_tools code; Bluetooth
# edges replaced by tests/fake/fake_adapter.m, the PCBUSB library by
# tests/fake/fake_pcan.c, so nothing here can reach real hardware), plus a
# strings check of both binaries.
#
#   tests/pcan_fake/run.sh CAN_TOOLS_FAKE CAN_TOOLS
#
# pcan_calls.txt (written by the fake library) holds LOAD, VERSION, SETVALUE,
# INIT, GETVALUE LISTEN_ONLY and UNINIT lines; cmds.txt (fake Bluetooth
# adapter) appears only if a Bluetooth method was called.
set -u
FAKE="${1:-./build/can_tools_fake}"
REAL="${2:-./build/can_tools}"
ROOT=$(pwd)
case "$FAKE" in /*) ;; *) FAKE="$ROOT/$FAKE" ;; esac
case "$REAL" in /*) ;; *) REAL="$ROOT/$REAL" ;; esac
case "$FAKE" in *can_tools_fake) ;; *) echo "refusing to run $FAKE: only the fake build"; exit 1 ;; esac
FX="$ROOT/tests/fixtures/tui"
PROFILE="$FX/test_profile.vsp"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/pcan_fake.XXXXXX") || exit 1
[ -n "${KEEP_WORK:-}" ] || trap 'rm -rf "$WORK"' EXIT
[ -n "${KEEP_WORK:-}" ] && echo "work folder: $WORK"
FAILED=0
TOTAL=0

fail() {
    echo "FAIL $1: $2"
    FAILED=$((FAILED + 1))
}
ok() { echo "ok   $1"; }

# run <name> <env...> -- <args...>   (in $WORK/<name>, HOME there too; stdin from $WORK/<name>/stdin if present)
run() {
    name="$1"; shift
    dir="$WORK/$name"
    mkdir -p "$dir/home"
    envs=""
    while [ "$1" != "--" ]; do envs="$envs $1"; shift; done
    shift
    [ -f "$dir/stdin" ] || : > "$dir/stdin"
    ( cd "$dir" && env HOME="$dir/home" $envs "$FAKE" "$@" < stdin > out.txt 2> err.txt; echo $? > rc.txt )
}

# tui <name> <env...> -- <tui args...>   (headless live hook of the fake build)
tui() {
    name="$1"; shift
    envs=""
    while [ "$1" != "--" ]; do envs="$envs $1"; shift; done
    shift
    run "$name" CAN_TOOLS_TUI_HEADLESS=1 $envs -- tui --captures cap "$@"
}

expect() {   # expect <name> <file> <fixed string> [absent]
    TOTAL=$((TOTAL + 1))
    if [ "${4:-}" = "absent" ]; then
        if grep -qF -- "$3" "$WORK/$1/$2" 2>/dev/null; then fail "$1" "unexpected '$3' in $2"; else ok "$1: no '$3'"; fi
    else
        if grep -qF -- "$3" "$WORK/$1/$2" 2>/dev/null; then ok "$1: '$3'"; else fail "$1" "missing '$3' in $2"; fi
    fi
}

rc_is() {   # rc_is <name> <rc>
    TOTAL=$((TOTAL + 1))
    rc=$(cat "$WORK/$1/rc.txt" 2>/dev/null)
    if [ "$rc" = "$2" ]; then ok "$1: exit $2"; else fail "$1" "exit $rc, want $2 ($(head -c 300 "$WORK/$1/err.txt"))"; fi
}

no_file() {   # no_file <name> <file> <why>
    TOTAL=$((TOTAL + 1))
    if [ -e "$WORK/$1/$2" ]; then fail "$1" "$2 exists ($3): $(head -3 "$WORK/$1/$2" | tr '\n' ' ')"; else ok "$1: no $2 ($3)"; fi
}

# The fake library's log: every INIT right after "SETVALUE <ch> LISTEN_ONLY=1"
# and "GETVALUE <ch> LISTEN_ONLY -> 1" (the pre-init read-back), with
# listen_only=1 at init; no other SETVALUE; then <n> INIT lines.
safe_inits() {   # safe_inits <name> <expected INIT count>
    TOTAL=$((TOTAL + 1))
    f="$WORK/$1/pcan_calls.txt"
    verdict=$(awk -v want="$2" '
        /^SETVALUE/ { if ($3 != "LISTEN_ONLY=1") bad = bad " " $0; ch = $2; st = 1; next }
        /^GETVALUE/ && $3 == "LISTEN_ONLY" { st = (st == 1 && $2 == ch && $5 == "1") ? 2 : 0; next }
        /^INIT/ {
            n++
            if (st != 2 || $2 != ch) bad = bad " [INIT without SETVALUE LISTEN_ONLY=1 + read-back 1 first: " $0 "]"
            if ($4 != "listen_only=1") bad = bad " [" $0 "]"
            st = 0; next
        }
        { st = 0 }
        END {
            if (n != want) bad = bad " [" n " INIT lines, want " want "]"
            if (bad == "") print "ok"; else print bad
        }' "$f" 2>/dev/null)
    if [ "$verdict" = "ok" ]; then ok "$1: $2 safe init(s)"; else fail "$1" "pcan_calls.txt:$verdict"; fi
}

calls_are() {   # calls_are <name> <expected lines joined by |>
    TOTAL=$((TOTAL + 1))
    got=$(tr '\n' '|' < "$WORK/$1/pcan_calls.txt" 2>/dev/null | sed 's/|$//')
    if [ "$got" = "$2" ]; then ok "$1: call sequence"; else fail "$1" "calls '$got', want '$2'"; fi
}

bt_untouched() {   # bt_untouched <name>: no Bluetooth method was called at all
    no_file "$1" cmds.txt "no Bluetooth object used"
}

bt_scan_only() {   # bt_scan_only <name>: the connect screen may scan, nothing more
    TOTAL=$((TOTAL + 1))
    if grep -qvE '^(SCAN|WAIT_BLE)$' "$WORK/$1/cmds.txt" 2>/dev/null; then
        fail "$1" "Bluetooth adapter did more than scan: $(grep -vE '^(SCAN|WAIT_BLE)$' "$WORK/$1/cmds.txt" | head -3 | tr '\n' ' ')"
    else ok "$1: Bluetooth only scanned"; fi
}

PRE="LOAD|VERSION|SETVALUE 0x51 LISTEN_ONLY=1"
SAFE_OPEN="$PRE|GETVALUE 0x51 LISTEN_ONLY -> 1|INIT 0x51 btr=0x001C listen_only=1|GETVALUE 0x51 LISTEN_ONLY -> 1|UNINIT 0x51"
POST_OFF="$PRE|GETVALUE 0x51 LISTEN_ONLY -> 1|INIT 0x51 btr=0x001C listen_only=1|GETVALUE 0x51 LISTEN_ONLY -> 0|UNINIT 0x51"

# ---- commands that transmit refuse PCAN before anything is created or loaded ----
refuse() {   # refuse <name> <what> <args...>
    name="$1"; what="$2"; shift 2
    run "$name" FAKE_PCAN=1 -- "$@"
    TOTAL=$((TOTAL + 1))
    rc=$(cat "$WORK/$name/rc.txt")
    if [ "$rc" != 0 ] && grep -qF "PCAN is listen-only: $what would transmit" "$WORK/$name/err.txt"; then
        ok "$name: refused ($what, exit $rc)"
    else fail "$name" "exit $rc: $(head -2 "$WORK/$name/err.txt" | tr '\n' ' ')"; fi
    no_file "$name" pcan_calls.txt "library not loaded"
    bt_untouched "$name"
}
refuse r_query query query pcan 0C
refuse r_supported supported supported pcan1
refuse r_raw raw raw PCAN 0100
refuse r_p992 p992 p992 read pcan
refuse r_p992_fastlog p992 p992 fastlog pcan2
refuse r_detect_probe "detect --probe" detect pcan --probe
refuse r_detect_probe2 "detect --probe" detect pcan --bitrate 250k --probe
refuse r_probe_obd "probe --mode obd" probe "$PROFILE" --device pcan --mode obd
refuse r_probe_auto "probe --mode auto" probe "$PROFILE" --device pcan1
refuse r_probe_auto2 "probe --mode auto" probe "$PROFILE" --device pcan --mode auto --seconds 1

# ---- receive commands (parallel) ----
run monitor FAKE_PCAN=1 FAKE_NAME= -- monitor pcan 1 --out mon.json &
run monitor_filter FAKE_PCAN=1 -- monitor pcan1 1 0A5 18DAF110 &
run monitor_auto FAKE_PCAN=1 FAKE_PCAN_BITRATE=250000 -- monitor pcan 1 --bitrate auto &
run monitor_none FAKE_PCAN=1 FAKE_PCAN_BITRATE=20000 -- monitor pcan 1 --bitrate auto &
run monitor_readback FAKE_PCAN=1 FAKE_PCAN_LO_READBACK=0 -- monitor pcan 1 &
run monitor_readback_auto FAKE_PCAN=1 FAKE_PCAN_LO_READBACK=0 -- monitor pcan 1 --bitrate auto &
run monitor_setfail FAKE_PCAN=1 FAKE_PCAN_LO_SET_FAIL=1 -- monitor pcan 1 &
run monitor_pre_off FAKE_PCAN=1 FAKE_PCAN_LO_PRE=off -- monitor pcan 1 &
run monitor_pre_fail FAKE_PCAN=1 FAKE_PCAN_LO_PRE=fail -- monitor pcan 1 &
run monitor_pre_off_auto FAKE_PCAN=1 FAKE_PCAN_LO_PRE=off -- monitor pcan 1 --bitrate auto &
run detect_pre_fail FAKE_PCAN=1 FAKE_PCAN_LO_PRE=fail -- detect pcan &
run monitor_oldver FAKE_PCAN=1 FAKE_PCAN_VERSION=PCBUSB_0.12.2 -- monitor pcan 1 &
run monitor_major FAKE_PCAN=1 FAKE_PCAN_VERSION=PCBUSB_1.0.0 -- monitor pcan 1 &
run monitor_badver FAKE_PCAN=1 FAKE_PCAN_VERSION=garbage -- monitor pcan 1 &
run monitor_verfail FAKE_PCAN=1 FAKE_PCAN_VERSION=fail -- monitor pcan 1 &
run monitor_newer FAKE_PCAN=1 FAKE_PCAN_VERSION=PCBUSB_0.14.1 -- monitor pcan 1 &
run scan_oldver FAKE_PCAN=1 FAKE_PCAN_VERSION=PCBUSB_0.12.2 -- scan &
run monitor_missing FAKE_PCAN=missing -- monitor pcan 1 &
run monitor_occupied FAKE_PCAN=1 FAKE_PCAN_OCCUPIED=1 -- monitor pcan 1 &
run monitor_absent FAKE_PCAN=1 -- monitor pcan3 1 &
run monitor_badrate FAKE_PCAN=1 -- monitor pcan 1 --bitrate 499k &
run monitor_bt_bitrate FAKE_PCAN=1 -- monitor vLinker 1 --bitrate 250k &
run monitor_unplug FAKE_PCAN=1 FAKE_PCAN_UNPLUG_AFTER_MS=500 -- monitor pcan 2 &
run detect FAKE_PCAN=1 -- detect pcan &
run probe_can FAKE_PCAN=1 -- probe "$PROFILE" --device pcan --mode can --seconds 1 &
run scan FAKE_PCAN=1,3 -- scan &
run scan_missing FAKE_PCAN=missing -- scan &
mkdir -p "$WORK/discover"
printf '\ns\ns\ns\ns\ns\ns\ns\n' > "$WORK/discover/stdin"
run discover FAKE_PCAN=1 -- discover pcan --dump-dir dd &

# ---- TUI (headless live hook) ----
tui tui_x FAKE_PCAN=1 FAKE_NAME= -- --device pcan --profile "$PROFILE" --snapshot-at 3.5 --keys "2.5:x;3:y" &
tui tui_list FAKE_PCAN=1 FAKE_NAME=vLinker_FAKE -- --profiles "$FX/profiles" --snapshot-at 1.5 &
tui tui_auto FAKE_PCAN=1 FAKE_PCAN_BITRATE=250000 FAKE_NAME= -- --device pcan --bitrate auto --profile none \
    --snapshot-at 3.5 &
tui tui_unplug FAKE_PCAN=1 FAKE_PCAN_UNPLUG_AFTER_MS=1500 FAKE_NAME= -- --device pcan1 --profile none --snapshot-at 3 &
tui tui_readback FAKE_PCAN=1 FAKE_PCAN_LO_READBACK=0 FAKE_NAME= -- --device pcan --profile none --snapshot-at 2 &
tui tui_missing FAKE_PCAN=missing FAKE_NAME= -- --device pcan --profile none --snapshot-at 1 &
tui tui_pre_off FAKE_PCAN=1 FAKE_PCAN_LO_PRE=off FAKE_NAME= -- --device pcan --profile none --snapshot-at 2 &
tui tui_pre_auto FAKE_PCAN=1 FAKE_PCAN_LO_PRE=fail FAKE_NAME= -- --device pcan --bitrate auto --profile none \
    --snapshot-at 2 &
tui tui_oldver FAKE_PCAN=1 FAKE_PCAN_VERSION=PCBUSB_0.12.2 FAKE_NAME=vLinker_FAKE -- --device pcan --profile none \
    --snapshot-at 1.5 &
(
    tui tui_enter FAKE_PCAN=1 FAKE_NAME= -- --profile none --snapshot-at 0.8,3 --keys "1:enter"
    # same HOME again: the "last:" line connects the PCAN channel
    mkdir -p "$WORK/tui_last"
    cp -R "$WORK/tui_enter/home" "$WORK/tui_last/"
    tui tui_last FAKE_PCAN=1 FAKE_NAME= -- --profile none --snapshot-at 0.8,3 --keys "1:enter"
) &
wait

# monitor: listen-only JSON, hardware µs, exact library calls, no Bluetooth
rc_is monitor 0
expect monitor mon.json '"interface":"pcan","library_version":"PCBUSB 0.13.0.10 (fake)","bitrate":500000'
expect monitor mon.json '"mode":"listen-only"'
expect monitor mon.json '"timestamp_us":'
expect monitor mon.json '"id":"18DAF110","is_extended":true'
expect monitor mon.json '"not_delivered":{"rtr":'
calls_are monitor "$SAFE_OPEN"
bt_untouched monitor
TOTAL=$((TOTAL + 1))
n=$(grep -c '"timestamp_us"' "$WORK/monitor/mon.json")
if [ "$n" -ge 100 ]; then ok "monitor: $n frames with timestamp_us"; else fail monitor "$n frames"; fi
TOTAL=$((TOTAL + 1))
bad=$(awk -F'"timestamp_ms":' 'NF > 1 { split($2, a, ","); ms = a[1]; split($0, b, "\"timestamp_us\":"); split(b[2], c, ","); if (int(c[1] / 1000) != ms) print }' \
      "$WORK/monitor/mon.json" | head -1)
if [ -z "$bad" ]; then ok "monitor: timestamp_ms = floor(timestamp_us / 1000)"; else fail monitor "ms/us mismatch: $bad"; fi

rc_is monitor_filter 0
expect monitor_filter out.txt '"filters":["0A5","18DAF110"]'
expect monitor_filter out.txt '"id":"0F3"' absent
expect monitor_filter out.txt '"id":"0A5"'
expect monitor_filter out.txt '"id":"18DAF110"'

rc_is monitor_auto 0
expect monitor_auto out.txt '"bitrate":250000,"bitrate_auto":true'
expect monitor_auto err.txt "bitrate 250k detected (tried: 500k: no frames; 250k:"
safe_inits monitor_auto 3      # 500k attempt, 250k attempt, the capture
expect monitor_auto pcan_calls.txt "INIT 0x51 btr=0x001C listen_only=1"
expect monitor_auto pcan_calls.txt "INIT 0x51 btr=0x011C listen_only=1"

rc_is monitor_none 1
expect monitor_none err.txt "bitrate detection failed"
safe_inits monitor_none 8

rc_is monitor_readback 1
expect monitor_readback err.txt "listen-only is not active after init"
calls_are monitor_readback "$POST_OFF"
expect monitor_readback out.txt '"frames"' absent

rc_is monitor_readback_auto 1
calls_are monitor_readback_auto "$POST_OFF"

rc_is monitor_setfail 1
expect monitor_setfail err.txt "cannot set listen-only before init"
calls_are monitor_setfail "$PRE"

# pre-init read-back OFF / failing: CAN_Initialize is never called (also not by auto)
rc_is monitor_pre_off 1
expect monitor_pre_off err.txt "reads back 0, not ON, before init; not initialized"
calls_are monitor_pre_off "$PRE|GETVALUE 0x51 LISTEN_ONLY -> 0"
rc_is monitor_pre_fail 1
expect monitor_pre_fail err.txt "cannot be read back before init"
calls_are monitor_pre_fail "$PRE|GETVALUE 0x51 LISTEN_ONLY -> error"
rc_is monitor_pre_off_auto 1
calls_are monitor_pre_off_auto "$PRE|GETVALUE 0x51 LISTEN_ONLY -> 0"
safe_inits monitor_pre_off_auto 0
rc_is detect_pre_fail 1
calls_are detect_pre_fail "$PRE|GETVALUE 0x51 LISTEN_ONLY -> error"

# version gate: before any channel use; only the version is read
for n in monitor_oldver monitor_major monitor_badver monitor_verfail; do
    rc_is "$n" 1
    calls_are "$n" "LOAD|VERSION"
    expect "$n" err.txt "0.13"
    bt_untouched "$n"
done
expect monitor_oldver err.txt 'PCBUSB version "PCBUSB 0.12.2" (0.12) is not supported'
expect monitor_major err.txt 'PCBUSB version "PCBUSB 1.0.0" (1.0) is not supported'
expect monitor_badver err.txt 'unrecognized PCBUSB version "garbage"'
expect monitor_verfail err.txt "cannot read the PCBUSB version"
rc_is monitor_newer 0
expect monitor_newer out.txt '"library_version":"PCBUSB 0.14.1"'
rc_is scan_oldver 0
expect scan_oldver err.txt 'PCAN not listed: PCAN: PCBUSB version "PCBUSB 0.12.2"'
expect scan_oldver out.txt '"transport":"usb"' absent
expect scan_oldver out.txt '"name":"vLinker FAKE"'
calls_are scan_oldver "LOAD|VERSION"

rc_is monitor_missing 1
expect monitor_missing err.txt "https://github.com/mac-can/PCBUSB-Library"
bt_untouched monitor_missing

rc_is monitor_occupied 1
expect monitor_occupied err.txt "in use by another application"
calls_are monitor_occupied "LOAD|VERSION"

rc_is monitor_absent 1
expect monitor_absent err.txt "pcan3 not found"
calls_are monitor_absent "LOAD|VERSION"

rc_is monitor_badrate 1
expect monitor_badrate err.txt "Bad --bitrate 499k"
no_file monitor_badrate pcan_calls.txt "bad option: library not loaded"

rc_is monitor_bt_bitrate 1
expect monitor_bt_bitrate err.txt "--bitrate applies to PCAN devices only"
bt_untouched monitor_bt_bitrate

rc_is monitor_unplug 0
expect monitor_unplug err.txt "stopped answering"
expect monitor_unplug out.txt '"device_lost":true'

rc_is detect 0
expect detect out.txt '"interface":"pcan","library_version":"PCBUSB 0.13.0.10 (fake)"'
expect detect out.txt '"probe_sent":false'
expect detect out.txt '"mode":"broadcast"'
expect detect out.txt '"voltage":null'
expect detect err.txt "Phase 2 skipped: PCAN is listen-only"
calls_are detect "$SAFE_OPEN"
bt_untouched detect

rc_is probe_can 0
expect probe_can out.txt '"strategy" : "pcan_listen_only"'
expect probe_can out.txt '"interface" : "pcan"'
expect probe_can out.txt '"library_version" : "PCBUSB 0.13.0.10 (fake)"'
calls_are probe_can "$SAFE_OPEN"
bt_untouched probe_can

rc_is scan 0
expect scan out.txt '"name":"pcan1 PCAN-USB","transport":"usb","channel":"pcan1","condition":"available"'
expect scan out.txt '"channel":"pcan3"'
expect scan out.txt '"name":"vLinker FAKE"'
calls_are scan "LOAD|VERSION"
rc_is scan_missing 0
expect scan_missing err.txt "PCAN not listed"
expect scan_missing out.txt '"name":"vLinker FAKE"'

rc_is discover 0
calls_are discover "$SAFE_OPEN"
bt_untouched discover
TOTAL=$((TOTAL + 1))
D="$WORK/discover/dd"
if head -1 "$D/00_baseline.frames.tsv" 2>/dev/null | grep -q "	timestamp_us$" &&
   [ "$(sed -n 2p "$D/00_baseline.frames.tsv" | awk -F'\t' '{ print NF }')" = 6 ] &&
   grep -q '"connected_device": *"pcan1 PCAN-USB"' "$D/manifest.json"; then
    ok "discover: 6-column phase dump, manifest names the PCAN"
else fail discover "dump/manifest in $D: $(head -2 "$D/00_baseline.frames.tsv" 2>/dev/null | tr '\n' ' ')"; fi
TOTAL=$((TOTAL + 1))
( cd "$WORK/discover" && "$FAKE" tui --replay dd/00_baseline.frames.tsv --snapshot-at 0.5 --size 80x24 > replay.txt 2>&1 )
if grep -q "REPLAY" "$WORK/discover/replay.txt"; then ok "discover: the 6-column dump replays"; else fail discover "replay"; fi

# TUI: x is refused up front, the monitor keeps running
expect tui_x out.txt "pcan1 PCAN-USB"
expect tui_x out.txt "listen-only 500k"
expect tui_x out.txt "TX unavailable: PCAN is listen-only"
expect tui_x out.txt "TX [listen-only]"
expect tui_x out.txt "Send OBD requests" absent
expect tui_x out.txt "TX ON" absent
expect tui_x out.txt "monitor paused" absent
expect tui_x out.txt "connected pcan1 PCAN-USB (USB), PCBUSB"
calls_are tui_x "$SAFE_OPEN"
bt_scan_only tui_x
TOTAL=$((TOTAL + 1))
S=$(ls -d "$WORK/tui_x"/cap/*/ 2>/dev/null | head -1)
if head -1 "$S/live.frames.tsv" 2>/dev/null | grep -q "	timestamp_us$" &&
   [ "$(sed -n 2p "$S/live.frames.tsv" | awk -F'\t' '{ print NF }')" = 6 ] &&
   grep -q '"interface": "pcan"' "$S/session.json" && grep -q '"mode": "listen-only"' "$S/session.json" &&
   grep -q '"library_version": "PCBUSB 0.13.0.10 (fake)"' "$S/session.json"; then
    ok "tui_x: 6-column recording, session.json interface pcan"
else fail tui_x "recording/session.json in $S"; fi

# connect screen: PCAN row (USB) next to the Bluetooth one; listing never initializes
expect tui_list out.txt "pcan1 PCAN-USB"
expect tui_list out.txt "USB"
expect tui_list out.txt "available"
expect tui_list out.txt "vLinker_FAKE"
calls_are tui_list "LOAD|VERSION"

expect tui_auto out.txt "listen-only 250k"
safe_inits tui_auto 3

expect tui_unplug out.txt "DISCONNECTED"
expect tui_unplug out.txt "adapter disconnected"

expect tui_readback out.txt "listen-only is not active"
calls_are tui_readback "$POST_OFF"
expect tui_pre_off out.txt "not ON, before init"
calls_are tui_pre_off "$PRE|GETVALUE 0x51 LISTEN_ONLY -> 0"
expect tui_pre_auto out.txt "no bitrate detected"
calls_are tui_pre_auto "$PRE|GETVALUE 0x51 LISTEN_ONLY -> error"
expect tui_oldver out.txt 'PCBUSB version "PCBUSB 0.12.2"'
expect tui_oldver out.txt "vLinker_FAKE"
expect tui_oldver out.txt "pcan1 PCAN-USB" absent
calls_are tui_oldver "LOAD|VERSION"

expect tui_missing out.txt "PCAN driver library not found"

expect tui_enter out.txt "connected pcan1 PCAN-USB (USB)"
expect tui_enter home/.can_tools/tui_last "device=pcan1 PCAN-USB"
expect tui_last out.txt "last: none"
expect tui_last out.txt "connected pcan1 PCAN-USB (USB)"
calls_are tui_last "$SAFE_OPEN"

# ---- the binaries hold no transmit symbol name to look up ----
for bin in "$REAL" "$FAKE"; do
    TOTAL=$((TOTAL + 1))
    b=$(basename "$bin")
    if strings "$bin" | grep -qE 'CAN_Write|CAN_WriteFD|CAN_Reset'; then
        fail strings "$b names CAN_Write / CAN_WriteFD / CAN_Reset"
    elif strings "$bin" | grep -q 'CAN_Initialize' && strings "$bin" | grep -q 'CAN_Read'; then
        ok "strings: $b resolves CAN_Initialize / CAN_Read, never CAN_Write / CAN_Reset"
    else
        fail strings "$b: receive symbol names missing (check is not meaningful)"
    fi
    TOTAL=$((TOTAL + 1))
    if strings "$bin" | grep -x 'libPCBUSB.dylib' > /dev/null; then
        fail strings "$b has a bare-name libPCBUSB.dylib (cwd / DYLD search)"
    elif strings "$bin" | grep -qx '/usr/local/lib/libPCBUSB.dylib'; then
        ok "strings: $b loads PCBUSB from absolute paths only"
    else
        fail strings "$b: library path missing (check is not meaningful)"
    fi
done

echo "$((TOTAL - FAILED))/$TOTAL PCAN fake checks pass"
[ "$FAILED" -eq 0 ]
