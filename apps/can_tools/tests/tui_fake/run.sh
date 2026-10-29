#!/bin/sh
# Live TUI path through the fake adapter: the safety proof of the step 3
# brief (section 0). Every scenario runs can_tools_fake (all can_tools code,
# Bluetooth edges replaced by tests/fake/fake_adapter.m, no CoreBluetooth or
# IOBluetooth object) with the headless test hook, then compares the adapter
# command log (cmds.txt) with tests/tui_fake/expected/<name>.cmds and checks
# the screen and the session folder.
#
#   tests/tui_fake/run.sh check  [CAN_TOOLS_FAKE]   compare (ctest)
#   tests/tui_fake/run.sh update [CAN_TOOLS_FAKE]   rewrite the expected logs
#
# Repeated poll queries (ATSH, ATFCSH, ATCRA, request) are collapsed to one
# "QUERIES <request>" line; their count is checked against the 10/s limit.
set -u
MODE="${1:-check}"
BIN="${2:-./build/can_tools_fake}"
ROOT=$(pwd)
case "$BIN" in /*) ;; *) BIN="$ROOT/$BIN" ;; esac
case "$BIN" in *can_tools_fake) ;; *) echo "refusing to run $BIN: only the fake-adapter build"; exit 1 ;; esac
EXP="$ROOT/tests/tui_fake/expected"
FX="$ROOT/tests/fixtures/tui"
PROFILE="$FX/test_profile.vsp"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/tui_fake.XXXXXX") || exit 1
[ -n "${KEEP_WORK:-}" ] || trap 'rm -rf "$WORK"' EXIT
[ -n "${KEEP_WORK:-}" ] && echo "work folder: $WORK"
FAILED=0
TOTAL=0

fail() {
    echo "FAIL $1: $2"
    FAILED=$((FAILED + 1))
}

# live <name> <env...> -- <tui args...>   (runs in $WORK/<name>, HOME there too)
live() {
    name="$1"; shift
    dir="$WORK/$name"
    mkdir -p "$dir/home"
    envs=""
    while [ "$1" != "--" ]; do envs="$envs $1"; shift; done
    shift
    ( cd "$dir" && env HOME="$dir/home" CAN_TOOLS_TUI_HEADLESS=1 $envs "$BIN" tui --captures cap "$@" \
        > screen.txt 2> err.txt; echo $? > rc.txt )
}

# Collapse the poll query blocks; print the query count to <dir>/queries.
normalize() {
    awk -v qfile="$1/queries" '
        /^CMD ATSH/ { pend[1] = $0; state = 1; next }
        state == 1 && /^CMD ATFCSH/ { pend[2] = $0; state = 2; next }
        state == 2 && /^CMD ATCRA/ { pend[3] = $0; state = 3; next }
        state == 3 && /^CMD [0-9A-F]+$/ {
            if (!inq) print "QUERIES " $2
            inq = 1; n++; state = 0; next
        }
        {
            for (i = 1; i <= state; i++) print pend[i]
            state = 0; inq = 0
            print
        }
        END { print n + 0 > qfile }
    ' "$1/cmds.txt"
}

check_cmds() {
    name="$1"; dir="$WORK/$name"
    TOTAL=$((TOTAL + 1))
    if [ ! -f "$dir/cmds.txt" ]; then fail "$name" "no cmds.txt (rc $(cat "$dir/rc.txt" 2>/dev/null))"; return; fi
    normalize "$dir" > "$dir/cmds.norm"
    if [ "$MODE" = "update" ]; then
        cp "$dir/cmds.norm" "$EXP/$name.cmds"
        echo "updated $EXP/$name.cmds"
        return
    fi
    if ! cmp -s "$dir/cmds.norm" "$EXP/$name.cmds"; then
        fail "$name" "adapter commands differ from $EXP/$name.cmds"
        diff "$EXP/$name.cmds" "$dir/cmds.norm" | head -20
        return
    fi
    if [ "$(cat "$dir/rc.txt")" != "0" ]; then fail "$name" "exit status $(cat "$dir/rc.txt")"; return; fi
    echo "ok   $name"
}

# expect <name> <file> <fixed string> [absent]
expect() {
    TOTAL=$((TOTAL + 1))
    if [ "${4:-}" = "absent" ]; then
        if grep -qF -- "$3" "$WORK/$1/$2"; then fail "$1" "unexpected '$3' in $2"; else echo "ok   $1: no '$3'"; fi
    else
        if grep -qF -- "$3" "$WORK/$1/$2"; then echo "ok   $1: '$3'"; else fail "$1" "missing '$3' in $2"; fi
    fi
}

queries_between() {
    TOTAL=$((TOTAL + 1))
    q=$(cat "$WORK/$1/queries" 2>/dev/null || echo 0)
    if [ "$q" -ge "$2" ] && [ "$q" -le "$3" ]; then echo "ok   $1: $q queries"; else fail "$1" "$q queries, want $2..$3"; fi
}

session_file() {
    ls -d "$WORK/$1"/cap/*/ 2>/dev/null | head -1
}

# ---- scenarios (parallel) ----
live direct_x FAKE_STREAM=direct -- --device vLinker --profile "$PROFILE" --snapshot-at 4.5 --keys "3.8:x" &
live gateway_tx FAKE_STREAM=gateway FAKE_RESP=probe -- --device vLinker --profile "$PROFILE" \
    --snapshot-at 4.2,5.5,7.5 --keys "4:x;4.4:y;6.5:x" &
live silent_tx FAKE_STREAM=silent FAKE_RESP=probe -- --device vLinker --profile "$PROFILE" \
    --snapshot-at 4.2,6 --keys "4:x;4.4:y" &
live no_tx FAKE_STREAM=gateway FAKE_RESP=probe -- --device vLinker --profile "$PROFILE" --no-tx \
    --snapshot-at 4.5 --keys "4:x" &
live detecting FAKE_STREAM=gateway FAKE_RESP=probe -- --device vLinker --profile "$PROFILE" \
    --snapshot-at 2 --keys "1.5:x" &
live sticky FAKE_STREAM=direct FAKE_SLOW_AFTER_MS=3500 FAKE_RESP=probe -- --device vLinker \
    --profile "$PROFILE" --snapshot-at 9.5 --keys "9:x" &
live disconnect FAKE_STREAM=direct FAKE_DISCONNECT_AFTER_MS=3000 -- --device vLinker --profile "$PROFILE" \
    --snapshot-at 4.5,6 --keys "1.5:d;1.6:enter;5:c" &
live no_connect FAKE_STREAM=direct "FAKE_NAME=OBDLink_MX+,vLinker_FAKE" -- --profiles "$FX/profiles" --snapshot-at 2 &
live recording FAKE_STREAM=direct -- --device vLinker --profile none --profiles "$FX/profiles" --no-rec \
    --snapshot-at 4 --keys "1.5:r;2.5:r;3:r" &
live discover_live FAKE_STREAM=direct -- --device vLinker --profile "$PROFILE" \
    --snapshot-at 8.5 --keys "1.5:d;1.6:enter;7:enter;8:a" &
(
    live connect_screen FAKE_STREAM=direct "FAKE_NAME=OBDLink_MX+,vLinker_FAKE" -- --profiles "$FX/profiles" \
        --snapshot-at 0.8,2.5,4 --keys "1:down;1.2:enter;3:enter"
    # second run, same HOME: the "last:" line connects the same adapter, the picker preselects the profile
    mkdir -p "$WORK/connect_last"
    cp -R "$WORK/connect_screen/home" "$WORK/connect_last/"
    live connect_last FAKE_STREAM=direct "FAKE_NAME=OBDLink_MX+,vLinker_FAKE" -- --profiles "$FX/profiles" \
        --snapshot-at 0.8,2.5,4 --keys "1:enter;3:enter"
) &
# replay list over cap/old (--no-rec keeps the live session out of the list)
for n in list_wait list_gone_connect list_gone_live tx_replay connect_r; do
    mkdir -p "$WORK/$n/cap/old" && cp "$FX/stream_gateway.frames.tsv" "$WORK/$n/cap/old/live.frames.tsv"
done
empty_later() { sleep "$2"; head -1 "$FX/stream_gateway.frames.tsv" > "$WORK/$1/cap/old/live.frames.tsv"; }
live list_wait FAKE_STREAM=direct -- --profile "$PROFILE" --snapshot-at 1.5,3.5 --keys "0.5:enter R;2:esc" &
empty_later list_gone_connect 2 &
live list_gone_connect FAKE_STREAM=direct -- --profile "$PROFILE" --snapshot-at 3.5 --keys "1:R;3:enter" &
empty_later list_gone_live 2.5 &
live list_gone_live FAKE_STREAM=direct -- --device vLinker --profile "$PROFILE" --no-rec \
    --snapshot-at 4 --keys "1.5:R;3.5:enter" &
live tx_replay FAKE_STREAM=gateway FAKE_RESP=probe -- --device vLinker --profile "$PROFILE" --no-rec \
    --snapshot-at 5.8,6.6,7.6,10 --keys "4:x;4.4:y;5:R;5.4:enter;6.2:n;7:esc;7.2:q;8:esc;8.4:R;8.8:enter;9.2:y" &
# connect screen: r only shows a status (no list, no adapter command); s rescans and drops the pending target
live connect_r FAKE_STREAM=direct -- --profile "$PROFILE" --snapshot-at 1.5 --keys "1:r" &
live connect_s FAKE_STREAM=direct -- --device NOPE --profile "$PROFILE" --snapshot-at 1.5 --keys "1:s" &
# silent mode not confirmed (ATCSM1 "?" or no reply): no monitor command, the reason on the connect screen
live csm_refused FAKE_STREAM=direct "FAKE_CSM_REPLY=?" -- --device vLinker --profile "$PROFILE" --snapshot-at 2.5 &
live csm_timeout FAKE_STREAM=direct FAKE_CSM_REPLY=timeout -- --device vLinker --profile "$PROFILE" --snapshot-at 2.5 &
# TX off whose PASSIVE re-init is refused: the monitor stays off and the header says NOT SILENT
live tx_not_silent FAKE_STREAM=gateway FAKE_RESP=probe FAKE_CSM_FROM=2 "FAKE_CSM_REPLY=?" -- --device vLinker \
    --profile "$PROFILE" --no-rec --snapshot-at 4.2,6 --keys "4:x;4.4:y;5:x" &
wait

# ---- no terminal, no hook: nothing is created (run in a folder of their own) ----
mkdir -p "$WORK/refuse" && cd "$WORK/refuse"
TOTAL=$((TOTAL + 1))
"$BIN" tui --captures cap < /dev/null > out.txt 2> err.txt; rc=$?
if [ "$rc" = 2 ] && [ ! -e cmds.txt ] && grep -q "needs a terminal" err.txt; then echo "ok   refuse: tui without a terminal"; else fail refuse "tui without a terminal: rc $rc"; fi
TOTAL=$((TOTAL + 1))
"$BIN" < /dev/null > out.txt 2> err.txt; rc=$?
if [ "$rc" = 2 ] && [ ! -e cmds.txt ] && grep -q "Usage: can_tools" err.txt; then echo "ok   refuse: no arguments without a terminal"; else fail refuse "no arguments: rc $rc"; fi
TOTAL=$((TOTAL + 1))
"$BIN" tui --device "" > out.txt 2> err.txt; rc=$?
if [ "$rc" = 2 ] && [ ! -e cmds.txt ]; then echo "ok   refuse: --device \"\""; else fail refuse "--device \"\": rc $rc"; fi
TOTAL=$((TOTAL + 1))
"$BIN" tui --device vLinker --snapshot-at 1 > out.txt 2> err.txt; rc=$?
if [ "$rc" = 2 ] && [ ! -e cmds.txt ]; then echo "ok   refuse: --snapshot-at without the hook"; else fail refuse "--snapshot-at without hook: rc $rc"; fi
cd "$ROOT"

# ---- adapter command logs ----
for n in direct_x gateway_tx silent_tx no_tx detecting sticky disconnect no_connect recording discover_live \
         connect_screen connect_last list_wait list_gone_connect list_gone_live tx_replay connect_r connect_s \
         csm_refused csm_timeout tx_not_silent; do
    check_cmds "$n"
done
[ "$MODE" = "update" ] && exit 0

# ---- screens and files ----
expect direct_x screen.txt "TX locked: DIRECT bus"
expect direct_x screen.txt "Send OBD requests" absent
expect direct_x screen.txt "x TX mode [locked: DIRECT]"
expect gateway_tx screen.txt "Bus: GATEWAY (OBD port). Continue? [y/N]"
expect gateway_tx screen.txt "TX ON"
expect gateway_tx screen.txt "Raw CAN (monitor paused: TX mode)"
expect gateway_tx screen.txt "TX off: adapter back in silent mode (ATCSM1)"
queries_between gateway_tx 5 23
expect silent_tx screen.txt "No frames seen. Only continue on the car's OBD port,"
expect silent_tx screen.txt "TX ON"
queries_between silent_tx 5 20
expect no_tx screen.txt "TX disabled (--no-tx)"
expect detecting screen.txt "TX locked: bus not classified yet"
expect sticky screen.txt "bus mode GATEWAY"
expect sticky screen.txt "TX locked: DIRECT bus"
expect disconnect screen.txt "DISCONNECTED"
expect disconnect screen.txt "discover aborted (disconnected)"
expect disconnect screen.txt "BT ready"
expect no_connect screen.txt "OBDLink_MX+"
expect no_connect screen.txt "vLinker_FAKE"
expect recording screen.txt "rec off"
expect connect_screen screen.txt "Profile"
expect connect_screen home/.can_tools/tui_last "device=vLinker_FAKE"
expect connect_last screen.txt "last: other__roadster__r2__2009_2014.vsp"
expect connect_last screen.txt "other roadster r2"
# no connect while the replay list is open; the pending target connects after esc
expect list_wait screen.txt "waiting for vLinker FAKE"
expect list_wait screen.txt "1 capture"
expect list_wait screen.txt "connected vLinker FAKE (BLE)"
# a capture emptied after the scan is refused and the list stays open
expect list_gone_connect screen.txt "cannot replay old: no frames"
expect list_gone_connect screen.txt "1 capture"
expect list_gone_live screen.txt "[WARN] cannot replay old: no frames"
expect list_gone_live screen.txt "1 capture"
# TX mode on: picking a capture asks first; n goes back and drops the pick
expect tx_replay screen.txt "TX mode is on. Leave TX mode and replay old? [y/N]"
expect tx_replay screen.txt "TX mode is on. Quit? [y/N]"
expect tx_replay screen.txt "snapshot T=10.000" absent
expect connect_r screen.txt "recording starts after connecting"
expect connect_r screen.txt "1 capture" absent
expect connect_s screen.txt "looking for NOPE" absent
expect connect_s screen.txt "R replay list  s rescan"
expect csm_refused screen.txt 'adapter init failed: silent mode not confirmed: ATCSM1 answered "?"'
expect csm_timeout screen.txt "adapter init failed: silent mode not confirmed: ATCSM1 timed out"
expect tx_not_silent screen.txt "NOT SILENT"
expect tx_not_silent screen.txt "[ERROR] TX off: silent mode (ATCSM1) not con"

S=$(session_file disconnect)
TOTAL=$((TOTAL + 1))
if grep -q '"status": *"disconnected"' "$S/manifest.json" 2>/dev/null && [ -s "$S/live.frames.tsv" ]; then
    echo "ok   disconnect: manifest disconnected, recording kept"
else fail disconnect "manifest/recording in $S"; fi
S=$(session_file recording)
TOTAL=$((TOTAL + 1))
if [ -s "$S/live.frames.tsv" ] && [ -s "$S/live_02.frames.tsv" ] && grep -q '"live_02.frames.tsv"' "$S/session.json"; then
    echo "ok   recording: two segments listed in session.json"
else fail recording "segments in $S"; fi
TOTAL=$((TOTAL + 1))
( cd "$WORK/recording" && "$BIN" tui --replay "$S" --snapshot-at 0.5 --size 80x24 > replay.txt 2>&1 )
if grep -q "REPLAY" "$WORK/recording/replay.txt"; then echo "ok   recording: the session folder replays"; else fail recording "replay of $S"; fi
S=$(session_file discover_live)
TOTAL=$((TOTAL + 1))
if [ -s "$S/00_baseline.frames.tsv" ] && [ -s "$S/01_steering.frames.tsv" ] && grep -q '"status": *"interrupted"' "$S/manifest.json"; then
    echo "ok   discover_live: phases dumped, manifest interrupted"
else fail discover_live "session files in $S"; fi

echo "$((TOTAL - FAILED))/$TOTAL fake-adapter checks pass"
[ "$FAILED" -eq 0 ]
