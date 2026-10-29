#!/bin/sh
# TUI snapshot goldens.
#
#   tests/golden/tui/snapshots.sh check  [CAN_TOOLS]   compare (ctest)
#   tests/golden/tui/snapshots.sh update [CAN_TOOLS]   rewrite the .txt files
#
# Run from the repository root (ctest does). Every case is a replay snapshot:
# a manual clock over tests/fixtures/tui, no terminal, no adapter, no
# Bluetooth. Cases run inside a temporary folder, so discover sessions land in
# <tmp>/cap/snapshot_replay and nothing is written to the repository or to
# ~/.can_tools.
set -u
MODE="${1:-check}"
BIN="${2:-./build/can_tools}"
ROOT=$(pwd)
case "$BIN" in /*) ;; *) BIN="$ROOT/$BIN" ;; esac
DIR="$ROOT/tests/golden/tui"
FX="$ROOT/tests/fixtures/tui"
PROFILE="--profile $FX/test_profile.vsp"
DIRECT="--replay $FX/stream_direct.frames.tsv"
GATEWAY="--replay $FX/stream_gateway.frames.tsv"
SESSION="--replay $FX/discover_session"
TEMPS="--replay $FX/stream_temps.frames.tsv"
STEER="--replay $FX/stream_steer.frames.tsv"
PARTIAL="--profile $FX/partial_profile.vsp"
PROFILES="--profiles $FX/profiles"
NOPROF="--profile none"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/tui_snapshots.XXXXXX") || exit 1
trap 'rm -rf "$WORK"' EXIT
cd "$WORK" || exit 1
FAILED=0
TOTAL=0

run_case() {
    name="$1"; shift
    TOTAL=$((TOTAL + 1))
    out="$DIR/$name.txt"
    rm -rf cap
    if [ "$MODE" = "update" ]; then
        "$BIN" tui --captures cap "$@" > "$out" || { echo "FAIL $name (exit $?)"; FAILED=$((FAILED + 1)); }
        echo "updated $out"
        return
    fi
    tmp="$WORK/snapshot.txt"
    if ! "$BIN" tui --captures cap "$@" > "$tmp"; then
        echo "FAIL $name: exit status"
        FAILED=$((FAILED + 1))
    elif ! cmp -s "$tmp" "$out"; then
        echo "FAIL $name: differs from $out"
        diff "$out" "$tmp" | head -20
        FAILED=$((FAILED + 1))
    else
        echo "ok   $name"
    fi
}

DISC_KEYS="0.5:d;0.6:enter;6:enter;15:enter;24:enter;32.2:s;32.3:s"

run_case dashboard_120x40    $DIRECT $PROFILE --snapshot-at 5 --size 120x40 --color-map
run_case dashboard_80x24     $DIRECT $PROFILE --snapshot-at 5 --size 80x24 --color-map
run_case tabs_60x20          $DIRECT $PROFILE --snapshot-at 5,6,7 --size 60x20 --keys "5.5:tab;6.5:tab" --color-map
run_case no_profile_120x40   $DIRECT $NOPROF --snapshot-at 5 --size 120x40 --color-map
run_case no_profile_tabs_60x20 $DIRECT $NOPROF --snapshot-at 5 --size 60x20 --color-map
run_case gateway_120x40      $GATEWAY $PROFILE --snapshot-at 6 --size 120x40 --color-map
run_case stale_120x40        $DIRECT $PROFILE --snapshot-at 4,14 --size 120x40 --color-map
run_case controls_120x40     $DIRECT $PROFILE --snapshot-at 4.5 --size 120x40 --keys "1:];3:space;3.5:s s;4:/ w h e e l enter" --color-map
run_case ascii_80x24         $DIRECT $PROFILE --snapshot-at 5 --size 80x24 --ascii
run_case raw_ext_80x24       $DIRECT $PROFILE --snapshot-at 5 --size 80x24 --keys "4:tab;4.2:pgdn" --color-map
run_case discover_mid_120x40 $SESSION $PROFILE --snapshot-at 3,10 --size 120x40 --keys "$DISC_KEYS" --color-map
run_case discover_result_120x40 $SESSION $PROFILE --snapshot-at 33 --size 120x40 --keys "$DISC_KEYS" --color-map
run_case discover_load_120x40 $SESSION $NOPROF --snapshot-at 33.5 --size 120x40 --keys "$DISC_KEYS;33.2:enter"
run_case discover_json_100x30 $SESSION $NOPROF --snapshot-at 33.5 --size 100x30 --keys "$DISC_KEYS;33.2:v"
run_case discover_abort_120x40 $SESSION $NOPROF --snapshot-at 8 --size 120x40 --keys "0.5:d;0.6:enter;6:enter;7.5:a"
run_case picker_120x40       $DIRECT $PROFILES $NOPROF --snapshot-at 1.5,2.5 --size 120x40 --keys "1:p;2:b e n c h space o" --color-map
# replay without --profile: the profile in the capture's session.json, else the picker
run_case replay_picker_100x30 $DIRECT $PROFILES --snapshot-at 1.5 --size 100x30
mkdir -p sess_caps/20260104_120000 sess_profiles
cp "$FX/stream_direct.frames.tsv" sess_caps/20260104_120000/live.frames.tsv
cp "$FX/test_profile.vsp" sess_profiles/
# recorded in another checkout: the path is gone, the file name is in --profiles
cat > sess_caps/20260104_120000/session.json <<'JSON'
{
  "format": "can_tools_tui_session/v1",
  "device": "vLinker \"profile\": x",
  "classification": [{"t_s": 3.0, "profile": "decoy.vsp"}],
  "profile": "../old_checkout/profiles/test_profile.vsp",
  "recordings": ["live.frames.tsv"]
}
JSON
run_case replay_session_profile_120x40 --replay sess_caps/20260104_120000 --profiles sess_profiles --snapshot-at 5 --size 120x40
# replay list: a fixed captures folder, newest first
mkdir -p "$WORK/replay_caps/20260101_120000" "$WORK/replay_caps/20260102_120000"
cp "$FX/stream_direct.frames.tsv" "$WORK/replay_caps/20260101_120000/live.frames.tsv"
cp "$FX/stream_gateway.frames.tsv" "$WORK/replay_caps/20260102_120000/live.frames.tsv"
touch -t 202601011200 "$WORK/replay_caps/20260101_120000"
touch -t 202601021200 "$WORK/replay_caps/20260102_120000"
# newest, header only (the link dropped before any frame): not listed
mkdir -p "$WORK/replay_caps/20260103_120000"
head -1 "$FX/stream_direct.frames.tsv" > "$WORK/replay_caps/20260103_120000/live.frames.tsv"
touch -t 202601031200 "$WORK/replay_caps/20260103_120000"
run_case replay_list_120x40  $DIRECT $PROFILE --captures "$WORK/replay_caps" --snapshot-at 1.5 --size 120x40 --keys "1:R"
# r in replay: no list, one Log line (r is recording only)
run_case replay_r_120x40     $DIRECT $PROFILE --captures "$WORK/replay_caps" --snapshot-at 1.5 --size 120x40 --keys "1:r"
run_case help_100x30         $DIRECT $PROFILE --snapshot-at 1.5 --size 100x30 --keys "1:?"
run_case x_direct_120x40     $DIRECT $PROFILE --snapshot-at 4 --size 120x40 --keys "3.5:x" --color-map
run_case x_gateway_120x40    $GATEWAY $PROFILE --snapshot-at 4 --size 120x40 --keys "3.5:x"
# step 4: gauges panel + braille timeline (fg-map = 256-color index per cell)
run_case gauges_120x40       $DIRECT $PROFILE --snapshot-at 12 --size 120x40 --color-map
run_case gauges_80x24        $DIRECT $PROFILE --snapshot-at 12 --size 80x24 --color-map
run_case gauges_100x30       $DIRECT $PROFILE --snapshot-at 12 --size 100x30 --color-map
run_case gauges_tabs_60x20   $DIRECT $PROFILE --snapshot-at 12 --size 60x20 --color-map
run_case gauges_temps_cold   $TEMPS $PROFILE --snapshot-at 4.5 --size 120x40 --color-map
run_case gauges_temps_hot    $TEMPS $PROFILE --snapshot-at 10.5 --size 120x40 --color-map
run_case gauges_steer_over   $STEER $PROFILE --snapshot-at 5 --size 120x40 --color-map
run_case gauges_steer_lock   $STEER $PROFILE --snapshot-at 10 --size 120x40 --color-map
run_case gauges_missing      $DIRECT $PARTIAL --snapshot-at 5 --size 120x40 --color-map
run_case gauges_8color       $DIRECT $PROFILE --snapshot-at 12 --size 120x40 --colors 8 --color-map
run_case gauges_nocolor_80x24 $DIRECT $PROFILE --snapshot-at 12 --size 80x24 --colors 0 --color-map

if [ "$MODE" != "update" ]; then
    echo "$((TOTAL - FAILED))/$TOTAL snapshot cases match"
fi
[ "$FAILED" -eq 0 ]
