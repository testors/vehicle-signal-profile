#!/bin/sh
# The real can_tools has no headless test hook (it is compiled only into
# can_tools_fake). With CAN_TOOLS_TUI_HEADLESS=1, `tui --snapshot-at` without
# --replay must still exit 2 during option checking, before any adapter
# object exists (tui_main.m: the --snapshot-at check runs before tui_live_run,
# the only place an OBDAdapter is created), so this cannot reach Bluetooth.
#
#   tests/tui_hooks_check.sh CAN_TOOLS
set -u
BIN="${1:-./build/can_tools}"
case "$BIN" in /*) ;; *) BIN="$(pwd)/$BIN" ;; esac
case "$BIN" in *can_tools_fake) echo "use the real can_tools, not the fake build"; exit 1 ;; esac
WORK=$(mktemp -d "${TMPDIR:-/tmp}/tui_hooks.XXXXXX") || exit 1
trap 'rm -rf "$WORK"' EXIT
cd "$WORK" || exit 1
mkdir home
FAILED=0

if strings "$BIN" | grep -q CAN_TOOLS_TUI_HEADLESS; then
    echo "FAIL the real binary contains the CAN_TOOLS_TUI_HEADLESS hook"
    FAILED=1
else
    echo "ok   no CAN_TOOLS_TUI_HEADLESS string in $(basename "$BIN")"
fi

for keys in "" "0.5:enter;1:x y"; do
    if [ -n "$keys" ]; then
        env HOME="$WORK/home" CAN_TOOLS_TUI_HEADLESS=1 "$BIN" tui --snapshot-at 1 --keys "$keys" \
            < /dev/null > out.txt 2> err.txt
    else
        env HOME="$WORK/home" CAN_TOOLS_TUI_HEADLESS=1 "$BIN" tui --snapshot-at 1 < /dev/null > out.txt 2> err.txt
    fi
    rc=$?
    left=$(ls -A "$WORK/home" | wc -l | tr -d ' ')
    if [ "$rc" = 2 ] && grep -q -- "--snapshot-at needs --replay" err.txt && [ ! -s out.txt ] &&
       [ ! -e captures ] && [ "$left" = 0 ]; then
        echo "ok   hook ignored, exit 2 at option checking${keys:+ (keys: $keys)}"
    else
        echo "FAIL hook run${keys:+ (keys: $keys)}: rc $rc, stderr: $(head -1 err.txt)"
        FAILED=1
    fi
done
[ "$FAILED" -eq 0 ]
