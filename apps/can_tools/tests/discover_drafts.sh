#!/bin/sh
# The draft VSP of every discover_regression scenario, written the way a
# user gets it (the session dump replayed with `can_tools replay`), through
# the repository's tools/validate_vsp.py: no errors and no warnings.
#
#   tests/discover_drafts.sh DISCOVER_REGRESSION CAN_TOOLS PYTHON3 VALIDATE_VSP_PY
set -u
REG="$1"
BIN="$2"
PY="$3"
VALIDATE="$4"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/discover_drafts.XXXXXX") || exit 1
trap 'rm -rf "$WORK"' EXIT
FAILED=0
TOTAL=0

for name in $("$REG" --list); do
    TOTAL=$((TOTAL + 1))
    dir="$WORK/$name"
    mkdir -p "$dir"
    if ! "$REG" --dump "$name" "$dir" > /dev/null; then
        echo "FAIL $name: dump"
        FAILED=$((FAILED + 1))
        continue
    fi
    if ! "$BIN" replay "$dir" --out-dir "$dir/out" > "$WORK/replay.log" 2>&1; then
        echo "FAIL $name: replay"
        tail -5 "$WORK/replay.log"
        FAILED=$((FAILED + 1))
        continue
    fi
    out=$("$PY" "$VALIDATE" "$dir/out/ptcan_discover_draft.vsp" 2>&1)
    if echo "$out" | grep -q " errors=0 warnings=0$"; then
        echo "ok   $name"
    else
        echo "FAIL $name"
        echo "$out" | sed "s#$WORK/##" | tail -10
        FAILED=$((FAILED + 1))
    fi
done

echo "$((TOTAL - FAILED))/$TOTAL draft VSPs valid without warnings"
[ "$TOTAL" -gt 0 ] && [ "$FAILED" -eq 0 ]
