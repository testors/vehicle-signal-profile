#!/usr/bin/env python3
"""Generate common/can_discover_priors.h from vehicle-signal-profile (*.vsp) files.

Usage:
    tools/gen_can_discover_priors.py [profiles_path] -o common/can_discover_priors.h
    tools/gen_can_discover_priors.py --check        # exit 1 if the header is stale

Only classic CAN layouts the engine can use are emitted: DLC 1..8, bit length
1..32, field inside the frame, not multiplexed, and strings that fit the C
struct. Everything else is skipped and counted in the stderr summary.

Sign-magnitude fields (VSP decode.sign_bit) are emitted with sign 'S' and the
sign bit position; the magnitude may then be at most 31 bits.

The header records where it came from: the profiles path, the
vehicle-signal-profile git revision, the file count, and a digest. The digest
is sha256 over "<sha256 of file>  <path relative to profiles_path>\\n" lines
sorted by path, which matches:
    cd profiles && find . -name '*.vsp' -type f | sed 's|^\\./||' | LC_ALL=C sort \\
        | xargs shasum -a 256 | shasum -a 256
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field
from decimal import Decimal
from pathlib import Path


CANONICAL_MAP = {
    "steering.wheel.angle": "DISC_SIG_STEERING",
    "engine.speed": "DISC_SIG_RPM",
    "throttle.position": "DISC_SIG_THROTTLE",
    "accelerator.pedal.position": "DISC_SIG_THROTTLE",
    "brake.pedal.position": "DISC_SIG_BRAKE",
    "brake.pressure": "DISC_SIG_BRAKE",
    "brake.pressure.front": "DISC_SIG_BRAKE",
    "brake.pressure.rear": "DISC_SIG_BRAKE",
    "transmission.gear.selected": "DISC_SIG_GEAR",
    "transmission.gear.requested": "DISC_SIG_GEAR",
    "vehicle.speed": "DISC_SIG_WHEEL_SPEED",
    "wheel.speed": "DISC_SIG_WHEEL_SPEED",
    "wheel.speed.front_left": "DISC_SIG_WHEEL_SPEED",
    "wheel.speed.front_right": "DISC_SIG_WHEEL_SPEED",
    "wheel.speed.rear_left": "DISC_SIG_WHEEL_SPEED",
    "wheel.speed.rear_right": "DISC_SIG_WHEEL_SPEED",
}

# Limits of the C side (common/can_discover.h).
MAX_DLC = 8
MAX_BIT_LENGTH = 32  # values are extracted into uint32_t

# Max string bytes (excluding NUL) per disc_discover_prior_t char field.
STRING_LIMITS = {
    "name": 63,       # name[64]
    "factor": 31,     # factor[32]
    "offset": 31,     # offset[32]
    "min": 31,        # min_val[32]
    "max": 31,        # max_val[32]
    "unit": 31,       # unit[32]
    "canonical": 47,  # canonical[48]
}

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "common" / "can_discover_priors.h"


@dataclass(frozen=True)
class Provenance:
    profiles_path: str  # as displayed: relative to the repo root when possible
    revision: str       # "<git sha>[-dirty]" or "unknown"
    file_count: int
    digest: str         # full sha256 hex

    @property
    def source_id(self) -> str:
        return f"{self.revision} sha256:{self.digest[:16]}"


def signal_enum(canonical_id: str | None) -> str | None:
    if not canonical_id:
        return None
    return CANONICAL_MAP.get(canonical_id)


def c_string(value: str) -> str:
    raw = value.encode("utf-8", errors="replace")
    out: list[str] = ['"']
    prev = 0
    for b in raw:
        if b == 0x22:
            out.append(r"\"")
        elif b == 0x5C:
            out.append(r"\\")
        elif b == 0x3F and prev == 0x3F:
            out.append(r"\?")  # avoid "??x" trigraphs
        elif 0x20 <= b <= 0x7E:
            out.append(chr(b))
        else:
            # Octal escapes stop after 3 digits; \xNN would swallow a
            # following hex digit (e.g. "\xB0C" for degree-C).
            out.append(f"\\{b:03o}")
        prev = b
    out.append('"')
    return "".join(out)


def c_comment(value: str) -> str:
    return value.replace("*/", "* /")


def number_string(value) -> str:
    if value is None:
        return "0"
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, Decimal):
        s = format(value, "f")
    elif isinstance(value, int):
        s = str(value)
    elif isinstance(value, float):
        s = format(value, ".12g")
    else:
        s = str(value)

    if "." in s:
        s = s.rstrip("0").rstrip(".")
    if s in ("", "-0"):
        return "0"
    return s


# A canonical "preferred unit" that names a kind of value, not a scale: safe to show for a
# decode without a unit.
VALUE_KIND_UNITS = {"gear", "bool", "state", "count"}


def prior_unit(decode: dict, canonical: dict) -> str:
    """The decode's own unit. "" is a plain number (a raw count or code), so a scaled unit
    such as "%" or "km/h" is not borrowed from the canonical id for it; a kind (gear,
    bool, state, count) is, and a decode with no unit field takes the preferred unit."""
    preferred = str(canonical.get("preferred_unit") or "")
    unit = decode.get("unit")
    if unit is None:
        return preferred
    if unit == "" and preferred in VALUE_KIND_UNITS:
        return preferred
    return str(unit)


def is_finite_number(value) -> bool:
    if value is None or isinstance(value, bool):
        return True
    if isinstance(value, Decimal):
        return value.is_finite()
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    try:
        return math.isfinite(float(str(value)))
    except ValueError:
        return False


def int_or_none(value) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    if isinstance(value, Decimal):
        return int(value)
    if isinstance(value, str):
        s = value.strip()
        if not s:
            return None
        base = 16 if s.lower().startswith("0x") else 10
        return int(s, base)
    return int(value)


def load_vsp(data: bytes):
    return json.loads(data.decode("utf-8"),
                      parse_float=Decimal,
                      parse_int=int)


def iter_vsp_files(path: Path) -> list[tuple[str, Path]]:
    """Return (relative posix path, path) pairs sorted by relative path."""
    if path.is_file():
        return [(path.name, path)] if path.suffix == ".vsp" else []
    files = [(p.relative_to(path).as_posix(), p)
             for p in path.rglob("*.vsp") if p.is_file()]
    files.sort(key=lambda item: item[0])
    return files


def git_revision(profiles_path: Path) -> str:
    if profiles_path.is_dir():
        git_dir, pathspec = profiles_path, "."
    else:
        git_dir, pathspec = profiles_path.parent, profiles_path.name

    def git(*args: str) -> str | None:
        try:
            proc = subprocess.run(
                ["git", "--no-optional-locks", "-C", str(git_dir), *args],
                capture_output=True, text=True, check=False)
        except OSError:
            return None
        return proc.stdout if proc.returncode == 0 else None

    # Last commit that touched the profiles, so unrelated commits elsewhere in
    # the vehicle-signal-profile repo do not make --check fail.
    head = git("log", "-1", "--format=%H", "--", pathspec)
    if not head or not head.strip():
        return "unknown"
    rev = head.strip()
    status = git("status", "--porcelain", "--", pathspec)
    if status is None or status.strip():
        rev += "-dirty"
    return rev


def display_path(path: Path) -> str:
    """Path relative to the repo root (e.g. ../vehicle-signal-profile/profiles),
    or absolute when it is not inside the repo or a sibling of it."""
    absolute = os.path.abspath(path)
    try:
        rel = Path(os.path.relpath(absolute, REPO_ROOT)).as_posix()
    except ValueError:  # different drive on Windows
        return Path(absolute).as_posix()
    if rel.startswith("../../"):
        return Path(absolute).as_posix()
    return rel


def field_bits(start_bit: int, bit_length: int, byte_order: int) -> list[int]:
    """DBC bit position of every bit of a field (byte_order 1 = Intel, 0 = Motorola)."""
    out: list[int] = []
    bit = start_bit
    for _ in range(bit_length):
        out.append(bit)
        if byte_order == 1:
            bit += 1
        else:
            bit = bit + 15 if bit % 8 == 0 else bit - 1
    return out


def field_bytes(start_bit: int, bit_length: int, byte_order: int) -> list[int]:
    """Byte index of every bit of a DBC field (byte_order 1 = Intel, 0 = Motorola)."""
    return [bit // 8 for bit in field_bits(start_bit, bit_length, byte_order)]


def value_table_strings(values) -> tuple[str, str]:
    """decode.values as (VSP JSON array, DBC VAL_ pairs), or ("", "") when absent.

    Both are carried to discovered signals, so a matched prior keeps its raw code
    meanings (BMW 0x3F9 gear: raw 5 = 1st, value 1; raw 3 = P, value null).
    """
    if not isinstance(values, list) or not values:
        return "", ""
    entries = []
    for entry in values:
        if not isinstance(entry, dict) or not isinstance(entry.get("raw"), int) or "label" not in entry:
            return "", ""
        item = {"raw": entry["raw"], "label": str(entry["label"])}
        if "value" in entry:
            item["value"] = entry["value"]
        entries.append(item)
    values_json = json.dumps(entries, ensure_ascii=False, separators=(", ", ": "))
    values_dbc = " ".join(f'{e["raw"]} "{e["label"].replace(chr(34), chr(39))}"' for e in entries)
    return values_json, values_dbc


def prior_key(signal: dict, frame_dlcs: dict[tuple[str | None, int], int]):
    """Return (row key, None) for a usable signal, or (None, skip reason)."""
    acquisition = signal.get("acquisition") or {}
    canonical = signal.get("canonical") or {}
    decode = signal.get("decode") or {}
    canonical_id = canonical.get("id")
    sig_enum = signal_enum(canonical_id)

    if decode.get("mux"):
        return None, "multiplexed"
    if decode.get("value_type") not in (None, "integer") or decode.get("polynomial"):
        return None, "non_integer_decode"   # priors describe linear integer fields

    try:
        frame_id = int_or_none(acquisition.get("frame_id"))
        start_bit = int_or_none(decode.get("start_bit"))
        bit_length = int_or_none(decode.get("bit_length"))
    except (TypeError, ValueError):
        return None, "invalid_field"
    if frame_id is None or start_bit is None or bit_length is None:
        return None, "missing_field"

    byte_order_name = str(decode.get("byte_order") or "")
    if byte_order_name in ("little_endian", "little", "intel"):
        byte_order = 1
    elif byte_order_name in ("big_endian", "big", "motorola"):
        byte_order = 0
    else:
        return None, "unknown_byte_order"

    if bit_length < 1 or bit_length > MAX_BIT_LENGTH:
        return None, "bit_length_out_of_range"
    if start_bit < 0:
        return None, "negative_start_bit"

    sign_bit = decode.get("sign_bit")
    if sign_bit is not None:
        if isinstance(sign_bit, bool) or not isinstance(sign_bit, int):
            return None, "invalid_sign_bit"
        if not 0 <= sign_bit < MAX_DLC * 8:
            return None, "sign_bit_out_of_range"
        if decode.get("signed"):
            return None, "sign_bit_with_signed"
        if bit_length > MAX_BIT_LENGTH - 1:
            return None, "sign_magnitude_too_wide"  # magnitude + sign must fit uint32_t
        if sign_bit in field_bits(start_bit, bit_length, byte_order):
            return None, "sign_bit_overlaps_field"

    used_bytes = field_bytes(start_bit, bit_length, byte_order)
    if sign_bit is not None:
        used_bytes.append(sign_bit // 8)
    bus = acquisition.get("bus")
    dlc = frame_dlcs.get((bus, frame_id))
    if dlc is None:
        dlc = frame_dlcs.get((None, frame_id))
    if dlc is None:
        dlc = max(used_bytes) + 1
        if dlc > MAX_DLC:
            return None, "inferred_dlc_too_large"
    elif dlc < 1 or dlc > MAX_DLC:
        return None, "dlc_out_of_range"
    if any(b < 0 or b >= dlc for b in used_bytes):
        return None, "field_outside_dlc"

    numbers = {
        "factor": decode.get("factor", 1),
        "offset": decode.get("offset", 0),
        "min": decode.get("minimum", 0),
        "max": decode.get("maximum", 0),
    }
    if not all(is_finite_number(v) for v in numbers.values()):
        return None, "non_finite_number"
    if number_string(numbers["factor"]) == "0":
        return None, "zero_factor"

    strings = {
        "name": str(signal.get("name") or canonical_id or "Signal"),
        **{k: number_string(v) for k, v in numbers.items()},
        "unit": prior_unit(decode, canonical),
        "canonical": str(canonical_id),
    }
    for field, limit in STRING_LIMITS.items():
        if len(strings[field].encode("utf-8", errors="replace")) > limit:
            return None, f"string_too_long:{field}"
    values_json, values_dbc = value_table_strings(decode.get("values"))

    key = (
        sig_enum,
        frame_id,
        dlc,
        start_bit,
        bit_length,
        byte_order,
        "S" if sign_bit is not None else ("-" if decode.get("signed") else "+"),
        sign_bit if sign_bit is not None else -1,
        strings["name"],
        strings["factor"],
        strings["offset"],
        strings["min"],
        strings["max"],
        strings["unit"],
        strings["canonical"],
        values_json,
        values_dbc,
    )
    return key, None


def build_frame_dlc_map(profile: dict) -> dict[tuple[str | None, int], int]:
    frame_dlcs: dict[tuple[str | None, int], int] = {}
    for frame in profile.get("can_frames", []) or []:
        if not isinstance(frame, dict):
            continue
        try:
            frame_id = int_or_none(frame.get("frame_id"))
            dlc = int_or_none(frame.get("dlc"))
        except (TypeError, ValueError):
            continue
        if frame_id is None or dlc is None:
            continue
        bus = frame.get("bus")
        frame_dlcs[(bus, frame_id)] = dlc
        frame_dlcs.setdefault((None, frame_id), dlc)
    return frame_dlcs


@dataclass
class Stats:
    profiles: int = 0
    signals: int = 0
    considered: int = 0
    accepted: int = 0
    rows: int = 0
    skipped: collections.Counter = field(default_factory=collections.Counter)


def parse_vsps(profiles_path: Path):
    support = collections.Counter()
    stats = Stats()
    manifest: list[str] = []

    for rel, path in iter_vsp_files(profiles_path):
        data = path.read_bytes()
        manifest.append(f"{hashlib.sha256(data).hexdigest()}  {rel}\n")
        profile = load_vsp(data)
        stats.profiles += 1
        frame_dlcs = build_frame_dlc_map(profile)

        for signal in profile.get("signals", []) or []:
            if not isinstance(signal, dict):
                continue
            stats.signals += 1
            acquisition = signal.get("acquisition") or {}
            if acquisition.get("type") != "can_frame":
                continue
            canonical = signal.get("canonical") or {}
            if signal_enum(canonical.get("id")) is None:
                continue

            stats.considered += 1
            key, reason = prior_key(signal, frame_dlcs)
            if key is None:
                stats.skipped[reason] += 1
                continue
            stats.accepted += 1
            support[key] += 1

    rows = [(*key, count) for key, count in support.items()]
    # Most supported layout first per (signal, can_id); the trailing full
    # row makes the order total, so it never depends on file order.
    rows.sort(key=lambda r: (r[0], r[1], -r[-1], r[3], r[4], r[8], r))
    stats.rows = len(rows)

    provenance = Provenance(
        profiles_path=display_path(profiles_path),
        revision=git_revision(profiles_path),
        file_count=stats.profiles,
        digest=hashlib.sha256("".join(manifest).encode("utf-8")).hexdigest(),
    )
    return rows, provenance, stats


def emit_header(rows, provenance: Provenance) -> str:
    lines = [
        "/* Auto-generated by tools/gen_can_discover_priors.py. */",
        "/* Do not edit manually; regenerate from VSP profiles. */",
        "/* Source profiles: %s */" % c_comment(provenance.profiles_path),
        "/* Source revision: %s */" % c_comment(provenance.revision),
        "/* Source files: %d *.vsp */" % provenance.file_count,
        "/* Source digest: sha256:%s */" % provenance.digest,
        "/*   = sha256 of sorted \"<sha256>  <relative path>\" lines, one per file */",
        "#ifndef CAN_DISCOVER_PRIORS_H",
        "#define CAN_DISCOVER_PRIORS_H",
        "",
        "#define DISC_CAN_DISCOVER_PRIORS_SOURCE %s" % c_string(provenance.source_id),
        "",
        "static const disc_discover_prior_t s_can_discover_priors[] = {",
    ]

    for row in rows:
        (
            sig_enum,
            can_id,
            dlc,
            start_bit,
            bit_length,
            byte_order,
            sign_char,
            sign_bit,
            name,
            factor,
            offset,
            min_val,
            max_val,
            unit,
            canonical,
            values_json,
            values_dbc,
            support_count,
        ) = row

        values = ", %s, %s" % (c_string(values_json), c_string(values_dbc)) if values_json else ", NULL, NULL"
        lines.append(
            "    { %s, %du, %uu, %d, %d, %d, '%s', %d, %s, %s, %s, %s, %s, %s, %s, %d%s },"
            % (
                sig_enum,
                can_id,
                dlc,
                start_bit,
                bit_length,
                byte_order,
                sign_char,
                sign_bit,
                c_string(name),
                c_string(factor),
                c_string(offset),
                c_string(min_val),
                c_string(max_val),
                c_string(unit),
                c_string(canonical),
                support_count,
                values,
            )
        )

    lines.extend(
        [
            "};",
            "",
            "#define DISC_CAN_DISCOVER_PRIOR_COUNT ((int)(sizeof(s_can_discover_priors) / sizeof(s_can_discover_priors[0])))",
            "",
            "#endif",
        ]
    )
    return "\n".join(lines) + "\n"


def print_summary(stats: Stats, provenance: Provenance) -> None:
    err = sys.stderr
    print(f"source: {provenance.profiles_path} @ {provenance.source_id}", file=err)
    print(f"profiles scanned:   {stats.profiles}", file=err)
    print(f"signals scanned:    {stats.signals}", file=err)
    print(f"signals considered: {stats.considered} (CAN frame, mapped canonical id)", file=err)
    print(f"signals accepted:   {stats.accepted}", file=err)
    print(f"rows emitted:       {stats.rows}", file=err)
    total = sum(stats.skipped.values())
    print(f"signals skipped:    {total}", file=err)
    for reason, count in sorted(stats.skipped.items()):
        print(f"  {reason}: {count}", file=err)


def main() -> int:
    default_profiles_dir = REPO_ROOT.parent / "vehicle-signal-profile" / "profiles"
    parser = argparse.ArgumentParser(
        description="Generate embedded can_discover priors from VSP profiles."
    )
    parser.add_argument("profiles_path", nargs="?", default=str(default_profiles_dir))
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output header path, or - for stdout (default: stdout; "
             "with --check: common/can_discover_priors.h)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Regenerate in memory and exit 1 if the output file differs",
    )
    args = parser.parse_args()

    profiles_path = Path(args.profiles_path)
    if not profiles_path.exists():
        parser.error(f"VSP profile path not found: {profiles_path}")

    rows, provenance, stats = parse_vsps(profiles_path)
    header = emit_header(rows, provenance).encode("utf-8")
    print_summary(stats, provenance)

    if args.check:
        if args.output == "-":
            parser.error("--check needs an output file, not stdout")
        output = Path(args.output) if args.output else DEFAULT_OUTPUT
        try:
            current = output.read_bytes()
        except FileNotFoundError:
            current = None
        if current == header:
            print(f"{output}: up to date", file=sys.stderr)
            return 0
        state = "missing" if current is None else "out of date"
        print(f"{output}: {state}; regenerate with "
              f"tools/gen_can_discover_priors.py -o {output}", file=sys.stderr)
        return 1

    if args.output is None or args.output == "-":
        sys.stdout.buffer.write(header)
    else:
        Path(args.output).write_bytes(header)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
