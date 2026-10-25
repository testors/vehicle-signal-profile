#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import json
import sys
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


def signal_enum(canonical_id: str | None) -> str | None:
    if not canonical_id:
        return None
    return CANONICAL_MAP.get(canonical_id)


def c_string(value: str) -> str:
    raw = value.encode("utf-8", errors="replace")
    out: list[str] = ['"']
    for b in raw:
        if b == 0x22:
            out.append(r"\"")
        elif b == 0x5C:
            out.append(r"\\")
        elif 0x20 <= b <= 0x7E:
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02X}")
    out.append('"')
    return "".join(out)


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


def load_vsp(path: Path):
    return json.loads(path.read_text(encoding="utf-8"),
                      parse_float=Decimal,
                      parse_int=int)


def iter_vsp_files(path: Path):
    if path.is_file():
        if path.suffix == ".vsp":
            yield path
        return
    yield from sorted(path.rglob("*.vsp"))


def build_frame_dlc_map(profile: dict) -> dict[tuple[str | None, int], int]:
    frame_dlcs: dict[tuple[str | None, int], int] = {}
    for frame in profile.get("can_frames", []) or []:
        if not isinstance(frame, dict):
            continue
        frame_id = int_or_none(frame.get("frame_id"))
        dlc = int_or_none(frame.get("dlc"))
        if frame_id is None or dlc is None:
            continue
        bus = frame.get("bus")
        frame_dlcs[(bus, frame_id)] = dlc
        frame_dlcs.setdefault((None, frame_id), dlc)
    return frame_dlcs


def inferred_dlc(start_bit: int, bit_length: int, byte_order: int) -> int:
    bit = start_bit
    max_byte = 0
    for _ in range(bit_length):
        if bit >= 0:
            max_byte = max(max_byte, bit // 8)
        bit = bit + 1 if byte_order == 1 else (bit + 15 if bit % 8 == 0 else bit - 1)
    return min(max_byte + 1, 8)


def parse_vsps(profiles_path: Path):
    support = collections.Counter()

    for path in iter_vsp_files(profiles_path):
        profile = load_vsp(path)
        frame_dlcs = build_frame_dlc_map(profile)

        for signal in profile.get("signals", []) or []:
            if not isinstance(signal, dict):
                continue
            acquisition = signal.get("acquisition") or {}
            if acquisition.get("type") != "can_frame":
                continue

            canonical = signal.get("canonical") or {}
            sig_enum = signal_enum(canonical.get("id"))
            if sig_enum is None:
                continue

            decode = signal.get("decode") or {}
            frame_id = int_or_none(acquisition.get("frame_id"))
            start_bit = int_or_none(decode.get("start_bit"))
            bit_length = int_or_none(decode.get("bit_length"))
            if frame_id is None or start_bit is None or bit_length is None:
                continue

            byte_order_name = str(decode.get("byte_order") or "")
            if byte_order_name in ("little_endian", "little", "intel"):
                byte_order = 1
            elif byte_order_name in ("big_endian", "big", "motorola"):
                byte_order = 0
            else:
                continue

            bus = acquisition.get("bus")
            dlc = frame_dlcs.get((bus, frame_id)) or frame_dlcs.get((None, frame_id))
            if dlc is None:
                dlc = inferred_dlc(start_bit, bit_length, byte_order)

            key = (
                sig_enum,
                frame_id,
                dlc,
                start_bit,
                bit_length,
                byte_order,
                "-" if decode.get("signed") else "+",
                str(signal.get("name") or canonical.get("id") or "Signal"),
                number_string(decode.get("factor", 1)),
                number_string(decode.get("offset", 0)),
                number_string(decode.get("minimum", 0)),
                number_string(decode.get("maximum", 0)),
                str(decode.get("unit") or canonical.get("preferred_unit") or ""),
            )
            support[key] += 1

    rows = []
    for key, count in support.items():
        rows.append((*key, count))

    rows.sort(
        key=lambda r: (
            r[0],
            r[1],
            -r[13],
            r[3],
            r[4],
            r[7],
        )
    )
    return rows


def emit_header(rows) -> str:
    lines = [
        "/* Auto-generated by tools/gen_can_discover_priors.py. */",
        "/* Do not edit manually; regenerate from VSP profiles. */",
        "#ifndef CAN_DISCOVER_PRIORS_H",
        "#define CAN_DISCOVER_PRIORS_H",
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
            name,
            factor,
            offset,
            min_val,
            max_val,
            unit,
            support_count,
        ) = row

        lines.append(
            "    { %s, %du, %uu, %d, %d, %d, '%s', %s, %s, %s, %s, %s, %s, %d },"
            % (
                sig_enum,
                can_id,
                dlc,
                start_bit,
                bit_length,
                byte_order,
                sign_char,
                c_string(name),
                c_string(factor),
                c_string(offset),
                c_string(min_val),
                c_string(max_val),
                c_string(unit),
                support_count,
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


def main() -> int:
    default_profiles_dir = (
        Path(__file__).resolve().parents[2]
        / "vehicle-signal-profile"
        / "profiles"
    )
    parser = argparse.ArgumentParser(
        description="Generate embedded can_discover priors from VSP profiles."
    )
    parser.add_argument("profiles_path", nargs="?", default=str(default_profiles_dir))
    parser.add_argument(
        "-o",
        "--output",
        default="-",
        help="Output header path, or - for stdout",
    )
    args = parser.parse_args()

    profiles_path = Path(args.profiles_path)
    if not profiles_path.exists():
        parser.error(f"VSP profile path not found: {profiles_path}")

    header = emit_header(parse_vsps(profiles_path))
    if args.output == "-":
        sys.stdout.write(header)
    else:
        Path(args.output).write_text(header, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
