#!/usr/bin/env python3
"""Shared helpers for public Vehicle Signal Profile tools."""

from __future__ import annotations

import copy
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]

PROFILE_EXTENSION = ".vsp"
PROFILE_MIME_TYPE = "application/vnd.vehicle-signal-profile+json"

BO_RE = re.compile(r"^BO_\s+(?P<id>\d+)\s+(?P<name>[^:]+):\s+(?P<dlc>\d+)\s+")
SG_RE = re.compile(
    r'^ SG_\s+(?P<name>[^ :]+)(?:\s+(?P<mux>M|m\d+))?\s*:\s*'
    r'(?P<start>\d+)\|(?P<length>\d+)@(?P<byte_order>[01])(?P<sign>[+-])\s+'
    r'\((?P<factor>[-+0-9.eE]+),(?P<offset>[-+0-9.eE]+)\)\s+'
    r'\[(?P<min>[-+0-9.eE]+)\|(?P<max>[-+0-9.eE]+)\]\s+'
    r'"(?P<unit>[^"]*)"'
)
CANONICAL_ID_RE = re.compile(r"^[a-z][a-z0-9_]*(?:\.[a-z0-9_]+)*$")


@dataclass(frozen=True)
class DbcMessage:
    frame_id: int
    name: str
    dlc: int


@dataclass(frozen=True)
class DbcSignal:
    frame_id: int
    message_name: str
    name: str
    mux: str | None
    start_bit: int
    bit_length: int
    byte_order: int
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    sign_bit: int | None = None
    values: tuple[tuple[int, str], ...] = ()
    value_type: str = "integer"
    polynomial: tuple[float, ...] = ()


DBC_VAL_RE = re.compile(r"^VAL_\s+(?P<id>\d+)\s+(?P<name>\w+)\s+(?P<body>(?:-?\d+\s+\"[^\"]*\"\s*)*);", re.M)
DBC_VAL_ENTRY_RE = re.compile(r"(-?\d+)\s+\"([^\"]*)\"")
# SIG_VALTYPE_ <id> <name> : 1 (IEEE float) | 2 (IEEE double)
DBC_VALTYPE_RE = re.compile(r"^SIG_VALTYPE_\s+(?P<id>\d+)\s+(?P<name>\w+)\s*:\s*(?P<type>[12])\s*;", re.M)
DBC_VALUE_TYPES = {"1": "float32", "2": "float64"}
# profile_to_dbc keeps a polynomial conversion in a signal comment.
DBC_POLYNOMIAL_RE = re.compile(r'^CM_\s+SG_\s+(?P<id>\d+)\s+(?P<name>\w+)\s+"VSP polynomial c0\.\.cn: (?P<terms>[^"]*)"\s*;', re.M)


def hex_id(value: int | None) -> str | None:
    return None if value is None else f"0x{value:X}"


def id_format(frame_id: int) -> str:
    return "extended29" if frame_id > 0x7FF else "standard11"


def byte_order_name(value: int) -> str:
    return "little_endian" if value == 1 else "big_endian"


def root_path(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else ROOT / path


def profile_output_path(path: str | Path) -> Path:
    output = root_path(path)
    if output.suffix:
        return output
    return output.with_suffix(PROFILE_EXTENSION)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def parse_dbc(path: Path) -> tuple[dict[int, DbcMessage], list[DbcSignal]]:
    messages: dict[int, DbcMessage] = {}
    signals: list[DbcSignal] = []
    current: DbcMessage | None = None
    text = path.read_text(errors="ignore")

    for line in text.splitlines():
        bo = BO_RE.match(line)
        if bo:
            raw_frame_id = int(bo.group("id"))
            frame_id = raw_frame_id & 0x1FFFFFFF if raw_frame_id & 0x80000000 else raw_frame_id
            current = DbcMessage(
                frame_id=frame_id,
                name=bo.group("name").strip(),
                dlc=int(bo.group("dlc")),
            )
            messages[current.frame_id] = current
            continue

        sg = SG_RE.match(line)
        if sg and current is not None:
            signals.append(
                DbcSignal(
                    frame_id=current.frame_id,
                    message_name=current.name,
                    name=sg.group("name"),
                    mux=sg.group("mux"),
                    start_bit=int(sg.group("start")),
                    bit_length=int(sg.group("length")),
                    byte_order=int(sg.group("byte_order")),
                    signed=sg.group("sign") == "-",
                    factor=float(sg.group("factor")),
                    offset=float(sg.group("offset")),
                    minimum=float(sg.group("min")),
                    maximum=float(sg.group("max")),
                    unit=sg.group("unit"),
                )
            )

    value_tables: dict[tuple[int, str], tuple[tuple[int, str], ...]] = {}
    for val in DBC_VAL_RE.finditer(text):
        raw_frame_id = int(val.group("id"))
        frame_id = raw_frame_id & 0x1FFFFFFF if raw_frame_id & 0x80000000 else raw_frame_id
        entries = [(int(raw), label.strip()) for raw, label in DBC_VAL_ENTRY_RE.findall(val.group("body"))]
        entries = [(raw, label) for raw, label in entries if label]
        if entries:
            value_tables[(frame_id, val.group("name"))] = tuple(sorted(dict(entries).items()))
    signals = [
        replace(signal, values=value_tables[(signal.frame_id, signal.name)])
        if (signal.frame_id, signal.name) in value_tables
        else signal
        for signal in signals
    ]
    value_types: dict[tuple[int, str], str] = {}
    for valtype in DBC_VALTYPE_RE.finditer(text):
        raw_frame_id = int(valtype.group("id"))
        frame_id = raw_frame_id & 0x1FFFFFFF if raw_frame_id & 0x80000000 else raw_frame_id
        value_types[(frame_id, valtype.group("name"))] = DBC_VALUE_TYPES[valtype.group("type")]
    signals = [
        replace(signal, value_type=value_types[(signal.frame_id, signal.name)], signed=False)
        if (signal.frame_id, signal.name) in value_types
        else signal
        for signal in signals
    ]
    polynomials: dict[tuple[int, str], tuple[float, ...]] = {}
    for comment in DBC_POLYNOMIAL_RE.finditer(text):
        raw_frame_id = int(comment.group("id"))
        frame_id = raw_frame_id & 0x1FFFFFFF if raw_frame_id & 0x80000000 else raw_frame_id
        polynomials[(frame_id, comment.group("name"))] = tuple(float(t) for t in comment.group("terms").split())
    signals = [
        replace(signal, polynomial=polynomials[(signal.frame_id, signal.name)], factor=1.0, offset=0.0)
        if (signal.frame_id, signal.name) in polynomials
        else signal
        for signal in signals
    ]
    return messages, fold_sign_signals(signals)


DBC_SIGN_SUFFIX = "_Sign"


def dbc_signal_bits(signal: DbcSignal) -> set[int]:
    """Payload bit positions covered by a DBC signal (DBC bit numbering)."""
    if signal.byte_order == 1:
        return set(range(signal.start_bit, signal.start_bit + signal.bit_length))
    positions: set[int] = set()
    pos = signal.start_bit
    for _ in range(signal.bit_length):
        positions.add(pos)
        pos = pos + 15 if pos % 8 == 0 else pos - 1
    return positions


def fold_sign_signals(signals: list[DbcSignal]) -> list[DbcSignal]:
    """Fold a 1-bit `<name>_Sign` signal into `<name>` as a sign-magnitude sign_bit.

    DBC has no sign-magnitude type, so `<name>` carries the unsigned magnitude and
    `<name>_Sign` (same message and mux) carries the sign; profile_to_dbc emits the same pair.
    """
    by_key = {(signal.frame_id, signal.name): signal for signal in signals}
    sign_bits: dict[tuple[int, str], int] = {}
    consumed: set[tuple[int, str]] = set()
    for signal in signals:
        if signal.bit_length != 1 or not signal.name.endswith(DBC_SIGN_SUFFIX):
            continue
        base = by_key.get((signal.frame_id, signal.name[: -len(DBC_SIGN_SUFFIX)]))
        if base is None or base.signed or base.mux != signal.mux:
            continue
        if signal.start_bit in dbc_signal_bits(base):
            continue
        sign_bits[(base.frame_id, base.name)] = signal.start_bit
        consumed.add((signal.frame_id, signal.name))
    return [
        replace(signal, sign_bit=sign_bits[(signal.frame_id, signal.name)])
        if (signal.frame_id, signal.name) in sign_bits
        else signal
        for signal in signals
        if (signal.frame_id, signal.name) not in consumed
    ]


def dbc_signal_decode(signal: DbcSignal) -> dict[str, Any]:
    mux = None
    if signal.mux == "M":
        mux = {"role": "multiplexor"}
    elif signal.mux and signal.mux.startswith("m"):
        mux = {"value": int(signal.mux[1:])}

    decode: dict[str, Any] = {
        "start_bit": signal.start_bit,
        "bit_length": signal.bit_length,
        "byte_order": byte_order_name(signal.byte_order),
        "signed": signal.signed,
    }
    if signal.value_type != "integer":
        decode["value_type"] = signal.value_type
    if signal.sign_bit is not None:
        decode["sign_bit"] = signal.sign_bit
    decode.update(
        {
            "factor": signal.factor,
            "offset": signal.offset,
            **({"polynomial": list(signal.polynomial)} if signal.polynomial else {}),
            "minimum": signal.minimum,
            "maximum": signal.maximum,
            "unit": signal.unit,
            "mux": mux,
        }
    )
    if signal.values:
        decode = with_decode_values(decode, [{"raw": raw, "label": label} for raw, label in signal.values])
    return decode


def with_decode_values(decode: dict[str, Any], values: list[dict[str, Any]]) -> dict[str, Any]:
    """Return decode with "values" placed right before "mux" (spec field order)."""
    result: dict[str, Any] = {}
    for key, item in decode.items():
        if key == "values":
            continue
        if key == "mux":
            result["values"] = values
        result[key] = item
    if "values" not in result:
        result["values"] = values
    return result


def profile_from_dbc(path: Path, *, manufacturer: str, model: str, bitrate: int) -> dict[str, Any]:
    messages, signals = parse_dbc(path)
    grouped: dict[int, list[DbcSignal]] = defaultdict(list)
    for signal in signals:
        grouped[signal.frame_id].append(signal)

    can_frames = []
    for frame_id, message in sorted(messages.items()):
        frame_signals = grouped.get(frame_id, [])
        can_frames.append(
            {
                "bus": "can0",
                "frame_id": frame_id,
                "frame_id_hex": hex_id(frame_id),
                "id_format": id_format(frame_id),
                "name": message.name,
                "dlc": message.dlc,
                "signals": [
                    {"name": signal.name, "decode": dbc_signal_decode(signal)}
                    for signal in sorted(frame_signals, key=lambda item: (item.start_bit, item.name))
                ],
            }
        )

    runtime_signals = []
    for signal in sorted(signals, key=lambda item: (item.frame_id, item.name)):
        runtime_signals.append(
            {
                "name": signal.name,
                "acquisition": {
                    "type": "can_frame",
                    "bus": "can0",
                    "frame_id": signal.frame_id,
                    "frame_id_hex": hex_id(signal.frame_id),
                    "frame_name": signal.message_name,
                },
                "decode": dbc_signal_decode(signal),
            }
        )

    return {
        "schema": "vehicle_data_profile.v1",
        "vehicle": {"manufacturer": manufacturer, "model": model},
        "applicability": {},
        "buses": [{"id": "can0", "type": "can", "bitrate": bitrate}],
        "can_frames": can_frames,
        "diagnostic_queries": [],
        "signals": runtime_signals,
    }


def dbc_identifier(value: Any, fallback: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_]", "_", str(value or "")).strip("_")
    text = re.sub(r"_+", "_", text)
    if not text:
        text = fallback
    if text[0].isdigit():
        text = f"_{text}"
    return text[:128]


def dbc_number(value: Any, default: float) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if number.is_integer():
        return str(int(number))
    return f"{number:.12g}"


def dbc_mux_suffix(decode: dict[str, Any]) -> str:
    mux = decode.get("mux")
    if not isinstance(mux, dict):
        return ""
    if mux.get("role") == "multiplexor":
        return " M"
    if "value" in mux:
        return f" m{int(mux['value'])}"
    return ""


DBC_VALUE_TYPE_CODES = {"float32": 1, "float64": 2}


def dbc_signal_line(signal: dict[str, Any], name: str) -> str:
    decode = signal.get("decode") or {}
    byte_order = 1 if decode.get("byte_order") == "little_endian" else 0
    sign = "-" if decode.get("signed") else "+"
    return (
        f" SG_ {name}{dbc_mux_suffix(decode)} : "
        f"{int(decode.get('start_bit') or 0)}|{int(decode.get('bit_length') or 1)}@{byte_order}{sign} "
        f"({dbc_number(decode.get('factor'), 1)},{dbc_number(decode.get('offset'), 0)}) "
        f"[{dbc_number(decode.get('minimum'), 0)}|{dbc_number(decode.get('maximum'), 0)}] "
        f"\"{str(decode.get('unit') or '')}\" Vector__XXX"
    )


def dbc_sign_signal_line(signal: dict[str, Any], name: str) -> str:
    """Export a sign-magnitude sign bit as its own 1-bit DBC signal."""
    decode = signal.get("decode") or {}
    return (
        f" SG_ {name}{dbc_mux_suffix(decode)} : "
        f"{int(decode['sign_bit'])}|1@1+ (1,0) [0|1] \"\" Vector__XXX"
    )


def dbc_muxor_signal_line(mux: dict[str, Any], name: str) -> str:
    start_bit = int(mux.get("start_bit") or 0)
    bit_length = int(mux.get("bit_length") or 1)
    order = 0 if mux.get("byte_order") == "big_endian" else 1
    maximum = (1 << bit_length) - 1 if 0 < bit_length < 64 else 0
    return (
        f" SG_ {name} M : {start_bit}|{bit_length}@{order}+ "
        f"(1,0) [0|{maximum}] \"\" Vector__XXX"
    )


def synthetic_muxors(signals: list[dict[str, Any]]) -> list[dict[str, Any]]:
    has_muxor = any(
        isinstance((signal.get("decode") or {}).get("mux"), dict)
        and (signal.get("decode") or {})["mux"].get("role") == "multiplexor"
        for signal in signals
    )
    if has_muxor:
        return []

    muxors: dict[tuple[int, int, str], dict[str, Any]] = {}
    for signal in signals:
        mux = (signal.get("decode") or {}).get("mux")
        if not isinstance(mux, dict) or "value" not in mux:
            continue
        if mux.get("start_bit") is None or mux.get("bit_length") is None:
            continue
        key = (int(mux["start_bit"]), int(mux["bit_length"]), str(mux.get("byte_order") or "little_endian"))
        muxors.setdefault(key, mux)
    return [muxors[key] for key in sorted(muxors)]


def dbc_frame_id(frame: dict[str, Any]) -> int:
    frame_id = int(frame.get("frame_id") or 0)
    if frame.get("id_format") == "extended29" or frame_id > 0x7FF:
        return frame_id | 0x80000000
    return frame_id


def profile_to_dbc(profile: dict[str, Any], *, include_diagnostic_responses: bool = False) -> str:
    frame_keys: set[tuple[str, int]] = set()
    for signal in profile.get("signals") or []:
        acquisition = signal.get("acquisition") or {}
        if acquisition.get("type") == "can_frame" and acquisition.get("frame_id") is not None:
            frame_keys.add((str(acquisition.get("bus")), int(acquisition["frame_id"])))
        elif (
            include_diagnostic_responses
            and acquisition.get("type") == "diagnostic_query"
            and acquisition.get("response_frame_id") is not None
        ):
            frame_keys.add((str(acquisition.get("bus")), int(acquisition["response_frame_id"])))

    lines = [
        'VERSION ""',
        "",
        "NS_ :",
        "",
        "BS_:",
        "",
        "BU_: Vector__XXX",
        "",
    ]

    value_tables: list[str] = []
    comments: list[str] = []
    value_types: list[str] = []
    for frame in sorted(profile.get("can_frames") or [], key=lambda item: (str(item.get("bus")), int(item.get("frame_id") or 0))):
        key = (str(frame.get("bus")), int(frame.get("frame_id") or 0))
        if key not in frame_keys:
            continue
        frame_name = dbc_identifier(frame.get("name"), f"FRAME_{int(frame.get('frame_id') or 0):X}")
        lines.append(f"BO_ {dbc_frame_id(frame)} {frame_name}: {int(frame.get('dlc') or 8)} Vector__XXX")
        used_names: set[str] = set()
        for index, mux in enumerate(synthetic_muxors(frame.get("signals") or []), start=1):
            base_name = dbc_identifier(f"{frame_name}_MUX" if index == 1 else f"{frame_name}_MUX_{index}", "MUX")
            mux_name = base_name
            suffix = 2
            while mux_name in used_names:
                mux_name = dbc_identifier(f"{base_name}_{suffix}", "MUX")
                suffix += 1
            used_names.add(mux_name)
            lines.append(dbc_muxor_signal_line(mux, mux_name))
        for signal in frame.get("signals") or []:
            base_name = dbc_identifier(signal.get("name"), "SIGNAL")
            signal_name = base_name
            suffix = 2
            while signal_name in used_names:
                signal_name = dbc_identifier(f"{base_name}_{suffix}", "SIGNAL")
                suffix += 1
            used_names.add(signal_name)
            lines.append(dbc_signal_line(signal, signal_name))
            if (signal.get("decode") or {}).get("sign_bit") is not None:
                sign_name = dbc_identifier(f"{signal_name}_Sign", "SIGN")
                suffix = 2
                while sign_name in used_names:
                    sign_name = dbc_identifier(f"{signal_name}_Sign_{suffix}", "SIGN")
                    suffix += 1
                used_names.add(sign_name)
                lines.append(dbc_sign_signal_line(signal, sign_name))
            decode = signal.get("decode") or {}
            values = decode.get("values") or []
            if values:
                labels = " ".join(f'{int(v["raw"])} "{v["label"]}"' for v in values)
                value_tables.append(f"VAL_ {dbc_frame_id(frame)} {signal_name} {labels} ;")
            if decode.get("value_type") in DBC_VALUE_TYPE_CODES:
                value_types.append(f"SIG_VALTYPE_ {dbc_frame_id(frame)} {signal_name} : {DBC_VALUE_TYPE_CODES[decode['value_type']]};")
            if decode.get("polynomial"):
                # DBC has no polynomial conversion: the signal exports as raw (1,0).
                terms = " ".join(dbc_number(c, 0) for c in decode["polynomial"])
                comments.append(f'CM_ SG_ {dbc_frame_id(frame)} {signal_name} "VSP polynomial c0..cn: {terms}";')
        lines.append("")

    lines.extend(comments)
    lines.extend(value_tables)
    lines.extend(value_types)
    return "\n".join(lines).rstrip() + "\n"


def normalize_hex_text(value: Any) -> str:
    return re.sub(r"[^0-9A-Fa-f]", "", str(value or "")).upper()


def canonical_number(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 12)
    return value


def canonical_decode(decode: dict[str, Any]) -> dict[str, Any]:
    keys = ("start_bit", "bit_length", "byte_order", "signed", "value_type", "sign_bit", "factor", "offset",
            "polynomial", "minimum", "maximum", "unit", "values", "mux")
    result = {key: canonical_number(decode.get(key)) for key in keys if key in decode}
    if result.get("mux") is None:
        result["mux"] = None
    if result.get("sign_bit") is None:
        result.pop("sign_bit", None)
    if not result.get("values"):
        result.pop("values", None)
    if result.get("value_type") in (None, "integer"):
        result.pop("value_type", None)
    if result.get("polynomial"):
        result["polynomial"] = [canonical_number(c) for c in result["polynomial"]]
    else:
        result.pop("polynomial", None)
    return result


def decode_bit_positions(decode: dict[str, Any]) -> set[int]:
    """Payload bit positions covered by a decode field (DBC bit numbering)."""
    start = int(decode.get("start_bit") or 0)
    length = int(decode.get("bit_length") or 0)
    if decode.get("byte_order") == "big_endian":
        positions: set[int] = set()
        pos = start
        for _ in range(length):
            positions.add(pos)
            pos = pos + 15 if pos % 8 == 0 else pos - 1
        return positions
    return set(range(start, start + length))


def decode_raw_range(decode: dict[str, Any]) -> tuple[int, int]:
    bits = int(decode.get("bit_length") or 0)
    if decode.get("sign_bit") is not None:
        return -((1 << bits) - 1), (1 << bits) - 1
    if decode.get("signed"):
        return -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return 0, (1 << bits) - 1


def validate_values(label: str, decode: dict[str, Any], errors: list[str]) -> None:
    values = decode.get("values")
    if values is None:
        return
    if not isinstance(values, list) or not values:
        errors.append(f"{label}: values must be a non-empty list")
        return
    lo, hi = decode_raw_range(decode)
    seen: set[int] = set()
    for entry in values:
        if not isinstance(entry, dict):
            errors.append(f"{label}: values entries must be objects")
            continue
        raw = entry.get("raw")
        if isinstance(raw, bool) or not isinstance(raw, int):
            errors.append(f"{label}: values raw must be an integer")
            continue
        if not lo <= raw <= hi:
            errors.append(f"{label}: values raw {raw} is outside the field range {lo}..{hi}")
        if raw in seen:
            errors.append(f"{label}: duplicate values raw {raw}")
        seen.add(raw)
        if not isinstance(entry.get("label"), str) or not entry["label"]:
            errors.append(f"{label}: values raw {raw} needs a non-empty label")
        if "value" in entry and entry["value"] is not None and (
            isinstance(entry["value"], bool) or not isinstance(entry["value"], (int, float))
        ):
            errors.append(f"{label}: values raw {raw} value must be a number or null")


def validate_sign_bit(label: str, decode: dict[str, Any], errors: list[str]) -> None:
    sign_bit = decode.get("sign_bit")
    if sign_bit is None:
        return
    if isinstance(sign_bit, bool) or not isinstance(sign_bit, int) or not 0 <= sign_bit < 64:
        errors.append(f"{label}: sign_bit must be an integer bit position 0..63")
        return
    if decode.get("signed"):
        errors.append(f"{label}: sign_bit requires signed=false")
    if sign_bit in decode_bit_positions(decode):
        errors.append(f"{label}: sign_bit {sign_bit} overlaps the magnitude field")


def validate_fits_frame(label: str, decode: dict[str, Any], dlc: Any, errors: list[str]) -> None:
    """Every bit a signal reads (field, sign_bit, mux field) lies inside the frame's dlc bytes.

    A big-endian start_bit numbered from the LSB instead of the MSB (DBC numbering)
    usually shows up here, as a field that runs past the end of the payload.
    """
    if isinstance(dlc, bool) or not isinstance(dlc, int) or dlc <= 0:
        return
    limit = dlc * 8
    parts = [("field", decode_bit_positions(decode), decode.get("byte_order"))]
    sign_bit = decode.get("sign_bit")
    if isinstance(sign_bit, int) and not isinstance(sign_bit, bool):
        parts.append(("sign_bit", {sign_bit}, None))
    mux = decode.get("mux")
    if isinstance(mux, dict) and isinstance(mux.get("start_bit"), int) and isinstance(mux.get("bit_length"), int):
        mux_order = mux.get("byte_order") or "little_endian"
        mux_field = {"start_bit": mux["start_bit"], "bit_length": mux["bit_length"], "byte_order": mux_order}
        parts.append(("mux field", decode_bit_positions(mux_field), mux_order))
    for what, positions, byte_order in parts:
        if not positions or (min(positions) >= 0 and max(positions) < limit):
            continue
        hint = " (big_endian start_bit is the field's MSB)" if byte_order == "big_endian" else ""
        errors.append(f"{label}: {what} covers payload bits {min(positions)}..{max(positions)}, outside dlc {dlc}{hint}")


def warn_lsb_numbered(label: str, decode: dict[str, Any], warnings: list[str]) -> None:
    """Flag a whole-byte big-endian field whose start_bit looks like an LSB number.

    In DBC numbering a byte-aligned big-endian field starts at bit 7 of its first
    byte. start_bit % 8 == 0 is what LSB-based numbering gives for the same field.
    """
    if decode.get("byte_order") != "big_endian":
        return
    start, length = decode.get("start_bit"), decode.get("bit_length")
    if isinstance(start, int) and isinstance(length, int) and length >= 16 and length % 8 == 0 and start % 8 == 0:
        warnings.append(
            f"{label}: big_endian {start}|{length} starts at bit 0 of a byte; "
            f"start_bit may be the LSB, not the MSB (a byte-aligned field starts at bit 7)"
        )


def signal_matches_frame(signal: dict[str, Any], frame: dict[str, Any]) -> bool:
    expected = canonical_decode(signal.get("decode") or {})
    return any(
        item.get("name") == signal.get("name") and canonical_decode(item.get("decode") or {}) == expected
        for item in frame.get("signals", [])
    )


MUX_BYTE_ORDERS = {"little_endian", "big_endian"}
VALUE_TYPE_BITS = {"float32": 32, "float64": 64}


def validate_value_type(label: str, decode: dict[str, Any], errors: list[str]) -> None:
    value_type = decode.get("value_type")
    if value_type is not None and value_type != "integer":
        if value_type not in VALUE_TYPE_BITS:
            errors.append(f"{label}: value_type must be integer, float32 or float64")
        else:
            if decode.get("bit_length") != VALUE_TYPE_BITS[value_type]:
                errors.append(f"{label}: {value_type} needs bit_length {VALUE_TYPE_BITS[value_type]}")
            if decode.get("signed") or decode.get("sign_bit") is not None or decode.get("values"):
                errors.append(f"{label}: a float field has signed false and no sign_bit or values")
    polynomial = decode.get("polynomial")
    if polynomial is not None:
        if (
            not isinstance(polynomial, list)
            or len(polynomial) < 3
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool) and math.isfinite(c) for c in polynomial)
        ):
            errors.append(f"{label}: polynomial must list at least three finite coefficients")
        if decode.get("factor") != 1 or decode.get("offset") != 0:
            errors.append(f"{label}: a polynomial signal has factor 1 and offset 0")
        if value_type not in (None, "integer"):
            errors.append(f"{label}: polynomial applies to integer fields only")


def validate_mux(label: str, decode: dict[str, Any], errors: list[str]) -> None:
    mux = decode.get("mux")
    if mux is None:
        return
    if not isinstance(mux, dict):
        errors.append(f"{label}: mux must be an object or null")
        return
    if mux.get("role") == "multiplexor":
        return
    value = mux.get("value")
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        errors.append(f"{label}: mux.value must be a non-negative integer")
        return
    if mux.get("byte_order") is not None and mux["byte_order"] not in MUX_BYTE_ORDERS:
        errors.append(f"{label}: mux.byte_order must be little_endian or big_endian")
    start_bit, bit_length = mux.get("start_bit"), mux.get("bit_length")
    if start_bit is None and bit_length is None:
        return  # DBC style: the frame's multiplexor signal carries the field
    if not isinstance(start_bit, int) or not isinstance(bit_length, int) or not 0 <= start_bit < 512 or not 1 <= bit_length <= 64:
        errors.append(f"{label}: mux.start_bit/bit_length must be integers (0-511, 1-64)")
        return
    if value >= 1 << bit_length:
        errors.append(f"{label}: mux.value {value} does not fit in {bit_length} bits")


def validate_frame_id_mask(frame: dict[str, Any], errors: list[str]) -> None:
    mask = frame.get("frame_id_mask")
    if mask is None:
        return
    label = f"frame {frame.get('name') or frame.get('frame_id')}"
    width = 0x7FF if frame.get("id_format") == "standard11" else 0x1FFFFFFF
    if not isinstance(mask, int) or isinstance(mask, bool) or mask <= 0 or mask & ~width:
        errors.append(f"{label}: frame_id_mask must be a positive integer within the id width")
        return
    if frame.get("frame_id_mask_hex") is not None and frame["frame_id_mask_hex"] != f"0x{mask:X}":
        errors.append(f"{label}: frame_id_mask_hex does not match frame_id_mask")


def validate_profile(profile: dict[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []

    if profile.get("schema") != "vehicle_data_profile.v1":
        errors.append("schema must be vehicle_data_profile.v1")

    buses = profile.get("buses") or []
    bus_ids = {bus.get("id") for bus in buses if bus.get("id")}
    if not bus_ids:
        errors.append("at least one bus is required")

    frames: dict[tuple[str, int], dict[str, Any]] = {}
    for frame in profile.get("can_frames") or []:
        bus = frame.get("bus")
        frame_id = frame.get("frame_id")
        if bus not in bus_ids:
            errors.append(f"frame {frame.get('name') or frame_id}: unknown bus {bus!r}")
        if frame_id is None:
            errors.append(f"frame {frame.get('name') or '<unnamed>'}: missing frame_id")
            continue
        key = (str(bus), int(frame_id))
        if key in frames:
            errors.append(f"duplicate frame on {bus} id=0x{int(frame_id):X}")
        frames[key] = frame
        validate_frame_id_mask(frame, errors)
        for frame_signal in frame.get("signals") or []:
            frame_label = f"frame {frame.get('name') or frame_id} signal {frame_signal.get('name') or '<unnamed>'}"
            validate_sign_bit(frame_label, frame_signal.get("decode") or {}, errors)
            validate_values(frame_label, frame_signal.get("decode") or {}, errors)
            validate_mux(frame_label, frame_signal.get("decode") or {}, errors)
            validate_value_type(frame_label, frame_signal.get("decode") or {}, errors)
            validate_fits_frame(frame_label, frame_signal.get("decode") or {}, frame.get("dlc"), errors)
            warn_lsb_numbered(frame_label, frame_signal.get("decode") or {}, warnings)
        expected_format = id_format(int(frame_id))
        if frame.get("id_format") and frame.get("id_format") != expected_format:
            errors.append(f"frame {frame.get('name') or frame_id}: id_format should be {expected_format}")

    queries: dict[str, dict[str, Any]] = {}
    for query in profile.get("diagnostic_queries") or []:
        query_id = query.get("id")
        if not query_id:
            errors.append("diagnostic query without id")
            continue
        if query_id in queries:
            errors.append(f"duplicate diagnostic query id {query_id}")
        queries[query_id] = query
        if query.get("bus") not in bus_ids:
            errors.append(f"diagnostic query {query_id}: unknown bus {query.get('bus')!r}")
        if query.get("tx_id") is None:
            errors.append(f"diagnostic query {query_id}: missing tx_id")
        if not normalize_hex_text(query.get("request_hex")):
            errors.append(f"diagnostic query {query_id}: missing request_hex")
        payload = normalize_hex_text(query.get("payload_hex"))
        request = normalize_hex_text(query.get("request_hex"))
        if payload and request and request not in payload:
            warnings.append(f"diagnostic query {query_id}: request_hex is not visible inside payload_hex")

    canonical_ids: dict[str, dict[str, list[str]]] = defaultdict(lambda: defaultdict(list))
    for signal in profile.get("signals") or []:
        name = signal.get("name") or "<unnamed>"
        acquisition = signal.get("acquisition") or {}
        acquisition_type = acquisition.get("type")
        canonical = signal.get("canonical")
        validate_sign_bit(f"signal {name}", signal.get("decode") or {}, errors)
        validate_values(f"signal {name}", signal.get("decode") or {}, errors)
        validate_mux(f"signal {name}", signal.get("decode") or {}, errors)
        validate_value_type(f"signal {name}", signal.get("decode") or {}, errors)

        if canonical is not None:
            if not isinstance(canonical, dict):
                errors.append(f"signal {name}: canonical must be an object")
            else:
                canonical_id = canonical.get("id")
                if not canonical_id:
                    errors.append(f"signal {name}: canonical.id is required when canonical is present")
                elif not CANONICAL_ID_RE.match(str(canonical_id)):
                    errors.append(f"signal {name}: invalid canonical.id {canonical_id!r}")
                else:
                    canonical_ids[str(canonical_id)][str(acquisition_type)].append(str(name))

        bus = acquisition.get("bus")
        if bus not in bus_ids:
            errors.append(f"signal {name}: unknown bus {bus!r}")

        if acquisition_type == "can_frame":
            frame_id = acquisition.get("frame_id")
            if frame_id is None:
                errors.append(f"signal {name}: missing frame_id")
                continue
            frame = frames.get((str(bus), int(frame_id)))
            if frame is None:
                errors.append(f"signal {name}: missing referenced frame {bus}/0x{int(frame_id):X}")
            elif not signal_matches_frame(signal, frame):
                errors.append(f"signal {name}: decode does not match referenced frame {bus}/0x{int(frame_id):X}")
        elif acquisition_type == "diagnostic_query":
            query_id = acquisition.get("query_id")
            query = queries.get(query_id)
            if query is None:
                errors.append(f"signal {name}: missing referenced diagnostic query {query_id!r}")
                continue
            if query.get("bus") != bus:
                errors.append(f"signal {name}: query bus {query.get('bus')!r} does not match signal bus {bus!r}")
            if query.get("tx_id") != acquisition.get("tx_id"):
                errors.append(f"signal {name}: tx_id does not match diagnostic query {query_id}")
            if normalize_hex_text(query.get("request_hex")) != normalize_hex_text(acquisition.get("request_hex")):
                errors.append(f"signal {name}: request_hex does not match diagnostic query {query_id}")

            response_frame_id = acquisition.get("response_frame_id")
            if response_frame_id is None:
                errors.append(f"signal {name}: missing response_frame_id")
                continue
            if query.get("rx_id") is not None and int(query["rx_id"]) != int(response_frame_id):
                errors.append(f"signal {name}: response_frame_id does not match diagnostic query rx_id")
            frame = frames.get((str(bus), int(response_frame_id)))
            if frame is None:
                errors.append(f"signal {name}: missing response frame {bus}/0x{int(response_frame_id):X}")
            elif not signal_matches_frame(signal, frame):
                errors.append(f"signal {name}: decode does not match response frame {bus}/0x{int(response_frame_id):X}")
        else:
            errors.append(f"signal {name}: unknown acquisition type {acquisition_type!r}")

    for canonical_id, names_by_acquisition in sorted(canonical_ids.items()):
        for acquisition_type, names in sorted(names_by_acquisition.items()):
            if len(names) > 1:
                warnings.append(
                    f"canonical.id {canonical_id!r} is used by multiple {acquisition_type} signals: {', '.join(names)}"
                )

    return {
        "errors": errors,
        "warnings": warnings,
        "counts": {
            "buses": len(buses),
            "can_frames": len(profile.get("can_frames") or []),
            "diagnostic_queries": len(profile.get("diagnostic_queries") or []),
            "signals": len(profile.get("signals") or []),
            "canonical_signals": sum(1 for signal in profile.get("signals") or [] if signal.get("canonical")),
        },
    }


def stable_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def signal_compose_key(signal: dict[str, Any]) -> tuple[Any, ...]:
    acquisition = signal.get("acquisition") or {}
    acquisition_type = acquisition.get("type")
    if acquisition_type == "can_frame":
        return (
            acquisition_type,
            acquisition.get("bus"),
            acquisition.get("frame_id"),
            signal.get("name"),
        )
    if acquisition_type == "diagnostic_query":
        return (
            acquisition_type,
            acquisition.get("bus"),
            acquisition.get("query_id"),
            signal.get("name"),
        )
    return (acquisition_type, signal.get("name"), stable_json(acquisition))


def choose_item(
    existing: Any,
    incoming: Any,
    *,
    prefer: str,
    label: str,
    conflicts: list[str],
) -> Any:
    if stable_json(existing) == stable_json(incoming):
        return existing
    if prefer == "first":
        return existing
    if prefer == "last":
        return copy.deepcopy(incoming)
    conflicts.append(label)
    return existing


def compose_item_list(
    existing_items: list[dict[str, Any]],
    incoming_items: list[dict[str, Any]],
    *,
    key_name: str,
    key_func: Any,
    prefer: str,
    conflicts: list[str],
) -> list[dict[str, Any]]:
    result = copy.deepcopy(existing_items)
    positions = {key_func(item): index for index, item in enumerate(result)}

    for item in incoming_items:
        key = key_func(item)
        if key not in positions:
            positions[key] = len(result)
            result.append(copy.deepcopy(item))
            continue
        index = positions[key]
        result[index] = choose_item(
            result[index],
            item,
            prefer=prefer,
            label=f"{key_name} conflict: {key!r}",
            conflicts=conflicts,
        )

    return result


def compose_profiles(profiles: list[dict[str, Any]], *, prefer: str = "error") -> dict[str, Any]:
    if prefer not in {"error", "first", "last"}:
        raise ValueError("prefer must be one of: error, first, last")
    if not profiles:
        raise ValueError("at least one profile is required")

    result = copy.deepcopy(profiles[0])
    conflicts: list[str] = []

    for profile in profiles[1:]:
        if profile.get("schema") != result.get("schema"):
            conflicts.append("schema conflict")

        for key in ("vehicle", "applicability"):
            result[key] = choose_item(
                result.get(key),
                profile.get(key),
                prefer=prefer,
                label=f"{key} conflict",
                conflicts=conflicts,
            )

        result["buses"] = compose_item_list(
            result.get("buses") or [],
            profile.get("buses") or [],
            key_name="bus",
            key_func=lambda item: item.get("id"),
            prefer=prefer,
            conflicts=conflicts,
        )
        result["can_frames"] = compose_item_list(
            result.get("can_frames") or [],
            profile.get("can_frames") or [],
            key_name="can_frame",
            key_func=lambda item: (item.get("bus"), item.get("frame_id")),
            prefer=prefer,
            conflicts=conflicts,
        )
        result["diagnostic_queries"] = compose_item_list(
            result.get("diagnostic_queries") or [],
            profile.get("diagnostic_queries") or [],
            key_name="diagnostic_query",
            key_func=lambda item: item.get("id"),
            prefer=prefer,
            conflicts=conflicts,
        )
        result["signals"] = compose_item_list(
            result.get("signals") or [],
            profile.get("signals") or [],
            key_name="signal",
            key_func=signal_compose_key,
            prefer=prefer,
            conflicts=conflicts,
        )

    if conflicts and prefer == "error":
        details = "\n".join(f"- {conflict}" for conflict in conflicts[:20])
        extra = "" if len(conflicts) <= 20 else f"\n- ... {len(conflicts) - 20} more"
        raise ValueError(f"cannot compose profiles with conflicts:\n{details}{extra}")

    return result


def load_profiles(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text())
    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        return [data]
    raise SystemExit(f"{path}: expected profile object or list")


def print_validation_report(label: str, report: dict[str, Any]) -> None:
    counts = report["counts"]
    status = "valid" if not report["errors"] else "invalid"
    print(
        f"{label}: {status} "
        f"frames={counts['can_frames']} "
        f"queries={counts['diagnostic_queries']} "
        f"signals={counts['signals']} "
        f"canonical={counts['canonical_signals']} "
        f"errors={len(report['errors'])} "
        f"warnings={len(report['warnings'])}"
    )
    for error in report["errors"]:
        print(f"  ERROR: {error}")
    for warning in report["warnings"]:
        print(f"  WARN: {warning}")
