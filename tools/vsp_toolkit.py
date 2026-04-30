#!/usr/bin/env python3
"""Shared helpers for public Vehicle Signal Profile tools."""

from __future__ import annotations

import copy
import json
import re
from collections import defaultdict
from dataclasses import dataclass
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

    for line in path.read_text(errors="ignore").splitlines():
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

    return messages, signals


def dbc_signal_decode(signal: DbcSignal) -> dict[str, Any]:
    mux = None
    if signal.mux == "M":
        mux = {"role": "multiplexor"}
    elif signal.mux and signal.mux.startswith("m"):
        mux = {"value": int(signal.mux[1:])}

    return {
        "start_bit": signal.start_bit,
        "bit_length": signal.bit_length,
        "byte_order": byte_order_name(signal.byte_order),
        "signed": signal.signed,
        "factor": signal.factor,
        "offset": signal.offset,
        "minimum": signal.minimum,
        "maximum": signal.maximum,
        "unit": signal.unit,
        "mux": mux,
    }


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

    for frame in sorted(profile.get("can_frames") or [], key=lambda item: (str(item.get("bus")), int(item.get("frame_id") or 0))):
        key = (str(frame.get("bus")), int(frame.get("frame_id") or 0))
        if key not in frame_keys:
            continue
        frame_name = dbc_identifier(frame.get("name"), f"FRAME_{int(frame.get('frame_id') or 0):X}")
        lines.append(f"BO_ {dbc_frame_id(frame)} {frame_name}: {int(frame.get('dlc') or 8)} Vector__XXX")
        used_names: set[str] = set()
        for signal in frame.get("signals") or []:
            base_name = dbc_identifier(signal.get("name"), "SIGNAL")
            signal_name = base_name
            suffix = 2
            while signal_name in used_names:
                signal_name = dbc_identifier(f"{base_name}_{suffix}", "SIGNAL")
                suffix += 1
            used_names.add(signal_name)
            lines.append(dbc_signal_line(signal, signal_name))
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def normalize_hex_text(value: Any) -> str:
    return re.sub(r"[^0-9A-Fa-f]", "", str(value or "")).upper()


def canonical_number(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 12)
    return value


def canonical_decode(decode: dict[str, Any]) -> dict[str, Any]:
    keys = ("start_bit", "bit_length", "byte_order", "signed", "factor", "offset", "minimum", "maximum", "unit", "mux")
    result = {key: canonical_number(decode.get(key)) for key in keys if key in decode}
    if result.get("mux") is None:
        result["mux"] = None
    return result


def signal_matches_frame(signal: dict[str, Any], frame: dict[str, Any]) -> bool:
    expected = canonical_decode(signal.get("decode") or {})
    return any(
        item.get("name") == signal.get("name") and canonical_decode(item.get("decode") or {}) == expected
        for item in frame.get("signals", [])
    )


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
