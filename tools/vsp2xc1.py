#!/usr/bin/env python3
"""
Build a Race Studio 3 custom CAN driver (.xc1) straight from a VSP profile.

Only passive CAN signals (signals[] with acquisition.type == "can_frame") are written;
a custom CAN driver cannot poll diagnostic queries. Each signal becomes one AiM channel:

- layout: CANP sBit/bLen/CFlg (Intel start, or the LSB for Motorola, as Race Studio's
  own DBC import stores it);
- sign: RawFmt 1/0 signed, 4/3 unsigned (> / <= 16 bits), 25 sign-magnitude when
  decode.sign_bit is the bit right above the magnitude's MSB, 6 for float32/float64;
  a sign bit anywhere else goes out as its own 1-bit <name>_Sign channel (1 = negative);
- mux: CFlg bit 1 with mpVal/mpSBit/mpBLen. AiM reads the mux in its signal's byte order,
  so a mux in the other order is rewritten over the same bits with its value's bits moved;
- scale: SENS ConvFunc polynomial (c0, c1[, c2 ...]) and the same terms in CANUserConv
  MapPunti; value tables as CANUserConv MapCodici (codeVal in output units);
- ID mask: GConfParam CANIDMask (one per driver) with CIDMasked = CID & mask;
- AiM sensor: unit, physical quantity, object and FID as AiM's own drivers write them
  (engine RPM, vehicle and wheel speeds, steering, water/oil temperature ...); a unit AiM
  keeps as a display of a base unit (kPa, psi, mbar of bar; F of C; mph; km ...) gets that
  conversion. --aim-sensors off keeps Race Studio's DBC import form ("#" numbers);
- frequency: a frame with can_frames[].cycle_ms sets its channels' MaxFreqIus to the
  fastest AiM step that is not faster than the frame.

Anything AiM cannot hold (a mux whose bits are not one field in the signal's order, frames
needing different ID masks) is reported and the signal is left out. CustECUNote marks the
file as vsp2xc1 output and lists the split sign channels, so a reader can take the names,
scaled code values and RawFmt signedness as written. Race Studio "silent" (listen-only)
defaults to on and the logger's 120 ohm terminator to off: a VSP profile is usually read
from a tap on a bus that is already terminated.

The output starts from a Race Studio 3 DBC-import driver used as a template: by default
tools/aim/rs3_can_template.xml (one placeholder channel), or --template with an XC1 (or its
XML) of your own. The XC1 written is the XML encrypted with Race Studio's no-password key
(aim_xc1.py).
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from aim_xc1 import (
    DbcSignal, dbc_info_text, display_decimals, find_first, fmt_number, load_template, motorola_lsb_start,
    rewrite_i_attributes, safe_stem, set_prop, update_header, write_index_fragments, write_xc1, xml_bytes,
)

RAWFMT_SIGNED_16, RAWFMT_SIGNED_32 = "1", "0"
RAWFMT_UNSIGNED_16, RAWFMT_UNSIGNED_32 = "4", "3"
RAWFMT_SIGN_MAGNITUDE = "25"
RAWFMT_FLOAT = "6"
CFLG_INTEL, CFLG_MUX = 0x1, 0x2
NO_MUX = 4294967295
GENERATOR_NOTE = "vsp2xc1:"
# A Race Studio 3 DBC-import driver cut down to one placeholder channel, the structural template.
DEFAULT_TEMPLATE = Path(__file__).resolve().parent / "aim" / "rs3_can_template.xml"


@dataclass
class Channel:
    name: str
    can_id: int
    start_bit: int          # DBC numbering of the AiM field (sign bit included for RawFmt 25)
    bit_length: int
    byte_order: int         # 1 Intel, 0 Motorola
    raw_format: str
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    polynomial: tuple[float, ...] = ()
    codes: list[tuple[float, str]] = field(default_factory=list)
    mux: tuple[int, int, int] | None = None   # (value, mpSBit, mpBLen) in the channel's numbering
    split_sign_bit: int | None = None         # a sign bit AiM cannot join to the field: own channel
    canonical: str = ""                       # the VSP canonical id, for the AiM sensor identity
    cycle_ms: float | None = None             # the frame's period (can_frames[].cycle_ms)


def more_significant_bit(bit: int, byte_order: int) -> int:
    if byte_order == 1:
        return bit + 1
    return bit - 15 if bit % 8 == 7 else bit + 1


def field_msb(start_bit: int, bit_length: int, byte_order: int) -> int:
    return start_bit + bit_length - 1 if byte_order == 1 else start_bit


def field_bits(start_bit: int, bit_length: int, byte_order: int) -> set[int]:
    if byte_order == 1:
        return set(range(start_bit, start_bit + bit_length))
    bits, bit = set(), start_bit
    for _ in range(bit_length):
        bits.add(bit)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return bits


def canp_start(start_bit: int, bit_length: int, byte_order: int) -> int:
    """CANP sBit: the Intel start bit, or the LSB of a Motorola field."""
    return start_bit if byte_order == 1 else motorola_lsb_start(start_bit, bit_length)


def resolve_mux(decode: dict[str, Any], frame_signals: list[dict[str, Any]]) -> dict[str, Any] | None:
    """A full mux object: {value, start_bit, bit_length, byte_order}, resolving the DBC-style
    {"value": n} form through the frame's multiplexor signal."""
    mux = decode.get("mux")
    if not mux:
        return None
    if mux.get("role") == "multiplexor":
        return None
    if "start_bit" in mux:
        return {**mux, "byte_order": mux.get("byte_order") or "little_endian"}
    for item in frame_signals:
        item_decode = item.get("decode") or {}
        if (item_decode.get("mux") or {}).get("role") == "multiplexor":
            return {"value": mux["value"], "start_bit": item_decode["start_bit"], "bit_length": item_decode["bit_length"],
                    "byte_order": item_decode["byte_order"]}
    return None


def field_bits_lsb_first(start_bit: int, bit_length: int, byte_order: int) -> list[int]:
    """The field's bit positions from its LSB up (DBC numbering; a BE start is the MSB)."""
    if byte_order == 1:
        return list(range(start_bit, start_bit + bit_length))
    bits, bit = [], start_bit
    for _ in range(bit_length):
        bits.append(bit)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return bits[::-1]


def mux_in_channel_numbering(mux: dict[str, Any], byte_order: int) -> tuple[int, int, int] | None:
    """(mpVal, mpSBit, mpBLen). AiM reads the mux in its signal's byte order. A mux in the other
    order is rewritten as the field over the same bits in the signal's order, with its value's
    bits moved to match (a big-endian 16-bit 0x0200 over bytes 0-1 is little-endian 0x0002);
    None when those bits are not one field in the signal's order."""
    mux_order = 1 if mux.get("byte_order") == "little_endian" else 0
    start, length, value = int(mux["start_bit"]), int(mux["bit_length"]), int(mux["value"])
    if mux_order != byte_order:
        source = field_bits_lsb_first(start, length, mux_order)
        bit_value = {bit: (value >> weight) & 1 for weight, bit in enumerate(source)}
        candidates = [min(source)] if byte_order == 1 else list(source)
        for candidate in candidates:
            target = field_bits_lsb_first(candidate, length, byte_order)
            if set(target) == set(source):
                start = candidate
                value = sum(bit_value[bit] << weight for weight, bit in enumerate(target))
                break
        else:
            return None
    return value, canp_start(start, length, byte_order), length


def channel_from_signal(signal: dict[str, Any], frame: dict[str, Any] | None) -> tuple[Channel | None, str | None]:
    decode = signal.get("decode") or {}
    acquisition = signal.get("acquisition") or {}
    name = str(signal.get("name") or "")
    if (decode.get("mux") or {}).get("role") == "multiplexor":
        pass  # the multiplexor itself is a plain field
    byte_order = 1 if decode.get("byte_order") == "little_endian" else 0
    start, length = int(decode["start_bit"]), int(decode["bit_length"])
    value_type = decode.get("value_type") or "integer"
    sign_bit = decode.get("sign_bit")
    split_sign = None
    if value_type != "integer":
        if length not in (32, 64):
            return None, f"{name}: {value_type} needs 32 or 64 bits"
        raw_format = RAWFMT_FLOAT
    elif sign_bit is not None and int(sign_bit) != more_significant_bit(field_msb(start, length, byte_order), byte_order):
        # AiM sign-magnitude needs the sign right above the MSB; otherwise the magnitude and
        # the sign go out as two channels, as AiM's own files do (TEVES MK60 YAW_RATE_RAW).
        split_sign = int(sign_bit)
        raw_format = RAWFMT_UNSIGNED_32 if length > 16 else RAWFMT_UNSIGNED_16
    elif sign_bit is not None:
        raw_format = RAWFMT_SIGN_MAGNITUDE
        if byte_order == 0:
            start = int(sign_bit)
        length += 1
    elif decode.get("signed"):
        raw_format = RAWFMT_SIGNED_32 if length > 16 else RAWFMT_SIGNED_16
    else:
        raw_format = RAWFMT_UNSIGNED_32 if length > 16 else RAWFMT_UNSIGNED_16
    mux = resolve_mux(decode, (frame or {}).get("signals") or [])
    aim_mux = None
    if decode.get("mux") and (decode["mux"].get("role") != "multiplexor"):
        if mux is None:
            return None, f"{name}: mux {decode['mux']} has no multiplexor field"
        aim_mux = mux_in_channel_numbering(mux, byte_order)
        if aim_mux is None:
            return None, f"{name}: a multi-byte mux in the other byte order than the signal"
    factor, offset = float(decode.get("factor", 1.0)), float(decode.get("offset", 0.0))
    codes = []
    for entry in decode.get("values") or []:
        codes.append((float(entry["raw"]) * factor + offset, str(entry["label"])))
    return Channel(
        name=name,
        can_id=int(acquisition["frame_id"]),
        start_bit=start,
        bit_length=length,
        byte_order=byte_order,
        raw_format=raw_format,
        factor=factor,
        offset=offset,
        minimum=float(decode.get("minimum") if decode.get("minimum") is not None else -1e6),
        maximum=float(decode.get("maximum") if decode.get("maximum") is not None else 1e6),
        unit=str(decode.get("unit") or ""),
        polynomial=tuple(float(c) for c in decode.get("polynomial") or ()),
        codes=codes,
        mux=aim_mux,
        split_sign_bit=split_sign,
        canonical=str((signal.get("canonical") or {}).get("id") or ""),
        cycle_ms=(frame or {}).get("cycle_ms"),
    ), None


SIGN_CHANNEL_SUFFIX = "_Sign"

# AiM sensor identity of a custom CAN channel, as AiM's own drivers write it (SN_CUSTOM_CAN,
# UIB = In = UIV = dvOutUnit, multUsr2Base 1). FID = object bits | quantity code. Learnt from
# the AiM driver corpus (773 files, 2026-10-26): a VSP unit maps to AiM's base unit spelling ...
AIM_UNIT_ALIASES = {
    "L": "l", "L/s": "l/s", "lambda": "\u03bb", "Bar": "bar", "Km/h": "km/h", "kmh": "km/h", "RPM": "rpm", "1/min": "rpm",
    "sec": "s", "Sec": "s", "Volt": "V", "volt": "V", "Amps": "A",
}
# ... each base unit has one quantity, with the FID AiM gives it when the object is undefined
# (km/h: 0x70010, AiM's generic speed; lambda only appears with its object) ...
AIM_UNIT_SENSOR = {
    "%": ("FID_PHI_PERCENT", "FID_OBJ_UNDEFINED", 0x1),
    "%/s": ("FID_PHI_PERCENT_SEC_RATIO", "FID_OBJ_UNDEFINED", 0x2),
    "g": ("FID_PHI_ACCELERATION", "FID_OBJ_UNDEFINED", 0x3),
    "deg": ("FID_PHI_ANGLE", "FID_OBJ_UNDEFINED", 0x4),
    "deg/s": ("FID_PHI_ANGLE_SEC_RATIO", "FID_OBJ_UNDEFINED", 0x5),
    "m": ("FID_PHI_DISTANCE", "FID_OBJ_UNDEFINED", 0x8),
    "Hz": ("FID_PHI_FREQUENCY", "FID_OBJ_UNDEFINED", 0x9),
    "kg/s": ("FID_PHI_MASS_SEC_RATIO", "FID_OBJ_UNDEFINED", 0xA),
    "mm": ("FID_PHI_POSITION", "FID_OBJ_UNDEFINED", 0xC),
    "kW": ("FID_PHI_POWER", "FID_OBJ_UNDEFINED", 0xD),
    "bar": ("FID_PHI_PRESSURE", "FID_OBJ_UNDEFINED", 0xE),
    "rpm": ("FID_PHI_RPM", "FID_OBJ_UNDEFINED", 0xF),
    "km/h": ("FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x70010),
    "C": ("FID_PHI_TEMPERATURE", "FID_OBJ_UNDEFINED", 0x11),
    "ms": ("FID_PHI_TIME", "FID_OBJ_UNDEFINED", 0x12),
    "s": ("FID_PHI_TIME", "FID_OBJ_UNDEFINED", 0x12),
    "Nm": ("FID_PHI_TORQUE", "FID_OBJ_UNDEFINED", 0x13),
    "mV": ("FID_PHI_VOLTAGE", "FID_OBJ_UNDEFINED", 0x15),
    "V": ("FID_PHI_VOLTAGE", "FID_OBJ_UNDEFINED", 0x15),
    "l": ("FID_PHI_VOLUME", "FID_OBJ_UNDEFINED", 0x16),
    "l/s": ("FID_PHI_VOLUME_SEC_RATIO", "FID_OBJ_UNDEFINED", 0x18),
    "A": ("FID_PHI_CURRENT", "FID_OBJ_UNDEFINED", 0x1B),
    "rpm/s": ("FID_PHI_RPM_SEC", "FID_OBJ_UNDEFINED", 0x1D),
    "gear": ("FID_PHI_GEAR", "FID_OBJ_UNDEFINED", 0x1F),
    "krpm": ("FID_PHI_HI_RPM", "FID_OBJ_UNDEFINED", 0x26),
    "kgf": ("FID_PHI_FORCE", "FID_OBJ_UNDEFINED", 0x29),
    "\u03bb": ("FID_PHI_LAMBDA", "FID_OBJ_LAMBDA", 0x91E),
}
# ... and a canonical id AiM's drivers give one object in at least 80% of its channels (n >= 17)
# keeps that object: the dash's RPM, the speed reference, wheel speeds per corner, alarms on
# water/oil temperature. (base unit, quantity, object, FID)
AIM_CANONICAL_SENSOR = {
    "brake.pedal.position": ("%", "FID_PHI_PERCENT", "FID_OBJ_BRAKE_LOAD", 0x281),
    "brake.pressure": ("bar", "FID_PHI_PRESSURE", "FID_OBJ_BRAKE_FLUID", 0x1820E),
    "engine.coolant.temperature": ("C", "FID_PHI_TEMPERATURE", "FID_OBJ_WATER", 0x1091),
    "engine.oil.pressure": ("bar", "FID_PHI_PRESSURE", "FID_OBJ_OIL", 0xB8E),
    "engine.oil.temperature": ("C", "FID_PHI_TEMPERATURE", "FID_OBJ_OIL", 0xB91),
    "engine.speed": ("rpm", "FID_PHI_RPM", "FID_OBJ_UNDEFINED", 0xF),
    "exhaust.gas.temperature": ("C", "FID_PHI_TEMPERATURE", "FID_OBJ_AIR", 0x40091),
    "fuel.pressure": ("bar", "FID_PHI_PRESSURE", "FID_OBJ_FUEL", 0x58E),
    "fuel.remaining": ("l", "FID_PHI_VOLUME", "FID_OBJ_FUEL", 0x596),
    "steering.wheel.angle": ("deg", "FID_PHI_ANGLE", "FID_OBJ_STEERING", 0xD84),
    "transmission.gear.selected": ("gear", "FID_PHI_GEAR", "FID_OBJ_GEAR", 0x61F),
    "vehicle.acceleration.lateral": ("g", "FID_PHI_ACCELERATION", "FID_OBJ_Y", 0x1183),
    "vehicle.acceleration.longitudinal": ("g", "FID_PHI_ACCELERATION", "FID_OBJ_X", 0x1103),
    "vehicle.acceleration.vertical": ("g", "FID_PHI_ACCELERATION", "FID_OBJ_Z", 0x1203),
    "vehicle.roll_rate": ("deg/s", "FID_PHI_ANGLE_SEC_RATIO", "FID_OBJ_X", 0x1105),
    "vehicle.pitch_rate": ("deg/s", "FID_PHI_ANGLE_SEC_RATIO", "FID_OBJ_Y", 0x1185),   # 13 of 13
    "vehicle.speed": ("km/h", "FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x68010),
    "vehicle.yaw_rate": ("deg/s", "FID_PHI_ANGLE_SEC_RATIO", "FID_OBJ_Z", 0x1205),
    "wheel.speed.front_left": ("km/h", "FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x5070010),
    "wheel.speed.front_right": ("km/h", "FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x4870010),
    "wheel.speed.rear_left": ("km/h", "FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x3070010),
    "wheel.speed.rear_right": ("km/h", "FID_PHI_SPEED", "FID_OBJ_UNDEFINED", 0x2870010),
}


# A switch or flag: AiM's drivers mark state words and switches (ECU_BRAKE_SW, CLUTCH_SWITCH)
# as a number of object STATUS (FID 0x158B, 1136 channels).
AIM_STATUS_SENSOR = ("#", "FID_PHI_NUMBER", "FID_OBJ_STATUS", 0x158B)
AIM_STATUS_UNITS = {"on/off"}
# AiM base units that name a kind of value, not a scale.
AIM_KIND_UNITS = {"gear"}


# A VSP unit AiM shows through a display conversion of one of its base units, as AiM's
# drivers do (CANUserConv user value = base value * multUsr2Base + offsetUsr2Base; bar shown
# as kPa x100 in 300 channels, as psi x14.5038 in 59, C as F x1.8 +32 in 17 ...).
# VSP unit: (AiM base unit, AiM display unit, mult, offset)
AIM_DISPLAY_UNITS = {
    "kPa": ("bar", "kPa", 100.0, 0.0),
    "mbar": ("bar", "mbar", 1000.0, 0.0),
    "psi": ("bar", "psi", 14.503773773, 0.0),
    "F": ("C", "F", 1.8, 32.0),
    "mph": ("km/h", "mph", 0.621371192, 0.0),
    "km": ("m", "km", 0.001, 0.0),
    "us": ("ms", "us", 1000.0, 0.0),
    "L/h": ("l/s", "l/h", 3600.0, 0.0),
    "N": ("kgf", "N", 9.80665, 0.0),
    "m/s^2": ("g", "m/s2", 9.80665, 0.0),
    "g/s": ("kg/s", "g/s", 1000.0, 0.0),
}


@dataclass(frozen=True)
class AimSensor:
    base_unit: str       # SENS UIB/In: the unit of SENS ConvFunc
    unit: str            # UIV / CANUserConv dvOutUnit: the unit shown
    mult: float          # shown = base * mult + add
    add: float
    quantity: str
    measured: str
    fid: int


def aim_sensor(channel: Channel) -> AimSensor | None:
    """The AiM sensor identity of the channel, or None to keep the generic "#" number (a
    unit AiM has no form for: gauge pressure, miles, L/100km ...). A polynomial channel
    gets no display conversion (its curve stays in the VSP unit)."""
    if channel.unit in AIM_STATUS_UNITS:
        return AimSensor("#", "#", 1.0, 0.0, *AIM_STATUS_SENSOR[1:])
    display = AIM_DISPLAY_UNITS.get(channel.unit) if not channel.polynomial else None
    if display:
        base, shown, mult, add = display
    else:
        base = shown = AIM_UNIT_ALIASES.get(channel.unit, channel.unit)
        mult, add = 1.0, 0.0
    known = AIM_CANONICAL_SENSOR.get(channel.canonical)
    if known and base == "" and known[0] in AIM_KIND_UNITS:
        base = shown = known[0]   # a gear decode without a unit is still a gear
    if known and known[0] == base:
        quantity, measured, fid = known[1:]
    elif base in AIM_UNIT_SENSOR:
        quantity, measured, fid = AIM_UNIT_SENSOR[base]
    else:
        return None
    return AimSensor(base, shown, mult, add, quantity, measured, fid)


def set_aim_sensor(channel_el: ET.Element, sensor: AimSensor) -> None:
    sens = find_first(channel_el, "./e[@c='SENS']", "SENS")
    for name, value in (("UIB", sensor.base_unit), ("In", sensor.base_unit),
                        ("PhysicalQuantity", sensor.quantity), ("MeasuredObject", sensor.measured)):
        set_prop(sens, name, value)
    set_prop(channel_el, "UIV", sensor.unit)
    set_prop(channel_el, "FID", sensor.fid)
    user = channel_el.find("./e[@c='CANUserConv']")
    if user is not None:
        set_prop(user, "dvOutUnit", sensor.unit)


# AiM channel frequencies (MaxFreqIus, us) in its drivers: 500 Hz ... 1 Hz.
AIM_INTERVALS_US = (2000, 5000, 10000, 20000, 40000, 50000, 100000, 200000, 500000, 1000000)


def aim_interval_us(cycle_ms: float) -> int:
    """The fastest AiM channel frequency that is not faster than the frame."""
    target = float(cycle_ms) * 1000 * 0.99
    return next((interval for interval in AIM_INTERVALS_US if interval >= target), AIM_INTERVALS_US[-1])


def sign_channel(base: Channel) -> Channel:
    """The 1-bit sign of a split sign-magnitude field (1 = negative)."""
    return Channel(
        name=base.name + SIGN_CHANNEL_SUFFIX, can_id=base.can_id, start_bit=int(base.split_sign_bit), bit_length=1,
        byte_order=base.byte_order, raw_format=RAWFMT_UNSIGNED_16, factor=1.0, offset=0.0, minimum=0.0, maximum=1.0,
        unit="", mux=base.mux, cycle_ms=base.cycle_ms,
    )


ID_WIDTH_STANDARD, ID_WIDTH_EXTENDED = 0x7FF, 0x1FFFFFFF


def id_width(frame_id: int) -> int:
    return ID_WIDTH_STANDARD if frame_id <= ID_WIDTH_STANDARD else ID_WIDTH_EXTENDED


def driver_id_mask(frames: list[dict[str, Any]]) -> tuple[int, set[int]]:
    """AiM keeps one CAN ID mask per driver (GConfParam CANIDMask; each channel's CIDMasked is
    CID & mask). Returns that mask and the IDs of masked frames it cannot serve: frames with
    another mask, or every masked frame when the mask would also loosen an unmasked frame."""
    masked = [(int(frame["frame_id"]), int(frame["frame_id_mask"])) for frame in frames if frame.get("frame_id_mask") is not None]
    if not masked:
        return ID_WIDTH_EXTENDED, set()
    extended = [mask & ID_WIDTH_EXTENDED for frame_id, mask in masked if id_width(frame_id) == ID_WIDTH_EXTENDED]
    standard = [mask & ID_WIDTH_STANDARD for frame_id, mask in masked if id_width(frame_id) == ID_WIDTH_STANDARD]
    if extended:
        mask = max(set(extended), key=extended.count)
    else:
        mask = (ID_WIDTH_EXTENDED & ~ID_WIDTH_STANDARD) | max(set(standard), key=standard.count)
    unserved = {frame_id for frame_id, frame_mask in masked if frame_mask & id_width(frame_id) != mask & id_width(frame_id)}
    unmasked = [int(frame["frame_id"]) for frame in frames if frame.get("frame_id_mask") is None]
    if any(mask & id_width(frame_id) != id_width(frame_id) for frame_id in unmasked):
        return ID_WIDTH_EXTENDED, {frame_id for frame_id, _ in masked}
    return mask, unserved


def channels_from_profile(profile: dict[str, Any], *, include_names: set[str] | None = None) -> tuple[list[Channel], list[str]]:
    frames = {int(frame["frame_id"]): frame for frame in profile.get("can_frames") or []}
    _, unserved = driver_id_mask(list(frames.values()))
    channels: list[Channel] = []
    notes: list[str] = []
    skipped_diag = 0
    reassembled = 0
    for signal in profile.get("signals") or []:
        acquisition = signal.get("acquisition") or {}
        if acquisition.get("type") != "can_frame":
            skipped_diag += 1
            continue
        if include_names and signal.get("name") not in include_names:
            continue
        if (frames.get(int(acquisition["frame_id"])) or {}).get("reassembled"):
            reassembled += 1
            continue
        if int(acquisition["frame_id"]) in unserved:
            notes.append(f"{signal.get('name')}: frame 0x{int(acquisition['frame_id']):X} needs an ID mask the driver's one mask cannot give")
            continue
        channel, problem = channel_from_signal(signal, frames.get(int(acquisition["frame_id"])))
        if problem:
            notes.append(problem)
        if channel is not None:
            channels.append(channel)
            if channel.split_sign_bit is not None:
                channels.append(sign_channel(channel))
                notes.append(f"{channel.name}: sign bit {channel.split_sign_bit} is not right above the field, "
                             f"so it is its own channel {channel.name}{SIGN_CHANNEL_SUFFIX} (1 = negative)")
    if skipped_diag:
        notes.append(f"{skipped_diag} diagnostic-query signals left out (a custom CAN driver only listens)")
    if reassembled:
        notes.append(f"{reassembled} signals of reassembled vendor-stream frames left out (a custom CAN driver reads single frames)")
    channels.sort(key=lambda item: (item.can_id, item.mux[0] if item.mux else -1, item.start_bit, item.name))
    return channels, notes


def assign_copy_numbers(channels: list[Channel]) -> list[int]:
    """AiM cpyNum: one receive slot per CAN ID and mux condition, plus a fresh slot for a
    channel that reads bits an earlier channel of its slot already reads (as in AiM's files)."""
    next_slot: dict[int, int] = {}
    slots: dict[tuple[int, Any], int] = {}
    seen_bits: dict[tuple[int, Any], list[set[int]]] = {}
    numbers = []
    for channel in channels:
        key = (channel.can_id, channel.mux)
        if key not in slots:
            slots[key] = next_slot.get(channel.can_id, 0)
            next_slot[channel.can_id] = slots[key] + 1
        number = slots[key]
        bits = field_bits(channel.start_bit, channel.bit_length, channel.byte_order)
        if any(bits & earlier for earlier in seen_bits.get(key, [])):
            number = next_slot[channel.can_id]
            next_slot[channel.can_id] = number + 1
        seen_bits.setdefault(key, []).append(bits)
        numbers.append(number)
    return numbers


def write_conversion(channel_el: ET.Element, channel: Channel, index: int, aim: AimSensor | None = None) -> None:
    """SENS ConvFunc in the AiM base unit, CANUserConv MapPunti and codes in the shown unit."""
    terms = channel.polynomial or (channel.offset, channel.factor)
    mult, add = (aim.mult, aim.add) if aim else (1.0, 0.0)
    base_terms = [(terms[0] - add) / mult, *(coefficient / mult for coefficient in terms[1:])]
    sensor = find_first(channel_el, "./e[@c='SENS']", "SENS")
    conv = find_first(sensor, "./e[@c='ConvFunc']", "SENS/ConvFunc")
    for prop in list(conv.findall("./p")):
        conv.remove(prop)
    for power, coefficient in enumerate(base_terms):
        set_prop(conv, str(power), fmt_number(coefficient))
    user_conv = find_first(channel_el, "./e[@c='CANUserConv']", "CANUserConv")
    set_prop(user_conv, "multUsr2Base", fmt_number(mult))
    set_prop(user_conv, "offsetUsr2Base", fmt_number(add))
    points = find_first(user_conv, "./e[@c='MapPunti']", "CANUserConv/MapPunti")
    template_point = points.find("./e[@c='Punto']")
    for point in list(points):
        points.remove(point)
    for power, coefficient in enumerate(terms):
        point = copy.deepcopy(template_point)
        point.set("i", f"Punto.1.1.2.{index}.1.1.3.1.{power + 1}")
        set_prop(point, "CoordX", power)
        set_prop(point, "CoordY", fmt_number(coefficient))
        points.append(point)
    codes = user_conv.find("./e[@c='MapCodici']")
    if codes is None:
        codes = ET.SubElement(user_conv, "e", {"c": "MapCodici", "i": f"MapCodici.1.1.2.{index}.1.1.3.2"})
    for code in list(codes):
        codes.remove(code)
    for number, (value, label) in enumerate(channel.codes, start=1):
        code = ET.SubElement(codes, "e", {"c": "Codice", "i": f"Codice.1.1.2.{index}.1.1.3.2.{number}"})
        set_prop(code, "codeAbil", "1")
        set_prop(code, "codeLabel", label)
        set_prop(code, "codeVal", fmt_number(value))


def dbc_view(channel: Channel) -> DbcSignal:
    """DBCInfo text of the channel's value field (the magnitude for sign-magnitude)."""
    start, length = channel.start_bit, channel.bit_length
    if channel.raw_format == RAWFMT_SIGN_MAGNITUDE:
        length -= 1
        if channel.byte_order == 0:
            start = start + 15 if start % 8 == 0 else start - 1
    return DbcSignal(
        message_id=channel.can_id, message_name=f"MSG_{channel.can_id:03X}", name=channel.name, start_bit=start,
        bit_length=length, byte_order=channel.byte_order,
        signed=channel.raw_format in (RAWFMT_SIGNED_16, RAWFMT_SIGNED_32), factor=channel.factor,
        offset=channel.offset, minimum=channel.minimum, maximum=channel.maximum, unit=channel.unit, canonical=None,
    )


def fill_channel(meta: ET.Element, channel: Channel, index: int, copy_number: int, id_mask: int = ID_WIDTH_EXTENDED,
                 aim_sensors: bool = True) -> None:
    rewrite_i_attributes(meta, index)
    channel_el = find_first(meta, ".//e[@c='Canale']", "Canale")
    canp = find_first(channel_el, "./e[@c='CANP']", "CANP")
    cid = f"0x{channel.can_id:08x}"
    set_prop(channel_el, "CHID", f"CC{index:02d}")
    set_prop(channel_el, "ChSortPos", 1)  # AiM and Race Studio write 1 for every channel
    set_prop(channel_el, "IDFis", index - 1)
    set_prop(channel_el, "DBCInfo", dbc_info_text(dbc_view(channel)))
    set_prop(channel_el, "DBCMeasName", channel.name)
    set_prop(channel_el, "DBCMeasUdm", channel.unit)
    set_prop(channel_el, "IsByDBC", "1")
    set_prop(channel_el, "IsDBCValidated", "0")
    set_prop(channel_el, "LabelLongName", channel.name)
    set_prop(channel_el, "LabelShortName", f"e{index:03d}")
    set_prop(channel_el, "LongName", channel.name)
    set_prop(channel_el, "ShortName", f"e{index:03d}")
    set_prop(channel_el, "RawFmt", channel.raw_format)
    set_prop(channel_el, "NumDec", display_decimals(channel.factor) if not channel.polynomial else 1)
    set_prop(channel_el, "ToSendToFw", "1")
    if channel.raw_format == RAWFMT_FLOAT:
        set_prop(channel_el, "ChMin", fmt_number(channel.minimum))
        set_prop(channel_el, "ChMax", fmt_number(channel.maximum))
    cflag = (CFLG_INTEL if channel.byte_order == 1 else 0) | (4 if channel.byte_order == 1 else 0)
    if channel.mux:
        cflag |= CFLG_MUX
    set_prop(canp, "CFlg", cflag)
    set_prop(canp, "CID", cid)
    set_prop(canp, "CIDMasked", f"0x{channel.can_id & id_mask:08x}")
    set_prop(canp, "bLen", channel.bit_length)
    set_prop(canp, "cpyNum", copy_number)
    set_prop(canp, "mpVal", channel.mux[0] if channel.mux else NO_MUX)
    set_prop(canp, "mpSBit", channel.mux[1] if channel.mux else 0)
    set_prop(canp, "mpBLen", channel.mux[2] if channel.mux else 0)
    set_prop(canp, "sBit", canp_start(channel.start_bit, channel.bit_length, channel.byte_order))
    set_prop(canp, "tcAbil", "1")
    aim = aim_sensor(channel) if aim_sensors else None
    write_conversion(channel_el, channel, index, aim)
    if aim:
        set_aim_sensor(channel_el, aim)
    if channel.cycle_ms:
        set_prop(channel_el, "MaxFreqIus", aim_interval_us(channel.cycle_ms))


def build_xc1(
    profile: dict[str, Any], *, template_path: Path, manufacturer: str, model: str, uid: str, bitrate: int,
    bus_mode: int, silent: bool, termination: bool, source_name: str, aim_sensors: bool = True,
) -> tuple[ET.ElementTree, list[Channel], list[str]]:
    channels, notes = channels_from_profile(profile)
    id_mask, _ = driver_id_mask(profile.get("can_frames") or [])
    if not channels:
        raise SystemExit("no CAN signal can be written")
    root = load_template(template_path)
    metas = root.findall(".//e[@c='MetaCanale']")
    prototype = metas[0]
    meta_list = find_first(root, ".//e[@c='ElencoMetaCanali']", "ElencoMetaCanali")
    for child in list(meta_list):
        meta_list.remove(child)
    for index, (channel, number) in enumerate(zip(channels, assign_copy_numbers(channels)), start=1):
        meta = copy.deepcopy(prototype)
        fill_channel(meta, channel, index, number, id_mask, aim_sensors)
        meta_list.append(meta)
    update_header(root, dbc_path=Path(source_name), manufacturer=manufacturer, model=model, uid=uid,
                  bitrate=bitrate, bus_mode=bus_mode, channel_count=len(channels))
    conf = find_first(root, ".//e[@c='GConfParam']", "GConfParam")
    set_prop(conf, "R120abil", int(termination))
    set_prop(conf, "CANIDMask", f"0x{id_mask:08x}")
    # Marks the file as vsp2xc1 output: names, scaled code values and split signs as written.
    split = [channel.name for channel in channels if channel.split_sign_bit is not None]
    note = f"{GENERATOR_NOTE} {source_name}"
    if split:
        note += "\nsplit sign: " + json.dumps(split, ensure_ascii=False)
    set_prop(conf, "CustECUNote", note)
    set_prop(find_first(root, ".//e[@c='BusConfiguration']", "BusConfiguration"), "silent", int(silent))
    ET.indent(root, space="  ")
    return ET.ElementTree(root), channels, notes


def profile_bitrate(profile: dict[str, Any], default: int) -> int:
    for bus in profile.get("buses") or []:
        if bus.get("bitrate"):
            return int(bus["bitrate"])
    return default


def command_build(args: argparse.Namespace) -> int:
    vsp_path = Path(args.vsp)
    profile = json.loads(vsp_path.read_text())
    vehicle = profile.get("vehicle") or {}
    manufacturer = (args.manufacturer or vehicle.get("manufacturer") or "CUSTOM").upper()
    model = args.model or " ".join(str(v) for v in (vehicle.get("model"), (profile.get("applicability") or {}).get("generation")) if v)
    uid = safe_stem(args.uid or vsp_path.stem)
    bitrate = args.bitrate or profile_bitrate(profile, 500000)
    template = Path(args.template) if args.template else DEFAULT_TEMPLATE
    tree, channels, notes = build_xc1(
        profile, template_path=template, manufacturer=manufacturer, model=model, uid=uid, bitrate=bitrate,
        bus_mode=args.bus_mode, silent=args.silent == "on", termination=args.termination == "on", source_name=vsp_path.name,
        aim_sensors=args.aim_sensors == "on",
    )
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    payload = xml_bytes(tree)
    (out / f"{uid}.xml").write_bytes(payload)
    write_xc1(payload, out / f"{uid}.xc1")
    (out / f"{uid}.manifest.json").write_text(json.dumps({
        "vsp": str(vsp_path), "template_xc1": str(template), "manufacturer": manufacturer, "model": model, "uid": uid,
        "bitrate": bitrate, "silent": args.silent == "on", "termination": args.termination == "on",
        "channels": [asdict(channel) for channel in channels], "notes": notes,
    }, indent=2, ensure_ascii=False) + "\n")
    if args.index_fragments:
        write_index_fragments(out, manufacturer=manufacturer, model=model, uid=uid, bitrate=bitrate,
                              bus_mode=args.bus_mode, dbc_path=vsp_path)
    print(f"written: {out / (uid + '.xc1')}  channels: {len(channels)}")
    for note in notes:
        print(f"  note: {note}", file=sys.stderr)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("vsp")
    parser.add_argument("-o", "--output-dir", default="aim_xc1_out")
    parser.add_argument("--template", help="a Race Studio DBC-imported no-password XC1 or its XML "
                                           "(default: tools/aim/rs3_can_template.xml)")
    parser.add_argument("--manufacturer")
    parser.add_argument("--model")
    parser.add_argument("--uid", help="custom protocol id / output stem (default: the VSP file stem)")
    parser.add_argument("--bitrate", type=int, help="default: the profile's bus bitrate")
    parser.add_argument("--bus-mode", type=int, default=2)
    parser.add_argument("--silent", choices=("on", "off"), default="on", help="listen-only bus mode (default on)")
    parser.add_argument("--termination", choices=("on", "off"), default="off", help="logger 120 ohm terminator (default off)")
    parser.add_argument("--aim-sensors", choices=("on", "off"), default="on",
                        help="AiM unit/quantity/FID per channel as AiM's drivers do (off: Race Studio's DBC import '#')")
    parser.add_argument("--index-fragments", action="store_true", help="also write Race Studio index-file entries")
    parser.set_defaults(func=command_build)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
