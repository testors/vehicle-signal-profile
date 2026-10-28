#!/usr/bin/env python3
"""Generate the TUI test fixtures (deterministic; no randomness).

    python3 tests/fixtures/tui/gen_fixtures.py            # (re)write the fixtures next to this file
    python3 tests/fixtures/tui/gen_fixtures.py --perf OUT  # 2,000 fr/s stream for the perf check

Files:
  test_profile.vsp            12 CAN signals + 1 diagnostic (OBD 01 2F); all 9 gauges by
                              canonical id, a 7-gear value table, and a throttle duplicate
                              (0x0DA) that never transmits
  partial_profile.vsp         RPM, steering and coolant only (gauges with missing slots)
  stream_direct.frames.tsv    20 s, 0x0A5/0x0F3 at 100 Hz, ... -> classifier DIRECT;
                              0x2C4 stops after 3 s (stale path)
  stream_gateway.frames.tsv   20 s, every ID <= 5 Hz -> classifier GATEWAY
  stream_temps.frames.tsv     12 s: temperatures cold for 6 s, then hot
  stream_steer.frames.tsv     12 s: steering 240° left (over-range), then 470° (full lock)
  profiles/                   3 small profiles (profile picker)
  discover_session/           synthetic phase dumps + manifest (discover overlay / replay)
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
T0 = 1700000000000


def row(ts, can_id, data):
    ext = 1 if can_id > 0x7FF else 0
    return "%d\t0x%X\t%d\t%d\t%s\n" % (ts, can_id, ext, len(data), "".join("%02X" % b for b in data))


def write_tsv(path, rows):
    rows.sort(key=lambda r: (r[0], r[1]))
    with open(path, "w") as f:
        f.write("timestamp_ms\tcan_id\tis_extended\tdlc\tdata_hex\n")
        for ts, can_id, data in rows:
            f.write(row(ts, can_id, data))


def le16(v):
    v &= 0xFFFF
    return [v & 0xFF, v >> 8]


def be16(v):
    v &= 0xFFFF
    return [v >> 8, v & 0xFF]


def signal(name, frame_id, start, bits, order, factor=1.0, offset=0.0, unit="", signed=False,
           canonical=None, values=None):
    s = {
        "name": name,
        "acquisition": {"type": "can_frame", "bus": "can0", "frame_id": frame_id,
                        "frame_id_hex": "0x%03X" % frame_id},
        "decode": {"start_bit": start, "bit_length": bits, "byte_order": order, "signed": signed,
                   "factor": factor, "offset": offset, "unit": unit, "mux": None},
    }
    if values:
        s["decode"]["values"] = values
    if canonical:
        s["canonical"] = {"id": canonical}
    return s


def test_profile():
    frames = [
        (0x0F3, "ENGINE", 8, 10), (0x0A5, "STEERING", 8, 10), (0x1A0, "BRAKE_WHEEL", 8, 20),
        (0x0D9, "PEDALS", 6, 50), (0x1E0, "TRANS", 8, 50), (0x3B0, "COOLANT", 8, 1000),
        (0x2C4, "LAP", 8, 20), (0x3B2, "GEARBOX", 8, 1000), (0x0DA, "PEDALS_ECU", 8, 20),
    ]
    can_frames = [{"bus": "can0", "frame_id": fid, "frame_id_hex": "0x%03X" % fid, "id_format": "standard",
                   "name": name, "dlc": dlc, "cycle_ms": cyc, "signals": []} for fid, name, dlc, cyc in frames]
    # canonical numeric convention: R -1, N 0, forward gears 1.., P / D null
    gear_values = [{"raw": 0, "label": "P", "value": None}, {"raw": 1, "label": "R", "value": -1},
                   {"raw": 2, "label": "N", "value": 0}, {"raw": 3, "label": "D", "value": None}]
    gear_values += [{"raw": 3 + g, "label": str(g), "value": g} for g in range(1, 8)]
    signals = [
        signal("Engine_RPM", 0x0F3, 39, 16, "big_endian", 0.25, 0, "rpm", canonical="engine.speed"),
        signal("Steering_Angle", 0x0A5, 16, 16, "little_endian", 0.1, 0, "deg", True,
               canonical="steering.wheel.angle"),
        # a duplicate throttle candidate earlier in profile order that never transmits
        signal("Throttle_ECU", 0x0DA, 16, 12, "little_endian", 0.025, 0, "%", canonical="throttle.position"),
        signal("Throttle_Pos", 0x0D9, 16, 12, "little_endian", 0.025, 0, "%", canonical="throttle.position"),
        signal("Brake_Pressure", 0x1A0, 0, 16, "little_endian", 0.01, 0, "bar", canonical="brake.pressure"),
        signal("Wheel_Speed_FL", 0x1A0, 16, 16, "little_endian", 0.01, 0, "km/h",
               canonical="wheel.speed.front_left"),
        signal("Vehicle_Speed", 0x0D9, 32, 16, "little_endian", 0.01, 0, "km/h", canonical="vehicle.speed"),
        signal("Gear", 0x1E0, 0, 4, "little_endian", 1, 0, "", canonical="transmission.gear.selected",
               values=gear_values),
        signal("Coolant_Temp", 0x3B0, 0, 8, "little_endian", 1, -40, "°C", canonical="engine.coolant.temperature"),
        signal("Oil_Temp", 0x3B0, 8, 8, "little_endian", 1, -40, "°C", canonical="engine.oil.temperature"),
        signal("Gearbox_Oil_Temp", 0x3B2, 0, 8, "little_endian", 1, -40, "°C",
               canonical="transmission.oil.temperature"),
        signal("Lap_Trigger", 0x2C4, 0, 1, "little_endian", 1, 0, ""),
    ]
    signals.append({
        "name": "Fuel_Level",
        "acquisition": {"type": "diagnostic_query", "bus": "can0", "query_id": "diag_7DF_012F",
                        "tx_id": 0x7DF, "rx_id": 0x7E8, "request_hex": "01 2F"},
        "decode": {"start_bit": 31, "bit_length": 8, "byte_order": "big_endian", "signed": False,
                   "factor": 0.392156862745098, "offset": 0, "unit": "%", "mux": None},
        "canonical": {"id": "fuel.level"},
    })
    return {
        "schema": "vehicle_data_profile.v1",
        "vehicle": {"manufacturer": "Test", "model": "Bench Car"},
        "applicability": {"generation": "T1", "platform": None, "year_start": 2020, "year_end": None,
                          "market": None, "trim": None, "powertrain": None},
        "buses": [{"id": "can0", "type": "can", "bitrate": 500000}],
        "can_frames": can_frames,
        "diagnostic_queries": [{"id": "diag_7DF_012F", "bus": "can0", "kind": "obd_mode_01",
                                "role": "channel_query", "tx_id": 0x7DF, "rx_id": 0x7E8,
                                "request_hex": "01 2F", "payload_hex": "02 01 2F 55 55 55 55 55"}],
        "signals": signals,
    }


def direct_stream(seconds=20):
    rows = []
    for ms in range(0, seconds * 1000, 10):          # 100 Hz
        t = ms / 1000.0
        rpm = int((3000 + 2500 * math.sin(t * 0.8) ** 2) * 4)
        rows.append((T0 + ms, 0x0F3, [ms // 10 & 0xFF, 0, 0, 0] + be16(rpm) + [0x12, 0x34]))
        steer = int(1800 * math.sin(t * 1.3))
        rows.append((T0 + ms + 1, 0x0A5, [(ms // 10) & 0x0F, 0] + le16(steer) + [0, 0, 0x7A, 0x03]))
    for ms in range(0, seconds * 1000, 20):          # 50 Hz
        t = ms / 1000.0
        brake = int(max(0.0, math.sin(t * 0.5)) * 9000)
        wheel = int(14260 + 300 * math.sin(t * 0.2))
        rows.append((T0 + ms + 2, 0x1A0, le16(brake) + le16(wheel) + le16(wheel + 5) + le16(wheel - 3)))
        if ms < 3000:
            rows.append((T0 + ms + 3, 0x2C4, [1 if (ms // 500) % 2 else 0, 0, 0, 0, 0, 0, 0, 0]))
    for ms in range(0, seconds * 1000, 50):          # 20 Hz
        t = ms / 1000.0
        thr = int(abs(math.sin(t * 0.9)) * 3600)
        speed = int((120 + 40 * math.sin(t * 0.3)) * 100)
        rows.append((T0 + ms + 4, 0x0D9, [0, 0x40] + le16(thr) + le16(speed)))
        gear = [3, 4, 5, 6, 5, 4][(ms // 3000) % 6]      # D, 1, 2, 3, 2, 1
        rows.append((T0 + ms + 5, 0x1E0, [gear, 0, 0, 0, 0, 0, 0, 0]))
    for ms in range(0, seconds * 1000, 100):         # 10 Hz, not in the profile, constant
        rows.append((T0 + ms + 6, 0x316, [0x0F] * 8))
    for ms in range(0, seconds * 1000, 200):         # 5 Hz, not in the profile, counter
        rows.append((T0 + ms + 7, 0x4F0, [(ms // 200) & 0xFF, 0xAA]))
    for ms in range(0, seconds * 1000, 500):         # 2 Hz extended, not in the profile
        rows.append((T0 + ms + 8, 0x18DAF110, [0x03, 0x22, 0xF1, (ms // 500) & 0xFF]))
    for ms in range(0, seconds * 1000, 1000):        # 1 Hz
        rows.append((T0 + ms + 9, 0x3B0, [92 + 40 + (ms // 5000), 104 + 40, 0, 0, 0, 0, 0, 0]))
        rows.append((T0 + ms + 10, 0x3B2, [48 + 40 + (1 if ms >= 10000 else 0), 0, 0, 0, 0, 0, 0, 0]))
    return rows


def temps_stream(seconds=12):
    """Cold for the first 6 s (coolant 45, oil 55, gearbox 30 °C), then hot (116, 136, 126)."""
    rows = []
    for ms in range(0, seconds * 1000, 10):          # rpm at 100 Hz so the dashboard is alive
        rows.append((T0 + ms, 0x0F3, [0, 0, 0, 0] + be16(int((900 + 50 * math.sin(ms / 700.0)) * 4)) + [0, 0]))
    for ms in range(0, seconds * 1000, 1000):        # 1 Hz
        hot = ms >= 6000
        rows.append((T0 + ms + 9, 0x3B0, [(116 if hot else 45) + 40, (136 if hot else 55) + 40, 0, 0, 0, 0, 0, 0]))
        rows.append((T0 + ms + 10, 0x3B2, [(126 if hot else 30) + 40, 0, 0, 0, 0, 0, 0, 0]))
    return rows


def steer_stream(seconds=12):
    """Steering (positive = left): ramp to 240° and hold until 6 s, ramp to 470° and hold."""
    rows = []
    for ms in range(0, seconds * 1000, 10):          # 100 Hz
        t = ms / 1000.0
        if t < 2:
            deg = 120 * t
        elif t < 6:
            deg = 240
        elif t < 8:
            deg = 240 + (t - 6) * 115
        else:
            deg = 470
        rows.append((T0 + ms, 0x0A5, [(ms // 10) & 0x0F, 0] + le16(int(round(deg * 10))) + [0, 0, 0, 0]))
        rows.append((T0 + ms + 1, 0x0F3, [0, 0, 0, 0] + be16(1500 * 4) + [0, 0]))
    return rows


def partial_profile():
    p = test_profile()
    keep = {"Engine_RPM", "Steering_Angle", "Coolant_Temp"}
    p["signals"] = [s for s in p["signals"] if s["name"] in keep]
    p["vehicle"] = {"manufacturer": "Test", "model": "Partial"}
    p["diagnostic_queries"] = []
    return p


def gateway_stream(seconds=20):
    rows = []
    for ms in range(0, seconds * 1000, 200):         # 5 Hz
        t = ms / 1000.0
        rpm = int((2200 + 400 * math.sin(t * 0.5)) * 4)
        rows.append((T0 + ms, 0x0F3, [0, 0, 0, 0] + be16(rpm) + [0, 0]))
    for ms in range(0, seconds * 1000, 500):         # 2 Hz
        rows.append((T0 + ms + 1, 0x0A5, [0, 0] + le16(int(50 * math.sin(ms / 1000.0))) + [0, 0, 0, 0]))
    for ms in range(0, seconds * 1000, 1000):        # 1 Hz
        rows.append((T0 + ms + 2, 0x1A0, le16(0) + le16(0) + le16(0) + le16(0)))
        rows.append((T0 + ms + 3, 0x3B0, [130, 0, 0, 0, 0, 0, 0, 0]))
    return rows


def small_profile(manufacturer, model, generation, years, frame_id, name, diag=False):
    p = {
        "schema": "vehicle_data_profile.v1",
        "vehicle": {"manufacturer": manufacturer, "model": model},
        "applicability": {"generation": generation, "platform": None, "year_start": years[0],
                          "year_end": years[1], "market": None, "trim": None, "powertrain": None},
        "buses": [{"id": "can0", "type": "can", "bitrate": 500000}],
        "can_frames": [{"bus": "can0", "frame_id": frame_id, "frame_id_hex": "0x%03X" % frame_id,
                        "id_format": "standard", "name": "F", "dlc": 8, "signals": []}],
        "diagnostic_queries": [],
        "signals": [signal(name, frame_id, 39, 16, "big_endian", 0.25, 0, "rpm", canonical="engine.speed")],
    }
    if diag:
        p["signals"].append({
            "name": "RPM_OBD",
            "acquisition": {"type": "diagnostic_query", "bus": "can0", "query_id": "q", "tx_id": 0x7DF,
                            "rx_id": 0x7E8, "request_hex": "01 0C"},
            "decode": {"start_bit": 31, "bit_length": 16, "byte_order": "big_endian", "signed": False,
                       "factor": 0.25, "offset": 0, "unit": "rpm", "mux": None},
            "canonical": {"id": "engine.speed"},
        })
    return p


def discover_session(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    plan = [(0, "baseline", 5), (1, "steering", 8), (2, "throttle", 8), (3, "brake", 8)]
    t = T0 + 600000
    phases = []
    for idx, slug, seconds in plan:
        rows = []
        for ms in range(0, seconds * 1000, 20):      # 50 Hz
            tt = ms / 1000.0
            steer = int(4000 * math.sin(tt * 2.0)) if slug == "steering" else (ms // 20) % 3 - 1
            rows.append((t + ms, 0x0A5, [(ms // 20) & 0x0F, 0] + le16(steer) + [0, 0, 0, 0]))
            rpm = 3000 + (int(2500 * abs(math.sin(tt))) if slug == "throttle" else 0) + (ms // 20) % 5
            rows.append((t + ms + 1, 0x0F3, [0, 0, 0, 0] + be16(rpm * 4) + [0, 0]))
        for ms in range(0, seconds * 1000, 50):      # 20 Hz
            tt = ms / 1000.0
            thr = int(4000 * abs(math.sin(tt))) if slug == "throttle" else 0
            rows.append((t + ms + 2, 0x0D9, [0, 0] + le16(thr & 0x0FFF) + [0, 0]))
            brk = int(30000 * abs(math.sin(tt))) if slug == "brake" else 0
            rows.append((t + ms + 3, 0x1A0, le16(brk) + [0] * 6))
        for ms in range(0, seconds * 1000, 200):     # 5 Hz
            rows.append((t + ms + 4, 0x316, [0x0F] * 8))
        fn = "%02d_%s.frames.tsv" % (idx, slug)
        write_tsv(os.path.join(out_dir, fn), rows)
        phases.append({"index": idx, "slug": slug, "seconds": seconds, "frames": len(rows), "file": fn})
        t += seconds * 1000 + 15000
    names = ["Baseline", "Steering", "Throttle", "Brake", "Gear", "Wheel Speed"]
    slugs = ["baseline", "steering", "throttle", "brake", "gear", "wheel_speed"]
    planned = [5, 8, 8, 8, 10, 15]
    manifest = {"format": "can_discover_dump/v1", "status": "completed", "device_prefix": None,
                "connected_device": None, "monitor_cmd": "ATMA", "bus_mode": "direct", "phases": []}
    for i in range(6):
        p = next((x for x in phases if x["index"] == i), None)
        manifest["phases"].append({
            "index": i, "name": names[i], "slug": slugs[i], "planned_duration_s": planned[i],
            "skipped": p is None, "captured": p is not None,
            "actual_duration_s": float(p["seconds"]) if p else 0.0,
            "analyzed_duration_s": float(p["seconds"]) if p else 0.0,
            "frame_count": p["frames"] if p else 0, "id_count": 5 if p else 0, "dropped_frames": 0,
            "capture_dropped_frames": 0, "dump_file": p["file"] if p else None})
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")


def perf_stream(path, seconds=30, fps=2000):
    rows = []
    ids = list(range(0x100, 0x100 + 40))
    per_id = fps // len(ids)                          # 50 Hz each
    step = 1000 // per_id
    for k, can_id in enumerate(ids):
        for ms in range(0, seconds * 1000, step):
            rows.append((T0 + ms + (k % step), can_id,
                         [(ms // step + k) & 0xFF, k, 0, 0, (ms >> 8) & 0xFF, ms & 0xFF, 0, 0]))
    write_tsv(path, rows)


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--perf":
        perf_stream(sys.argv[2])
        return
    with open(os.path.join(HERE, "test_profile.vsp"), "w") as f:
        json.dump(test_profile(), f, indent=2, ensure_ascii=False)
        f.write("\n")
    with open(os.path.join(HERE, "partial_profile.vsp"), "w") as f:
        json.dump(partial_profile(), f, indent=2, ensure_ascii=False)
        f.write("\n")
    write_tsv(os.path.join(HERE, "stream_direct.frames.tsv"), direct_stream())
    write_tsv(os.path.join(HERE, "stream_gateway.frames.tsv"), gateway_stream())
    write_tsv(os.path.join(HERE, "stream_temps.frames.tsv"), temps_stream())
    write_tsv(os.path.join(HERE, "stream_steer.frames.tsv"), steer_stream())
    pdir = os.path.join(HERE, "profiles")
    os.makedirs(pdir, exist_ok=True)
    for fname, args in [
        ("test__bench_car__t1__can.vsp", ("Test", "Bench Car", "T1", (2020, None), 0x0F3, "Engine_RPM")),
        ("test__bench_car__t1__obd.vsp", ("Test", "Bench Car", "T1", (2020, None), 0x0F3, "Engine_RPM", True)),
        ("other__roadster__r2__2009_2014.vsp", ("Other", "Roadster", "R2", (2009, 2014), 0x1F0, "RPM")),
    ]:
        with open(os.path.join(pdir, fname), "w") as f:
            json.dump(small_profile(*args), f, indent=2)
            f.write("\n")
    discover_session(os.path.join(HERE, "discover_session"))


if __name__ == "__main__":
    main()
