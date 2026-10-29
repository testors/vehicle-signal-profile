#!/usr/bin/env python3
"""Tests for the VSP validator and vsp2xc1's channel range.

    python3 tools/test_vsp_tools.py        (or: python3 -m unittest discover -s tools)

Every profile in profiles/ validates without errors; malformed input is reported,
never a crash.
"""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import vsp2xc1  # noqa: E402
from vsp_toolkit import ROOT, load_profiles, validate_profile  # noqa: E402

PROFILES = ROOT / "profiles"
# A profile with passive CAN signals, diagnostic queries and canonical ids; valid as is.
BASE = PROFILES / "porsche__911_gt3__992__can_obd.vsp"


def can_signal(profile: dict) -> tuple[dict, dict]:
    """The first passive CAN signal and its frame."""
    signal = next(s for s in profile["signals"] if s["acquisition"]["type"] == "can_frame")
    frame = next(f for f in profile["can_frames"] if f["frame_id"] == signal["acquisition"]["frame_id"])
    return signal, frame


def set_decode(profile: dict, key: str, value) -> None:
    """Change a decode field of a signal and its frame layout entry alike."""
    signal, frame = can_signal(profile)
    signal["decode"][key] = value
    for entry in frame["signals"]:
        if entry["name"] == signal["name"]:
            entry["decode"][key] = value


class ValidatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base = json.loads(BASE.read_text())

    def errors_of(self, mutate) -> list[str]:
        profile = copy.deepcopy(self.base)
        mutate(profile)
        return validate_profile(profile)["errors"]

    def assert_error(self, mutate, needle: str) -> None:
        errors = self.errors_of(mutate)
        self.assertTrue(any(needle in e for e in errors), f"no error with {needle!r} in {errors}")

    def test_base_profile_is_valid(self) -> None:
        self.assertEqual(self.errors_of(lambda p: None), [])

    def test_every_profile_is_valid(self) -> None:
        failed = {}
        for path in sorted(PROFILES.glob("*.vsp")):
            for profile in load_profiles(path):
                errors = validate_profile(profile)["errors"]
                if errors:
                    failed[path.name] = errors[:3]
        self.assertEqual(failed, {})

    # malformed types: an error, not an exception
    def test_profile_not_an_object(self) -> None:
        self.assertEqual(validate_profile([])["errors"], ["a profile must be a JSON object"])

    def test_decode_not_an_object(self) -> None:
        self.assert_error(lambda p: can_signal(p)[0].update(decode="x"), "decode must be an object")

    def test_acquisition_not_an_object(self) -> None:
        self.assert_error(lambda p: p["signals"][0].update(acquisition="x"), "acquisition must be an object")

    def test_buses_not_a_list(self) -> None:
        self.assert_error(lambda p: p.update(buses="can0"), "buses must be a list")

    def test_frame_id_not_an_integer(self) -> None:
        self.assert_error(lambda p: p["can_frames"][0].update(frame_id="abc"), "frame_id must be an integer CAN id")

    def test_dlc_not_an_integer(self) -> None:
        self.assert_error(lambda p: p["can_frames"][0].update(dlc="abc"), "dlc must be a positive integer")

    def test_dlc_zero(self) -> None:
        self.assert_error(lambda p: p["can_frames"][0].update(dlc=0), "dlc must be a positive integer")

    def test_frame_id_beyond_29_bits(self) -> None:
        def mutate(p):
            signal, frame = can_signal(p)
            frame["frame_id"] = signal["acquisition"]["frame_id"] = 0x2FFFFFFF
        self.assert_error(mutate, "frame_id must be an integer CAN id")

    # decode values
    def test_byte_order(self) -> None:
        self.assert_error(lambda p: set_decode(p, "byte_order", "middle"), "byte_order must be")

    def test_signed_is_a_bool(self) -> None:
        # "false" is truthy in Python and NO in Objective-C: decoders would disagree
        self.assert_error(lambda p: set_decode(p, "signed", "false"), "signed must be true or false")

    def test_bit_length_range(self) -> None:
        self.assert_error(lambda p: set_decode(p, "bit_length", 0), "bit_length must be an integer 1..64")

    def test_factor_finite(self) -> None:
        self.assert_error(lambda p: set_decode(p, "factor", float("nan")), "factor must be a finite number")

    def test_value_table_value_finite(self) -> None:
        values = [{"raw": 1, "label": "a", "value": float("inf")}]
        self.assert_error(lambda p: set_decode(p, "values", values), "finite number or null")

    def test_values_and_polynomial(self) -> None:
        def mutate(p):
            set_decode(p, "values", [{"raw": 1, "label": "a"}])
            set_decode(p, "polynomial", [0, 1, 0.5])
            set_decode(p, "factor", 1)
            set_decode(p, "offset", 0)
        self.assert_error(mutate, "values and polynomial do not go together")

    def test_minimum_above_maximum_warns(self) -> None:
        def mutate(p):
            set_decode(p, "minimum", 10)
            set_decode(p, "maximum", 0)
        profile = copy.deepcopy(self.base)
        mutate(profile)
        report = validate_profile(profile)
        self.assertEqual(report["errors"], [])
        self.assertTrue(any("is above maximum" in w for w in report["warnings"]))

    # ids written twice, queries, canonical
    def test_frame_id_hex_mismatch(self) -> None:
        self.assert_error(lambda p: p["can_frames"][0].update(frame_id_hex="0x123"), "frame_id_hex")

    def test_request_hex_bytes(self) -> None:
        self.assert_error(lambda p: p["diagnostic_queries"][0].update(request_hex="01 0C ZZ"),
                          "request_hex must be hex bytes")

    def test_query_kind(self) -> None:
        self.assert_error(lambda p: p["diagnostic_queries"][0].update(kind="obd_mode_1"), "kind must be one of")

    def test_canonical_confidence(self) -> None:
        self.assert_error(lambda p: can_signal(p)[0].setdefault("canonical", {"id": "engine.speed"})
                          .update(confidence="mapedd"), "canonical.confidence must be one of")

    def test_canonical_preferred_unit(self) -> None:
        self.assert_error(lambda p: can_signal(p)[0].setdefault("canonical", {"id": "engine.speed"})
                          .update(preferred_unit="rmp"), "canonical.preferred_unit")


class ChannelRangeTest(unittest.TestCase):
    def channel(self, minimum: float, maximum: float) -> vsp2xc1.Channel:
        return vsp2xc1.Channel(name="x", can_id=1, start_bit=0, bit_length=32, byte_order=1,
                               raw_format=vsp2xc1.RAWFMT_FLOAT, factor=1.0, offset=0.0,
                               minimum=minimum, maximum=maximum, unit="")

    def test_range_kept(self) -> None:
        self.assertEqual(vsp2xc1.channel_range(self.channel(-40.0, 150.0)), (-40.0, 150.0))

    def test_dbc_zero_zero_is_no_range(self) -> None:
        self.assertEqual(vsp2xc1.channel_range(self.channel(0.0, 0.0)), vsp2xc1.NO_RANGE)

    def test_minimum_above_maximum_is_no_range(self) -> None:
        self.assertEqual(vsp2xc1.channel_range(self.channel(1.0, 0.0)), vsp2xc1.NO_RANGE)


if __name__ == "__main__":
    unittest.main()
