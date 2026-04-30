#!/usr/bin/env python3
"""Compose multiple Vehicle Signal Profiles into one profile."""

from __future__ import annotations

import argparse

from vsp_toolkit import compose_profiles, load_profiles, print_validation_report, root_path, validate_profile, write_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="+", help="VSP input paths")
    parser.add_argument("-o", "--output", required=True, help="VSP output path")
    parser.add_argument(
        "--prefer",
        choices=("error", "first", "last"),
        default="error",
        help="conflict policy for identical buses, frames, queries, or signal acquisition keys",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profiles = []
    for value in args.input:
        profiles.extend(load_profiles(root_path(value)))

    try:
        profile = compose_profiles(profiles, prefer=args.prefer)
    except ValueError as error:
        print(error)
        return 1

    report = validate_profile(profile)
    print_validation_report("output", report)
    if report["errors"]:
        return 1

    output = root_path(args.output)
    write_json(output, profile)
    print(f"written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
