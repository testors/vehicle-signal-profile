#!/usr/bin/env python3
"""Generate a passive-CAN Vehicle Signal Profile from a DBC file."""

from __future__ import annotations

import argparse

from vsp_toolkit import profile_from_dbc, profile_output_path, root_path, write_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="DBC input path")
    parser.add_argument("--manufacturer", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("-o", "--output", required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profile = profile_from_dbc(
        root_path(args.input),
        manufacturer=args.manufacturer,
        model=args.model,
        bitrate=args.bitrate,
    )
    output = profile_output_path(args.output)
    write_json(output, profile)
    print(f"written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
