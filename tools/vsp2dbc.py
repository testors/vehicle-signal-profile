#!/usr/bin/env python3
"""Export passive CAN frame layouts from a Vehicle Signal Profile to DBC."""

from __future__ import annotations

import argparse

from vsp_toolkit import load_profiles, profile_to_dbc, root_path, write_text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="VSP input path")
    parser.add_argument("-o", "--output", required=True, help="DBC output path")
    parser.add_argument(
        "--include-diagnostic-responses",
        action="store_true",
        help="also export diagnostic response frame layouts; DBC cannot represent the active queries",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profiles = load_profiles(root_path(args.input))
    if len(profiles) != 1:
        raise SystemExit("vsp2dbc expects one profile object")
    dbc = profile_to_dbc(profiles[0], include_diagnostic_responses=args.include_diagnostic_responses)
    output = root_path(args.output)
    write_text(output, dbc)
    print(f"written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
