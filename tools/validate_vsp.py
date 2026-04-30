#!/usr/bin/env python3
"""Validate Vehicle Signal Profile runtime consistency."""

from __future__ import annotations

import argparse
from pathlib import Path

from vsp_toolkit import load_profiles, print_validation_report, root_path, validate_profile


def profile_paths(values: list[str]) -> list[Path]:
    paths: list[Path] = []
    for value in values:
        path = root_path(value)
        if path.is_dir():
            paths.extend(sorted(path.glob("*.vsp")))
        else:
            paths.append(path)
    return paths


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", nargs="+", help="VSP file or directory")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    failed = False
    for path in profile_paths(args.profile):
        for index, profile in enumerate(load_profiles(path)):
            label = str(path) if index == 0 else f"{path}[{index}]"
            report = validate_profile(profile)
            print_validation_report(label, report)
            failed = failed or bool(report["errors"])
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
