#!/usr/bin/env python3
"""Extract FSR_TRACE records from a monitor log into an Excel-ready CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
TRACE_RE = re.compile(
    r"FSR_TRACE,"
    r"(?P<cycle>-?\d+),"
    r"(?P<angle_deg>-?\d+(?:\.\d+)?),"
    r"(?P<sample_index>\d+),"
    r"(?P<elapsed_ms>\d+),"
    r"(?P<raw_mv>-?\d+),"
    r"(?P<model_mv>-?\d+),"
    r"(?P<force_g>-?\d+(?:\.\d+)?)"
)
FIELDS = (
    "cycle",
    "angle_deg",
    "sample_index",
    "elapsed_ms",
    "raw_mv",
    "model_mv",
    "force_g",
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert FSR_TRACE monitor lines to CSV."
    )
    parser.add_argument("input", type=Path, help="Monitor text log")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output CSV; defaults to <input>_fsr_trace.csv",
    )
    args = parser.parse_args()

    output = args.output or args.input.with_name(
        f"{args.input.stem}_fsr_trace.csv"
    )
    rows = []
    for line in args.input.read_text(
            encoding="utf-8", errors="replace").splitlines():
        match = TRACE_RE.search(ANSI_ESCAPE_RE.sub("", line))
        if match:
            rows.append(match.groupdict())

    if not rows:
        parser.error("no FSR_TRACE records found")

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Extracted {len(rows)} samples to: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
