#!/usr/bin/env python3
"""Extract per-cycle PRESS_CAPTURE records into an Excel-ready CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import re


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
CYCLE_RE = re.compile(r"FSR fixed test:\s*(\d+)/(\d+)")
CAPTURE_RE = re.compile(
    r"PRESS_CAPTURE: angle=(?P<angle>-?\d+(?:\.\d+)?)"
    r" samples=(?P<samples>\d+)"
    r" avg=(?P<avg>\d+(?:\.\d+)?)g"
    r" median=(?P<median>\d+(?:\.\d+)?)g"
    r" peak=(?P<peak>\d+(?:\.\d+)?)g"
    r" max3=(?P<max3>\d+(?:\.\d+)?)g"
    r" final=(?P<final>\d+(?:\.\d+)?)g"
)
FIELDS = (
    "run_index",
    "cycle_index",
    "angle_deg",
    "samples",
    "avg_g",
    "median_g",
    "peak_g",
    "max3_g",
    "final_g",
    "peak_minus_final_g",
    "drop_percent",
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert PRESS_CAPTURE logs to per-cycle CSV."
    )
    parser.add_argument("input", type=Path, help="Monitor text log")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output CSV; defaults to <input>_fsr_cycles.csv",
    )
    args = parser.parse_args()

    output = args.output or args.input.with_name(
        f"{args.input.stem}_fsr_cycles.csv"
    )
    rows = []
    run_index = 0
    cycle_index = 0

    for line in args.input.read_text(
            encoding="utf-8", errors="replace").splitlines():
        clean = ANSI_ESCAPE_RE.sub("", line)
        cycle_match = CYCLE_RE.search(clean)
        if cycle_match:
            detected_cycle = int(cycle_match.group(1))
            if detected_cycle == 1:
                run_index += 1
            cycle_index = detected_cycle
            continue

        capture_match = CAPTURE_RE.search(clean)
        if not capture_match:
            continue

        values = capture_match.groupdict()
        peak = float(values["peak"])
        final = float(values["final"])
        drop = peak - final
        rows.append({
            "run_index": max(run_index, 1),
            "cycle_index": cycle_index or len(rows) + 1,
            "angle_deg": values["angle"],
            "samples": values["samples"],
            "avg_g": values["avg"],
            "median_g": values["median"],
            "peak_g": values["peak"],
            "max3_g": values["max3"],
            "final_g": values["final"],
            "peak_minus_final_g": f"{drop:.1f}",
            "drop_percent": f"{drop / peak * 100.0:.2f}",
        })

    if not rows:
        parser.error("no PRESS_CAPTURE records found")

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Extracted {len(rows)} cycles to: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
