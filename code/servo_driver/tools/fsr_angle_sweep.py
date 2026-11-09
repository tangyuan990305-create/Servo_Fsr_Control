#!/usr/bin/env python3
"""Sweep fixed FSR angles and save per-angle pressure statistics."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import re
import statistics
import sys
import time
from pathlib import Path

import serial


CAPTURE_RE = re.compile(
    r"PRESS_CAPTURE:.*?"
    r"avg=(?P<avg>-?\d+(?:\.\d+)?)g.*?"
    r"median=(?P<median>-?\d+(?:\.\d+)?)g.*?"
    r"peak=(?P<peak>-?\d+(?:\.\d+)?)g.*?"
    r"max3=(?P<max3>-?\d+(?:\.\d+)?)g.*?"
    r"final=(?P<final>-?\d+(?:\.\d+)?)g"
)

READY_PROBE_COMMAND = b"AT+FSR?\r\n"
PROBE_RESPONSE_MARKER = "+FSR:"


def _stat(values: list[float]) -> tuple[float, float, float]:
    return (
        statistics.mean(values),
        min(values),
        max(values),
    )


def _write_summary_row(
    writer: csv.writer,
    angle: float,
    requested_count: int,
    captures: list[dict[str, float]],
) -> None:
    if not captures:
        writer.writerow([
            f"{angle:.1f}",
            requested_count,
            0,
            "ERROR",
        ])
        return

    rows = []
    for metric in ("avg", "median", "peak", "max3", "final"):
        values = [capture[metric] for capture in captures]
        mean, minimum, maximum = _stat(values)
        rows.extend([
            f"{mean:.1f}",
            f"{minimum:.1f}",
            f"{maximum:.1f}",
        ])

    writer.writerow([
        f"{angle:.1f}",
        requested_count,
        len(captures),
        *rows,
    ])


def _wait_for_firmware_ready(
    port: serial.Serial,
    text_file,
    timeout_s: float = 15.0,
) -> bool:
    deadline = time.monotonic() + timeout_s
    next_probe_at = 0.0
    probe_response_seen = False

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_probe_at:
            port.write(READY_PROBE_COMMAND)
            port.flush()
            next_probe_at = now + 2.0

        raw = port.readline()
        if not raw:
            continue

        decoded = raw.decode("utf-8", errors="replace")
        print(decoded, end="")
        text_file.write(decoded)
        text_file.flush()
        if PROBE_RESPONSE_MARKER in decoded:
            probe_response_seen = True
        if probe_response_seen and "run over" in decoded:
            time.sleep(0.3)
            return True

    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sweep fixed FSR angles and record pressure statistics."
    )
    parser.add_argument("-p", "--port", required=True, help="Serial port")
    parser.add_argument(
        "--servo",
        type=int,
        default=1,
        choices=(1, 2),
        help="Servo ID",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=20,
        help="Test count per angle, 1-100",
    )
    parser.add_argument(
        "--start-tenths",
        type=int,
        default=-750,
        help="Start angle multiplied by 10",
    )
    parser.add_argument(
        "--end-tenths",
        type=int,
        default=-751,
        help="End angle multiplied by 10",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Timeout per angle in seconds; 0 waits until run over",
    )
    args = parser.parse_args()

    if args.count < 1 or args.count > 100:
        parser.error("--count must be between 1 and 100")
    if args.start_tenths < args.end_tenths:
        parser.error("start-tenths must be greater than end-tenths")

    project_dir = Path(__file__).resolve().parent.parent
    output_dir = project_dir / "logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    text_path = output_dir / f"fsr_angle_sweep_{timestamp}.txt"
    csv_path = output_dir / f"fsr_angle_sweep_{timestamp}.csv"

    angles = [
        tenths / 10.0
        for tenths in range(
            args.start_tenths,
            args.end_tenths - 1,
            -1,
        )
    ]

    header = ["angle", "requested_count", "captured_count"]
    for metric in ("avg", "median", "peak", "max3", "final"):
        header.extend([
            f"{metric}_mean",
            f"{metric}_min",
            f"{metric}_max",
        ])

    try:
        port = serial.Serial(
            args.port,
            baudrate=115200,
            timeout=1.0,
            write_timeout=2.0,
        )
    except serial.SerialException as error:
        print(f"ERROR: cannot open {args.port}: {error}", file=sys.stderr)
        return 1

    with text_path.open("w", encoding="utf-8", newline="") as text_file, \
            csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)

        try:
            if not _wait_for_firmware_ready(port, text_file):
                print(
                    "ERROR: firmware did not become ready within 15 seconds",
                    file=sys.stderr,
                )
                return 1

            for angle in angles:
                command = (
                    f"AT+FSRFIXED={args.servo},{angle:.1f},{args.count}"
                )
                line = f"\n===== {command} =====\n"
                print(line, end="")
                text_file.write(line)
                text_file.flush()

                port.write((command + "\r\n").encode("ascii"))
                port.flush()

                captures: list[dict[str, float]] = []
                deadline = (
                    time.monotonic() + args.timeout
                    if args.timeout > 0.0
                    else None
                )
                timed_out = False

                while deadline is None or time.monotonic() < deadline:
                    raw = port.readline()
                    if not raw:
                        if deadline is not None and \
                                time.monotonic() >= deadline:
                            timed_out = True
                            break
                        continue

                    decoded = raw.decode("utf-8", errors="replace")
                    print(decoded, end="")
                    text_file.write(decoded)
                    text_file.flush()

                    match = CAPTURE_RE.search(decoded)
                    if match:
                        captures.append({
                            metric: float(match.group(metric))
                            for metric in (
                                "avg",
                                "median",
                                "peak",
                                "max3",
                                "final",
                            )
                        })

                    if "run over" in decoded:
                        break
                if timed_out:
                    print(
                        f"WARNING: timeout waiting for {angle:.1f}",
                        file=sys.stderr,
                    )
                    _write_summary_row(
                        writer,
                        angle,
                        args.count,
                        captures,
                    )
                    csv_file.flush()
                    print(f"\nPartial log: {text_path}")
                    print(f"Partial CSV: {csv_path}")
                    return 1

                _write_summary_row(
                    writer,
                    angle,
                    args.count,
                    captures,
                )
                csv_file.flush()
        finally:
            port.close()

    print(f"\nComplete log: {text_path}")
    print(f"CSV summary:  {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
