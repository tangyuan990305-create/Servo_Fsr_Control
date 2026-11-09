#!/usr/bin/env python3
"""Build an Excel workbook with per-cycle FSR pressure charts."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import statistics

import xlsxwriter


def read_cycles(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def add_line_chart(
    workbook: xlsxwriter.Workbook,
    sheet: xlsxwriter.worksheet.Worksheet,
    title: str,
    columns: list[tuple[int, str, int]],
    last_row: int,
    position: str,
) -> None:
    chart = workbook.add_chart({"type": "line"})
    for column, name, color in columns:
        chart.add_series({
            "name": name,
            "categories": ["Data", 1, 1, last_row, 1],
            "values": ["Data", 1, column, last_row, column],
            "line": {"color": color, "width": 1.75},
        })
    chart.set_title({"name": title})
    chart.set_x_axis({
        "name": "Press cycle",
        "major_gridlines": {"visible": False},
    })
    chart.set_y_axis({
        "name": "Force (g)",
        "major_gridlines": {"visible": True, "line": {"color": "#D9E2EC"}},
    })
    chart.set_legend({"position": "bottom"})
    chart.set_size({"width": 1000, "height": 420})
    sheet.insert_chart(position, chart)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build Excel charts from a per-cycle FSR CSV."
    )
    parser.add_argument("input", type=Path, help="Per-cycle CSV")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output XLSX; defaults to <input>.xlsx",
    )
    args = parser.parse_args()

    rows = read_cycles(args.input)
    if not rows:
        parser.error("input CSV contains no cycles")

    output = args.output or args.input.with_suffix(".xlsx")
    workbook = xlsxwriter.Workbook(output)
    workbook.set_calc_mode("auto")

    title_format = workbook.add_format({
        "bold": True,
        "font_size": 18,
        "font_color": "#17324D",
    })
    label_format = workbook.add_format({
        "bold": True,
        "font_color": "#52616B",
    })
    number_format = workbook.add_format({"num_format": "0.0"})
    integer_format = workbook.add_format({"num_format": "0"})

    summary = workbook.add_worksheet("Summary")
    data = workbook.add_worksheet("Data")

    summary.hide_gridlines(2)
    summary.set_column("A:A", 30)
    summary.set_column("B:B", 18)
    summary.write("A1", "FSR 100-press pressure curves", title_format)
    summary.write("A2", f"Angle: {rows[0]['angle_deg']} deg")
    summary.write("A3", f"Cycles: {len(rows)}")

    peak_values = [float(row["peak_g"]) for row in rows]
    max3_values = [float(row["max3_g"]) for row in rows]
    final_values = [float(row["final_g"]) for row in rows]
    drop_values = [float(row["peak_minus_final_g"]) for row in rows]

    summary_rows = [
        ("Peak mean (g)", statistics.mean(peak_values)),
        ("Peak minimum (g)", min(peak_values)),
        ("Peak maximum (g)", max(peak_values)),
        ("Final mean (g)", statistics.mean(final_values)),
        ("Final minimum (g)", min(final_values)),
        ("Final maximum (g)", max(final_values)),
        ("Peak-to-final mean (g)", statistics.mean(drop_values)),
        ("Peak-to-final minimum (g)", min(drop_values)),
        ("Peak-to-final maximum (g)", max(drop_values)),
    ]
    for row_index, (label, value) in enumerate(summary_rows, start=5):
        summary.write(row_index - 1, 0, label, label_format)
        summary.write_number(row_index - 1, 1, value, number_format)

    headers = [
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
    ]
    for column, header in enumerate(headers):
        data.write(0, column, header, label_format)

    numeric_fields = set(headers) - {"angle_deg"}
    for row_index, row in enumerate(rows, start=1):
        for column, header in enumerate(headers):
            value = row[header]
            if header in numeric_fields:
                number = float(value)
                data.write_number(
                    row_index,
                    column,
                    number,
                    integer_format if number.is_integer() else number_format,
                )
            else:
                data.write(row_index, column, value)

    data.freeze_panes(1, 2)
    data.autofilter(0, 0, len(rows), len(headers) - 1)
    data.set_column(0, 1, 12)
    data.set_column(2, 2, 12)
    data.set_column(3, 4, 12)
    data.set_column(5, 10, 16)

    last_row = len(rows)
    add_line_chart(
        workbook,
        summary,
        "Peak, moving maximum and final force",
        [
            (6, "Peak", "#C0392B"),
            (7, "Max3", "#2E86C1"),
            (8, "Final", "#27AE60"),
        ],
        last_row,
        "A16",
    )
    add_line_chart(
        workbook,
        summary,
        "Peak-to-final pressure drop",
        [(9, "Peak - final", "#8E44AD")],
        last_row,
        "A40",
    )

    summary.write(
        "A64",
        "Each point is one press cycle. The charts show raw per-cycle values.",
    )
    data.activate()
    workbook.close()

    print(f"Created {len(rows)}-cycle workbook: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
