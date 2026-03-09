#!/usr/bin/env python3
"""Build a smoothed racing path JSON from setup-lap odom CSV.

Expected CSV columns: x,y (header optional). Extra columns are ignored.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def read_points(csv_path: Path) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                points.append((x, y))
            except Exception:
                # Skip header or malformed rows.
                continue
    return points


def smooth(points: list[tuple[float, float]], window: int) -> list[tuple[float, float]]:
    if window <= 1 or len(points) < window:
        return points
    half = window // 2
    out: list[tuple[float, float]] = []
    for i in range(len(points)):
        lo = max(0, i - half)
        hi = min(len(points), i + half + 1)
        xs = [p[0] for p in points[lo:hi]]
        ys = [p[1] for p in points[lo:hi]]
        out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return out


def downsample(points: list[tuple[float, float]], min_step: float) -> list[tuple[float, float]]:
    if not points:
        return []
    out = [points[0]]
    min_step_sq = min_step * min_step
    for x, y in points[1:]:
        px, py = out[-1]
        dx = x - px
        dy = y - py
        if (dx * dx + dy * dy) >= min_step_sq:
            out.append((x, y))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Build racing path JSON from odom CSV")
    parser.add_argument("--input", required=True, help="Input CSV path with x,y columns")
    parser.add_argument("--output", required=True, help="Output JSON path")
    parser.add_argument("--smooth-window", type=int, default=7, help="Moving-average window")
    parser.add_argument("--min-step", type=float, default=0.06, help="Minimum point spacing (m)")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    raw = read_points(input_path)
    if not raw:
        raise SystemExit(f"No valid points read from {input_path}")

    smoothed = smooth(raw, max(1, int(args.smooth_window)))
    sampled = downsample(smoothed, max(0.01, float(args.min_step)))
    payload = {"points": [{"x": x, "y": y} for x, y in sampled]}
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {len(sampled)} points -> {output_path}")


if __name__ == "__main__":
    main()
