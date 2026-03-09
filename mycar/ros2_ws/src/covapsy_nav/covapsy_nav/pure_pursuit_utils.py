"""Pure helper utilities for pure pursuit path tracking."""

from __future__ import annotations

import math

import numpy as np


def front_min_distance(
    ranges_in: list[float] | np.ndarray,
    angle_min: float,
    angle_increment: float,
    half_angle_deg: float = 20.0,
) -> float:
    ranges = np.array(ranges_in, dtype=np.float32)
    if ranges.size == 0:
        return float('inf')

    angles = float(angle_min) + np.arange(ranges.size) * float(angle_increment)
    front_mask = np.abs(angles) <= math.radians(half_angle_deg)
    selected = ranges[front_mask] if np.any(front_mask) else ranges
    selected = selected[np.isfinite(selected)]
    if selected.size == 0:
        return float('inf')
    return float(np.min(selected))


def path_is_looped(path: list[tuple[float, float]], closure_tol: float = 0.6) -> bool:
    if len(path) < 3:
        return False
    start = path[0]
    end = path[-1]
    return math.hypot(end[0] - start[0], end[1] - start[1]) <= closure_tol


def find_lookahead_index(
    path: list[tuple[float, float]],
    start_idx: int,
    lookahead_dist: float,
    looped: bool,
) -> int:
    n = len(path)
    if n < 2:
        return max(0, min(start_idx, n - 1))

    idx = max(0, min(start_idx, n - 1))
    traversed = 0.0
    max_steps = n if looped else (n - 1 - idx)

    for _ in range(max_steps):
        next_idx = (idx + 1) % n if looped else idx + 1
        if not looped and next_idx >= n:
            return n - 1
        seg = math.hypot(path[next_idx][0] - path[idx][0], path[next_idx][1] - path[idx][1])
        traversed += seg
        idx = next_idx
        if traversed >= lookahead_dist:
            return idx
    return idx
