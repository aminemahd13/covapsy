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


def to_vehicle_frame(
    target_x: float,
    target_y: float,
    pose_x: float,
    pose_y: float,
    yaw_rad: float,
) -> tuple[float, float]:
    dx = float(target_x) - float(pose_x)
    dy = float(target_y) - float(pose_y)
    local_x = dx * math.cos(-yaw_rad) - dy * math.sin(-yaw_rad)
    local_y = dx * math.sin(-yaw_rad) + dy * math.cos(-yaw_rad)
    return float(local_x), float(local_y)


def choose_progress_index(
    nearest_idx: int,
    prev_progress_idx: int | None,
    path_length: int,
    looped: bool,
    nearest_dist: float,
    relocalization_distance_m: float,
    max_backward_index_jump: int,
) -> int:
    n = int(path_length)
    if n <= 0:
        return 0
    nearest_idx = max(0, min(int(nearest_idx), n - 1))
    if prev_progress_idx is None:
        return nearest_idx
    prev_idx = max(0, min(int(prev_progress_idx), n - 1))
    max_back = max(0, int(max_backward_index_jump))

    if nearest_dist > float(relocalization_distance_m):
        return nearest_idx

    if not looped:
        if nearest_idx + max_back < prev_idx:
            return prev_idx
        return max(prev_idx, nearest_idx)

    backward_jump = (prev_idx - nearest_idx) % n
    forward_jump = (nearest_idx - prev_idx) % n
    if 0 < backward_jump <= max_back and forward_jump > backward_jump:
        return prev_idx
    return nearest_idx


def find_forward_lookahead_index(
    path: list[tuple[float, float]],
    start_idx: int,
    lookahead_dist: float,
    looped: bool,
    pose_x: float,
    pose_y: float,
    yaw_rad: float,
    min_forward_x_m: float,
) -> int | None:
    n = len(path)
    if n == 0:
        return None

    idx = find_lookahead_index(
        path=path,
        start_idx=start_idx,
        lookahead_dist=lookahead_dist,
        looped=looped,
    )
    max_steps = n if looped else max(1, n - 1 - idx)
    for _ in range(max_steps + 1):
        tx, ty = path[idx]
        local_x, _ = to_vehicle_frame(
            target_x=tx,
            target_y=ty,
            pose_x=pose_x,
            pose_y=pose_y,
            yaw_rad=yaw_rad,
        )
        if local_x > float(min_forward_x_m):
            return idx
        next_idx = (idx + 1) % n if looped else min(n - 1, idx + 1)
        if next_idx == idx:
            break
        idx = next_idx
    return None
