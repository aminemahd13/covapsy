"""Follow-the-gap utility logic."""

from __future__ import annotations

import math
import numpy as np


_NEAREST_HEADING_PENALTY = 0.45
_ANGLE_PENALTY_PER_RAD = 0.22
_GAP_SCORE_WIDTH_WEIGHT = 0.45
_GAP_SCORE_CLEARANCE_WEIGHT = 0.45
_GAP_SCORE_HEADING_WEIGHT = 0.10
_GAP_EDGE_PENALTY = 0.22
_SPEED_LOOKAHEAD_WINDOW_DEG = 14.0
_SPEED_TTC_WINDOW_DEG = 8.0


def _find_free_gaps(mask: np.ndarray) -> list[tuple[int, int]]:
    gaps: list[tuple[int, int]] = []
    start: int | None = None

    for i, valid in enumerate(mask):
        if valid and start is None:
            start = i
        elif not valid and start is not None:
            gaps.append((start, i - 1))
            start = None
    if start is not None:
        gaps.append((start, len(mask) - 1))
    return gaps


def _choose_best_gap(
    f_ranges: np.ndarray,
    f_angles: np.ndarray,
    gaps: list[tuple[int, int]],
    max_range: float,
) -> tuple[int, int] | None:
    if not gaps:
        return None

    total = max(len(f_ranges), 1)
    max_heading = max(float(np.max(np.abs(f_angles))), 1e-6)

    def score(gap: tuple[int, int]) -> float:
        start, end = gap
        width = (end - start + 1) / float(total)
        section = f_ranges[start : end + 1]
        clearance = float(np.median(section)) / max(max_range, 1e-6)
        clearance = max(0.0, min(1.0, clearance))
        center_idx = (start + end) // 2
        heading = 1.0 - min(abs(float(f_angles[center_idx])) / max_heading, 1.0)
        return (
            _GAP_SCORE_WIDTH_WEIGHT * width
            + _GAP_SCORE_CLEARANCE_WEIGHT * clearance
            + _GAP_SCORE_HEADING_WEIGHT * heading
        )

    return max(gaps, key=score)


def _forward_clearance_for_heading(
    f_ranges: np.ndarray,
    f_angles: np.ndarray,
    heading_rad: float,
    half_window_deg: float,
    fallback: float,
) -> float:
    half_window_rad = math.radians(half_window_deg)
    mask = np.abs(f_angles - heading_rad) <= half_window_rad
    if np.any(mask):
        return float(np.min(f_ranges[mask]))
    return float(fallback)


def compute_gap_command(
    ranges_in: np.ndarray,
    angle_min: float,
    angle_increment: float,
    car_width: float,
    max_speed: float,
    min_speed: float,
    max_range: float,
    safety_radius: float,
    disparity_threshold: float,
    steering_gain: float,
    fov_degrees: float = 200.0,
    prev_steering: float = 0.0,
    steering_slew_rate: float = 0.07,
    max_steering: float = 0.5,
    ttc_target_sec: float = 1.2,
) -> tuple[float, float]:
    """Compute (speed_m_s, steering_rad) from filtered scan data."""
    ranges = np.array(ranges_in, dtype=np.float64)
    n = len(ranges)
    if n == 0:
        return 0.0, 0.0

    ranges = np.where(np.isfinite(ranges), ranges, max_range)
    ranges = np.clip(ranges, 0.0, max_range)

    angles = angle_min + np.arange(n) * angle_increment

    fov = math.radians(fov_degrees)
    front_mask = np.abs(angles) <= (fov * 0.5)
    front_idx = np.where(front_mask)[0]
    if len(front_idx) == 0:
        return 0.0, 0.0

    f_ranges = ranges[front_idx].copy()
    reference_ranges = f_ranges.copy()
    f_angles = angles[front_idx]
    fn = len(f_ranges)
    angle_inc = abs(angle_increment) if angle_increment != 0.0 else (2.0 * math.pi / n)

    # Disparity extension
    half_width = car_width * 0.5
    for i in range(1, fn):
        diff = abs(f_ranges[i] - f_ranges[i - 1])
        if diff > disparity_threshold:
            ci = i if f_ranges[i] < f_ranges[i - 1] else i - 1
            cr = f_ranges[ci]
            if cr > 0.05:
                ext_angle = math.atan2(half_width, cr)
                ext_count = int(ext_angle / angle_inc)
                lo = max(0, ci - ext_count)
                hi = min(fn, ci + ext_count + 1)
                for j in range(lo, hi):
                    f_ranges[j] = min(f_ranges[j], cr)

    # Safety bubble
    nearest_idx = int(np.argmin(reference_ranges + _NEAREST_HEADING_PENALTY * np.abs(f_angles)))
    nearest_dist = float(reference_ranges[nearest_idx])
    if nearest_dist < safety_radius * 5.0:
        for i in range(fn):
            arc_dist = nearest_dist * abs(f_angles[i] - f_angles[nearest_idx])
            if arc_dist < safety_radius:
                f_ranges[i] = 0.0

    # Best gap among all free gaps
    nonzero = f_ranges > 0.05
    gaps = _find_free_gaps(nonzero)
    if not gaps:
        return 0.0, 0.0

    best_gap = _choose_best_gap(f_ranges=f_ranges, f_angles=f_angles, gaps=gaps, max_range=max_range)
    if best_gap is None:
        return 0.0, 0.0

    # Best point in selected gap with center penalty.
    gap_start, gap_end = best_gap
    gap_center = 0.5 * (gap_start + gap_end)
    gap_half_span = max(0.5 * (gap_end - gap_start + 1), 1.0)
    idx = np.arange(gap_start, gap_end + 1)
    point_scores = (
        f_ranges[idx]
        - _ANGLE_PENALTY_PER_RAD * np.abs(f_angles[idx])
        - _GAP_EDGE_PENALTY * np.abs((idx - gap_center) / gap_half_span)
    )
    best_idx = int(idx[int(np.argmax(point_scores))])
    target_angle = float(f_angles[best_idx])

    raw_steering = steering_gain * target_angle
    raw_steering = max(-max_steering, min(max_steering, raw_steering))
    steering = max(
        float(prev_steering) - float(steering_slew_rate),
        min(float(prev_steering) + float(steering_slew_rate), raw_steering),
    )

    projected_clearance = _forward_clearance_for_heading(
        f_ranges=reference_ranges,
        f_angles=f_angles,
        heading_rad=steering,
        half_window_deg=_SPEED_LOOKAHEAD_WINDOW_DEG,
        fallback=nearest_dist,
    )
    projected_ttc_clearance = _forward_clearance_for_heading(
        f_ranges=reference_ranges,
        f_angles=f_angles,
        heading_rad=steering,
        half_window_deg=_SPEED_TTC_WINDOW_DEG,
        fallback=nearest_dist,
    )

    steer_factor = 1.0 - 0.7 * abs(steering) / max(max_steering, 1e-6)
    free_space_factor = max(0.0, min(1.0, (projected_clearance - 0.14) / 1.35))
    speed = min_speed + (max_speed - min_speed) * steer_factor * free_space_factor

    # TTC guard: slow down when projected front time-to-collision is short.
    speed = max(0.0, min(max_speed, speed))
    ttc = projected_ttc_clearance / max(speed, 0.05)
    target_ttc = max(float(ttc_target_sec), 0.2)
    if ttc < target_ttc:
        speed = min(speed, projected_ttc_clearance / target_ttc)

    # Keep low-speed creeping when space is available, allow near-stop in dense traffic.
    if projected_clearance > 0.35:
        speed = max(min_speed, speed)
    else:
        speed = max(0.0, speed)
    speed = min(max_speed, speed)
    return float(speed), float(steering)
