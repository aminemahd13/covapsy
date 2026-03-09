"""Follow-the-gap utility logic with AI-enhanced speed control."""

from __future__ import annotations

import math
from typing import List, Optional, TYPE_CHECKING

import numpy as np

from covapsy_nav.racing_intelligence import (
    adaptive_disparity_threshold,
    adaptive_safety_radius as ai_adaptive_safety_radius,
    adaptive_steering_gain,
    compute_optimal_speed,
    compute_optimal_speed_fused,
    compute_passage_width,
    estimate_curvature,
    extract_sector_ranges,
    fuse_curvature_with_imu,
)

if TYPE_CHECKING:
    from covapsy_nav.vehicle_state_estimator import VehicleState

_NEAREST_HEADING_PENALTY = 0.30
_ANGLE_PENALTY_PER_RAD = 0.14
_GAP_SCORE_WIDTH_WEIGHT = 0.35
_GAP_SCORE_CLEARANCE_WEIGHT = 0.35
_GAP_SCORE_HEADING_WEIGHT = 0.30
_GAP_EDGE_PENALTY = 0.14
_SPEED_LOOKAHEAD_WINDOW_DEG = 20.0
_SPEED_TTC_WINDOW_DEG = 12.0

# Forward-direction safety: prevent wall collisions
_EMERGENCY_WALL_DIST_M = 0.30
_FORWARD_TTC_MIN_SEC = 0.40
_SLEW_URGENCY_DIST_M = 0.55
_SLEW_URGENCY_BOOST = 2.5
_GAP_REACH_MULTIPLIER = 2.8

# Module-level state for curvature smoothing across calls
_prev_curvature: float = 0.0
_prev_speed: float = 0.0


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
    steering_slew_rate: float = 0.15,
    max_steering: float = 0.5,
    ttc_target_sec: float = 1.2,
    use_ai_speed: bool = True,
    depth_front_dist: float = float("inf"),
    vehicle_state: Optional["VehicleState"] = None,
) -> tuple[float, float]:
    """Compute (speed_m_s, steering_rad) from filtered scan data.

    When ``use_ai_speed=True`` (default), uses the racing intelligence module
    for curvature-aware predictive speed control.  Falls back to the classic
    heuristic formula otherwise.
    """
    global _prev_curvature, _prev_speed

    ranges = np.array(ranges_in, dtype=np.float64)
    n = len(ranges)
    if n == 0:
        return 0.0, 0.0

    ranges = np.where(np.isfinite(ranges), ranges, max_range)
    ranges = np.clip(ranges, 0.0, max_range)

    angles = angle_min + np.arange(n) * angle_increment

    # --- AI: curvature estimation from full 360° scan ---
    ranges_360: List[float] = []
    if use_ai_speed and n >= 180:
        # Build 360-entry range array indexed by degree (0=front)
        for deg in range(360):
            rad = math.radians(deg if deg <= 180 else deg - 360)
            # Find nearest scan index
            idx_f = (rad - angle_min) / angle_increment if angle_increment != 0 else 0
            idx_i = max(0, min(n - 1, int(round(idx_f))))
            ranges_360.append(float(ranges[idx_i]))

        sector_ranges = extract_sector_ranges(ranges_360)
        _prev_curvature = estimate_curvature(sector_ranges, _prev_curvature)
        # Fuse with IMU for faster curvature response
        if vehicle_state is not None:
            _prev_curvature = fuse_curvature_with_imu(_prev_curvature, vehicle_state)
    else:
        sector_ranges = []

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

    # --- AI: adaptive disparity threshold based on curvature ---
    effective_disparity = disparity_threshold
    if use_ai_speed:
        effective_disparity = adaptive_disparity_threshold(_prev_curvature, disparity_threshold)

    # Disparity extension
    half_width = car_width * 0.5
    for i in range(1, fn):
        diff = abs(f_ranges[i] - f_ranges[i - 1])
        if diff > effective_disparity:
            ci = i if f_ranges[i] < f_ranges[i - 1] else i - 1
            cr = f_ranges[ci]
            if cr > 0.05:
                ext_angle = math.atan2(half_width, cr)
                ext_count = int(ext_angle / angle_inc)
                lo = max(0, ci - ext_count)
                hi = min(fn, ci + ext_count + 1)
                for j in range(lo, hi):
                    f_ranges[j] = min(f_ranges[j], cr)

    # Mask unreachable gaps (too far off-axis for the car to steer to)
    gap_reach_limit = max_steering * _GAP_REACH_MULTIPLIER
    if gap_reach_limit < fov * 0.5:
        for i in range(fn):
            if abs(f_angles[i]) > gap_reach_limit:
                f_ranges[i] = 0.0

    # --- AI: adaptive safety radius based on passage width ---
    passage_width = compute_passage_width(
        list(reference_ranges), list(f_angles),
    )
    if use_ai_speed:
        effective_safety = ai_adaptive_safety_radius(passage_width, safety_radius)
    else:
        effective_safety = safety_radius

    # Safety bubble
    nearest_idx = int(np.argmin(reference_ranges + _NEAREST_HEADING_PENALTY * np.abs(f_angles)))
    nearest_dist = float(reference_ranges[nearest_idx])
    if nearest_dist < effective_safety * 5.0:
        for i in range(fn):
            arc_dist = nearest_dist * abs(f_angles[i] - f_angles[nearest_idx])
            if arc_dist < effective_safety:
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

    # --- AI: adaptive steering gain in curves ---
    effective_gain = steering_gain
    if use_ai_speed:
        effective_gain = adaptive_steering_gain(_prev_curvature, steering_gain)

    # Forward clearance: where the car is actually heading (for adaptive steering + emergency brake)
    actual_forward_clearance = _forward_clearance_for_heading(
        f_ranges=reference_ranges,
        f_angles=f_angles,
        heading_rad=0.0,
        half_window_deg=_SPEED_LOOKAHEAD_WINDOW_DEG,
        fallback=nearest_dist,
    )

    # Adaptive slew rate: faster steering when wall is close ahead
    effective_slew = steering_slew_rate
    if actual_forward_clearance < _SLEW_URGENCY_DIST_M:
        urgency = 1.0 - actual_forward_clearance / _SLEW_URGENCY_DIST_M
        effective_slew = steering_slew_rate * (1.0 + _SLEW_URGENCY_BOOST * urgency)

    raw_steering = effective_gain * target_angle
    raw_steering = max(-max_steering, min(max_steering, raw_steering))
    steering = max(
        float(prev_steering) - float(effective_slew),
        min(float(prev_steering) + float(effective_slew), raw_steering),
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

    # Fuse depth camera with LiDAR clearance (depth wins if closer)
    if math.isfinite(depth_front_dist) and depth_front_dist > 0:
        projected_clearance = min(projected_clearance, depth_front_dist)
        projected_ttc_clearance = min(projected_ttc_clearance, depth_front_dist)

    # --- Speed computation ---
    if use_ai_speed and sector_ranges:
        if vehicle_state is not None:
            # IMU-fused path: faster response, grip-aware
            speed = compute_optimal_speed_fused(
                max_speed=max_speed,
                min_speed=min_speed,
                steering_rad=steering,
                max_steering=max_steering,
                projected_clearance=projected_clearance,
                passage_width=passage_width,
                curvature=_prev_curvature,
                prev_speed=_prev_speed,
                sector_ranges=sector_ranges,
                ttc_clearance=projected_ttc_clearance,
                vehicle_state=vehicle_state,
                ttc_target_sec=ttc_target_sec,
                speed_ramp_up=0.35,
                speed_ramp_down=0.25,
            )
        else:
            speed = compute_optimal_speed(
                max_speed=max_speed,
                min_speed=min_speed,
                steering_rad=steering,
                max_steering=max_steering,
                projected_clearance=projected_clearance,
                passage_width=passage_width,
                curvature=_prev_curvature,
                prev_speed=_prev_speed,
                sector_ranges=sector_ranges,
                ttc_clearance=projected_ttc_clearance,
                ttc_target_sec=ttc_target_sec,
                speed_ramp_up=0.35,
                speed_ramp_down=0.25,
            )
    else:
        # Classic heuristic fallback
        steer_factor = 1.0 - 0.45 * abs(steering) / max(max_steering, 1e-6)
        free_space_factor = max(0.0, min(1.0, (projected_clearance - 0.08) / 1.2))
        speed = min_speed + (max_speed - min_speed) * steer_factor * free_space_factor

        speed = max(0.0, min(max_speed, speed))
        ttc = projected_ttc_clearance / max(speed, 0.05)
        target_ttc = max(float(ttc_target_sec), 0.2)
        if ttc < target_ttc:
            speed = min(speed, projected_ttc_clearance / target_ttc)

        if projected_clearance > 0.35:
            speed = max(min_speed, speed)
        else:
            speed = max(0.0, speed)
        speed = min(max_speed, speed)

    # Forward-direction safety: TTC guard and emergency wall brake
    if speed > 0.05 and actual_forward_clearance < 2.0:
        fwd_ttc = actual_forward_clearance / speed
        if fwd_ttc < _FORWARD_TTC_MIN_SEC:
            speed = min(speed, max(min_speed * 0.3, actual_forward_clearance / _FORWARD_TTC_MIN_SEC))
    if actual_forward_clearance < _EMERGENCY_WALL_DIST_M:
        speed = min(speed, min_speed)

    _prev_speed = speed
    return float(speed), float(steering)
