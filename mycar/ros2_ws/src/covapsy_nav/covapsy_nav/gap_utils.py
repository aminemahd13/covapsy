"""Follow-the-gap utility logic."""

from __future__ import annotations

import math
import numpy as np


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
    nearest_idx = int(np.argmin(f_ranges))
    nearest_dist = float(f_ranges[nearest_idx])
    if nearest_dist < safety_radius * 5.0:
        for i in range(fn):
            arc_dist = nearest_dist * abs(f_angles[i] - f_angles[nearest_idx])
            if arc_dist < safety_radius:
                f_ranges[i] = 0.0

    # Largest gap
    nonzero = f_ranges > 0.05
    gaps = []
    start = None
    for i in range(fn):
        if nonzero[i] and start is None:
            start = i
        elif not nonzero[i] and start is not None:
            gaps.append((start, i - 1))
            start = None
    if start is not None:
        gaps.append((start, fn - 1))
    if not gaps:
        return 0.0, 0.0

    # Best point in largest gap
    best_gap = max(gaps, key=lambda g: g[1] - g[0])
    gap_ranges = f_ranges[best_gap[0] : best_gap[1] + 1]
    best_idx = best_gap[0] + int(np.argmax(gap_ranges))
    target_angle = float(f_angles[best_idx])

    raw_steering = steering_gain * target_angle
    raw_steering = max(-max_steering, min(max_steering, raw_steering))
    steering = max(
        float(prev_steering) - float(steering_slew_rate),
        min(float(prev_steering) + float(steering_slew_rate), raw_steering),
    )

    steer_factor = 1.0 - 0.7 * abs(steering) / max(max_steering, 1e-6)
    free_space_factor = min(1.0, nearest_dist / 2.0)
    speed = min_speed + (max_speed - min_speed) * steer_factor * free_space_factor

    # TTC guard: slow down when projected front time-to-collision is short.
    speed = max(0.0, min(max_speed, speed))
    ttc = nearest_dist / max(speed, 0.05)
    target_ttc = max(float(ttc_target_sec), 0.2)
    if ttc < target_ttc:
        speed = min(speed, nearest_dist / target_ttc)

    # Keep low-speed creeping when space is available, allow near-stop in dense traffic.
    if nearest_dist > 0.35:
        speed = max(min_speed, speed)
    else:
        speed = max(0.0, speed)
    speed = min(max_speed, speed)
    return float(speed), float(steering)
