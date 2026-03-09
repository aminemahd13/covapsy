"""AI-enhanced racing intelligence — pure Python, no ROS2 dependency.

Provides lightweight algorithms that run at 50 Hz on Raspberry Pi 5:
  - Multi-sector curvature estimation from LiDAR geometry
  - Optimal speed computation with predictive braking
  - Track section classification (straight / gentle / tight / chicane)
  - Adaptive parameter tuning based on real-time conditions

Used by both ROS2 nodes and the standalone Webots controller.
"""

from __future__ import annotations

import math
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Sector configuration — divide front FOV into sectors for analysis
# ---------------------------------------------------------------------------
_NUM_SECTORS = 12
_SECTOR_HALF_FOV_DEG = 90.0  # analyse ±90° in front

# Curvature estimation
_CURVATURE_SMOOTHING = 0.55  # EMA alpha — lower = smoother/more stable signal
_MIN_AVG_RANGE = 0.10

# Speed profiles — wider bands so fewer scans are classified as "tight"
_STRAIGHT_THRESHOLD = 0.15        # curvature below this = straight
_GENTLE_CURVE_THRESHOLD = 0.45    # curvature below this = gentle curve
_TIGHT_CURVE_THRESHOLD = 0.72     # above this = tight curve

# Speed factors per section type — much less aggressive reduction in turns
_SPEED_FACTOR_STRAIGHT = 1.00
_SPEED_FACTOR_GENTLE = 0.92
_SPEED_FACTOR_TIGHT = 0.72
_SPEED_FACTOR_CHICANE = 0.58

# Predictive braking
_BRAKE_LOOKAHEAD_SECTORS = 3     # how many sectors ahead to check
_BRAKE_ANTICIPATION = 0.22       # lighter pre-braking (was 0.35)

# Acceleration boost
_ACCEL_BOOST_FACTOR = 1.20       # speed multiplier on exits of corners
_ACCEL_BOOST_MIN_CLEARANCE = 1.2 # metres ahead needed to boost (was 1.5)

# Passage width adaptive margin
_NARROW_PASSAGE_M = 0.60
_WIDE_PASSAGE_M = 1.60


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Sector analysis
# ---------------------------------------------------------------------------

def extract_sector_ranges(
    ranges_360: List[float],
    num_sectors: int = _NUM_SECTORS,
    half_fov_deg: float = _SECTOR_HALF_FOV_DEG,
) -> List[float]:
    """Return median range for each sector spanning [-half_fov, +half_fov].

    ``ranges_360`` is indexed 0..359 where 0 = front, positive = left.
    """
    sector_width_deg = (2.0 * half_fov_deg) / num_sectors
    sector_ranges: List[float] = []

    for s in range(num_sectors):
        start_deg = -half_fov_deg + s * sector_width_deg
        end_deg = start_deg + sector_width_deg
        samples: List[float] = []
        for deg_i in range(int(start_deg), int(end_deg)):
            idx = int(deg_i) % 360
            r = ranges_360[idx]
            if math.isfinite(r) and 0.05 <= r <= 12.0:
                samples.append(r)
        if samples:
            samples.sort()
            sector_ranges.append(samples[len(samples) // 2])
        else:
            sector_ranges.append(5.0)

    return sector_ranges


def estimate_curvature(
    sector_ranges: List[float],
    prev_curvature: float = 0.0,
    smoothing: float = _CURVATURE_SMOOTHING,
) -> float:
    """Estimate normalised curvature [0..1] from sector range asymmetry.

    0 = perfectly straight, 1 = very tight turn.
    """
    n = len(sector_ranges)
    if n < 4:
        return prev_curvature

    mid = n // 2
    left_sectors = sector_ranges[:mid]
    right_sectors = sector_ranges[mid:]

    left_avg = sum(left_sectors) / max(len(left_sectors), 1)
    right_avg = sum(right_sectors) / max(len(right_sectors), 1)
    total_avg = 0.5 * (left_avg + right_avg)

    if total_avg < _MIN_AVG_RANGE:
        return prev_curvature

    # Asymmetry = how different left vs right distances are
    asymmetry = abs(left_avg - right_avg) / total_avg

    # Also check front narrowing (both sides closing in → tight curve or chicane)
    front_sectors = sector_ranges[mid - 1: mid + 2] if mid >= 1 else [sector_ranges[mid]]
    front_avg = sum(front_sectors) / max(len(front_sectors), 1)
    narrowing = _clamp(1.0 - front_avg / max(total_avg, 0.1), 0.0, 1.0)

    raw_curvature = _clamp(0.60 * asymmetry + 0.40 * narrowing, 0.0, 1.0)

    # EMA: smoothing = retention of previous value (higher = more stable)
    return (1.0 - smoothing) * raw_curvature + smoothing * prev_curvature


def classify_section(curvature: float) -> str:
    """Classify track section from curvature estimate."""
    if curvature < _STRAIGHT_THRESHOLD:
        return "straight"
    if curvature < _GENTLE_CURVE_THRESHOLD:
        return "gentle"
    if curvature < _TIGHT_CURVE_THRESHOLD:
        return "tight"
    return "chicane"


def section_speed_factor(section: str) -> float:
    """Return [0..1] speed multiplier for the classified section."""
    return {
        "straight": _SPEED_FACTOR_STRAIGHT,
        "gentle": _SPEED_FACTOR_GENTLE,
        "tight": _SPEED_FACTOR_TIGHT,
        "chicane": _SPEED_FACTOR_CHICANE,
    }.get(section, _SPEED_FACTOR_TIGHT)


# ---------------------------------------------------------------------------
# Predictive braking
# ---------------------------------------------------------------------------

def predictive_brake_factor(
    sector_ranges: List[float],
    current_curvature: float,
) -> float:
    """Return [0..1] factor: 1.0 = no braking needed, <1.0 = brake for upcoming curve.

    Analyses sector gradient to detect if ranges are closing (approaching a wall/turn).
    """
    n = len(sector_ranges)
    if n < 6:
        return 1.0

    mid = n // 2
    # Front sectors vs wide sectors
    front = sector_ranges[max(0, mid - 1): min(n, mid + 2)]
    side = sector_ranges[:max(1, mid - 2)] + sector_ranges[min(n, mid + 2):]

    front_avg = sum(front) / max(len(front), 1)
    side_avg = sum(side) / max(len(side), 1)

    if side_avg < 0.1:
        return 1.0

    # If front is much shorter than sides → approaching a wall/curve end
    closing_ratio = _clamp(1.0 - front_avg / max(side_avg, 0.1), 0.0, 1.0)

    # Combine with current curvature for anticipation
    urgency = _clamp(
        closing_ratio * 0.6 + current_curvature * 0.4,
        0.0, 1.0,
    )

    return 1.0 - _BRAKE_ANTICIPATION * urgency


# ---------------------------------------------------------------------------
# Optimal speed computation
# ---------------------------------------------------------------------------

def compute_optimal_speed(
    max_speed: float,
    min_speed: float,
    steering_rad: float,
    max_steering: float,
    projected_clearance: float,
    passage_width: float,
    curvature: float,
    prev_speed: float,
    sector_ranges: List[float],
    ttc_clearance: float,
    ttc_target_sec: float = 1.0,
    speed_ramp_up: float = 0.22,
    speed_ramp_down: float = 0.16,
) -> float:
    """Compute AI-optimised speed combining multiple factors.

    Returns the target speed in m/s.
    """
    section = classify_section(curvature)
    base_factor = section_speed_factor(section)

    # Steering penalty — mild, the section factor already handles most of it
    steer_ratio = _clamp(abs(steering_rad) / max(max_steering, 1e-6), 0.0, 1.0)
    steer_penalty = 1.0 - 0.35 * steer_ratio  # light touch — curvature does the heavy lifting

    # Clearance factor — ramp to 1.0 faster so open track ≈ full speed
    clearance_factor = _clamp((projected_clearance - 0.08) / 1.3, 0.0, 1.0)

    # Passage width factor — be bolder in wide passages
    width_factor = _clamp(
        (passage_width - _NARROW_PASSAGE_M) / (_WIDE_PASSAGE_M - _NARROW_PASSAGE_M),
        0.0, 1.0,
    )
    width_boost = 1.0 + 0.18 * width_factor  # up to 18% faster in wide sections

    # Predictive braking
    brake_factor = predictive_brake_factor(sector_ranges, curvature)

    # Acceleration boost on corner exit
    accel_boost = 1.0
    if (
        section == "straight"
        and projected_clearance > _ACCEL_BOOST_MIN_CLEARANCE
        and prev_speed > 0.5
    ):
        accel_boost = _ACCEL_BOOST_FACTOR

    # Combine all factors
    raw_speed = max_speed * base_factor * steer_penalty * clearance_factor
    raw_speed *= width_boost * brake_factor * accel_boost
    raw_speed = _clamp(raw_speed, min_speed, max_speed)

    # TTC guard
    if raw_speed > 0.05:
        ttc = ttc_clearance / max(raw_speed, 0.05)
        if ttc < max(ttc_target_sec, 0.2):
            raw_speed = min(raw_speed, ttc_clearance / max(ttc_target_sec, 0.2))

    # Ensure minimum creep when space available
    if projected_clearance > 0.35:
        raw_speed = max(min_speed, raw_speed)
    raw_speed = _clamp(raw_speed, 0.0, max_speed)

    # Smooth ramp (skip on cold-start when prev_speed is near zero)
    if prev_speed > 0.05:
        if raw_speed > prev_speed:
            speed = min(raw_speed, prev_speed + speed_ramp_up)
        else:
            speed = max(raw_speed, prev_speed - speed_ramp_down)
    else:
        speed = raw_speed

    return _clamp(speed, 0.0, max_speed)


# ---------------------------------------------------------------------------
# Adaptive tuning
# ---------------------------------------------------------------------------

def adaptive_safety_radius(
    passage_width: float,
    base_radius: float = 0.20,
    min_radius: float = 0.06,
) -> float:
    """Reduce safety bubble in narrow passages to avoid deadlocks."""
    if passage_width >= _WIDE_PASSAGE_M:
        return base_radius
    if passage_width <= _NARROW_PASSAGE_M:
        return min_radius
    t = (passage_width - _NARROW_PASSAGE_M) / (_WIDE_PASSAGE_M - _NARROW_PASSAGE_M)
    return min_radius + t * (base_radius - min_radius)


def adaptive_disparity_threshold(
    curvature: float,
    base_threshold: float = 0.38,
) -> float:
    """Tighten disparity detection in curves, relax on straights."""
    if curvature < _STRAIGHT_THRESHOLD:
        return base_threshold * 1.15  # slightly wider on straights
    if curvature > _TIGHT_CURVE_THRESHOLD:
        return base_threshold * 0.75  # tighter in corners
    return base_threshold


def adaptive_steering_gain(
    curvature: float,
    base_gain: float = 1.0,
) -> float:
    """Increase steering responsiveness in curves."""
    if curvature > _GENTLE_CURVE_THRESHOLD:
        return base_gain * 1.12
    return base_gain


def compute_passage_width(
    front_ranges: List[float],
    front_angles: List[float],
    min_rad: float = 0.38,
    max_rad: float = 1.48,
) -> float:
    """Estimate passage width from LiDAR left/right clearance."""
    left_samples = [
        r for r, a in zip(front_ranges, front_angles) if min_rad <= a <= max_rad
    ]
    right_samples = [
        r for r, a in zip(front_ranges, front_angles) if -max_rad <= a <= -min_rad
    ]

    def _median(lst: List[float], default: float) -> float:
        if not lst:
            return default
        s = sorted(lst)
        return s[len(s) // 2]

    left_clear = _median(left_samples, 5.0)
    right_clear = _median(right_samples, 5.0)
    return left_clear + right_clear
