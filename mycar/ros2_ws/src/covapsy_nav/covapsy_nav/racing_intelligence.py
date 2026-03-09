"""AI-enhanced racing intelligence — pure Python, no ROS2 dependency.

Provides lightweight algorithms that run at 50–100 Hz on Raspberry Pi 5:
  - Multi-sector curvature estimation from LiDAR geometry
  - IMU-fused curvature estimation (instant, gyro-based)
  - Grip-aware speed limiting via lateral acceleration
  - Optimal speed computation with predictive braking
  - Track section classification (straight / gentle / tight / chicane)
  - Adaptive parameter tuning based on real-time conditions

Used by both ROS2 nodes and the standalone Webots controller.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from covapsy_nav.vehicle_state_estimator import VehicleState

# ---------------------------------------------------------------------------
# Sector configuration — divide front FOV into sectors for analysis
# ---------------------------------------------------------------------------
_NUM_SECTORS = 12
_SECTOR_HALF_FOV_DEG = 90.0  # analyse ±90° in front

# Curvature estimation
_CURVATURE_SMOOTHING = 0.35  # EMA alpha — lower = faster response to track changes
_MIN_AVG_RANGE = 0.10

# Speed profiles — wide bands: only truly tight corners slow down
_STRAIGHT_THRESHOLD = 0.22        # curvature below this = straight
_GENTLE_CURVE_THRESHOLD = 0.55    # curvature below this = gentle curve
_TIGHT_CURVE_THRESHOLD = 0.82     # above this = tight curve

# Speed factors per section type — keep speed high through turns
_SPEED_FACTOR_STRAIGHT = 1.00
_SPEED_FACTOR_GENTLE = 0.96
_SPEED_FACTOR_TIGHT = 0.82
_SPEED_FACTOR_CHICANE = 0.70

# Predictive braking
_BRAKE_LOOKAHEAD_SECTORS = 3     # how many sectors ahead to check
_BRAKE_ANTICIPATION = 0.14       # light pre-braking — trust the section factor

# Acceleration boost
_ACCEL_BOOST_FACTOR = 1.30       # speed multiplier on exits of corners
_ACCEL_BOOST_MIN_CLEARANCE = 0.9 # metres ahead needed to boost

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
    ttc_target_sec: float = 0.70,
    speed_ramp_up: float = 0.35,
    speed_ramp_down: float = 0.25,
) -> float:
    """Compute AI-optimised speed combining multiple factors.

    Returns the target speed in m/s.
    """
    section = classify_section(curvature)
    base_factor = section_speed_factor(section)

    # Steering penalty — very mild, the section factor already handles turn severity
    steer_ratio = _clamp(abs(steering_rad) / max(max_steering, 1e-6), 0.0, 1.0)
    steer_penalty = 1.0 - 0.18 * steer_ratio  # minimal — curvature does the heavy lifting

    # Clearance factor — ramp to 1.0 quickly so open track ≈ full speed
    clearance_factor = _clamp((projected_clearance - 0.06) / 0.9, 0.0, 1.0)

    # Passage width factor — be bolder in wide passages
    width_factor = _clamp(
        (passage_width - _NARROW_PASSAGE_M) / (_WIDE_PASSAGE_M - _NARROW_PASSAGE_M),
        0.0, 1.0,
    )
    width_boost = 1.0 + 0.22 * width_factor  # up to 22% faster in wide sections

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

    # TTC guard — only intervene when collision is truly imminent
    if raw_speed > 0.10:
        ttc = ttc_clearance / max(raw_speed, 0.10)
        if ttc < max(ttc_target_sec, 0.15):
            raw_speed = min(raw_speed, ttc_clearance / max(ttc_target_sec, 0.15))

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


# ---------------------------------------------------------------------------
# IMU-fused curvature estimation (FAST PATH — 100 Hz via gyro)
# ---------------------------------------------------------------------------
_IMU_LIDAR_CURVATURE_BLEND = 0.30   # IMU weight when both available
_GRIP_LIMIT_G = 0.65                # max lateral g before limiting speed
_GRIP_MARGIN_G = 0.15               # soft zone before hard limit
_GRIP_SPEED_FLOOR = 0.55            # minimum fraction of max_speed under grip limit
_YAW_RATE_CURVATURE_SCALE = 2.8     # scale factor: yaw_rate → normalised curvature

# Predictive speed via longitudinal acceleration
_LON_ACCEL_BRAKE_THRESHOLD = -0.8   # m/s² — detect hard braking
_LON_ACCEL_BOOST_THRESHOLD = 0.5    # m/s² — detect firm acceleration
_ACCEL_PREDICTIVE_HORIZON = 0.3     # seconds to look ahead for speed prediction


def fuse_curvature_with_imu(
    lidar_curvature: float,
    vehicle_state: Optional["VehicleState"],
) -> float:
    """Fuse LiDAR curvature with IMU yaw-rate for faster, more accurate estimate.

    The IMU yaw rate responds instantly to turn entry (~5 ms latency)
    while LiDAR curvature has ~100 ms latency from scan processing.
    Blending gives fast response with LiDAR confirmation.

    Returns normalised curvature in [0..1], same scale as LiDAR.
    """
    if vehicle_state is None:
        return lidar_curvature

    imu_curvature = _clamp(
        abs(vehicle_state.yaw_rate) * _YAW_RATE_CURVATURE_SCALE,
        0.0, 1.0,
    )

    # When car is barely moving, yaw_rate is unreliable → trust LiDAR
    if vehicle_state.speed < 0.15:
        return lidar_curvature

    # Weighted blend: IMU is faster but noisier; LiDAR is lagged but geometric
    w_imu = _IMU_LIDAR_CURVATURE_BLEND
    # Increase IMU weight when it detects a curve the LiDAR hasn't seen yet
    # (imu_curvature significantly higher → we're entering a turn)
    if imu_curvature > lidar_curvature + 0.15:
        w_imu = min(0.70, w_imu + 0.25)
    # Decrease IMU weight when exiting a curve (lidar still sees walls)
    elif lidar_curvature > imu_curvature + 0.20:
        w_imu = max(0.15, w_imu - 0.10)

    fused = w_imu * imu_curvature + (1.0 - w_imu) * lidar_curvature
    return _clamp(fused, 0.0, 1.0)


def grip_limit_speed_factor(
    vehicle_state: Optional["VehicleState"],
) -> float:
    """Return [0..1] speed scaling based on lateral acceleration (grip limit).

    Uses IMU lateral acceleration to detect when tyres are near the
    grip limit. Reduces speed to prevent slides and maintain control.
    """
    if vehicle_state is None:
        return 1.0

    lat_g = vehicle_state.cornering_g
    if lat_g < (_GRIP_LIMIT_G - _GRIP_MARGIN_G):
        return 1.0  # plenty of grip
    if lat_g >= _GRIP_LIMIT_G:
        return _GRIP_SPEED_FLOOR  # at the limit

    # Smooth transition in the margin zone
    t = (lat_g - (_GRIP_LIMIT_G - _GRIP_MARGIN_G)) / max(_GRIP_MARGIN_G, 0.01)
    return 1.0 - (1.0 - _GRIP_SPEED_FLOOR) * t


def imu_predictive_speed_adjust(
    current_speed: float,
    vehicle_state: Optional["VehicleState"],
) -> float:
    """Adjust target speed using longitudinal acceleration prediction.

    If the car is accelerating, we can be slightly bolder.
    If braking hard (e.g. hitting a bump or overshoot), reduce target faster.
    """
    if vehicle_state is None:
        return current_speed

    lon_accel = vehicle_state.lon_accel
    predicted_speed = current_speed + lon_accel * _ACCEL_PREDICTIVE_HORIZON

    if lon_accel < _LON_ACCEL_BRAKE_THRESHOLD:
        # Hard braking detected — expect speed to drop, pre-adjust
        return max(0.0, min(current_speed, predicted_speed * 0.95))

    if lon_accel > _LON_ACCEL_BOOST_THRESHOLD:
        # Consistent acceleration — can trust slightly higher target
        return current_speed * 1.05

    return current_speed


def compute_optimal_speed_fused(
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
    vehicle_state: Optional["VehicleState"] = None,
    ttc_target_sec: float = 0.70,
    speed_ramp_up: float = 0.35,
    speed_ramp_down: float = 0.25,
) -> float:
    """Enhanced speed computation with IMU sensor fusion.

    When vehicle_state is provided (from VehicleStateEstimator):
      - Uses fused curvature (instant IMU + confirmed LiDAR)
      - Applies grip-limit speed reduction from lateral acceleration
      - Uses acceleration prediction for smoother speed transitions
      - Faster ramp rates when IMU confirms safe conditions

    Falls back cleanly to LiDAR-only when vehicle_state is None.
    """
    # Fuse curvature: IMU + LiDAR for faster corner detection
    fused_curvature = fuse_curvature_with_imu(curvature, vehicle_state)

    # Base speed from fused curvature
    section = classify_section(fused_curvature)
    base_factor = section_speed_factor(section)

    steer_ratio = _clamp(abs(steering_rad) / max(max_steering, 1e-6), 0.0, 1.0)
    steer_penalty = 1.0 - 0.18 * steer_ratio

    clearance_factor = _clamp((projected_clearance - 0.06) / 0.9, 0.0, 1.0)

    width_factor = _clamp(
        (passage_width - _NARROW_PASSAGE_M) / (_WIDE_PASSAGE_M - _NARROW_PASSAGE_M),
        0.0, 1.0,
    )
    width_boost = 1.0 + 0.22 * width_factor

    brake_factor = predictive_brake_factor(sector_ranges, fused_curvature)

    # Grip-limit factor from IMU lateral acceleration
    grip_factor = grip_limit_speed_factor(vehicle_state)

    accel_boost = 1.0
    if (
        section == "straight"
        and projected_clearance > _ACCEL_BOOST_MIN_CLEARANCE
        and prev_speed > 0.5
    ):
        accel_boost = _ACCEL_BOOST_FACTOR
        # Extra aggressive boost when IMU confirms we're on straight + accelerating
        if (
            vehicle_state is not None
            and vehicle_state.is_accelerating
            and vehicle_state.cornering_g < 0.15
        ):
            accel_boost = min(accel_boost * 1.08, 1.45)

    # Combine all factors
    raw_speed = max_speed * base_factor * steer_penalty * clearance_factor
    raw_speed *= width_boost * brake_factor * accel_boost * grip_factor
    raw_speed = _clamp(raw_speed, min_speed, max_speed)

    # TTC guard
    if raw_speed > 0.10:
        ttc = ttc_clearance / max(raw_speed, 0.10)
        if ttc < max(ttc_target_sec, 0.15):
            raw_speed = min(raw_speed, ttc_clearance / max(ttc_target_sec, 0.15))

    if projected_clearance > 0.35:
        raw_speed = max(min_speed, raw_speed)
    raw_speed = _clamp(raw_speed, 0.0, max_speed)

    # Acceleration-predictive refinement
    raw_speed = imu_predictive_speed_adjust(raw_speed, vehicle_state)
    raw_speed = _clamp(raw_speed, 0.0, max_speed)

    # Adaptive ramp rates: faster when IMU confirms stable conditions
    effective_ramp_up = speed_ramp_up
    effective_ramp_down = speed_ramp_down
    if vehicle_state is not None:
        # On straight with low lateral g → can accelerate faster
        if vehicle_state.cornering_g < 0.20 and section == "straight":
            effective_ramp_up = speed_ramp_up * 1.4
        # When grip is high and braking → allow faster deceleration response
        if vehicle_state.is_braking:
            effective_ramp_down = speed_ramp_down * 1.3

    if prev_speed > 0.05:
        if raw_speed > prev_speed:
            speed = min(raw_speed, prev_speed + effective_ramp_up)
        else:
            speed = max(raw_speed, prev_speed - effective_ramp_down)
    else:
        speed = raw_speed

    return _clamp(speed, 0.0, max_speed)
