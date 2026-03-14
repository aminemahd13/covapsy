"""Python wrapper for the C reactive driving core, with pure-Python fallback.

Loads libreactive_core.so via ctypes for deterministic, GC-free execution.
Falls back to an equivalent numpy implementation if the .so is missing.
"""

from __future__ import annotations

import ctypes
import math
import os
from typing import Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from covapsy_nav.vehicle_state_estimator import VehicleState

# ── Try to load the C library ──

_LIB = None
_C_AVAILABLE = False

_SO_SEARCH_PATHS = [
    os.path.join(os.path.dirname(__file__), "libreactive_core.so"),
    os.path.join(os.path.dirname(__file__), "..", "c_src", "libreactive_core.so"),
]

for _path in _SO_SEARCH_PATHS:
    if os.path.isfile(_path):
        try:
            _LIB = ctypes.CDLL(_path)
            _C_AVAILABLE = True
            break
        except OSError:
            pass


# ── C struct mirrors ──

class _ReactiveParams(ctypes.Structure):
    _fields_ = [
        ("car_width", ctypes.c_float),
        ("max_speed", ctypes.c_float),
        ("min_speed", ctypes.c_float),
        ("max_range", ctypes.c_float),
        ("max_steering", ctypes.c_float),
        ("fov_deg", ctypes.c_float),
        ("prev_steering", ctypes.c_float),
        ("slew_rate", ctypes.c_float),
        ("ttc_target", ctypes.c_float),
        ("depth_front", ctypes.c_float),
        ("camera_offset", ctypes.c_float),
        ("camera_gain", ctypes.c_float),
        ("imu_yaw_rate", ctypes.c_float),
        ("imu_speed", ctypes.c_float),
    ]


class _ReactiveResult(ctypes.Structure):
    _fields_ = [
        ("speed", ctypes.c_float),
        ("steering", ctypes.c_float),
        ("lateral_error", ctypes.c_float),
        ("forward_clearance", ctypes.c_float),
        ("corridor_width", ctypes.c_float),
    ]


if _C_AVAILABLE:
    _LIB.compute_reactive.argtypes = [
        ctypes.POINTER(ctypes.c_float),  # ranges
        ctypes.POINTER(ctypes.c_float),  # angles
        ctypes.c_int,                    # n
        ctypes.POINTER(_ReactiveParams), # params
        ctypes.POINTER(_ReactiveResult), # out
    ]
    _LIB.compute_reactive.restype = None


# ── Public API ──

def compute_reactive_drive(
    ranges: np.ndarray,
    angles: np.ndarray,
    car_width: float,
    max_speed: float,
    min_speed: float,
    max_range: float,
    max_steering: float,
    fov_degrees: float,
    prev_steering: float,
    slew_rate: float,
    ttc_target: float,
    depth_front: float,
    camera_offset: float,
    camera_gain: float,
    imu_yaw_rate: float,
    imu_speed: float,
) -> tuple[float, float]:
    """Compute (speed, steering) using corridor-following reactive control.

    Uses the C library if available, otherwise falls back to numpy.
    """
    if _C_AVAILABLE:
        return _compute_c(
            ranges, angles, car_width, max_speed, min_speed, max_range,
            max_steering, fov_degrees, prev_steering, slew_rate,
            ttc_target, depth_front, camera_offset, camera_gain,
            imu_yaw_rate, imu_speed,
        )
    return _compute_python(
        ranges, angles, car_width, max_speed, min_speed, max_range,
        max_steering, fov_degrees, prev_steering, slew_rate,
        ttc_target, depth_front, camera_offset, camera_gain,
        imu_yaw_rate, imu_speed,
    )


def is_c_available() -> bool:
    """Return True if the C library is loaded."""
    return _C_AVAILABLE


# ── C implementation ──

def _compute_c(
    ranges, angles, car_width, max_speed, min_speed, max_range,
    max_steering, fov_degrees, prev_steering, slew_rate,
    ttc_target, depth_front, camera_offset, camera_gain,
    imu_yaw_rate, imu_speed,
) -> tuple[float, float]:
    r = np.ascontiguousarray(ranges, dtype=np.float32)
    a = np.ascontiguousarray(angles, dtype=np.float32)
    n = len(r)

    params = _ReactiveParams(
        car_width=car_width,
        max_speed=max_speed,
        min_speed=min_speed,
        max_range=max_range,
        max_steering=max_steering,
        fov_deg=fov_degrees,
        prev_steering=prev_steering,
        slew_rate=slew_rate,
        ttc_target=ttc_target,
        depth_front=depth_front if math.isfinite(depth_front) else -1.0,
        camera_offset=camera_offset,
        camera_gain=camera_gain,
        imu_yaw_rate=imu_yaw_rate,
        imu_speed=imu_speed,
    )
    result = _ReactiveResult()

    _LIB.compute_reactive(
        r.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        a.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        ctypes.c_int(n),
        ctypes.byref(params),
        ctypes.byref(result),
    )

    return float(result.speed), float(result.steering)


# ── Pure Python fallback (mirrors C algorithm exactly) ──

# Constants matching the C implementation
_K_CENTER = 0.80
_K_HEADING = 0.50
_K_REPULSION = 0.90
_K_IMU_DAMP = 0.10
_REPULSION_DIST_FACTOR = 1.5
_CLEARANCE_REF_M = 1.80
_CURV_SPEED_PEN = 0.35
_STEER_SPEED_PEN = 0.25
_EMERGENCY_STOP_M = 0.15
_CREEP_CLEAR_M = 0.35
_CLOSE_WALL_FACTOR = 2.0
_W_CENTER_CLOSE = 0.75
_W_CENTER_OPEN = 0.40
_W_HEADING_CLOSE = 0.25
_W_HEADING_OPEN = 0.60
_FWD_CONE_RAD = math.radians(15.0)
_STEER_CONE_RAD = math.radians(18.0)
_PAIR_HALF_WIDTH = math.radians(6.0)
_N_PAIRS = 9
_HEADING_SCALE = 0.70
_MIN_USEFUL_RANGE = 0.30


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def _compute_python(
    ranges_raw, angles_raw, car_width, max_speed, min_speed, max_range,
    max_steering, fov_degrees, prev_steering, slew_rate,
    ttc_target, depth_front, camera_offset, camera_gain,
    imu_yaw_rate, imu_speed,
) -> tuple[float, float]:
    # Preprocess
    r = np.array(ranges_raw, dtype=np.float64)
    a = np.array(angles_raw, dtype=np.float64)
    n = len(r)
    if n < 10:
        return 0.0, 0.0

    r = np.where(np.isfinite(r), r, max_range)
    r = np.clip(r, 0.0, max_range)

    fov_half = math.radians(fov_degrees) / 2
    fov_mask = np.abs(a) <= fov_half
    r = r[fov_mask]
    a = a[fov_mask]

    if len(r) < 10:
        return 0.0, 0.0

    # ── Corridor model ──
    max_pair_angle = fov_half * 0.85
    pair_angles = np.array([
        max_pair_angle * (k + 1) / _N_PAIRS for k in range(_N_PAIRS)
    ])

    left_dist = np.full(_N_PAIRS, max_range)
    right_dist = np.full(_N_PAIRS, max_range)

    for k in range(_N_PAIRS):
        alpha = pair_angles[k]
        left_mask = np.abs(a - alpha) <= _PAIR_HALF_WIDTH
        right_mask = np.abs(a + alpha) <= _PAIR_HALF_WIDTH
        if np.any(left_mask):
            left_dist[k] = float(np.min(r[left_mask]))
        if np.any(right_mask):
            right_dist[k] = float(np.min(r[right_mask]))

    weights = np.cos(pair_angles)
    balance_num = float(np.sum(weights * (left_dist - right_dist)))
    balance_den = float(np.sum(weights * (left_dist + right_dist)))
    lateral_error = _clamp(balance_num / max(balance_den, 0.3), -1.0, 1.0)
    min_wall = min(float(np.min(left_dist)), float(np.min(right_dist)))

    # ── Steering ──

    # Corridor centering
    steer_center = _K_CENTER * lateral_error * max_steering

    # Best heading direction
    useful = r > _MIN_USEFUL_RANGE
    if np.any(useful):
        scores = r[useful] * np.cos(a[useful] * _HEADING_SCALE)
        scores = np.maximum(scores, 0.01)
        best_heading = float(np.average(a[useful], weights=scores))
    else:
        best_heading = 0.0
    steer_heading = _K_HEADING * best_heading

    # Blend
    close_to_wall = min_wall < car_width * _CLOSE_WALL_FACTOR
    if close_to_wall:
        w_c, w_h = _W_CENTER_CLOSE, _W_HEADING_CLOSE
    else:
        w_c, w_h = _W_CENTER_OPEN, _W_HEADING_OPEN
    steering = (w_c * steer_center + w_h * steer_heading) / (w_c + w_h)

    # Obstacle repulsion
    danger_dist = car_width * _REPULSION_DIST_FACTOR
    danger_mask = r < danger_dist
    if np.any(danger_mask):
        force = (danger_dist - r[danger_mask]) / danger_dist
        force = force ** 2
        steering -= float(np.sum(force * np.sin(a[danger_mask]))) * max_steering * _K_REPULSION

    # Camera offset
    if abs(camera_offset) > 0.01 and math.isfinite(camera_offset):
        steering += camera_gain * camera_offset

    # IMU yaw damping
    if abs(imu_yaw_rate) > 0.05:
        steering -= _K_IMU_DAMP * imu_yaw_rate

    # Clamp + slew
    steering = _clamp(steering, -max_steering, max_steering)
    steering = _clamp(steering, prev_steering - slew_rate, prev_steering + slew_rate)

    # ── Speed ──

    # Forward clearance
    steer_mask = np.abs(a - steering) <= _STEER_CONE_RAD
    straight_mask = np.abs(a) <= _FWD_CONE_RAD
    steer_clear = float(np.min(r[steer_mask])) if np.any(steer_mask) else max_range
    straight_clear = float(np.min(r[straight_mask])) if np.any(straight_mask) else max_range
    effective_clear = min(steer_clear, straight_clear)

    if math.isfinite(depth_front) and depth_front > 0:
        effective_clear = min(effective_clear, depth_front)

    # Curvature
    curvature = abs(lateral_error)
    if abs(imu_speed) > 0.15:
        imu_curv = _clamp(abs(imu_yaw_rate) * 2.5, 0.0, 1.0)
        curvature = max(curvature, imu_curv * 0.6)

    # Speed factors
    clearance_factor = _clamp(effective_clear / _CLEARANCE_REF_M, 0.0, 1.0)
    curvature_factor = 1.0 - _CURV_SPEED_PEN * curvature
    steer_factor = 1.0 - _STEER_SPEED_PEN * abs(steering) / max(max_steering, 0.01)

    speed = max_speed * clearance_factor * curvature_factor * steer_factor
    speed = _clamp(speed, 0.0, max_speed)

    # TTC
    ttc_t = max(ttc_target, 0.25)
    if speed > 0.05:
        ttc = effective_clear / speed
        if ttc < ttc_t:
            speed = min(speed, effective_clear / ttc_t)

    # Floor / emergency
    if effective_clear > _CREEP_CLEAR_M:
        speed = max(min_speed, speed)
    elif effective_clear < _EMERGENCY_STOP_M:
        speed = 0.0

    speed = _clamp(speed, 0.0, max_speed)
    return float(speed), float(steering)
