"""Reactive driving logic for COVAPSY — corridor-following algorithm.

Replaces the old gap-finding approach with a corridor model + potential field.
The car no longer gets stuck because:
  - No safety bubble that eliminates valid paths in narrow corridors
  - Continuous steering output (no discrete gap selection → no zigzag)
  - Direct IMU yaw-rate damping instead of 4-layer post-processing filters
  - Responsive obstacle repulsion for emergency avoidance

See reactive_core.py (and the C implementation in c_src/) for details.
"""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

import numpy as np

from covapsy_nav.reactive_core import compute_reactive_drive, is_c_available

if TYPE_CHECKING:
    from covapsy_nav.vehicle_state_estimator import VehicleState


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
    ttc_target_sec: float = 0.70,
    use_ai_speed: bool = True,
    depth_front_dist: float = float("inf"),
    vehicle_state: Optional["VehicleState"] = None,
    camera_offset: float = 0.0,
    use_close_far_fusion: bool = True,
    far_center_gain: float = 0.35,
    camera_center_gain: float = 0.25,
    far_weight_min: float = 0.10,
    far_weight_max: float = 0.55,
    fusion_clearance_ref_m: float = 1.8,
) -> tuple[float, float]:
    """Compute (speed_m_s, steering_rad) from filtered scan data.

    Interface-compatible with the old gap-following algorithm.
    Internally uses the new corridor-following reactive controller.
    """
    ranges = np.array(ranges_in, dtype=np.float64)
    n = len(ranges)
    if n == 0:
        return 0.0, 0.0

    angles = angle_min + np.arange(n) * angle_increment

    # Extract IMU data from vehicle state if available
    imu_yaw_rate = 0.0
    imu_speed = 0.0
    if vehicle_state is not None:
        imu_yaw_rate = float(vehicle_state.yaw_rate)
        imu_speed = float(vehicle_state.speed)

    return compute_reactive_drive(
        ranges=ranges,
        angles=angles,
        car_width=car_width,
        max_speed=max_speed,
        min_speed=min_speed,
        max_range=max_range,
        max_steering=max_steering,
        fov_degrees=fov_degrees,
        prev_steering=prev_steering,
        slew_rate=steering_slew_rate,
        ttc_target=ttc_target_sec,
        depth_front=depth_front_dist,
        camera_offset=camera_offset,
        camera_gain=camera_center_gain,
        imu_yaw_rate=imu_yaw_rate,
        imu_speed=imu_speed,
    )
