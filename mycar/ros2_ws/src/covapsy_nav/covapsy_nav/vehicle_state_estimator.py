"""Lightweight Extended Kalman Filter for vehicle state estimation.

Fuses IMU (yaw rate, lateral/longitudinal acceleration), wheel encoder
(speed), and LiDAR curvature into a coherent vehicle state at up to
100 Hz — an order of magnitude faster than LiDAR-only estimation.

State vector: [yaw, speed, yaw_rate, lat_accel, lon_accel]
  - yaw        : heading angle (rad)
  - speed      : longitudinal speed (m/s)
  - yaw_rate   : heading rate from gyro (rad/s)
  - lat_accel  : lateral acceleration (m/s²),  positive = left
  - lon_accel  : longitudinal acceleration (m/s²)

Pure Python + numpy.  No ROS2 dependency.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# ── Default noise parameters (tuned for BNO055 + TT-02 at 1/10 scale) ──

# Process noise — tuned for responsive state tracking on TT-02.
# Tighter than before: trust sensors more, respond faster to changes.
_Q_YAW = 0.005           # rad²/s
_Q_SPEED = 0.30          # (m/s)²/s — increased: speed changes fast on track
_Q_YAW_RATE = 1.2        # (rad/s)²/s — increased: allow fast yaw-rate changes
_Q_LAT_ACCEL = 3.0       # (m/s²)²/s — increased: lateral accel changes rapidly in corners
_Q_LON_ACCEL = 3.0       # (m/s²)²/s — increased: hard braking/acceleration events

# Measurement noise — trust BNO055 and wheel encoder
_R_GYRO_YAW_RATE = 0.008  # (rad/s)² — BNO055 NDOF mode is very good
_R_ACCEL_LAT = 0.10        # (m/s²)² — tighter: trust accelerometer more
_R_ACCEL_LON = 0.10        # (m/s²)²
_R_WHEEL_SPEED = 0.03      # (m/s)² — tighter: optical encoder is reliable
_R_LIDAR_YAW_RATE = 0.25   # (rad/s)² — LiDAR curvature is noisy

# State dimension
_N = 5

# Indices
_IDX_YAW = 0
_IDX_SPEED = 1
_IDX_YAW_RATE = 2
_IDX_LAT_ACCEL = 3
_IDX_LON_ACCEL = 4


@dataclass
class VehicleState:
    """Snapshot of the estimated vehicle state."""
    yaw: float = 0.0               # rad
    speed: float = 0.0             # m/s
    yaw_rate: float = 0.0          # rad/s (positive = turning left)
    lat_accel: float = 0.0         # m/s²  (positive = turning left)
    lon_accel: float = 0.0         # m/s²  (positive = accelerating)
    timestamp: float = 0.0

    @property
    def curvature(self) -> float:
        """Instantaneous curvature (1/m).  Positive = left turn."""
        if abs(self.speed) < 0.05:
            return 0.0
        return self.yaw_rate / self.speed

    @property
    def abs_curvature_normalised(self) -> float:
        """Normalised |curvature| in [0..1] range, comparable to LiDAR estimate."""
        # At 2 m/s and 0.35 rad/s yaw rate → curvature = 0.175 1/m
        # Map to 0..1 range: curvature 0 → 0, curvature ≥ 1.0 1/m → 1
        return min(abs(self.curvature) / 1.0, 1.0)

    @property
    def cornering_g(self) -> float:
        """Lateral acceleration in g-units for grip estimation."""
        return abs(self.lat_accel) / 9.81

    @property
    def is_braking(self) -> bool:
        return self.lon_accel < -0.3

    @property
    def is_accelerating(self) -> bool:
        return self.lon_accel > 0.3

    @property
    def turning_left(self) -> bool:
        return self.yaw_rate > 0.05

    @property
    def turning_right(self) -> bool:
        return self.yaw_rate < -0.05


class VehicleStateEstimator:
    """EKF fusing IMU + wheel speed + LiDAR curvature."""

    def __init__(self) -> None:
        self._x = np.zeros(_N)                     # state vector
        self._P = np.eye(_N) * 0.5                 # covariance
        self._last_time: float = 0.0
        self._initialised = False

        # Pre-compute noise matrices (diagonal)
        self._Q_per_sec = np.diag([
            _Q_YAW, _Q_SPEED, _Q_YAW_RATE, _Q_LAT_ACCEL, _Q_LON_ACCEL,
        ])

    def reset(self) -> None:
        self._x[:] = 0.0
        self._P = np.eye(_N) * 0.5
        self._last_time = 0.0
        self._initialised = False

    @property
    def state(self) -> VehicleState:
        return VehicleState(
            yaw=float(self._x[_IDX_YAW]),
            speed=float(self._x[_IDX_SPEED]),
            yaw_rate=float(self._x[_IDX_YAW_RATE]),
            lat_accel=float(self._x[_IDX_LAT_ACCEL]),
            lon_accel=float(self._x[_IDX_LON_ACCEL]),
            timestamp=self._last_time,
        )

    # ──────────────────────────────────────────────────────────────────
    # Predict step — called at every control tick (50–100 Hz)
    # ──────────────────────────────────────────────────────────────────

    def predict(self, timestamp: Optional[float] = None) -> VehicleState:
        now = timestamp if timestamp is not None else time.monotonic()
        if not self._initialised:
            self._last_time = now
            self._initialised = True
            return self.state

        dt = max(0.001, min(now - self._last_time, 0.5))
        self._last_time = now

        # State transition: simple kinematic model
        # yaw += yaw_rate * dt
        # speed += lon_accel * dt
        # yaw_rate, lat_accel, lon_accel assumed constant (random walk)
        self._x[_IDX_YAW] += self._x[_IDX_YAW_RATE] * dt
        self._x[_IDX_SPEED] += self._x[_IDX_LON_ACCEL] * dt
        self._x[_IDX_SPEED] = max(0.0, self._x[_IDX_SPEED])  # speed ≥ 0

        # Jacobian of state transition
        F = np.eye(_N)
        F[_IDX_YAW, _IDX_YAW_RATE] = dt
        F[_IDX_SPEED, _IDX_LON_ACCEL] = dt

        # Propagate covariance
        Q = self._Q_per_sec * dt
        self._P = F @ self._P @ F.T + Q

        return self.state

    # ──────────────────────────────────────────────────────────────────
    # Update steps — one per sensor measurement
    # ──────────────────────────────────────────────────────────────────

    def update_imu(
        self,
        yaw_rate: float,
        lat_accel: float,
        lon_accel: float,
        timestamp: Optional[float] = None,
    ) -> VehicleState:
        """Fuse IMU gyro (yaw rate) + accelerometer readings.

        Call at IMU rate (~100 Hz on BNO055).
        """
        self.predict(timestamp)

        # Measurement: z = [yaw_rate, lat_accel, lon_accel]
        z = np.array([yaw_rate, lat_accel, lon_accel])

        # Observation matrix: direct observation of states 2, 3, 4
        H = np.zeros((3, _N))
        H[0, _IDX_YAW_RATE] = 1.0
        H[1, _IDX_LAT_ACCEL] = 1.0
        H[2, _IDX_LON_ACCEL] = 1.0

        R = np.diag([_R_GYRO_YAW_RATE, _R_ACCEL_LAT, _R_ACCEL_LON])

        self._kf_update(z, H, R)
        return self.state

    def update_wheel_speed(
        self,
        speed_m_s: float,
        timestamp: Optional[float] = None,
    ) -> VehicleState:
        """Fuse wheel encoder speed.

        Call when new wheel speed is available (~50 Hz).
        """
        self.predict(timestamp)

        z = np.array([speed_m_s])
        H = np.zeros((1, _N))
        H[0, _IDX_SPEED] = 1.0
        R = np.array([[_R_WHEEL_SPEED]])

        self._kf_update(z, H, R)
        return self.state

    def update_lidar_curvature(
        self,
        lidar_curvature: float,
        timestamp: Optional[float] = None,
    ) -> VehicleState:
        """Fuse LiDAR-derived curvature as a yaw-rate observation.

        curvature ≈ yaw_rate / speed → yaw_rate ≈ curvature * speed.
        Only useful when speed > threshold.
        """
        speed = max(abs(self._x[_IDX_SPEED]), 0.1)
        implied_yaw_rate = lidar_curvature * speed

        self.predict(timestamp)

        z = np.array([implied_yaw_rate])
        H = np.zeros((1, _N))
        H[0, _IDX_YAW_RATE] = 1.0
        R = np.array([[_R_LIDAR_YAW_RATE]])

        self._kf_update(z, H, R)
        return self.state

    def update_yaw(
        self,
        yaw_rad: float,
        timestamp: Optional[float] = None,
    ) -> VehicleState:
        """Fuse absolute yaw from GPS/IMU fusion or SLAM.

        Handles wrap-around via innovation on shortest angular path.
        """
        self.predict(timestamp)

        # Innovation with angle wrapping
        innovation = yaw_rad - self._x[_IDX_YAW]
        innovation = (innovation + math.pi) % (2.0 * math.pi) - math.pi

        H = np.zeros((1, _N))
        H[0, _IDX_YAW] = 1.0
        R = np.array([[0.01]])  # reasonably tight from BNO055 absolute heading

        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x += (K @ np.array([innovation])).flatten()
        I_KH = np.eye(_N) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
        return self.state

    # ──────────────────────────────────────────────────────────────────
    # Internal KF update
    # ──────────────────────────────────────────────────────────────────

    def _kf_update(
        self,
        z: np.ndarray,
        H: np.ndarray,
        R: np.ndarray,
    ) -> None:
        innovation = z - H @ self._x
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x += (K @ innovation).flatten()
        self._x[_IDX_SPEED] = max(0.0, self._x[_IDX_SPEED])  # enforce speed ≥ 0

        I_KH = np.eye(_N) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
