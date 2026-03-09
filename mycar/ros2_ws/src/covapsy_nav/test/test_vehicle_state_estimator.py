"""Tests for VehicleStateEstimator (EKF sensor fusion)."""

import math
import pytest
import numpy as np
from covapsy_nav.vehicle_state_estimator import VehicleStateEstimator, VehicleState


class TestVehicleStateBasics:
    """Test basic state estimator lifecycle."""

    def test_initial_state_is_zero(self):
        est = VehicleStateEstimator()
        s = est.state
        assert s.yaw == 0.0
        assert s.speed == 0.0
        assert s.yaw_rate == 0.0

    def test_predict_without_update_stays_near_zero(self):
        est = VehicleStateEstimator()
        s = est.predict(timestamp=0.0)
        s = est.predict(timestamp=0.02)
        assert abs(s.speed) < 0.1
        assert abs(s.yaw) < 0.1

    def test_reset_clears_state(self):
        est = VehicleStateEstimator()
        est.update_wheel_speed(1.5, timestamp=0.0)
        est.reset()
        assert est.state.speed == 0.0


class TestIMUUpdate:
    """Test IMU measurement fusion."""

    def test_yaw_rate_converges(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(20):
            est.update_imu(
                yaw_rate=0.5, lat_accel=0.0, lon_accel=0.0,
                timestamp=0.01 * (i + 1),
            )
        s = est.state
        assert abs(s.yaw_rate - 0.5) < 0.15

    def test_lon_accel_increases_speed(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(50):
            est.update_imu(
                yaw_rate=0.0, lat_accel=0.0, lon_accel=2.0,
                timestamp=0.02 * (i + 1),
            )
        s = est.state
        # After 50 × 0.02s = 1s of 2 m/s² accel → speed should be near 2 m/s
        assert s.speed > 1.0

    def test_lat_accel_tracked(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(20):
            est.update_imu(
                yaw_rate=0.0, lat_accel=3.0, lon_accel=0.0,
                timestamp=0.01 * (i + 1),
            )
        assert abs(est.state.lat_accel - 3.0) < 1.5


class TestWheelSpeedUpdate:
    """Test wheel speed measurement fusion."""

    def test_speed_converges_to_measurement(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(30):
            est.update_wheel_speed(1.8, timestamp=0.02 * (i + 1))
        assert abs(est.state.speed - 1.8) < 0.3

    def test_speed_stays_non_negative(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        est.update_wheel_speed(0.0, timestamp=0.1)
        est.update_imu(0.0, 0.0, -5.0, timestamp=0.2)
        est.predict(timestamp=0.5)
        assert est.state.speed >= 0.0


class TestYawUpdate:
    """Test absolute yaw update with wrapping."""

    def test_yaw_converges(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(20):
            est.update_yaw(math.radians(45), timestamp=0.02 * (i + 1))
        assert abs(est.state.yaw - math.radians(45)) < math.radians(10)

    def test_yaw_wrapping(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        # Yaw near +pi
        for i in range(20):
            est.update_yaw(3.0, timestamp=0.02 * (i + 1))
        # Jump to -pi (close to +pi but wrapped)
        for i in range(20):
            est.update_yaw(-3.0, timestamp=0.4 + 0.02 * (i + 1))
        # Should not jump through zero but go the short way
        s = est.state
        assert abs(s.yaw) > 2.0 or abs(s.yaw + 3.0) < 0.5


class TestLidarCurvatureUpdate:
    """Test LiDAR curvature fusion."""

    def test_curvature_adjusts_yaw_rate(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        # Set speed first
        for i in range(10):
            est.update_wheel_speed(2.0, timestamp=0.02 * (i + 1))
        # Then feed curvature
        for i in range(10):
            est.update_lidar_curvature(0.3, timestamp=0.2 + 0.02 * (i + 1))
        # yaw_rate should reflect curvature * speed ≈ 0.6 rad/s
        assert est.state.yaw_rate > 0.2


class TestVehicleStateProperties:
    """Test computed properties of VehicleState."""

    def test_curvature_at_zero_speed(self):
        s = VehicleState(speed=0.0, yaw_rate=1.0)
        assert s.curvature == 0.0

    def test_curvature_normal(self):
        s = VehicleState(speed=2.0, yaw_rate=0.4)
        assert abs(s.curvature - 0.2) < 0.01

    def test_cornering_g(self):
        s = VehicleState(lat_accel=4.905)
        assert abs(s.cornering_g - 0.5) < 0.01

    def test_is_braking(self):
        assert VehicleState(lon_accel=-1.0).is_braking
        assert not VehicleState(lon_accel=0.0).is_braking

    def test_is_accelerating(self):
        assert VehicleState(lon_accel=1.0).is_accelerating
        assert not VehicleState(lon_accel=0.0).is_accelerating

    def test_turning_direction(self):
        assert VehicleState(yaw_rate=0.2).turning_left
        assert VehicleState(yaw_rate=-0.2).turning_right
        assert not VehicleState(yaw_rate=0.01).turning_left

    def test_abs_curvature_normalised_range(self):
        s = VehicleState(speed=2.0, yaw_rate=10.0)
        assert 0.0 <= s.abs_curvature_normalised <= 1.0


class TestMultiSensorFusion:
    """Test that combining IMU + wheel + LiDAR gives better estimates."""

    def test_fused_speed_is_stable(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(50):
            t = 0.02 * (i + 1)
            # IMU says accelerating
            est.update_imu(0.0, 0.0, 1.0, timestamp=t)
            # Wheel says constant 1.5 m/s
            est.update_wheel_speed(1.5, timestamp=t + 0.005)
        # Should be near 1.5 (wheel dominates constant speed)
        assert abs(est.state.speed - 1.5) < 0.5

    def test_fused_yaw_rate_with_imu_and_lidar(self):
        est = VehicleStateEstimator()
        est.predict(timestamp=0.0)
        for i in range(10):
            est.update_wheel_speed(2.0, timestamp=0.02 * (i + 1))
        for i in range(20):
            t = 0.2 + 0.02 * (i + 1)
            est.update_imu(0.5, 0.0, 0.0, timestamp=t)
            est.update_lidar_curvature(0.25, timestamp=t + 0.005)
        # Both IMU (0.5) and LiDAR (0.25 * 2.0 = 0.5) agree → converge fast
        assert abs(est.state.yaw_rate - 0.5) < 0.2
