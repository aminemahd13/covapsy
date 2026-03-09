"""Tests for IMU-enhanced racing intelligence functions."""

import math
import pytest

from covapsy_nav.racing_intelligence import (
    fuse_curvature_with_imu,
    grip_limit_speed_factor,
    imu_predictive_speed_adjust,
    compute_optimal_speed_fused,
    extract_sector_ranges,
    estimate_curvature,
)
from covapsy_nav.vehicle_state_estimator import VehicleState


class TestFuseCurvatureWithIMU:
    """Test fused curvature (IMU + LiDAR)."""

    def test_no_vehicle_state_returns_lidar(self):
        assert fuse_curvature_with_imu(0.5, None) == 0.5

    def test_low_speed_returns_lidar(self):
        s = VehicleState(speed=0.01, yaw_rate=1.0)
        assert fuse_curvature_with_imu(0.3, s) == 0.3

    def test_imu_detects_turn_earlier(self):
        # IMU sees high yaw rate but LiDAR hasn't updated yet (still straight)
        s = VehicleState(speed=2.0, yaw_rate=0.3)
        fused = fuse_curvature_with_imu(0.1, s)
        # Fused should be higher than LiDAR-only
        assert fused > 0.1

    def test_both_agree_on_straight(self):
        s = VehicleState(speed=2.0, yaw_rate=0.01)
        fused = fuse_curvature_with_imu(0.05, s)
        assert fused < 0.15  # still classified as straight

    def test_result_clamped_to_unit(self):
        s = VehicleState(speed=2.0, yaw_rate=5.0)
        fused = fuse_curvature_with_imu(0.9, s)
        assert 0.0 <= fused <= 1.0


class TestGripLimitSpeedFactor:
    """Test lateral-acceleration grip limiting."""

    def test_no_vehicle_state(self):
        assert grip_limit_speed_factor(None) == 1.0

    def test_low_lateral_g(self):
        s = VehicleState(lat_accel=1.0)  # ~0.10 g
        assert grip_limit_speed_factor(s) == 1.0

    def test_at_grip_limit(self):
        s = VehicleState(lat_accel=0.65 * 9.81)  # exactly at limit
        f = grip_limit_speed_factor(s)
        assert f < 1.0  # should be reducing speed
        assert f >= 0.55  # but not below floor

    def test_beyond_grip_limit(self):
        s = VehicleState(lat_accel=1.0 * 9.81)  # way over
        f = grip_limit_speed_factor(s)
        assert f == 0.55  # floor value


class TestIMUPredictiveSpeedAdjust:
    """Test acceleration-predictive speed refinement."""

    def test_no_state(self):
        assert imu_predictive_speed_adjust(2.0, None) == 2.0

    def test_hard_braking_reduces(self):
        s = VehicleState(lon_accel=-2.0)
        result = imu_predictive_speed_adjust(2.0, s)
        assert result < 2.0

    def test_accelerating_boosts(self):
        s = VehicleState(lon_accel=1.0)
        result = imu_predictive_speed_adjust(2.0, s)
        assert result > 2.0

    def test_neutral_unchanged(self):
        s = VehicleState(lon_accel=0.1)
        result = imu_predictive_speed_adjust(2.0, s)
        assert result == 2.0


class TestComputeOptimalSpeedFused:
    """Test the full fused speed computation."""

    def _straight_ranges(self):
        return [4.0] * 12

    def _curve_ranges(self):
        return [1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 4.0, 3.0, 2.0, 1.5, 1.2, 1.0]

    def test_fused_without_state_works(self):
        speed = compute_optimal_speed_fused(
            max_speed=2.5, min_speed=0.3, steering_rad=0.0,
            max_steering=0.5, projected_clearance=3.0,
            passage_width=2.0, curvature=0.1, prev_speed=1.5,
            sector_ranges=self._straight_ranges(),
            ttc_clearance=3.0, vehicle_state=None,
        )
        assert 0.0 <= speed <= 2.5

    def test_fused_with_straight_imu(self):
        s = VehicleState(speed=2.0, yaw_rate=0.01, lat_accel=0.1, lon_accel=0.5)
        speed = compute_optimal_speed_fused(
            max_speed=2.5, min_speed=0.3, steering_rad=0.0,
            max_steering=0.5, projected_clearance=3.0,
            passage_width=2.0, curvature=0.05, prev_speed=1.8,
            sector_ranges=self._straight_ranges(),
            ttc_clearance=3.0, vehicle_state=s,
        )
        # On straight with good clearance → should be near max
        assert speed > 1.5

    def test_fused_grip_limited_in_turn(self):
        s_no_grip = VehicleState(
            speed=2.0, yaw_rate=0.5, lat_accel=0.6 * 9.81, lon_accel=0.0,
        )
        s_normal = VehicleState(
            speed=2.0, yaw_rate=0.5, lat_accel=1.0, lon_accel=0.0,
        )
        speed_limited = compute_optimal_speed_fused(
            max_speed=2.5, min_speed=0.3, steering_rad=0.3,
            max_steering=0.5, projected_clearance=2.0,
            passage_width=1.5, curvature=0.6, prev_speed=2.0,
            sector_ranges=self._curve_ranges(),
            ttc_clearance=2.0, vehicle_state=s_no_grip,
        )
        speed_normal = compute_optimal_speed_fused(
            max_speed=2.5, min_speed=0.3, steering_rad=0.3,
            max_steering=0.5, projected_clearance=2.0,
            passage_width=1.5, curvature=0.6, prev_speed=2.0,
            sector_ranges=self._curve_ranges(),
            ttc_clearance=2.0, vehicle_state=s_normal,
        )
        # High lateral g → grip limit → lower speed
        assert speed_limited < speed_normal

    def test_output_always_in_range(self):
        s = VehicleState(speed=1.0, yaw_rate=0.3, lat_accel=2.0, lon_accel=-1.0)
        speed = compute_optimal_speed_fused(
            max_speed=2.5, min_speed=0.3, steering_rad=0.2,
            max_steering=0.5, projected_clearance=1.0,
            passage_width=1.0, curvature=0.5, prev_speed=1.0,
            sector_ranges=self._curve_ranges(),
            ttc_clearance=1.0, vehicle_state=s,
        )
        assert 0.0 <= speed <= 2.5
