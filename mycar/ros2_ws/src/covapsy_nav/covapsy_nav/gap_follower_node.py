"""Follow-the-Gap node for reactive obstacle avoidance."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32

from covapsy_nav.gap_utils import compute_gap_command
from covapsy_nav.race_profiles import resolve_profile_speed_cap
from covapsy_nav.vehicle_state_estimator import VehicleStateEstimator


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


class GapFollowerNode(Node):
    """Reactive follow-the-gap controller with disparity extension and AI speed."""

    def __init__(self):
        super().__init__("gap_follower")

        self.declare_parameter("car_width", 0.30)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("safety_radius", 0.20)
        self.declare_parameter("disparity_threshold", 0.5)
        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("fov_degrees", 200.0)
        self.declare_parameter("race_profile", "RACE_STABLE")
        self.declare_parameter("deployment_mode", "real")
        self.declare_parameter("max_speed_real_cap", 2.0)
        self.declare_parameter("max_speed_sim_cap", 2.5)
        self.declare_parameter("steering_slew_rate", 0.18)
        self.declare_parameter("max_steering", 0.5)
        self.declare_parameter("ttc_target_sec", 0.70)
        self.declare_parameter("use_ai_speed", True)
        self.declare_parameter("use_imu_fusion", True)
        self.declare_parameter("use_close_far_fusion", True)
        self.declare_parameter("far_center_gain", 0.35)
        self.declare_parameter("camera_center_gain", 0.25)
        self.declare_parameter("far_weight_min", 0.10)
        self.declare_parameter("far_weight_max", 0.55)
        self.declare_parameter("fusion_clearance_ref_m", 1.8)
        self.declare_parameter("steering_low_pass_alpha", 0.70)
        self.declare_parameter("speed_slew_up", 0.10)
        self.declare_parameter("speed_slew_down", 0.14)
        self.declare_parameter("use_odom_speed_fallback", True)
        self.declare_parameter("wheel_speed_timeout_sec", 0.30)
        self.declare_parameter("odom_speed_timeout_sec", 0.45)
        self.declare_parameter("depth_timeout_sec", 0.35)
        self.declare_parameter("camera_timeout_sec", 0.30)
        # Stability controls for anti-zigzag behaviour at higher speed.
        self.declare_parameter("steering_low_pass_alpha_min", 0.62)
        self.declare_parameter("steering_low_pass_alpha_max", 0.90)
        self.declare_parameter("steering_low_pass_speed_ref_m_s", 1.8)
        self.declare_parameter("steering_jerk_limit_rad", 0.035)
        self.declare_parameter("steering_sign_hysteresis_rad", 0.025)
        self.declare_parameter("steering_center_hold_speed_m_s", 0.80)
        self.declare_parameter("steering_slew_speed_scale", 0.45)
        # Camera confidence gating to reduce noisy visual steering influence.
        self.declare_parameter("camera_offset_conf_ref_rad", 0.20)
        self.declare_parameter("camera_offset_jitter_ref_rad", 0.08)
        self.declare_parameter("camera_offset_min_confidence", 0.12)

        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 10)
        self.create_subscription(Float32, "/depth_front_dist", self.depth_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 20)
        self.create_subscription(Float32, "/wheel_speed", self.wheel_speed_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Float32, "/camera_steering_offset", self.camera_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_reactive", 10)
        self.prev_steering = 0.0
        self.prev_steering_delta = 0.0
        self.prev_speed = 0.0
        self.depth_front_dist = float("inf")
        self.camera_offset = 0.0
        self.prev_camera_offset = 0.0
        self.wheel_speed_m_s = 0.0
        self.odom_speed_m_s = 0.0

        # Vehicle state estimator (EKF fusing IMU + wheel speed + LiDAR)
        self._state_estimator = VehicleStateEstimator()
        self._imu_active = False
        self._speed_active = False
        self._last_wheel_speed_time = 0.0
        self._last_odom_speed_time = 0.0
        self._last_depth_time = 0.0
        self._last_camera_time = 0.0

        self.get_logger().info("Gap follower node started (AI speed + IMU fusion enabled)")

    def depth_cb(self, msg: Float32):
        self.depth_front_dist = float(msg.data)
        self._last_depth_time = time.monotonic()

    def camera_cb(self, msg: Float32):
        self.camera_offset = float(msg.data)
        self._last_camera_time = time.monotonic()

    def imu_cb(self, msg: Imu):
        """Fuse IMU data into vehicle state at IMU rate (~100 Hz)."""
        if not bool(self.get_parameter("use_imu_fusion").value):
            return
        self._state_estimator.update_imu(
            yaw_rate=float(msg.angular_velocity.z),
            lat_accel=float(msg.linear_acceleration.y),
            lon_accel=float(msg.linear_acceleration.x),
        )
        self._imu_active = True

    def wheel_speed_cb(self, msg: Float32):
        """Fuse wheel encoder speed into vehicle state."""
        if not bool(self.get_parameter("use_imu_fusion").value):
            return
        self.wheel_speed_m_s = float(msg.data)
        self._state_estimator.update_wheel_speed(self.wheel_speed_m_s)
        self._speed_active = True
        self._last_wheel_speed_time = time.monotonic()

    def odom_cb(self, msg: Odometry):
        """Fallback speed fusion from odometry when wheel speed is stale or missing."""
        if not bool(self.get_parameter("use_imu_fusion").value):
            return
        if not bool(self.get_parameter("use_odom_speed_fallback").value):
            return

        now = time.monotonic()
        ws_timeout = float(self.get_parameter("wheel_speed_timeout_sec").value)
        wheel_recent = (
            self._last_wheel_speed_time > 0.0
            and (now - self._last_wheel_speed_time) <= ws_timeout
        )
        if wheel_recent:
            lin = msg.twist.twist.linear
            self.odom_speed_m_s = math.hypot(float(lin.x), float(lin.y))
            self._last_odom_speed_time = now
            return

        lin = msg.twist.twist.linear
        speed = math.hypot(float(lin.x), float(lin.y))
        self.odom_speed_m_s = speed
        self._last_odom_speed_time = now
        self._state_estimator.update_wheel_speed(speed)
        self._speed_active = True

    def scan_cb(self, msg: LaserScan):
        now = time.monotonic()
        configured_max_speed = float(self.get_parameter("max_speed").value)
        profile_cap = resolve_profile_speed_cap(
            race_profile=str(self.get_parameter("race_profile").value),
            deployment_mode=str(self.get_parameter("deployment_mode").value),
            max_speed_real_cap=float(self.get_parameter("max_speed_real_cap").value),
            max_speed_sim_cap=float(self.get_parameter("max_speed_sim_cap").value),
        )
        max_speed = min(configured_max_speed, profile_cap)

        # Get fused vehicle state whenever IMU/speed feedback has started.
        vehicle_state = None
        if bool(self.get_parameter("use_imu_fusion").value) and (self._imu_active or self._speed_active):
            self._state_estimator.predict(timestamp=now)
            vehicle_state = self._state_estimator.state
        est_speed = self._estimate_speed_for_control(vehicle_state=vehicle_state, now=now)
        speed_ratio = _clamp(est_speed / max(max_speed, 0.10), 0.0, 1.0)
        base_slew = float(self.get_parameter("steering_slew_rate").value)
        slew_scale = _clamp(float(self.get_parameter("steering_slew_speed_scale").value), 0.0, 0.95)
        dynamic_slew = max(0.02, base_slew * (1.0 - slew_scale * speed_ratio))
        camera_offset = self._gated_camera_offset(now=now)
        depth_front_dist = self._depth_front_for_control(now=now)

        speed, steering = compute_gap_command(
            ranges_in=msg.ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            car_width=float(self.get_parameter("car_width").value),
            max_speed=max_speed,
            min_speed=float(self.get_parameter("min_speed").value),
            max_range=float(self.get_parameter("max_range").value),
            safety_radius=float(self.get_parameter("safety_radius").value),
            disparity_threshold=float(self.get_parameter("disparity_threshold").value),
            steering_gain=float(self.get_parameter("steering_gain").value),
            fov_degrees=float(self.get_parameter("fov_degrees").value),
            prev_steering=self.prev_steering,
            steering_slew_rate=dynamic_slew,
            max_steering=float(self.get_parameter("max_steering").value),
            ttc_target_sec=float(self.get_parameter("ttc_target_sec").value),
            use_ai_speed=bool(self.get_parameter("use_ai_speed").value),
            depth_front_dist=depth_front_dist,
            vehicle_state=vehicle_state,
            camera_offset=camera_offset,
            use_close_far_fusion=bool(self.get_parameter("use_close_far_fusion").value),
            far_center_gain=float(self.get_parameter("far_center_gain").value),
            camera_center_gain=float(self.get_parameter("camera_center_gain").value),
            far_weight_min=float(self.get_parameter("far_weight_min").value),
            far_weight_max=float(self.get_parameter("far_weight_max").value),
            fusion_clearance_ref_m=float(self.get_parameter("fusion_clearance_ref_m").value),
        )

        # Extra damping for Webots to suppress high-frequency steering zigzags.
        alpha_base = float(self.get_parameter("steering_low_pass_alpha").value)
        alpha_min = min(alpha_base, float(self.get_parameter("steering_low_pass_alpha_min").value))
        alpha_max = max(alpha_base, float(self.get_parameter("steering_low_pass_alpha_max").value))
        alpha_ref = max(0.1, float(self.get_parameter("steering_low_pass_speed_ref_m_s").value))
        alpha_speed = _clamp(est_speed / alpha_ref, 0.0, 1.0)
        alpha = alpha_min + (alpha_max - alpha_min) * alpha_speed
        alpha = max(0.0, min(0.98, alpha))
        steering = alpha * self.prev_steering + (1.0 - alpha) * steering

        # Jerk limiting: bound how quickly steering rate itself can change.
        raw_delta = steering - self.prev_steering
        jerk_lim = max(0.001, float(self.get_parameter("steering_jerk_limit_rad").value))
        delta = _clamp(
            raw_delta,
            self.prev_steering_delta - jerk_lim,
            self.prev_steering_delta + jerk_lim,
        )
        steering = self.prev_steering + delta

        # Small sign flips around center create visible zigzags: damp them.
        sign_hys = max(0.0, float(self.get_parameter("steering_sign_hysteresis_rad").value))
        center_hold_speed = max(0.0, float(self.get_parameter("steering_center_hold_speed_m_s").value))
        if self.prev_steering * steering < 0.0 and max(abs(self.prev_steering), abs(steering)) < sign_hys * 1.8:
            steering = 0.0
        elif abs(steering) < sign_hys and est_speed > center_hold_speed:
            steering = 0.0
        actual_delta = steering - self.prev_steering
        self.prev_steering = steering
        self.prev_steering_delta = actual_delta

        up_step = max(0.0, float(self.get_parameter("speed_slew_up").value))
        down_step = max(0.0, float(self.get_parameter("speed_slew_down").value))
        if speed > self.prev_speed:
            speed = min(speed, self.prev_speed + up_step)
        else:
            speed = max(speed, self.prev_speed - down_step)
        self.prev_speed = speed

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)

    def _estimate_speed_for_control(self, vehicle_state, now: float) -> float:
        ws_timeout = max(0.05, float(self.get_parameter("wheel_speed_timeout_sec").value))
        odom_timeout = max(0.05, float(self.get_parameter("odom_speed_timeout_sec").value))

        if self._last_wheel_speed_time > 0.0 and (now - self._last_wheel_speed_time) <= ws_timeout:
            return abs(float(self.wheel_speed_m_s))
        if self._last_odom_speed_time > 0.0 and (now - self._last_odom_speed_time) <= odom_timeout:
            return abs(float(self.odom_speed_m_s))
        if vehicle_state is not None and math.isfinite(vehicle_state.speed):
            return abs(float(vehicle_state.speed))
        return max(0.0, float(self.prev_speed))

    def _depth_front_for_control(self, now: float) -> float:
        if not math.isfinite(self.depth_front_dist) or self.depth_front_dist <= 0.0:
            return float("inf")
        depth_timeout = max(0.05, float(self.get_parameter("depth_timeout_sec").value))
        if self._last_depth_time <= 0.0 or (now - self._last_depth_time) > depth_timeout:
            return float("inf")
        return float(self.depth_front_dist)

    def _gated_camera_offset(self, now: float) -> float:
        camera_timeout = max(0.05, float(self.get_parameter("camera_timeout_sec").value))
        if self._last_camera_time <= 0.0 or (now - self._last_camera_time) > camera_timeout:
            self.prev_camera_offset = 0.0
            return 0.0

        offset = float(self.camera_offset)
        mag_ref = max(0.01, float(self.get_parameter("camera_offset_conf_ref_rad").value))
        jitter_ref = max(0.01, float(self.get_parameter("camera_offset_jitter_ref_rad").value))
        min_conf = _clamp(float(self.get_parameter("camera_offset_min_confidence").value), 0.0, 1.0)

        mag_conf = _clamp(1.0 - abs(offset) / mag_ref, 0.0, 1.0)
        jitter = abs(offset - self.prev_camera_offset)
        jitter_conf = _clamp(1.0 - jitter / jitter_ref, 0.0, 1.0)
        conf = max(min_conf, mag_conf * jitter_conf)
        self.prev_camera_offset = offset
        return offset * conf


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
