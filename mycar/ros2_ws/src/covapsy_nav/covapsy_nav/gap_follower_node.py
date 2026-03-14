"""Reactive driving node for COVAPSY — corridor-following controller.

Replaces the old gap-follower's 4-layer steering damping pipeline with a
direct, responsive controller.  The new reactive_core algorithm produces
inherently smooth output through:
  - Weighted-average heading (no discrete gap jumps)
  - Continuous corridor centering (no safety bubble dead zones)
  - IMU yaw-rate damping (replaces low-pass + jerk limit + sign hysteresis)

The only post-processing is a speed slew for ESC protection.
"""

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
    """Reactive corridor-following controller with sensor fusion."""

    def __init__(self):
        super().__init__("gap_follower")

        # Core parameters
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
        self.declare_parameter("max_steering", 0.32)
        self.declare_parameter("ttc_target_sec", 0.70)
        self.declare_parameter("use_ai_speed", True)
        self.declare_parameter("use_imu_fusion", True)
        self.declare_parameter("use_close_far_fusion", True)
        self.declare_parameter("far_center_gain", 0.35)
        self.declare_parameter("camera_center_gain", 0.25)
        self.declare_parameter("far_weight_min", 0.10)
        self.declare_parameter("far_weight_max", 0.55)
        self.declare_parameter("fusion_clearance_ref_m", 1.8)
        self.declare_parameter("use_odom_speed_fallback", True)
        self.declare_parameter("wheel_speed_timeout_sec", 0.30)
        self.declare_parameter("odom_speed_timeout_sec", 0.45)
        self.declare_parameter("depth_timeout_sec", 0.35)
        self.declare_parameter("camera_timeout_sec", 0.30)

        # Steering slew rate: just the mechanical servo limit.
        # The new algorithm handles smoothness internally via:
        #   - Weighted-average heading (no discrete jumps)
        #   - IMU yaw-rate damping (suppresses oscillations)
        # No low-pass filter, no jerk limit, no sign hysteresis.
        self.declare_parameter("steering_slew_rate", 0.25)

        # Speed slew: ESC protection only.  Faster rates than before.
        self.declare_parameter("speed_slew_up", 0.25)
        self.declare_parameter("speed_slew_down", 0.35)

        # Subscriptions
        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 10)
        self.create_subscription(Float32, "/depth_front_dist", self.depth_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 20)
        self.create_subscription(Float32, "/wheel_speed", self.wheel_speed_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Float32, "/camera_steering_offset", self.camera_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_reactive", 10)

        # State
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.depth_front_dist = float("inf")
        self.camera_offset = 0.0
        self.wheel_speed_m_s = 0.0
        self.odom_speed_m_s = 0.0

        # EKF fusing IMU + wheel speed
        self._state_estimator = VehicleStateEstimator()
        self._imu_active = False
        self._speed_active = False
        self._last_wheel_speed_time = 0.0
        self._last_odom_speed_time = 0.0
        self._last_depth_time = 0.0
        self._last_camera_time = 0.0

        from covapsy_nav.reactive_core import is_c_available
        backend = "C" if is_c_available() else "Python"
        self.get_logger().info(
            f"Reactive driving node started (backend={backend}, "
            f"IMU fusion enabled)"
        )

    # ── Sensor callbacks ──

    def depth_cb(self, msg: Float32):
        self.depth_front_dist = float(msg.data)
        self._last_depth_time = time.monotonic()

    def camera_cb(self, msg: Float32):
        self.camera_offset = float(msg.data)
        self._last_camera_time = time.monotonic()

    def imu_cb(self, msg: Imu):
        if not bool(self.get_parameter("use_imu_fusion").value):
            return
        self._state_estimator.update_imu(
            yaw_rate=float(msg.angular_velocity.z),
            lat_accel=float(msg.linear_acceleration.y),
            lon_accel=float(msg.linear_acceleration.x),
        )
        self._imu_active = True

    def wheel_speed_cb(self, msg: Float32):
        if not bool(self.get_parameter("use_imu_fusion").value):
            return
        self.wheel_speed_m_s = float(msg.data)
        self._state_estimator.update_wheel_speed(self.wheel_speed_m_s)
        self._speed_active = True
        self._last_wheel_speed_time = time.monotonic()

    def odom_cb(self, msg: Odometry):
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

        lin = msg.twist.twist.linear
        speed = math.hypot(float(lin.x), float(lin.y))
        self.odom_speed_m_s = speed
        self._last_odom_speed_time = now

        if not wheel_recent:
            self._state_estimator.update_wheel_speed(speed)
            self._speed_active = True

    # ── Main control: LiDAR scan callback ──

    def scan_cb(self, msg: LaserScan):
        now = time.monotonic()

        # Resolve speed cap from race profile
        configured_max_speed = float(self.get_parameter("max_speed").value)
        profile_cap = resolve_profile_speed_cap(
            race_profile=str(self.get_parameter("race_profile").value),
            deployment_mode=str(self.get_parameter("deployment_mode").value),
            max_speed_real_cap=float(self.get_parameter("max_speed_real_cap").value),
            max_speed_sim_cap=float(self.get_parameter("max_speed_sim_cap").value),
        )
        max_speed = min(configured_max_speed, profile_cap)

        # Get fused vehicle state from EKF
        vehicle_state = None
        if bool(self.get_parameter("use_imu_fusion").value) and (
            self._imu_active or self._speed_active
        ):
            self._state_estimator.predict(timestamp=now)
            vehicle_state = self._state_estimator.state

        # Sensor freshness checks
        camera_offset = self._fresh_camera_offset(now)
        depth_front_dist = self._fresh_depth(now)

        # ── Core computation: corridor-following reactive drive ──
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
            steering_slew_rate=float(self.get_parameter("steering_slew_rate").value),
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

        # Store steering (no post-processing — algorithm output is clean)
        self.prev_steering = steering

        # Speed slew rate: ESC protection only
        up_step = max(0.0, float(self.get_parameter("speed_slew_up").value))
        down_step = max(0.0, float(self.get_parameter("speed_slew_down").value))
        if speed > self.prev_speed:
            speed = min(speed, self.prev_speed + up_step)
        else:
            speed = max(speed, self.prev_speed - down_step)
        self.prev_speed = speed

        # Publish
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)

    # ── Sensor freshness helpers ──

    def _fresh_depth(self, now: float) -> float:
        if not math.isfinite(self.depth_front_dist) or self.depth_front_dist <= 0.0:
            return float("inf")
        timeout = max(0.05, float(self.get_parameter("depth_timeout_sec").value))
        if self._last_depth_time <= 0.0 or (now - self._last_depth_time) > timeout:
            return float("inf")
        return float(self.depth_front_dist)

    def _fresh_camera_offset(self, now: float) -> float:
        timeout = max(0.05, float(self.get_parameter("camera_timeout_sec").value))
        if self._last_camera_time <= 0.0 or (now - self._last_camera_time) > timeout:
            return 0.0
        return float(self.camera_offset)


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
