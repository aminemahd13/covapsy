"""Opponent-aware tactical racing node with Kalman-tracked predictions.

Publishes /cmd_vel_tactical for RACING mode arbitration.
Uses multi-frame opponent tracking for stable passing decisions.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from covapsy_nav.race_profiles import resolve_profile_speed_cap
from covapsy_nav.tactical_utils import (
    apply_rule_guards,
    clamp,
    compute_ttc_limited_speed,
    fused_opponent_confidence,
    select_passing_side,
    traffic_mode_pass_margin,
    traffic_mode_speed_bias,
)

try:
    from covapsy_nav.opponent_tracker import (
        OpponentTracker,
        extract_opponent_clusters,
    )
    TRACKER_AVAILABLE = True
except ImportError:
    TRACKER_AVAILABLE = False


class TacticalRaceNode(Node):
    """Generate opponent-aware tactical commands with predictive tracking."""

    def __init__(self):
        super().__init__("tactical_race")

        self.declare_parameter("race_profile", "RACE_STABLE")
        self.declare_parameter("deployment_mode", "real")
        self.declare_parameter("max_speed_real_cap", 2.0)
        self.declare_parameter("max_speed_sim_cap", 2.5)
        self.declare_parameter("traffic_mode", "balanced")
        self.declare_parameter("follow_distance", 1.1)
        self.declare_parameter("opponent_detect_range", 2.8)
        self.declare_parameter("camera_weight", 0.20)
        self.declare_parameter("target_ttc_sec", 1.35)
        self.declare_parameter("max_steering", 0.5)
        self.declare_parameter("pass_lock_sec", 0.65)
        self.declare_parameter("steering_bias_gain", 0.18)
        self.declare_parameter("enable_predictive_tracking", True)

        self.reactive_cmd = Twist()
        self.pursuit_cmd = Twist()
        self.camera_offset = 0.0
        self.external_opp_conf = 0.0
        self.scan_msg: LaserScan | None = None
        self.odom_msg: Odometry | None = None
        self.pass_side = 0
        self.pass_lock_until = 0.0
        self.opponent_count = 0.0

        # Kalman tracker for multi-frame opponent prediction
        self.tracker: OpponentTracker | None = None
        if TRACKER_AVAILABLE and bool(self.get_parameter("enable_predictive_tracking").value):
            self.tracker = OpponentTracker()

        self.create_subscription(Twist, "/cmd_vel_reactive", self.reactive_cb, 20)
        self.create_subscription(Twist, "/cmd_vel_pursuit", self.pursuit_cb, 20)
        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 20)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.create_subscription(Float32, "/camera_steering_offset", self.camera_cb, 20)
        self.create_subscription(Float32, "/opponent_confidence", self.opp_conf_cb, 20)
        self.create_subscription(Float32, "/opponent_count", self.opp_count_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_tactical", 20)

        self.create_timer(0.02, self.control_loop)
        if self.tracker is not None:
            self.get_logger().info("Tactical race node started (predictive tracking ON)")
        else:
            self.get_logger().info("Tactical race node started")

    def reactive_cb(self, msg: Twist):
        self.reactive_cmd = msg

    def pursuit_cb(self, msg: Twist):
        self.pursuit_cmd = msg

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    def odom_cb(self, msg: Odometry):
        self.odom_msg = msg

    def camera_cb(self, msg: Float32):
        self.camera_offset = float(msg.data)

    def opp_conf_cb(self, msg: Float32):
        self.external_opp_conf = clamp(msg.data, 0.0, 1.0)

    def opp_count_cb(self, msg: Float32):
        self.opponent_count = max(0.0, float(msg.data))

    def _base_command(self) -> tuple[float, float]:
        pursuit_speed = float(self.pursuit_cmd.linear.x)
        if pursuit_speed > 0.05:
            return pursuit_speed, float(self.pursuit_cmd.angular.z)
        return float(self.reactive_cmd.linear.x), float(self.reactive_cmd.angular.z)

    def _scan_metrics(self) -> tuple[float, float, float, float]:
        """Return (front_min, left_clear, right_clear, lidar_confidence)."""
        if self.scan_msg is None:
            return 5.0, 5.0, 5.0, 0.0

        ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return 5.0, 5.0, 5.0, 0.0

        ranges = np.where(np.isfinite(ranges), ranges, float(self.scan_msg.range_max))
        ranges = np.clip(ranges, float(self.scan_msg.range_min), float(self.scan_msg.range_max))
        angles = float(self.scan_msg.angle_min) + np.arange(ranges.size) * float(self.scan_msg.angle_increment)

        front = ranges[np.abs(angles) < np.radians(18)]
        left = ranges[(angles > np.radians(20)) & (angles < np.radians(75))]
        right = ranges[(angles < -np.radians(20)) & (angles > -np.radians(75))]
        front_min = float(np.min(front)) if front.size > 0 else float(self.scan_msg.range_max)
        left_clear = float(np.median(left)) if left.size > 0 else float(self.scan_msg.range_max)
        right_clear = float(np.median(right)) if right.size > 0 else float(self.scan_msg.range_max)

        # Simple LiDAR cluster confidence near front.
        detect_range = float(self.get_parameter("opponent_detect_range").value)
        near_front = ranges[(np.abs(angles) < np.radians(65)) & (ranges < detect_range)]
        points_score = min(1.0, near_front.size / 30.0)
        dist_score = clamp((detect_range - front_min) / max(detect_range, 1e-3), 0.0, 1.0)
        lidar_conf = clamp(0.55 * points_score + 0.45 * dist_score, 0.0, 1.0)
        return front_min, left_clear, right_clear, lidar_conf

    def control_loop(self):
        base_speed, base_steer = self._base_command()
        profile_cap = resolve_profile_speed_cap(
            race_profile=str(self.get_parameter("race_profile").value),
            deployment_mode=str(self.get_parameter("deployment_mode").value),
            max_speed_real_cap=float(self.get_parameter("max_speed_real_cap").value),
            max_speed_sim_cap=float(self.get_parameter("max_speed_sim_cap").value),
        )
        max_speed = max(0.1, profile_cap)
        max_steering = float(self.get_parameter("max_steering").value)
        traffic_mode = str(self.get_parameter("traffic_mode").value)
        speed_bias = traffic_mode_speed_bias(traffic_mode)
        follow_distance = float(self.get_parameter("follow_distance").value)
        pass_margin = traffic_mode_pass_margin(traffic_mode)
        pass_lock_sec = float(self.get_parameter("pass_lock_sec").value)
        steering_bias_gain = float(self.get_parameter("steering_bias_gain").value)
        ttc_target_sec = float(self.get_parameter("target_ttc_sec").value)

        front_min, left_clear, right_clear, lidar_conf = self._scan_metrics()
        # Camera offset acts as a lightweight confirmation prior.
        camera_conf = clamp(1.0 - abs(self.camera_offset), 0.0, 1.0)
        confidence = max(
            fused_opponent_confidence(
                lidar_conf=lidar_conf,
                camera_conf=camera_conf,
                camera_weight=float(self.get_parameter("camera_weight").value),
            ),
            self.external_opp_conf,
        )

        now = time.monotonic()
        side = select_passing_side(
            front_dist=front_min,
            left_clear=left_clear,
            right_clear=right_clear,
            pass_margin=pass_margin,
            follow_distance=follow_distance,
        )
        if now < self.pass_lock_until and self.pass_side != 0:
            side = self.pass_side
        elif side != 0:
            self.pass_side = side
            self.pass_lock_until = now + pass_lock_sec
        else:
            self.pass_side = 0

        # Base tactical target starts from planner/reactive command.
        speed = clamp(base_speed, 0.0, max_speed) * speed_bias
        steer = clamp(base_steer, -max_steering, max_steering)

        if confidence > 0.30 and front_min < follow_distance:
            # Slow down in traffic and bias steering toward clearer side.
            slow_factor = clamp(front_min / max(follow_distance, 0.2), 0.30, 1.0)

            # Use Kalman-tracked predictions for smarter reactions
            if self.tracker is not None and self.scan_msg is not None:
                ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
                angles = float(self.scan_msg.angle_min) + np.arange(ranges.size) * float(self.scan_msg.angle_increment)
                clusters = extract_opponent_clusters(ranges, angles,
                                                     max_dist=float(self.get_parameter("opponent_detect_range").value))
                dt = 0.02  # control loop period
                self.tracker.predict(dt)
                self.tracker.update(clusters)

                # Find closest opponent ahead and check if it's moving away
                from covapsy_nav.opponent_tracker import nearest_opponent_ahead, predict_passing_opportunity
                nearest = nearest_opponent_ahead(self.tracker)
                if nearest is not None:
                    # Opponent tracked: use predicted behaviour
                    opp_dist = math.hypot(nearest.state[0], nearest.state[1])
                    opp_vx = nearest.state[2]  # positive = moving away in car frame
                    # If opponent is slower or stationary, start passing manoeuvre
                    passing = predict_passing_opportunity(self.tracker, car_speed=base_speed)
                    if passing:
                        # Opportunity to pass: be more aggressive
                        slow_factor = max(slow_factor, 0.70)
                    elif opp_vx > 0.3:
                        # Opponent moving away: less braking needed
                        slow_factor = max(slow_factor, 0.50)

            speed *= slow_factor
            steer += steering_bias_gain * float(side)
            steer = clamp(steer, -max_steering, max_steering)
        else:
            # No strong opponent signal: stay close to nominal planner command.
            speed = min(speed, max_speed)

        speed = compute_ttc_limited_speed(front_dist=front_min, desired_speed=speed, target_ttc_sec=ttc_target_sec)
        speed, steer = apply_rule_guards(
            speed_m_s=speed,
            steering_rad=steer,
            front_dist=front_min,
            left_clear=left_clear,
            right_clear=right_clear,
            max_speed_m_s=max_speed,
        )

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TacticalRaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
