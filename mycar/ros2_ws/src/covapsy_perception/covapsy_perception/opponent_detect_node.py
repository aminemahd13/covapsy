"""Lightweight opponent confidence estimator (LiDAR-first with camera confirmation)."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from covapsy_perception.opponent_detect_utils import fused_confidence, lidar_opponent_confidence


class OpponentDetectNode(Node):
    """Publish opponent confidence and coarse traffic state."""

    def __init__(self):
        super().__init__("opponent_detect")

        self.declare_parameter("detect_range", 2.8)
        self.declare_parameter("camera_weight", 0.2)
        self.declare_parameter("traffic_mode", "balanced")

        self.camera_offset = 0.0
        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 20)
        self.create_subscription(Float32, "/camera_steering_offset", self.camera_cb, 20)
        self.conf_pub = self.create_publisher(Float32, "/opponent_confidence", 20)
        self.state_pub = self.create_publisher(String, "/traffic_state", 20)

        self.get_logger().info("Opponent detect node started")

    def camera_cb(self, msg: Float32):
        self.camera_offset = float(msg.data)

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        detect_range = float(self.get_parameter("detect_range").value)
        lidar_conf, front_min, left_clear, right_clear = lidar_opponent_confidence(
            ranges=ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            detect_range=detect_range,
        )

        camera_conf = max(0.0, min(1.0, 1.0 - abs(self.camera_offset)))
        conf = fused_confidence(
            lidar_conf=lidar_conf,
            camera_conf=camera_conf,
            camera_weight=float(self.get_parameter("camera_weight").value),
        )

        conf_msg = Float32()
        conf_msg.data = float(conf)
        self.conf_pub.publish(conf_msg)

        state = "clear"
        if conf > 0.55 and front_min < 1.1:
            if left_clear > right_clear + 0.2:
                state = "opp_ahead_pass_left"
            elif right_clear > left_clear + 0.2:
                state = "opp_ahead_pass_right"
            else:
                state = "opp_ahead_hold"
        elif conf > 0.30:
            state = "traffic_watch"

        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
