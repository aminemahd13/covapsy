"""Publish a precomputed racing path from file for pure pursuit."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node


class RacingPathPublisherNode(Node):
    """Load racing path points from JSON and publish /racing_path."""

    def __init__(self):
        super().__init__("racing_path_publisher")
        self.declare_parameter("path_file", "")
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("frame_id", "odom")

        self.path_pub = self.create_publisher(NavPath, "/racing_path", 5)
        self.path_msg = self._load_path()

        period = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self.publish_path)
        if self.path_msg is not None:
            self.get_logger().info(f"Racing path publisher loaded {len(self.path_msg.poses)} waypoints")
        else:
            self.get_logger().warn("No valid path loaded; publisher idle")

    def _load_path(self) -> NavPath | None:
        file_param = str(self.get_parameter("path_file").value).strip()
        if not file_param:
            return None

        path = Path(file_param)
        if not path.exists():
            self.get_logger().warn(f"Path file not found: {path}")
            return None

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().warn(f"Failed to read path file: {exc}")
            return None

        points = data.get("points", data if isinstance(data, list) else [])
        if not points:
            self.get_logger().warn("Path file has no points")
            return None

        msg = NavPath()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        for pt in points:
            if isinstance(pt, dict):
                x = float(pt.get("x", 0.0))
                y = float(pt.get("y", 0.0))
            else:
                x = float(pt[0])
                y = float(pt[1])
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_path(self):
        if self.path_msg is None:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RacingPathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
