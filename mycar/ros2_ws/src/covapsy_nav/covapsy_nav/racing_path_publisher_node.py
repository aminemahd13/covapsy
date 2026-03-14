"""Publish a precomputed racing path from file for pure pursuit."""

from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Bool, String

from covapsy_nav.track_persistence_utils import load_points_from_json
from covapsy_nav.track_path_store import normalize_track_direction
from covapsy_nav.track_path_store import resolve_directional_path


class RacingPathPublisherNode(Node):
    """Load racing path points from JSON and publish /racing_path."""

    def __init__(self):
        super().__init__("racing_path_publisher")
        self.declare_parameter("path_file", "")
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("use_directional_loading", True)
        self.declare_parameter("storage_dir", "~/.ros/covapsy")
        self.declare_parameter("red_left_filename", "racing_path_red_left.json")
        self.declare_parameter("red_right_filename", "racing_path_red_right.json")
        self.declare_parameter("unknown_filename", "racing_path_unknown.json")
        self.declare_parameter("direction_topic", "/track_direction")

        self.path_pub = self.create_publisher(NavPath, "/racing_path", 5)
        self.learned_pub = self.create_publisher(Bool, "/track_learned", 10)
        self.saved_loaded_pub = self.create_publisher(Bool, "/saved_track_loaded", 10)

        self.path_msg: NavPath | None = None
        self.path_source = ""
        self.track_direction = "unknown"
        self.saved_track_loaded = False
        self._last_missing_file = ""

        self._legacy_path_file = str(self.get_parameter("path_file").value).strip()
        self._use_directional_loading = bool(
            self.get_parameter("use_directional_loading").value
        )
        if self._legacy_path_file:
            self.path_msg = self._load_from_file(Path(self._legacy_path_file).expanduser())
            if self.path_msg is not None:
                self.path_source = self._legacy_path_file
                self.saved_track_loaded = True
        elif self._use_directional_loading:
            direction_topic = str(self.get_parameter("direction_topic").value)
            self.create_subscription(String, direction_topic, self.direction_cb, 10)
            self.get_logger().info(
                "Racing path publisher waiting for direction on '%s'" % direction_topic
            )

        period = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self.publish_path)
        if self.path_msg is not None and self.saved_track_loaded:
            self.get_logger().info(
                f"Racing path publisher loaded {len(self.path_msg.poses)} waypoints "
                f"from '{self.path_source}'"
            )
        elif not self._use_directional_loading and not self._legacy_path_file:
            self.get_logger().warn("No path_file set and directional loading disabled; publisher idle")

    def direction_cb(self, msg: String) -> None:
        if self.path_msg is not None:
            return
        self.track_direction = normalize_track_direction(msg.data)

    def _load_from_file(self, path: Path) -> NavPath | None:
        if not path.exists():
            self.get_logger().warn(f"Path file not found: {path}")
            return None

        points = load_points_from_json(path)
        if not points:
            self.get_logger().warn(f"Path file has no valid points: {path}")
            return None

        msg = NavPath()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _resolve_directional_file(self) -> Path:
        return resolve_directional_path(
            storage_dir=str(self.get_parameter("storage_dir").value),
            direction=self.track_direction,
            red_left_filename=str(self.get_parameter("red_left_filename").value),
            red_right_filename=str(self.get_parameter("red_right_filename").value),
            unknown_filename=str(self.get_parameter("unknown_filename").value),
        )

    def _try_directional_load(self) -> None:
        if self.path_msg is not None or not self._use_directional_loading:
            return
        path = self._resolve_directional_file()
        if not path.exists():
            path_label = str(path)
            if self._last_missing_file != path_label:
                self._last_missing_file = path_label
                self.get_logger().info(
                    f"No saved path for direction '{self.track_direction}' at '{path_label}'; "
                    "fallback remains live track learning"
                )
            return
        loaded = self._load_from_file(path)
        if loaded is None:
            return
        self.path_msg = loaded
        self.path_source = str(path)
        self.saved_track_loaded = True
        self.get_logger().info(
            f"Directional racing path loaded ({len(self.path_msg.poses)} waypoints) "
            f"from '{self.path_source}' for direction '{self.track_direction}'"
        )

    def _publish_saved_loaded(self) -> None:
        msg = Bool()
        msg.data = bool(self.saved_track_loaded)
        self.saved_loaded_pub.publish(msg)

    def publish_path(self):
        if self.path_msg is None and not self._legacy_path_file and self._use_directional_loading:
            self._try_directional_load()

        self._publish_saved_loaded()
        if self.path_msg is None:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.path_pub.publish(self.path_msg)
        learned = Bool()
        learned.data = True
        self.learned_pub.publish(learned)


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
