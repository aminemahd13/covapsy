"""ROS2 Track Learner Node — learns track during setup laps.

During LEARNING mode:
  1. Records odometry positions
  2. Detects lap completion
  3. Computes optimised racing line
  4. Publishes racing path for pure pursuit
  5. Signals mode controller that track is learned

Subscribes: /odom (Odometry)
Publishes:  /racing_path (Path), /track_learned (Bool)
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

from covapsy_nav.track_persistence_utils import apply_saved_track_loaded
from covapsy_nav.track_persistence_utils import resolve_track_save_path
from covapsy_nav.track_learner import TrackLearner
from covapsy_nav.track_path_store import normalize_track_direction


class TrackLearnerNode(Node):

    def __init__(self):
        super().__init__("track_learner")

        self.declare_parameter("min_spacing", 0.06)
        self.declare_parameter("closure_tol", 0.50)
        self.declare_parameter("closure_tolerance", 0.50)
        self.declare_parameter("required_laps", 1)
        self.declare_parameter("smoothing_window", 7)
        self.declare_parameter("apex_iterations", 5)
        self.declare_parameter("auto_publish", True)
        self.declare_parameter("enable_persistence", True)
        self.declare_parameter("storage_dir", "~/.ros/covapsy")
        self.declare_parameter("red_left_filename", "racing_path_red_left.json")
        self.declare_parameter("red_right_filename", "racing_path_red_right.json")
        self.declare_parameter("unknown_filename", "racing_path_unknown.json")
        self.declare_parameter("direction_topic", "/track_direction")
        self.declare_parameter("saved_track_loaded_topic", "/saved_track_loaded")

        closure_tol = float(self.get_parameter("closure_tolerance").value)
        if closure_tol <= 0.0:
            closure_tol = float(self.get_parameter("closure_tol").value)
        self.learner = TrackLearner(
            min_spacing=float(self.get_parameter("min_spacing").value),
            closure_tol=closure_tol,
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            apex_iterations=int(self.get_parameter("apex_iterations").value),
        )
        self._required_laps = int(self.get_parameter("required_laps").value)
        self._published = False
        self._track_direction = "unknown"
        self._saved_track_loaded = False

        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.create_subscription(
            String,
            str(self.get_parameter("direction_topic").value),
            self.direction_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("saved_track_loaded_topic").value),
            self.saved_track_loaded_cb,
            10,
        )
        self.path_pub = self.create_publisher(Path, "/racing_path", 10)
        self.learned_pub = self.create_publisher(Bool, "/track_learned", 10)

        self.get_logger().info(
            f"Track learner started (need {self._required_laps} lap(s))"
        )

    def direction_cb(self, msg: String) -> None:
        self._track_direction = normalize_track_direction(msg.data)

    def saved_track_loaded_cb(self, msg: Bool) -> None:
        next_state = apply_saved_track_loaded(self._saved_track_loaded, bool(msg.data))
        if next_state and not self._saved_track_loaded:
            self._saved_track_loaded = next_state
            self._published = True
            self.get_logger().info(
                "Saved track was loaded from disk; pausing live learning for this run"
            )

    def odom_cb(self, msg: Odometry):
        if self._saved_track_loaded:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        lap_done = self.learner.add_position(x, y)

        if lap_done:
            self.get_logger().info(
                f"Lap {self.learner.laps_detected} detected "
                f"({len(self.learner.lap_points)} points)"
            )

        if (
            self.learner.laps_detected >= self._required_laps
            and not self._published
            and bool(self.get_parameter("auto_publish").value)
        ):
            self._publish_racing_line()

    def _save_racing_line(self) -> None:
        if not bool(self.get_parameter("enable_persistence").value):
            return
        save_path = resolve_track_save_path(
            storage_dir=str(self.get_parameter("storage_dir").value),
            direction=self._track_direction,
            red_left_filename=str(self.get_parameter("red_left_filename").value),
            red_right_filename=str(self.get_parameter("red_right_filename").value),
            unknown_filename=str(self.get_parameter("unknown_filename").value),
        )
        try:
            save_path.parent.mkdir(parents=True, exist_ok=True)
            self.learner.export_json(str(save_path))
        except Exception as exc:
            self.get_logger().warn(f"Failed to persist racing line to '{save_path}': {exc}")
            return
        self.get_logger().info(
            f"Persisted racing line to '{save_path}' (direction={self._track_direction})"
        )

    def _publish_racing_line(self) -> None:
        racing_line = self.learner.build_racing_line()
        if racing_line is None:
            self.get_logger().warn("Failed to build racing line")
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for wp in racing_line:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp["x"]
            pose.pose.position.y = wp["y"]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self._published = True

        learned_msg = Bool()
        learned_msg.data = True
        self.learned_pub.publish(learned_msg)
        self._save_racing_line()

        self.get_logger().info(
            f"Racing line published: {len(racing_line)} waypoints"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackLearnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
