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
from std_msgs.msg import Bool

from covapsy_nav.track_learner import TrackLearner


class TrackLearnerNode(Node):

    def __init__(self):
        super().__init__("track_learner")

        self.declare_parameter("min_spacing", 0.06)
        self.declare_parameter("closure_tol", 0.50)
        self.declare_parameter("required_laps", 1)
        self.declare_parameter("auto_publish", True)

        self.learner = TrackLearner(
            min_spacing=float(self.get_parameter("min_spacing").value),
            closure_tol=float(self.get_parameter("closure_tol").value),
        )
        self._required_laps = int(self.get_parameter("required_laps").value)
        self._published = False

        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.path_pub = self.create_publisher(Path, "/racing_path", 10)
        self.learned_pub = self.create_publisher(Bool, "/track_learned", 10)

        self.get_logger().info(
            f"Track learner started (need {self._required_laps} lap(s))"
        )

    def odom_cb(self, msg: Odometry):
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

    def _publish_racing_line(self):
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
