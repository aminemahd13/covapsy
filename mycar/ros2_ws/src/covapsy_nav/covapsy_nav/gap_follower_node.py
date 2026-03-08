"""Follow-the-Gap node for reactive obstacle avoidance."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from covapsy_nav.gap_utils import compute_gap_command


class GapFollowerNode(Node):
    """Reactive follow-the-gap controller with disparity extension."""

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

        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_reactive", 10)

        self.get_logger().info("Gap follower node started")

    def scan_cb(self, msg: LaserScan):
        speed, steering = compute_gap_command(
            ranges_in=msg.ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            car_width=float(self.get_parameter("car_width").value),
            max_speed=float(self.get_parameter("max_speed").value),
            min_speed=float(self.get_parameter("min_speed").value),
            max_range=float(self.get_parameter("max_range").value),
            safety_radius=float(self.get_parameter("safety_radius").value),
            disparity_threshold=float(self.get_parameter("disparity_threshold").value),
            steering_gain=float(self.get_parameter("steering_gain").value),
            fov_degrees=float(self.get_parameter("fov_degrees").value),
        )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)


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
