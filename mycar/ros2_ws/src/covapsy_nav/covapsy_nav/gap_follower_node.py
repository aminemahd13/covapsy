"""Follow-the-Gap node for reactive obstacle avoidance."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from covapsy_nav.gap_utils import compute_gap_command
from covapsy_nav.race_profiles import resolve_profile_speed_cap


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
        self.declare_parameter("steering_slew_rate", 0.10)
        self.declare_parameter("max_steering", 0.5)
        self.declare_parameter("ttc_target_sec", 0.70)
        self.declare_parameter("use_ai_speed", True)

        self.create_subscription(LaserScan, "/scan_filtered", self.scan_cb, 10)
        self.create_subscription(Float32, "/depth_front_dist", self.depth_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_reactive", 10)
        self.prev_steering = 0.0
        self.depth_front_dist = float("inf")

        self.get_logger().info("Gap follower node started (AI speed enabled)")

    def depth_cb(self, msg: Float32):
        self.depth_front_dist = float(msg.data)

    def scan_cb(self, msg: LaserScan):
        configured_max_speed = float(self.get_parameter("max_speed").value)
        profile_cap = resolve_profile_speed_cap(
            race_profile=str(self.get_parameter("race_profile").value),
            deployment_mode=str(self.get_parameter("deployment_mode").value),
            max_speed_real_cap=float(self.get_parameter("max_speed_real_cap").value),
            max_speed_sim_cap=float(self.get_parameter("max_speed_sim_cap").value),
        )
        max_speed = min(configured_max_speed, profile_cap)

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
            depth_front_dist=self.depth_front_dist,
        )
        self.prev_steering = steering

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
