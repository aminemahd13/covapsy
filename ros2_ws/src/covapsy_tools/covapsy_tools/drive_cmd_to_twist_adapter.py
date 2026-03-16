import rclpy
from rclpy.node import Node

from covapsy_interfaces.msg import DriveCommand
from geometry_msgs.msg import Twist


class DriveCmdToTwistAdapter(Node):
    """Adapt DriveCommand output from autonomy stack to Twist for Webots bridge."""

    def __init__(self) -> None:
        super().__init__('drive_cmd_to_twist_adapter')

        self.declare_parameter('input_topic', '/cmd_drive')
        self.declare_parameter('output_topic', '/cmd_vel')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._twist_pub = self.create_publisher(Twist, output_topic, 20)
        self._cmd_sub = self.create_subscription(DriveCommand, input_topic, self._cmd_cb, 20)

        self.get_logger().info(
            f'drive_cmd_to_twist_adapter listening on {input_topic} and publishing to {output_topic}'
        )

    def _cmd_cb(self, msg: DriveCommand) -> None:
        twist = Twist()

        if bool(msg.emergency_brake):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = float(msg.speed_mps)
            twist.angular.z = float(msg.steer_rad)

        self._twist_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = DriveCmdToTwistAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
