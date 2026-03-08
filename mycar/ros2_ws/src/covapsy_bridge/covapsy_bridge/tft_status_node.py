"""TFT Display Status Node for COVAPSY.

Publishes status updates to /tft_command which the STM32 bridge
forwards to the 1.6" TFT screen on the HAT.

Displays: mode, speed, IP address.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import subprocess


class TFTStatusNode(Node):

    def __init__(self):
        super().__init__('tft_status')

        self.mode_sub = self.create_subscription(
            String, '/car_mode', self.mode_cb, 10)
        self.speed_sub = self.create_subscription(
            Float32, '/wheel_speed', self.speed_cb, 10)

        self.status_pub = self.create_publisher(String, '/tft_command', 10)

        self.mode = 'BOOT'
        self.speed = 0.0

        # Update display at 1 Hz
        self.create_timer(1.0, self.send_status)

        # Send IP on startup
        self.create_timer(5.0, self.send_ip_once)
        self.ip_sent = False

    def mode_cb(self, msg):
        self.mode = msg.data

    def speed_cb(self, msg):
        self.speed = msg.data

    def send_status(self):
        msg = String()
        msg.data = f'TFT:{self.mode},{self.speed:.1f}'
        self.status_pub.publish(msg)

    def send_ip_once(self):
        if self.ip_sent:
            return
        self.ip_sent = True
        ip = self._get_ip()
        msg = String()
        msg.data = f'TFT:IP,{ip}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'IP address: {ip}')

    @staticmethod
    def _get_ip():
        try:
            result = subprocess.run(
                ['hostname', '-I'], capture_output=True, text=True, timeout=2)
            return result.stdout.strip().split()[0]
        except Exception:
            return '0.0.0.0'


def main(args=None):
    rclpy.init(args=args)
    node = TFTStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
