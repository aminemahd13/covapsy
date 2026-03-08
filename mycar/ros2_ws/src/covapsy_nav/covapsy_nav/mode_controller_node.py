"""Dual-Mode State Machine for COVAPSY.

Top-level controller selecting between driving modes:
  IDLE     - waiting for start signal
  REACTIVE - Follow-the-Gap (unknown track, first lap, or SLAM failure)
  MAPPING  - Slow driving while SLAM builds map
  RACING   - Pure Pursuit on optimized trajectory
  STOPPED  - Remote stop received or emergency

Subscribes:
  /cmd_vel_reactive (Twist)  - from gap_follower_node
  /cmd_vel_pursuit  (Twist)  - from pure_pursuit_node
  /race_start       (Bool)   - start signal
  /race_stop        (Bool)   - stop signal
  /scan_filtered    (LaserScan) - for emergency detection

Publishes:
  /cmd_vel   (Twist)  - final command to STM32 bridge
  /car_mode  (String) - current mode for TFT display
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import numpy as np


class ModeControllerNode(Node):

    def __init__(self):
        super().__init__('mode_controller')

        self.declare_parameter('mapping_speed_cap', 0.5)
        self.declare_parameter('initial_mode', 'IDLE')

        self.mode = self.get_parameter('initial_mode').value

        # Subscriptions
        self.create_subscription(
            Twist, '/cmd_vel_reactive', self.reactive_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel_pursuit', self.pursuit_cb, 10)
        self.create_subscription(
            Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(
            Bool, '/race_stop', self.stop_cb, 10)
        self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/car_mode', 10)

        # State
        self.reactive_cmd = Twist()
        self.pursuit_cmd = Twist()
        self.has_map = False
        self.min_front_dist = float('inf')

        # Control loop at 50 Hz
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'Mode controller started in {self.mode} mode')

    def start_cb(self, msg):
        if msg.data and self.mode != 'STOPPED':
            self.mode = 'REACTIVE'
            self.get_logger().info('Race started -> REACTIVE mode')

    def stop_cb(self, msg):
        if msg.data:
            self.mode = 'STOPPED'
            self.get_logger().info('Race STOPPED')

    def reactive_cb(self, msg):
        self.reactive_cmd = msg

    def pursuit_cb(self, msg):
        self.pursuit_cmd = msg

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        # Check front 30 degrees for emergency
        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        front_mask = np.abs(angles) < np.radians(15)
        front = ranges[front_mask]
        front = front[np.isfinite(front)]
        if len(front) > 0:
            self.min_front_dist = float(np.min(front))

    def set_mode(self, new_mode: str):
        """Externally set mode (e.g., from a service or parameter change)."""
        valid = ['IDLE', 'REACTIVE', 'MAPPING', 'RACING', 'STOPPED']
        if new_mode in valid:
            self.mode = new_mode
            self.get_logger().info(f'Mode changed to: {new_mode}')

    def control_loop(self):
        cmd = Twist()

        if self.mode in ('IDLE', 'STOPPED'):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.mode == 'MAPPING':
            cmd = self.reactive_cmd
            cap = self.get_parameter('mapping_speed_cap').value
            cmd.linear.x = min(cmd.linear.x, cap)

        elif self.mode == 'REACTIVE':
            cmd = self.reactive_cmd

        elif self.mode == 'RACING':
            # Use pursuit if available, fallback to reactive
            if abs(self.pursuit_cmd.linear.x) > 0.01:
                cmd = self.pursuit_cmd
            else:
                cmd = self.reactive_cmd

        # Emergency slowdown if obstacle very close
        if self.min_front_dist < 0.15 and cmd.linear.x > 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
