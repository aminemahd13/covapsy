#!/usr/bin/env python3
import time

import rclpy
from covapsy_interfaces.msg import DriveCommand
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class STM32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('stm32_bridge_node')
        self.declare_parameter('watchdog_timeout_s', 0.3)
        self.declare_parameter('max_speed_mps', 2.0)
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('competition_mode', True)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('allow_restart_after_stop', False)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.competition_mode = bool(self.get_parameter('competition_mode').value)
        self.require_start_signal = bool(self.get_parameter('require_start_signal').value)
        self.allow_restart_after_stop = bool(self.get_parameter('allow_restart_after_stop').value)

        self.started = not self.require_start_signal
        self.stopped = False
        self.last_cmd = DriveCommand()
        self.last_cmd_time = time.monotonic()

        self.create_subscription(DriveCommand, '/cmd_drive', self.cmd_cb, 20)
        self.create_subscription(Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(Bool, '/race_stop', self.stop_cb, 10)

        self.wheel_speed_pub = self.create_publisher(Float32, '/wheel_speed', 20)
        self.rear_obstacle_pub = self.create_publisher(Bool, '/rear_obstacle', 10)
        self.status_pub = self.create_publisher(String, '/bridge_status', 10)
        self.lowlevel_cmd_pub = self.create_publisher(DriveCommand, '/stm32/cmd_drive', 20)

        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info('stm32_bridge_node ready')

    def start_cb(self, msg: Bool) -> None:
        if msg.data and (not self.stopped or self.allow_restart_after_stop):
            self.started = True

    def stop_cb(self, msg: Bool) -> None:
        if msg.data:
            self.started = False
            self.stopped = True

    def cmd_cb(self, msg: DriveCommand) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _limited_cmd(self) -> DriveCommand:
        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if not self.started:
            cmd.speed_mps = 0.0
            cmd.steer_rad = 0.0
            cmd.emergency_brake = True
            return cmd

        age = time.monotonic() - self.last_cmd_time
        if age > self.watchdog_timeout:
            cmd.speed_mps = 0.0
            cmd.steer_rad = 0.0
            cmd.emergency_brake = True
            return cmd

        cmd.speed_mps = max(-0.8, min(self.max_speed, self.last_cmd.speed_mps))
        cmd.steer_rad = max(-self.max_steer, min(self.max_steer, self.last_cmd.steer_rad))
        cmd.emergency_brake = bool(self.last_cmd.emergency_brake)
        return cmd

    def tick(self) -> None:
        cmd = self._limited_cmd()
        self.lowlevel_cmd_pub.publish(cmd)

        wheel = Float32()
        wheel.data = float(cmd.speed_mps)
        self.wheel_speed_pub.publish(wheel)

        rear = Bool()
        rear.data = False
        self.rear_obstacle_pub.publish(rear)

        status = String()
        if not self.started:
            status.data = 'WAIT_START'
        elif abs(cmd.speed_mps) < 1e-4 and (time.monotonic() - self.last_cmd_time) > self.watchdog_timeout:
            status.data = 'WATCHDOG_BRAKE'
        else:
            status.data = 'RUN'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
