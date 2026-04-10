#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from covapsy_interfaces.msg import DriveCommand
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from covapsy_bridge.steering_policy import evaluate_steering_gate, steer_rad_to_goal_tick

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except Exception:  # pragma: no cover - optional import in non-hardware env
    COMM_SUCCESS = 0
    PacketHandler = None
    PortHandler = None


class DynamixelSteeringNode(Node):
    def __init__(self) -> None:
        super().__init__('dynamixel_steering_node')
        self.declare_parameter('watchdog_timeout_s', 0.25)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('allow_restart_after_stop', False)
        self.declare_parameter('serial_device', '/dev/waveshare_servo')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('serial_open_retry_s', 1.0)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('center_tick', 2048)
        self.declare_parameter('left_tick', 2560)
        self.declare_parameter('right_tick', 1536)
        self.declare_parameter('torque_enable_addr', 64)
        self.declare_parameter('goal_position_addr', 116)
        self.declare_parameter('present_position_addr', 132)
        self.declare_parameter('publish_present_position', True)
        self.declare_parameter('disable_torque_on_shutdown', True)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self.require_start_signal = bool(self.get_parameter('require_start_signal').value)
        self.allow_restart_after_stop = bool(
            self.get_parameter('allow_restart_after_stop').value
        )
        self.serial_device = str(self.get_parameter('serial_device').value)
        self.serial_enabled = bool(self.serial_device.strip())
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_open_retry = float(self.get_parameter('serial_open_retry_s').value)
        self.protocol_version = float(self.get_parameter('protocol_version').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.center_tick = int(self.get_parameter('center_tick').value)
        self.left_tick = int(self.get_parameter('left_tick').value)
        self.right_tick = int(self.get_parameter('right_tick').value)
        self.torque_enable_addr = int(self.get_parameter('torque_enable_addr').value)
        self.goal_position_addr = int(self.get_parameter('goal_position_addr').value)
        self.present_position_addr = int(self.get_parameter('present_position_addr').value)
        self.publish_present_position = bool(
            self.get_parameter('publish_present_position').value
        )
        self.disable_torque_on_shutdown = bool(
            self.get_parameter('disable_torque_on_shutdown').value
        )

        self.started = not self.require_start_signal
        self.stopped = False
        self.last_cmd = DriveCommand()
        self.last_cmd_time = time.monotonic()

        self.port_handler: Optional[object] = None
        self.packet_handler: Optional[object] = None
        self.last_serial_attempt = 0.0
        self.link_ready = False
        self.last_link_error = 'USB_DISCONNECTED'

        self.create_subscription(DriveCommand, '/cmd_drive', self.cmd_cb, 20)
        self.create_subscription(Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(Bool, '/race_stop', self.stop_cb, 10)

        self.status_pub = self.create_publisher(String, '/steering_status', 10)
        self.position_pub = self.create_publisher(Int32, '/steering_present_position', 10)

        self.timer = self.create_timer(0.02, self.tick)
        if PacketHandler is None or PortHandler is None:
            self.get_logger().error(
                'dynamixel_sdk not available; steering bridge will stay disconnected.'
            )
        self.get_logger().info('dynamixel_steering_node ready')

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

    def _close_link(self) -> None:
        if self.port_handler is None or self.packet_handler is None:
            self.link_ready = False
            self.port_handler = None
            self.packet_handler = None
            return

        try:
            if self.disable_torque_on_shutdown:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.dxl_id, self.torque_enable_addr, 0
                )
        except Exception:
            pass

        try:
            self.port_handler.closePort()
        except Exception:
            pass

        self.link_ready = False
        self.port_handler = None
        self.packet_handler = None

    def _ensure_link(self, now: float) -> bool:
        if not self.serial_enabled:
            self.last_link_error = 'USB_DISCONNECTED'
            return False
        if PacketHandler is None or PortHandler is None:
            self.last_link_error = 'USB_DISCONNECTED'
            return False
        if self.link_ready and self.port_handler is not None and self.packet_handler is not None:
            return True
        if now - self.last_serial_attempt < self.serial_open_retry:
            return False

        self.last_serial_attempt = now
        self._close_link()

        port_handler = PortHandler(self.serial_device)
        if not port_handler.openPort():
            self.last_link_error = 'USB_DISCONNECTED'
            return False
        if not port_handler.setBaudRate(self.baud_rate):
            self.last_link_error = 'USB_DISCONNECTED'
            try:
                port_handler.closePort()
            except Exception:
                pass
            return False

        packet_handler = PacketHandler(self.protocol_version)
        try:
            model, comm_result, dxl_error = packet_handler.ping(port_handler, self.dxl_id)
        except Exception:
            self.last_link_error = 'DXL_NO_RESPONSE'
            try:
                port_handler.closePort()
            except Exception:
                pass
            return False
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.last_link_error = 'DXL_NO_RESPONSE'
            try:
                port_handler.closePort()
            except Exception:
                pass
            return False

        comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, self.dxl_id, self.torque_enable_addr, 1
        )
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.last_link_error = 'DXL_NO_RESPONSE'
            try:
                port_handler.closePort()
            except Exception:
                pass
            return False

        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.link_ready = True
        self.last_link_error = 'OK'
        self.get_logger().info(
            f'Connected to DYNAMIXEL id={self.dxl_id} model={model} on {self.serial_device} @ {self.baud_rate}'
        )
        return True

    def _write_goal(self, goal_tick: int) -> bool:
        if self.port_handler is None or self.packet_handler is None:
            return False
        try:
            comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, self.goal_position_addr, int(goal_tick)
            )
        except Exception:
            comm_result, dxl_error = -1, 1
        if comm_result != COMM_SUCCESS or dxl_error != 0:
            self.last_link_error = 'DXL_NO_RESPONSE'
            self._close_link()
            return False

        if self.publish_present_position:
            try:
                pos, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, self.dxl_id, self.present_position_addr
                )
                if comm_result == COMM_SUCCESS and dxl_error == 0:
                    out = Int32()
                    out.data = int(pos)
                    self.position_pub.publish(out)
            except Exception:
                pass
        return True

    def _status_from_state(self, has_link: bool, wait_start: bool, watchdog_brake: bool) -> str:
        if not has_link:
            return self.last_link_error if self.last_link_error in (
                'USB_DISCONNECTED',
                'DXL_NO_RESPONSE',
            ) else 'USB_DISCONNECTED'
        if wait_start:
            return 'WAIT_START'
        if watchdog_brake:
            return 'WATCHDOG_BRAKE'
        return 'OK'

    def tick(self) -> None:
        now = time.monotonic()
        has_link = self._ensure_link(now)

        run_enable, watchdog_brake, wait_start = evaluate_steering_gate(
            started=self.started,
            cmd_age_s=now - self.last_cmd_time,
            watchdog_timeout_s=self.watchdog_timeout,
            emergency_brake=bool(self.last_cmd.emergency_brake),
        )

        goal_tick = self.center_tick
        if run_enable:
            goal_tick = steer_rad_to_goal_tick(
                steer_rad=float(self.last_cmd.steer_rad),
                max_steer_rad=self.max_steer,
                center_tick=self.center_tick,
                left_tick=self.left_tick,
                right_tick=self.right_tick,
            )

        if has_link:
            has_link = self._write_goal(goal_tick)

        status = String()
        status.data = self._status_from_state(has_link, wait_start, watchdog_brake)
        self.status_pub.publish(status)

    def destroy_node(self):
        self._close_link()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
