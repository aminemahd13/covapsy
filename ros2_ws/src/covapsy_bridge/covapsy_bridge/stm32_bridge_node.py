#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from covapsy_interfaces.msg import DriveCommand, RaceTelemetry, RecoveryState
from covapsy_bridge.lcd_status_formatter import LcdStatusSnapshot, build_lcd_lines
from covapsy_bridge.serial_probe import (
    build_baud_candidates,
    build_device_candidates,
    build_probe_pairs,
)
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from covapsy_bridge.usb_protocol import encode_command_line, encode_lcd_line, parse_telemetry_line
from covapsy_bridge.steering_policy import apply_external_steering_mode

try:
    import serial
    from serial import SerialException
except Exception:  # pragma: no cover - optional import in non-hardware env
    serial = None

    class SerialException(Exception):
        pass


class STM32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('stm32_bridge_node')
        self.declare_parameter('watchdog_timeout_s', 0.3)
        self.declare_parameter('max_speed_mps', 2.0)
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('external_steering_mode', False)
        self.declare_parameter('competition_mode', True)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('allow_restart_after_stop', False)
        self.declare_parameter('serial_device', '/dev/stm32_mcu')
        self.declare_parameter(
            'serial_device_candidates',
            '/dev/stm32_mcu,/dev/ttyACM1,/dev/ttyAMA0',
        )
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('serial_baud_candidates', '115200,57600,230400')
        self.declare_parameter('serial_open_retry_s', 1.0)
        self.declare_parameter('telemetry_timeout_s', 0.25)
        self.declare_parameter('telemetry_reconnect_s', 1.5)
        self.declare_parameter('allow_motion_without_telemetry', False)
        self.declare_parameter('lcd_enabled', True)
        self.declare_parameter('lcd_update_hz', 5.0)
        self.declare_parameter('lcd_page_period_s', 2.0)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.external_steering_mode = bool(self.get_parameter('external_steering_mode').value)
        self.competition_mode = bool(self.get_parameter('competition_mode').value)
        self.require_start_signal = bool(self.get_parameter('require_start_signal').value)
        self.allow_restart_after_stop = bool(self.get_parameter('allow_restart_after_stop').value)
        self.serial_device = str(self.get_parameter('serial_device').value).strip()
        serial_device_candidates_raw = str(
            self.get_parameter('serial_device_candidates').value
        ).strip()
        self.serial_devices = build_device_candidates(
            primary_device=self.serial_device,
            candidates_csv=serial_device_candidates_raw,
        )
        self.serial_enabled = bool(self.serial_devices)

        self.serial_baud = int(self.get_parameter('serial_baud').value)
        serial_baud_candidates_raw = str(
            self.get_parameter('serial_baud_candidates').value
        ).strip()
        self.serial_bauds = build_baud_candidates(
            primary_baud=self.serial_baud,
            candidates_csv=serial_baud_candidates_raw,
        )
        self.serial_open_retry = float(self.get_parameter('serial_open_retry_s').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout_s').value)
        self.telemetry_reconnect = max(
            self.telemetry_timeout,
            float(self.get_parameter('telemetry_reconnect_s').value),
        )
        self.allow_motion_without_telemetry = bool(
            self.get_parameter('allow_motion_without_telemetry').value
        )
        self.lcd_enabled = bool(self.get_parameter('lcd_enabled').value)
        lcd_update_hz = float(self.get_parameter('lcd_update_hz').value)
        self.lcd_period = 0.2 if lcd_update_hz <= 0.0 else (1.0 / lcd_update_hz)
        self.lcd_page_period = max(0.1, float(self.get_parameter('lcd_page_period_s').value))
        self.serial_probes = build_probe_pairs(self.serial_devices, self.serial_bauds)

        self.started = not self.require_start_signal
        self.stopped = False
        self.last_cmd = DriveCommand()
        self.last_cmd_time = time.monotonic()
        self.last_watchdog_brake = False

        self.serial_conn: Optional[object] = None
        self.last_serial_attempt = 0.0
        self.serial_probe_index = 0
        self.active_serial_device = ''
        self.active_serial_baud = 0
        self.serial_open_time = 0.0
        self.last_telemetry_time = 0.0
        self.last_wheel_speed = 0.0
        self.last_rear_obstacle = False
        self.last_status_code = 0
        self.parse_error_until = 0.0
        self.seq = 0
        self.lcd_seq = 0
        self.last_lcd_send_time = 0.0
        self.last_lcd_page_flip_time = time.monotonic()
        self.lcd_page_index = 0

        self.last_bridge_status = 'USB_DISCONNECTED'
        self.last_steering_status = 'USB_DISCONNECTED'
        self.last_car_mode = 'IDLE'
        self.last_race_telemetry = RaceTelemetry()
        self.last_recovery_state = RecoveryState()

        self.create_subscription(DriveCommand, '/cmd_drive', self.cmd_cb, 20)
        self.create_subscription(Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(Bool, '/race_stop', self.stop_cb, 10)
        self.create_subscription(String, '/bridge_status', self.bridge_status_cb, 10)
        self.create_subscription(String, '/steering_status', self.steering_status_cb, 10)
        self.create_subscription(String, '/car_mode', self.car_mode_cb, 10)
        self.create_subscription(RaceTelemetry, '/race_telemetry', self.race_telemetry_cb, 10)
        self.create_subscription(RecoveryState, '/recovery_state', self.recovery_state_cb, 10)

        self.wheel_speed_pub = self.create_publisher(Float32, '/wheel_speed', 20)
        self.rear_obstacle_pub = self.create_publisher(Bool, '/rear_obstacle', 10)
        self.status_pub = self.create_publisher(String, '/bridge_status', 10)
        self.lowlevel_cmd_pub = self.create_publisher(DriveCommand, '/stm32/cmd_drive', 20)

        self.timer = self.create_timer(0.02, self.tick)
        if serial is None:
            self.get_logger().error('pyserial not available; bridge will stay in USB_DISCONNECTED state.')
        elif self.serial_enabled:
            self.get_logger().info(
                f'STM32 serial candidates: devices={self.serial_devices}, bauds={self.serial_bauds}'
            )
        else:
            self.get_logger().warn('No STM32 serial device candidates configured; bridge disabled.')
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

    def bridge_status_cb(self, msg: String) -> None:
        self.last_bridge_status = str(msg.data)

    def steering_status_cb(self, msg: String) -> None:
        self.last_steering_status = str(msg.data)

    def car_mode_cb(self, msg: String) -> None:
        self.last_car_mode = str(msg.data)

    def race_telemetry_cb(self, msg: RaceTelemetry) -> None:
        self.last_race_telemetry = msg

    def recovery_state_cb(self, msg: RecoveryState) -> None:
        self.last_recovery_state = msg

    def _neutral_cmd(self) -> DriveCommand:
        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.speed_mps = 0.0
        cmd.steer_rad = 0.0
        cmd.emergency_brake = True
        return cmd

    def _limited_cmd(self) -> tuple[DriveCommand, bool, bool]:
        cmd = self._neutral_cmd()

        if not self.started:
            return cmd, False, False

        age = time.monotonic() - self.last_cmd_time
        if age > self.watchdog_timeout:
            return cmd, False, True

        cmd.speed_mps = max(-0.8, min(self.max_speed, self.last_cmd.speed_mps))
        cmd.steer_rad = max(-self.max_steer, min(self.max_steer, self.last_cmd.steer_rad))
        cmd.steer_rad = apply_external_steering_mode(
            steer_rad=float(cmd.steer_rad),
            external_steering_mode=self.external_steering_mode,
        )
        cmd.emergency_brake = bool(self.last_cmd.emergency_brake)
        return cmd, True, False

    def _advance_serial_probe(self) -> None:
        if not self.serial_probes:
            return
        self.serial_probe_index = (self.serial_probe_index + 1) % len(self.serial_probes)

    def _close_serial(self, *, advance_probe: bool = False) -> None:
        if self.serial_conn is None:
            return
        try:
            self.serial_conn.close()
        except Exception:
            pass
        self.serial_conn = None
        self.active_serial_device = ''
        self.active_serial_baud = 0
        self.serial_open_time = 0.0
        if advance_probe:
            self._advance_serial_probe()

    def _ensure_serial(self, now: float) -> bool:
        if serial is None:
            return False
        if not self.serial_enabled:
            return False
        if not self.serial_probes:
            return False
        if self.serial_conn is not None and self.serial_conn.is_open:
            return True
        if now - self.last_serial_attempt < self.serial_open_retry:
            return False

        device, baud = self.serial_probes[self.serial_probe_index]
        self.last_serial_attempt = now
        try:
            self.serial_conn = serial.Serial(
                port=device,
                baudrate=baud,
                timeout=0.0,
                write_timeout=0.0,
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.active_serial_device = device
            self.active_serial_baud = int(baud)
            self.serial_open_time = now
            self.get_logger().info(
                f'Connected to STM32 serial: {device} @ {baud} baud'
            )
            return True
        except Exception as exc:
            self.serial_conn = None
            self.get_logger().warn(
                f'Unable to open STM32 serial device {device} @ {baud}: {exc}'
            )
            self._advance_serial_probe()
            return False

    def _telemetry_stream_stale(self, now: float) -> bool:
        if self.serial_conn is None:
            return False
        # In no-telemetry override mode, keep the chosen link stable once race is armed.
        if self.allow_motion_without_telemetry and self.started:
            return False
        reference = self.last_telemetry_time if self.last_telemetry_time > 0.0 else self.serial_open_time
        if reference <= 0.0:
            return False
        return (now - reference) > self.telemetry_reconnect

    def _read_telemetry(self, now: float) -> None:
        if self.serial_conn is None:
            return
        for _ in range(20):
            try:
                raw = self.serial_conn.readline()
            except SerialException:
                self._close_serial(advance_probe=True)
                return

            if not raw:
                return

            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            try:
                seq, wheel_speed, rear_obstacle, status_code = parse_telemetry_line(line)
            except Exception:
                self.parse_error_until = now + 1.0
                self.get_logger().warn(f'Invalid STM32 telemetry line: {line!r}')
                continue

            self.seq = max(self.seq, seq + 1)
            self.last_wheel_speed = wheel_speed
            self.last_rear_obstacle = rear_obstacle
            self.last_status_code = status_code
            self.last_telemetry_time = now

    def _send_command(self, cmd: DriveCommand, run_enable: bool) -> None:
        if self.serial_conn is None:
            return
        try:
            line = encode_command_line(
                seq=self.seq,
                steer_deg=math.degrees(float(cmd.steer_rad)),
                speed_mps=float(cmd.speed_mps),
                run_enable=bool(run_enable),
                emergency_brake=bool(cmd.emergency_brake),
            )
            self.serial_conn.write(line.encode('ascii'))
            self.seq += 1
        except SerialException:
            self._close_serial(advance_probe=True)

    def _maybe_flip_lcd_page(self, now: float) -> None:
        if (now - self.last_lcd_page_flip_time) < self.lcd_page_period:
            return
        self.lcd_page_index = (self.lcd_page_index + 1) % 2
        self.last_lcd_page_flip_time = now

    def _build_lcd_snapshot(self) -> LcdStatusSnapshot:
        telemetry = self.last_race_telemetry
        recovery = self.last_recovery_state
        mode = (self.last_car_mode or '').strip() or telemetry.mode or 'IDLE'
        return LcdStatusSnapshot(
            bridge_status=self.last_bridge_status,
            steering_status=self.last_steering_status,
            car_mode=mode,
            cmd_speed_mps=float(telemetry.cmd_speed_mps),
            wheel_speed_mps=float(self.last_wheel_speed),
            front_clearance_m=float(telemetry.front_clearance_m),
            track_quality_score=float(telemetry.track_quality_score),
            emergency_brake=bool(telemetry.emergency_brake),
            safety_veto=bool(telemetry.safety_veto),
            recovery_state=int(recovery.state),
            recovery_attempt=int(recovery.attempt),
            recovery_reason=str(recovery.reason),
            wrong_direction_confidence=float(telemetry.wrong_direction_confidence),
            rear_obstacle=bool(self.last_rear_obstacle),
        )

    def _send_lcd(self, now: float) -> None:
        if not self.lcd_enabled:
            return
        if self.serial_conn is None:
            return
        if (now - self.last_lcd_send_time) < self.lcd_period:
            return

        self._maybe_flip_lcd_page(now)
        snapshot = self._build_lcd_snapshot()
        lines = build_lcd_lines(snapshot, self.lcd_page_index)

        try:
            line = encode_lcd_line(seq=self.lcd_seq, lines=lines)
            self.serial_conn.write(line.encode('ascii'))
            self.lcd_seq += 1
            self.last_lcd_send_time = now
        except SerialException:
            self._close_serial(advance_probe=True)

    def _usb_state(self, now: float, has_serial: bool) -> str:
        if not has_serial:
            return 'USB_DISCONNECTED'
        if (now - self.last_telemetry_time) > self.telemetry_timeout:
            if self.allow_motion_without_telemetry:
                return 'USB_NO_TELEMETRY'
            return 'USB_TIMEOUT'
        if now < self.parse_error_until:
            return 'USB_PARSE_ERROR'
        return 'OK'

    def tick(self) -> None:
        now = time.monotonic()
        has_serial = self._ensure_serial(now)
        if has_serial:
            self._read_telemetry(now)
            if self._telemetry_stream_stale(now):
                stale_for = now - (
                    self.last_telemetry_time
                    if self.last_telemetry_time > 0.0
                    else self.serial_open_time
                )
                self.get_logger().warn(
                    f'No STM32 telemetry from {self.active_serial_device} @ {self.active_serial_baud} '
                    f'for {stale_for:.2f}s; trying next serial candidate.'
                )
                self._close_serial(advance_probe=True)
                has_serial = False

        limited_cmd, run_enable, watchdog_brake = self._limited_cmd()
        self.last_watchdog_brake = watchdog_brake

        usb_state = self._usb_state(now, has_serial)
        motion_enabled = usb_state in ('OK', 'USB_NO_TELEMETRY')
        if not motion_enabled:
            cmd = self._neutral_cmd()
            run_enable = False
        else:
            cmd = limited_cmd

        self._send_command(cmd, run_enable)
        self.lowlevel_cmd_pub.publish(cmd)

        wheel = Float32()
        wheel.data = float(self.last_wheel_speed if usb_state == 'OK' else 0.0)
        self.wheel_speed_pub.publish(wheel)

        rear = Bool()
        rear.data = bool(self.last_rear_obstacle if usb_state == 'OK' else False)
        self.rear_obstacle_pub.publish(rear)

        status = String()
        if usb_state == 'USB_NO_TELEMETRY':
            if not self.started:
                status.data = 'WAIT_START'
            elif self.last_watchdog_brake:
                status.data = 'WATCHDOG_BRAKE'
            else:
                status.data = 'RUN_NO_TELEMETRY'
        elif usb_state != 'OK':
            status.data = usb_state
        elif not self.started or self.last_status_code == 2:
            status.data = 'WAIT_START'
        elif self.last_watchdog_brake or self.last_status_code in (1, 3):
            status.data = 'WATCHDOG_BRAKE'
        else:
            status.data = 'RUN'
        self.last_bridge_status = status.data
        self.status_pub.publish(status)
        self._send_lcd(now)

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
