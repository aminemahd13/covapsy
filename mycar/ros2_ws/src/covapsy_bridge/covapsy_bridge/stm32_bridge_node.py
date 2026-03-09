"""COVAPSY actuator bridge with pluggable backends: spi, uart, pi_pwm."""

from __future__ import annotations

import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String

from covapsy_bridge.command_mapping import DriveLimits
from covapsy_bridge.command_mapping import PwmCalibration
from covapsy_bridge.command_mapping import build_spi_frame_6b
from covapsy_bridge.command_mapping import build_uart_drive_frame
from covapsy_bridge.command_mapping import clamp_drive_command
from covapsy_bridge.command_mapping import parse_spi_frame_6b
from covapsy_bridge.command_mapping import pwm_duties_from_command
from covapsy_bridge.race_control import RaceControlPolicy
from covapsy_bridge.race_control import RaceControlState
from covapsy_bridge.race_control import apply_race_signal
from covapsy_bridge.race_control import race_command_allowed
from covapsy_bridge.race_profiles import resolve_profile_speed_cap


class STM32BridgeNode(Node):
    """Bridge ROS2 drive topics to hardware backends."""

    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        # Common parameters
        self.declare_parameter("backend", "spi")
        self.declare_parameter("watchdog_timeout", 0.25)
        self.declare_parameter("max_steering_deg", 18.0)
        self.declare_parameter("max_speed_fwd", 2.0)
        self.declare_parameter("max_speed_rev", -4.0)
        self.declare_parameter("race_profile", "RACE_STABLE")
        self.declare_parameter("deployment_mode", "real")
        self.declare_parameter("max_speed_real_cap", 2.0)
        self.declare_parameter("max_speed_sim_cap", 2.5)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("competition_mode", True)
        self.declare_parameter("require_start_signal", True)
        self.declare_parameter("allow_restart_after_stop", False)

        # UART parameters
        self.declare_parameter("uart_port", "/dev/ttyAMA0")
        self.declare_parameter("uart_baudrate", 115200)

        # SPI parameters
        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_device", 1)
        self.declare_parameter("spi_speed_hz", 1000000)
        self.declare_parameter("spi_mode", 0)
        self.declare_parameter("spi_header_0", 0x55)
        self.declare_parameter("spi_header_1", 0x55)
        self.declare_parameter("spi_steering_index", 2)
        self.declare_parameter("spi_speed_index", 3)
        self.declare_parameter("spi_flags_index", 4)

        # pi_pwm parameters
        self.declare_parameter("pwm_hz", 50)
        self.declare_parameter("pwm_prop_channel", 0)
        self.declare_parameter("pwm_dir_channel", 1)
        self.declare_parameter("pwm_prop_stop", 7.5)
        self.declare_parameter("pwm_prop_deadband", 0.4)
        self.declare_parameter("pwm_prop_delta_max", 1.5)
        self.declare_parameter("pwm_speed_hard", 8.0)
        self.declare_parameter("pwm_direction_sign", -1.0)
        self.declare_parameter("pwm_angle_deg_max", 18.0)
        self.declare_parameter("pwm_angle_min", 5.5)
        self.declare_parameter("pwm_angle_max", 9.3)
        self.declare_parameter("pwm_angle_center", 7.4)

        self.backend = str(self.get_parameter("backend").value).lower()
        self.watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.race_policy = RaceControlPolicy(
            competition_mode=bool(self.get_parameter("competition_mode").value),
            require_start_signal=bool(self.get_parameter("require_start_signal").value),
            allow_restart_after_stop=bool(self.get_parameter("allow_restart_after_stop").value),
        )
        self.race_state = RaceControlState(
            race_running=not self.race_policy.require_start_signal,
            stop_latched=False,
        )

        # ROS interface
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 20)
        self.create_subscription(Bool, "/race_start", self.race_start_cb, 10)
        self.create_subscription(Bool, "/race_stop", self.race_stop_cb, 10)
        self.create_subscription(String, "/tft_command", self.tft_callback, 10)

        self.speed_pub = self.create_publisher(Float32, "/wheel_speed", 10)
        self.rear_pub = self.create_publisher(Bool, "/rear_obstacle", 10)
        self.status_pub = self.create_publisher(String, "/mcu_status", 10)

        profile_speed_cap = resolve_profile_speed_cap(
            race_profile=str(self.get_parameter("race_profile").value),
            deployment_mode=str(self.get_parameter("deployment_mode").value),
            max_speed_real_cap=float(self.get_parameter("max_speed_real_cap").value),
            max_speed_sim_cap=float(self.get_parameter("max_speed_sim_cap").value),
        )
        max_speed_fwd = min(float(self.get_parameter("max_speed_fwd").value), profile_speed_cap)

        self.limits = DriveLimits(
            max_steering_deg=float(self.get_parameter("max_steering_deg").value),
            max_speed_fwd=max_speed_fwd,
            max_speed_rev=float(self.get_parameter("max_speed_rev").value),
        )
        self.pwm_cal = PwmCalibration(
            prop_stop=float(self.get_parameter("pwm_prop_stop").value),
            prop_deadband=float(self.get_parameter("pwm_prop_deadband").value),
            prop_delta_max=float(self.get_parameter("pwm_prop_delta_max").value),
            speed_hard=float(self.get_parameter("pwm_speed_hard").value),
            direction_sign=float(self.get_parameter("pwm_direction_sign").value),
            angle_deg_max=float(self.get_parameter("pwm_angle_deg_max").value),
            angle_min=float(self.get_parameter("pwm_angle_min").value),
            angle_max=float(self.get_parameter("pwm_angle_max").value),
            angle_center=float(self.get_parameter("pwm_angle_center").value),
        )

        self.serial = None
        self.spi = None
        self.pwm_prop = None
        self.pwm_dir = None

        self.current_steering_deg = 0.0
        self.current_speed_m_s = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_status = "starting"

        self._init_backend()

        self.create_timer(0.01, self.poll_backend)
        self.create_timer(0.05, self.watchdog_check)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"STM32 bridge started with backend='{self.backend}' "
            f"(cmd_topic='{self.cmd_topic}', competition_mode={self.race_policy.competition_mode}, "
            f"profile_cap={profile_speed_cap:.2f}m/s)"
        )
        if self.race_policy.competition_mode and self.race_policy.require_start_signal:
            self.get_logger().info("Drive commands are gated until /race_start=true")

    def _init_backend(self) -> None:
        if self.backend == "uart":
            self._init_uart_backend()
        elif self.backend == "spi":
            self._init_spi_backend()
        elif self.backend == "pi_pwm":
            self._init_pwm_backend()
        else:
            self.last_status = f"error:unknown_backend:{self.backend}"
            self.get_logger().error(self.last_status)

    def _init_uart_backend(self) -> None:
        port = str(self.get_parameter("uart_port").value)
        baud = int(self.get_parameter("uart_baudrate").value)
        try:
            import serial

            self.serial = serial.Serial(port, baud, timeout=0.01)
            self.last_status = f"ok:uart:{port}@{baud}"
        except Exception as exc:
            self.last_status = f"error:uart_open:{exc}"
            self.get_logger().warn(self.last_status)

    def _init_spi_backend(self) -> None:
        bus = int(self.get_parameter("spi_bus").value)
        dev = int(self.get_parameter("spi_device").value)
        speed = int(self.get_parameter("spi_speed_hz").value)
        mode = int(self.get_parameter("spi_mode").value)
        try:
            import spidev

            self.spi = spidev.SpiDev()
            self.spi.open(bus, dev)
            self.spi.max_speed_hz = speed
            self.spi.mode = mode
            self.last_status = f"ok:spi:bus{bus}.dev{dev}@{speed}"
        except Exception as exc:
            self.last_status = f"error:spi_open:{exc}"
            self.get_logger().warn(self.last_status)

    def _init_pwm_backend(self) -> None:
        hz = int(self.get_parameter("pwm_hz").value)
        prop_channel = int(self.get_parameter("pwm_prop_channel").value)
        dir_channel = int(self.get_parameter("pwm_dir_channel").value)
        try:
            from rpi_hardware_pwm import HardwarePWM

            self.pwm_prop = HardwarePWM(pwm_channel=prop_channel, hz=hz)
            self.pwm_dir = HardwarePWM(pwm_channel=dir_channel, hz=hz)
            self.pwm_prop.start(float(self.pwm_cal.prop_stop))
            self.pwm_dir.start(float(self.pwm_cal.angle_center))
            self.last_status = f"ok:pi_pwm:prop{prop_channel}/dir{dir_channel}"
        except Exception as exc:
            self.last_status = f"error:pi_pwm_init:{exc}"
            self.get_logger().warn(self.last_status)

    def race_start_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        prev = self.race_state
        self.race_state = apply_race_signal(
            self.race_state,
            start=True,
            stop=False,
            policy=self.race_policy,
        )
        if self.race_state == prev:
            if (
                self.race_policy.competition_mode
                and self.race_state.stop_latched
                and not self.race_policy.allow_restart_after_stop
            ):
                self.get_logger().warn("Ignoring /race_start because stop latch is active")
            return
        self.get_logger().info("Race control: start accepted, drive commands enabled")

    def race_stop_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.race_state = apply_race_signal(
            self.race_state,
            start=False,
            stop=True,
            policy=self.race_policy,
        )
        self._set_stop_output()
        self.get_logger().info("Race control: stop accepted, drive output forced to zero")

    def cmd_callback(self, msg: Twist) -> None:
        if not race_command_allowed(self.race_policy, self.race_state):
            return
        steering_deg, speed = clamp_drive_command(msg.angular.z, msg.linear.x, self.limits)
        self.current_steering_deg = steering_deg
        self.current_speed_m_s = speed
        self.last_cmd_time = time.monotonic()
        self._send_drive(steering_deg, speed)

    def tft_callback(self, msg: String) -> None:
        if self.backend != "uart" or self.serial is None:
            return
        try:
            self.serial.write((msg.data + "\n").encode("ascii", errors="ignore"))
        except Exception:
            pass

    def _send_drive(self, steering_deg: float, speed_m_s: float) -> None:
        if self.backend == "uart":
            self._send_uart_drive(steering_deg, speed_m_s)
        elif self.backend == "spi":
            self._send_spi_drive(steering_deg, speed_m_s)
        elif self.backend == "pi_pwm":
            self._send_pwm_drive(steering_deg, speed_m_s)

    def _set_stop_output(self) -> None:
        if abs(self.current_speed_m_s) > 1e-3 or abs(self.current_steering_deg) > 1e-3:
            self.current_speed_m_s = 0.0
            self.current_steering_deg = 0.0
            self._send_drive(0.0, 0.0)

    def _send_uart_drive(self, steering_deg: float, speed_m_s: float) -> None:
        if self.serial is None:
            return
        frame = build_uart_drive_frame(steering_deg, speed_m_s)
        try:
            self.serial.write(frame)
        except Exception as exc:
            self.last_status = f"error:uart_write:{exc}"

    def _send_spi_drive(self, steering_deg: float, speed_m_s: float) -> None:
        if self.spi is None:
            return
        command_flags = 0x01 if race_command_allowed(self.race_policy, self.race_state) else 0x00
        frame = build_spi_frame_6b(
            steering_deg=steering_deg,
            speed_m_s=speed_m_s,
            limits=self.limits,
            command_flags=command_flags,
            header0=int(self.get_parameter("spi_header_0").value),
            header1=int(self.get_parameter("spi_header_1").value),
            steering_index=int(self.get_parameter("spi_steering_index").value),
            speed_index=int(self.get_parameter("spi_speed_index").value),
            flags_index=int(self.get_parameter("spi_flags_index").value),
        )
        try:
            rx = bytes(self.spi.xfer2(list(frame)))
            speed, rear = parse_spi_frame_6b(
                rx,
                limits=self.limits,
                header0=int(self.get_parameter("spi_header_0").value),
                header1=int(self.get_parameter("spi_header_1").value),
                speed_index=int(self.get_parameter("spi_speed_index").value),
                flags_index=int(self.get_parameter("spi_flags_index").value),
            )
            self._publish_speed(speed)
            self._publish_rear(rear)
        except Exception as exc:
            self.last_status = f"error:spi_xfer:{exc}"

    def _send_pwm_drive(self, steering_deg: float, speed_m_s: float) -> None:
        if self.pwm_prop is None or self.pwm_dir is None:
            return
        prop_duty, dir_duty = pwm_duties_from_command(speed_m_s, steering_deg, self.pwm_cal)
        try:
            self.pwm_prop.change_duty_cycle(prop_duty)
            self.pwm_dir.change_duty_cycle(dir_duty)
            # Estimated telemetry in direct-Pi mode.
            self._publish_speed(speed_m_s)
            self._publish_rear(False)
        except Exception as exc:
            self.last_status = f"error:pi_pwm_write:{exc}"

    def poll_backend(self) -> None:
        if self.backend == "uart" and self.serial is not None:
            self._poll_uart()

    def _poll_uart(self) -> None:
        """Parse minimal UART telemetry frames from STM32."""
        try:
            while self.serial.in_waiting >= 8:
                if self.serial.read(1) != b"\xAA":
                    continue
                if self.serial.read(1) != b"\x55":
                    continue

                msg_type_raw = self.serial.read(1)
                if len(msg_type_raw) < 1:
                    continue
                msg_type = msg_type_raw[0]

                if msg_type == 0x02:
                    data = self.serial.read(5)
                    if len(data) >= 4:
                        speed = struct.unpack("<f", data[:4])[0]
                        self._publish_speed(speed)
                elif msg_type == 0x03:
                    data = self.serial.read(2)
                    if len(data) >= 1:
                        self._publish_rear(bool(data[0]))
                else:
                    _ = self.serial.read(1)
        except Exception:
            pass

    def watchdog_check(self) -> None:
        if not race_command_allowed(self.race_policy, self.race_state):
            self._set_stop_output()
            return
        if (time.monotonic() - self.last_cmd_time) > self.watchdog_timeout:
            self._set_stop_output()

    def _publish_speed(self, speed_m_s: float) -> None:
        msg = Float32()
        msg.data = float(speed_m_s)
        self.speed_pub.publish(msg)

    def _publish_rear(self, rear_obstacle: bool) -> None:
        msg = Bool()
        msg.data = bool(rear_obstacle)
        self.rear_pub.publish(msg)

    def publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"backend={self.backend};{self.last_status};"
            f"race_running={int(self.race_state.race_running)};"
            f"stop_latched={int(self.race_state.stop_latched)}"
        )
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._send_drive(0.0, 0.0)
        except Exception:
            pass

        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass

        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass

        if self.pwm_prop is not None:
            try:
                self.pwm_prop.stop()
            except Exception:
                pass

        if self.pwm_dir is not None:
            try:
                self.pwm_dir.stop()
            except Exception:
                pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
