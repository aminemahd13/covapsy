"""Periodic runtime monitor logs for both simulation and real-car bringup.

This node is intentionally read-only: it subscribes to key topics and emits
human-readable health snapshots when logging is enabled.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String

from covapsy_nav.runtime_monitor_utils import parse_status_blob


def _fmt_twist(cmd: Twist | None) -> str:
    if cmd is None:
        return "(n/a)"
    return f"({cmd.linear.x:+.2f},{cmd.angular.z:+.2f})"


class RuntimeMonitorNode(Node):
    """Runtime logger that can be toggled on/off by launch arg or param set."""

    def __init__(self) -> None:
        super().__init__("runtime_monitor")

        self.declare_parameter("enable_logs", False)
        self.declare_parameter("log_period_s", 1.0)
        self.declare_parameter("stale_timeout_s", 1.5)
        self.declare_parameter("command_topic", "/cmd_vel")

        command_topic = str(self.get_parameter("command_topic").value)

        self.create_subscription(String, "/car_mode", self.mode_cb, 10)
        self.create_subscription(Twist, command_topic, self.cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_reactive", self.reactive_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_pursuit", self.pursuit_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_tactical", self.tactical_cb, 10)
        self.create_subscription(Float32, "/wheel_speed", self.wheel_speed_cb, 10)
        self.create_subscription(Bool, "/rear_obstacle", self.rear_cb, 10)
        self.create_subscription(Float32, "/opponent_confidence", self.opp_cb, 10)
        self.create_subscription(String, "/mcu_status", self.mcu_status_cb, 10)

        self.mode = "n/a"
        self.final_cmd: Twist | None = None
        self.reactive_cmd: Twist | None = None
        self.pursuit_cmd: Twist | None = None
        self.tactical_cmd: Twist | None = None
        self.wheel_speed: float | None = None
        self.rear_blocked: bool | None = None
        self.opp_confidence: float | None = None
        self.mcu_fields: dict[str, str] = {}
        self.last_rx_time: dict[str, float] = {}

        now = time.monotonic()
        self.last_log_time = now
        self.last_rx_time["init"] = now
        self.prev_enabled = bool(self.get_parameter("enable_logs").value)

        # Fast timer; effective logging rate is controlled by log_period_s.
        self.create_timer(0.20, self.tick)
        self.get_logger().info(
            f"Runtime monitor ready (command_topic={command_topic}, "
            "toggle via param enable_logs)"
        )

    def mode_cb(self, msg: String) -> None:
        self.mode = str(msg.data)
        self.last_rx_time["mode"] = time.monotonic()

    def cmd_cb(self, msg: Twist) -> None:
        self.final_cmd = msg
        self.last_rx_time["final_cmd"] = time.monotonic()

    def reactive_cb(self, msg: Twist) -> None:
        self.reactive_cmd = msg
        self.last_rx_time["reactive_cmd"] = time.monotonic()

    def pursuit_cb(self, msg: Twist) -> None:
        self.pursuit_cmd = msg
        self.last_rx_time["pursuit_cmd"] = time.monotonic()

    def tactical_cb(self, msg: Twist) -> None:
        self.tactical_cmd = msg
        self.last_rx_time["tactical_cmd"] = time.monotonic()

    def wheel_speed_cb(self, msg: Float32) -> None:
        self.wheel_speed = float(msg.data)
        self.last_rx_time["wheel_speed"] = time.monotonic()

    def rear_cb(self, msg: Bool) -> None:
        self.rear_blocked = bool(msg.data)
        self.last_rx_time["rear"] = time.monotonic()

    def opp_cb(self, msg: Float32) -> None:
        self.opp_confidence = float(msg.data)
        self.last_rx_time["opp"] = time.monotonic()

    def mcu_status_cb(self, msg: String) -> None:
        self.mcu_fields = parse_status_blob(str(msg.data))
        self.last_rx_time["mcu"] = time.monotonic()

    def _is_stale(self, key: str, now: float, stale_timeout_s: float) -> bool:
        ts = self.last_rx_time.get(key)
        if ts is None:
            return True
        return (now - ts) > stale_timeout_s

    def tick(self) -> None:
        now = time.monotonic()
        enabled = bool(self.get_parameter("enable_logs").value)
        if enabled != self.prev_enabled:
            state = "ENABLED" if enabled else "DISABLED"
            self.get_logger().info(f"Runtime logs {state}")
            self.prev_enabled = enabled

        if not enabled:
            return

        period = max(0.20, float(self.get_parameter("log_period_s").value))
        if (now - self.last_log_time) < period:
            return
        self.last_log_time = now

        stale_timeout = max(0.20, float(self.get_parameter("stale_timeout_s").value))
        stale_tags = []
        for key in ("final_cmd", "reactive_cmd", "mode"):
            if self._is_stale(key, now, stale_timeout):
                stale_tags.append(key)

        backend = self.mcu_fields.get("backend", "n/a")
        race_running = self.mcu_fields.get("race_running", "n/a")
        stop_latched = self.mcu_fields.get("stop_latched", "n/a")

        wheel = "n/a" if self.wheel_speed is None else f"{self.wheel_speed:+.2f}"
        rear = "n/a" if self.rear_blocked is None else str(int(self.rear_blocked))
        opp = "n/a" if self.opp_confidence is None else f"{self.opp_confidence:.2f}"
        stale = "none" if not stale_tags else ",".join(stale_tags)

        self.get_logger().info(
            f"mode={self.mode} cmd={_fmt_twist(self.final_cmd)} "
            f"react={_fmt_twist(self.reactive_cmd)} "
            f"pursuit={_fmt_twist(self.pursuit_cmd)} "
            f"tactical={_fmt_twist(self.tactical_cmd)} "
            f"wheel={wheel} rear={rear} opp={opp} "
            f"backend={backend} race={race_running} stop={stop_latched} stale={stale}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
