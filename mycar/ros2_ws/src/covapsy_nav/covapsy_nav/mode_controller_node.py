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
  /rear_obstacle    (Bool)   - rear IR obstacle sensor
  /wheel_speed      (Float32)- actual wheel speed from bridge
  /set_mode         (String) - optional external mode switch (disabled by default)

Publishes:
  command_topic (Twist) - final command to bridge backend
  /car_mode  (String) - current mode for TFT display
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from covapsy_nav.mode_rules import DriveCommand, select_mode_command, select_recovery_command

_ACTIVE_MODES = ('REACTIVE', 'MAPPING', 'RACING')
_VALID_MODES = ('IDLE', 'REACTIVE', 'MAPPING', 'RACING', 'STOPPED')
_EMERGENCY_STOP_DIST = 0.15
_PHASE_DURATIONS = (0.20, 0.10)  # phase 0 (brake), phase 1 (neutral) in seconds


class ModeControllerNode(Node):

    def __init__(self):
        super().__init__('mode_controller')

        self.declare_parameter('mapping_speed_cap', 0.5)
        self.declare_parameter('initial_mode', 'IDLE')
        self.declare_parameter('start_mode', 'REACTIVE')
        self.declare_parameter('stuck_timeout', 2.0)
        self.declare_parameter('reverse_speed', -0.4)
        self.declare_parameter('reverse_duration', 1.0)
        self.declare_parameter('reverse_steer', 0.3)
        self.declare_parameter('max_reverse_distance', 0.8)
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('allow_runtime_mode_switch', False)
        self.declare_parameter('lock_mode_after_start', True)

        initial_mode = str(self.get_parameter('initial_mode').value).upper()
        self.mode = initial_mode if initial_mode in _VALID_MODES else 'IDLE'
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.race_started = self.mode in _ACTIVE_MODES

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel_reactive', self.reactive_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_pursuit', self.pursuit_cb, 10)
        self.create_subscription(Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(Bool, '/race_stop', self.stop_cb, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.create_subscription(Bool, '/rear_obstacle', self.rear_cb, 10)
        self.create_subscription(Float32, '/wheel_speed', self.wheel_speed_cb, 10)
        self.create_subscription(String, '/set_mode', self.set_mode_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.mode_pub = self.create_publisher(String, '/car_mode', 10)

        # Drive state
        self.reactive_cmd = Twist()
        self.pursuit_cmd = Twist()
        self.has_map = False
        self.min_front_dist = float('inf')
        self.rear_blocked = False
        self.wheel_speed = 0.0

        # Stuck detection & recovery state
        self.stopped_since: float = 0.0
        self.is_reversing = False
        self.reverse_phase = 0
        self.reverse_phase_start: float = 0.0
        self.reverse_distance = 0.0
        self.last_loop_time: float = 0.0
        self.recovery_count = 0
        self.recovery_steer_sign = 1.0

        # Control loop at 50 Hz
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f'Mode controller started in {self.mode} mode '
            f'(command_topic={self.command_topic})'
        )

    # ---- Callbacks ----

    def start_cb(self, msg):
        if not msg.data:
            return
        if self.mode == 'STOPPED':
            self.get_logger().warn('Ignoring race_start because mode is STOPPED')
            return
        if self.race_started:
            return
        start_mode = str(self.get_parameter('start_mode').value).upper()
        self.mode = start_mode if start_mode in _ACTIVE_MODES else 'REACTIVE'
        self.race_started = True
        self._reset_recovery()
        self.get_logger().info(f'Race started -> {self.mode} mode')

    def stop_cb(self, msg):
        if msg.data:
            self.mode = 'STOPPED'
            self.race_started = False
            self._reset_recovery()
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
        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        front_mask = np.abs(angles) < np.radians(15)
        front = ranges[front_mask]
        front = front[np.isfinite(front)]
        if len(front) > 0:
            self.min_front_dist = float(np.min(front))

    def rear_cb(self, msg: Bool):
        self.rear_blocked = bool(msg.data)

    def wheel_speed_cb(self, msg: Float32):
        self.wheel_speed = float(msg.data)

    def set_mode_cb(self, msg: String):
        self.set_mode(str(msg.data).strip().upper(), from_external=True)

    def set_mode(self, new_mode: str, from_external: bool = False):
        """Externally set mode (e.g., from a service or parameter change)."""
        if from_external:
            allow_runtime_switch = bool(self.get_parameter('allow_runtime_mode_switch').value)
            lock_after_start = bool(self.get_parameter('lock_mode_after_start').value)
            if not allow_runtime_switch:
                self.get_logger().warn('Ignoring /set_mode (allow_runtime_mode_switch=false)')
                return
            if lock_after_start and self.race_started:
                self.get_logger().warn('Ignoring /set_mode because race has started')
                return

        if new_mode in _VALID_MODES:
            self.mode = new_mode
            self.race_started = self.mode in _ACTIVE_MODES
            self._reset_recovery()
            self.get_logger().info(f'Mode changed to: {new_mode}')

    # ---- Control loop ----

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_loop_time if self.last_loop_time > 0.0 else 0.02
        self.last_loop_time = now

        if self.is_reversing:
            selected = self._run_reverse(now, dt)
        else:
            selected = self._run_normal(now)

        cmd = Twist()
        cmd.linear.x = selected.linear_x
        cmd.angular.z = selected.angular_z
        self.cmd_pub.publish(cmd)

        mode_msg = String()
        mode_msg.data = self.mode if not self.is_reversing else 'REVERSING'
        self.mode_pub.publish(mode_msg)

    def _run_normal(self, now: float) -> DriveCommand:
        selected = select_mode_command(
            mode=self.mode,
            reactive_cmd=DriveCommand(
                linear_x=float(self.reactive_cmd.linear.x),
                angular_z=float(self.reactive_cmd.angular.z),
            ),
            pursuit_cmd=DriveCommand(
                linear_x=float(self.pursuit_cmd.linear.x),
                angular_z=float(self.pursuit_cmd.angular.z),
            ),
            mapping_speed_cap=float(self.get_parameter('mapping_speed_cap').value),
            min_front_dist=float(self.min_front_dist),
        )

        # Stuck detection: car wanted to move but output is zero due to obstacle
        if self.mode in _ACTIVE_MODES and selected.linear_x == 0.0:
            if self.stopped_since == 0.0:
                self.stopped_since = now
            elif (now - self.stopped_since) > float(self.get_parameter('stuck_timeout').value):
                self._start_reverse(now)
                return self._run_reverse(now, 0.02)
        else:
            # Car is moving or in IDLE/STOPPED — reset stuck timer
            if self.stopped_since > 0.0:
                self.stopped_since = 0.0
                self.recovery_count = 0

        return selected

    def _start_reverse(self, now: float):
        self.is_reversing = True
        self.reverse_phase = 0
        self.reverse_phase_start = now
        self.reverse_distance = 0.0
        self.recovery_count += 1
        self.recovery_steer_sign *= -1.0
        self.get_logger().info(
            f'Recovery #{self.recovery_count}: starting ESC reverse '
            f'(steer sign={self.recovery_steer_sign:+.0f})'
        )

    def _run_reverse(self, now: float, dt: float) -> DriveCommand:
        elapsed = now - self.reverse_phase_start

        # Phase transitions based on timing
        if self.reverse_phase == 0 and elapsed >= _PHASE_DURATIONS[0]:
            self.reverse_phase = 1
            self.reverse_phase_start = now
            elapsed = 0.0
        elif self.reverse_phase == 1 and elapsed >= _PHASE_DURATIONS[1]:
            self.reverse_phase = 2
            self.reverse_phase_start = now
            self.reverse_distance = 0.0
            elapsed = 0.0

        # Phase 2 end conditions
        reverse_duration = float(self.get_parameter('reverse_duration').value)
        max_reverse_dist = float(self.get_parameter('max_reverse_distance').value)
        if self.reverse_phase == 2:
            self.reverse_distance += abs(self.wheel_speed) * dt
            if elapsed >= reverse_duration or self.reverse_distance >= max_reverse_dist:
                self._end_reverse()
                return DriveCommand()

        # Compute steer with escalating angle and alternating direction
        base_steer = float(self.get_parameter('reverse_steer').value)
        steer = base_steer * min(self.recovery_count, 3) * self.recovery_steer_sign

        return select_recovery_command(
            phase=self.reverse_phase,
            reverse_speed=float(self.get_parameter('reverse_speed').value),
            reverse_steer=steer,
            rear_blocked=self.rear_blocked,
        )

    def _end_reverse(self):
        self.get_logger().info(
            f'Recovery #{self.recovery_count} complete '
            f'(reversed ~{self.reverse_distance:.2f}m)'
        )
        self._reset_recovery()

    def _reset_recovery(self):
        self.is_reversing = False
        self.reverse_phase = 0
        self.reverse_phase_start = 0.0
        self.reverse_distance = 0.0
        self.stopped_since = 0.0


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
