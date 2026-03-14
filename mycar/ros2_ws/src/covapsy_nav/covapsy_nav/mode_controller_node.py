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
  /cmd_vel_tactical (Twist)  - from tactical_race_node
  /race_start       (Bool)   - start signal
  /race_stop        (Bool)   - stop signal
  /scan_filtered    (LaserScan) - for emergency detection
  /rear_obstacle    (Bool)   - rear IR obstacle sensor
  /wheel_speed      (Float32)- actual wheel speed from bridge
  /odom             (Odometry)- fallback speed estimate for stuck detection
  /set_mode         (String) - optional external mode switch (disabled by default)
  /opponent_confidence (Float32) - tactical context confidence
  /opponent_count   (Float32)- tactical context count
  /wrong_direction_confidence (Float32) - optional confidence supplement

Publishes:
  command_topic (Twist) - final command to bridge backend
  /car_mode  (String) - current mode for TFT display
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Float32, String

from covapsy_nav.mode_rules import DriveCommand, select_mode_command, select_recovery_command

_ACTIVE_MODES = ('REACTIVE', 'MAPPING', 'RACING', 'LEARNING')
_VALID_MODES = ('IDLE', 'REACTIVE', 'MAPPING', 'RACING', 'LEARNING', 'STOPPED')
_EMERGENCY_STOP_DIST = 0.15

# U-turn parameters
_UTURN_BRAKE_DURATION = 0.3  # seconds
_UTURN_REVERSE_DURATION = 1.2
_UTURN_FORWARD_DURATION = 0.8
_UTURN_REVERSE_SPEED = -0.45
_UTURN_FORWARD_SPEED = 0.45


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
        self.declare_parameter('stuck_cmd_speed_min', 0.08)
        self.declare_parameter('stuck_actual_speed_max', 0.06)
        self.declare_parameter('stuck_sensor_stale_sec', 0.40)
        self.declare_parameter('drive_input_stale_sec', 0.45)
        self.declare_parameter('stale_speed_cap', 0.20)
        self.declare_parameter('halt_on_scan_stale', True)
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('allow_runtime_mode_switch', False)
        self.declare_parameter('lock_mode_after_start', True)
        self.declare_parameter('enable_tactical_ai', False)
        self.declare_parameter('tactical_timeout', 0.25)
        self.declare_parameter('tactical_context_mode', 'manual')
        self.declare_parameter('tactical_opp_conf_threshold', 0.45)
        self.declare_parameter('tactical_opp_count_threshold', 0.5)
        self.declare_parameter('tactical_opp_persist_sec', 0.60)
        self.declare_parameter('tactical_clear_persist_sec', 1.20)
        self.declare_parameter('auto_switch_to_racing', True)
        self.declare_parameter('track_learned_handoff_confirm_sec', 0.50)
        self.declare_parameter('learning_speed_cap', 0.8)
        self.declare_parameter('wrong_direction_trigger_distance_m', 1.20)
        self.declare_parameter('wrong_direction_conf_enter', 0.55)
        self.declare_parameter('wrong_direction_conf_exit', 0.35)
        self.declare_parameter('wrong_direction_confirm_ticks', 6)
        self.declare_parameter('wrong_direction_distance_confirm_m', 0.25)
        self.declare_parameter('wrong_direction_correction_trigger_ticks', 8)
        self.declare_parameter('wrong_direction_correction_distance_m', 0.45)
        self.declare_parameter('wrong_direction_correction_speed_m_s', 0.18)
        self.declare_parameter('wrong_direction_correction_steer_rad', 0.22)
        self.declare_parameter('wrong_direction_correction_max_sec', 0.70)
        self.declare_parameter('wrong_direction_uturn_trigger_ticks', 20)
        self.declare_parameter('uturn_steer', 0.32)
        self.declare_parameter('uturn_blocked_forward_speed', 0.20)
        self.declare_parameter('uturn_blocked_forward_sec', 0.60)
        # ESC arming phase durations — set to 0.0 in simulation to skip arming sequence
        self.declare_parameter('phase0_duration', 0.20)
        self.declare_parameter('phase1_duration', 0.10)
        # Recovery cap: after this many consecutive failures, pause for cooldown
        self.declare_parameter('max_recovery_attempts', 30)
        self.declare_parameter('recovery_cooldown_sec', 10.0)

        initial_mode = str(self.get_parameter('initial_mode').value).upper()
        self.mode = initial_mode if initial_mode in _VALID_MODES else 'IDLE'
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.race_started = self.mode in _ACTIVE_MODES

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel_reactive', self.reactive_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_pursuit', self.pursuit_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_tactical', self.tactical_cb, 10)
        self.create_subscription(Bool, '/race_start', self.start_cb, 10)
        self.create_subscription(Bool, '/race_stop', self.stop_cb, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/rear_obstacle', self.rear_cb, 10)
        self.create_subscription(Float32, '/wheel_speed', self.wheel_speed_cb, 10)
        self.create_subscription(Float32, '/opponent_confidence', self.opponent_conf_cb, 10)
        self.create_subscription(Float32, '/opponent_count', self.opponent_count_cb, 10)
        self.create_subscription(String, '/set_mode', self.set_mode_cb, 10)
        self.create_subscription(Bool, '/track_learned', self.track_learned_cb, 10)
        self.create_subscription(Bool, '/depth_obstacle', self.depth_obstacle_cb, 10)
        self.create_subscription(Bool, '/wrong_direction', self.wrong_direction_cb, 10)
        self.create_subscription(Float32, '/wrong_direction_confidence', self.wrong_direction_conf_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 20)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.mode_pub = self.create_publisher(String, '/car_mode', 10)

        # Drive state
        self.reactive_cmd = Twist()
        self.pursuit_cmd = Twist()
        self.tactical_cmd = Twist()
        self.last_tactical_time = 0.0
        self.has_map = False
        self.min_front_dist = float('inf')
        self.rear_blocked = False
        self.depth_obstacle = False
        self.wheel_speed = 0.0
        self.odom_speed = 0.0
        self.track_learned = False
        self.track_learned_since: float = 0.0
        self.opponent_confidence = 0.0
        self.opponent_count = 0.0

        # IMU state for improved stuck/spin detection
        self.imu_yaw_rate = 0.0
        self.imu_lon_accel = 0.0

        # Stuck detection & recovery state
        self.stopped_since: float = 0.0
        self.is_reversing = False
        self.reverse_phase = 0
        self.reverse_phase_start: float = 0.0
        self.reverse_distance = 0.0
        self.last_loop_time: float = 0.0
        self.recovery_count = 0
        self.recovery_steer_sign = 1.0
        self.recovery_cooldown_until: float = 0.0
        # Guard: only use wheel_speed-based checks once data has been received
        self.wheel_speed_received = False
        self.odom_speed_received = False
        self.last_scan_time: float = 0.0
        self.last_reactive_time: float = 0.0
        self.last_pursuit_time: float = 0.0
        self.last_wheel_speed_time: float = 0.0
        self.last_odom_time: float = 0.0
        self._last_odom_xy: tuple[float, float] | None = None
        self.wrong_dir_distance_m: float = 0.0
        # Tactical context latching (auto mode)
        self._opp_candidate_since: float = 0.0
        self._opp_last_seen: float = 0.0
        self._opp_context_active: bool = False

        # U-turn (wrong-direction recovery)
        self.wrong_direction = False
        self.wrong_direction_confidence = 0.0
        self.wrong_direction_conf_ticks = 0
        self.wrong_direction_conf_active = False
        self.wrong_dir_counter = 0
        self.is_uturn = False
        self.uturn_phase = 0        # 0=off, 1=brake, 2=reverse, 3=forward, 4=rear-blocked forward
        self.uturn_phase_start = 0.0
        self.uturn_steer_sign = 1.0
        self.is_wrong_direction_correcting = False
        self.wrong_direction_correction_since = 0.0
        self.wrong_direction_correction_sign = 1.0

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
        self.last_reactive_time = time.monotonic()

    def pursuit_cb(self, msg):
        self.pursuit_cmd = msg
        self.last_pursuit_time = time.monotonic()

    def tactical_cb(self, msg):
        self.tactical_cmd = msg
        self.last_tactical_time = time.monotonic()

    def scan_cb(self, msg: LaserScan):
        self.last_scan_time = time.monotonic()
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

    def odom_cb(self, msg: Odometry):
        lin = msg.twist.twist.linear
        self.odom_speed = float(np.hypot(lin.x, lin.y))
        self.odom_speed_received = True
        self.last_odom_time = time.monotonic()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._last_odom_xy is not None:
            dx = x - self._last_odom_xy[0]
            dy = y - self._last_odom_xy[1]
            ds = math.hypot(dx, dy)
            if math.isfinite(ds) and ds < 2.0:
                wrong_signal = self._wrong_direction_signal_for_distance()
                if wrong_signal:
                    self.wrong_dir_distance_m += ds
                else:
                    self.wrong_dir_distance_m = max(0.0, self.wrong_dir_distance_m - ds * 1.5)
        self._last_odom_xy = (x, y)

    def wheel_speed_cb(self, msg: Float32):
        self.wheel_speed = float(msg.data)
        self.wheel_speed_received = True
        self.last_wheel_speed_time = time.monotonic()

    def opponent_conf_cb(self, msg: Float32):
        self.opponent_confidence = max(0.0, min(1.0, float(msg.data)))

    def opponent_count_cb(self, msg: Float32):
        self.opponent_count = max(0.0, float(msg.data))

    def set_mode_cb(self, msg: String):
        self.set_mode(str(msg.data).strip().upper(), from_external=True)

    def track_learned_cb(self, msg: Bool):
        if msg.data and not self.track_learned:
            self.track_learned = True
            self.track_learned_since = time.monotonic()
            self.get_logger().info('Track learned signal received')

    def depth_obstacle_cb(self, msg: Bool):
        self.depth_obstacle = bool(msg.data)

    def wrong_direction_cb(self, msg: Bool):
        self.wrong_direction = bool(msg.data)

    def wrong_direction_conf_cb(self, msg: Float32):
        self.wrong_direction_confidence = max(0.0, min(1.0, float(msg.data)))

    def imu_cb(self, msg: Imu):
        self.imu_yaw_rate = float(msg.angular_velocity.z)
        self.imu_lon_accel = float(msg.linear_acceleration.x)

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

        if self.is_uturn:
            selected = self._run_uturn(now)
        elif self.is_reversing:
            selected = self._run_reverse(now, dt)
        else:
            selected = self._run_normal(now)

        cmd = Twist()
        cmd.linear.x = selected.linear_x
        cmd.angular.z = selected.angular_z
        self.cmd_pub.publish(cmd)

        mode_msg = String()
        if self.is_uturn:
            mode_msg.data = 'UTURN'
        elif self.is_reversing:
            mode_msg.data = 'REVERSING'
        else:
            mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

    def _run_normal(self, now: float) -> DriveCommand:
        if (
            self.mode == 'LEARNING'
            and self.track_learned
            and bool(self.get_parameter('auto_switch_to_racing').value)
            and self.track_learned_since > 0.0
            and (now - self.track_learned_since)
            >= float(self.get_parameter('track_learned_handoff_confirm_sec').value)
        ):
            self.mode = 'RACING'
            self.get_logger().info('Auto-switched to RACING mode (track handoff confirmed)')

        tactical_timeout = float(self.get_parameter('tactical_timeout').value)
        tactical_context_ok = self._tactical_context_allows(now=now)
        tactical_enabled = (
            bool(self.get_parameter('enable_tactical_ai').value)
            and tactical_context_ok
            and (now - self.last_tactical_time) <= tactical_timeout
        )
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
            tactical_cmd=DriveCommand(
                linear_x=float(self.tactical_cmd.linear.x),
                angular_z=float(self.tactical_cmd.angular.z),
            ),
            tactical_enabled=tactical_enabled,
            mapping_speed_cap=float(self.get_parameter('mapping_speed_cap').value),
            min_front_dist=float(self.min_front_dist),
        )

        drive_stale_sec = max(0.05, float(self.get_parameter('drive_input_stale_sec').value))
        stale_speed_cap = max(0.0, float(self.get_parameter('stale_speed_cap').value))
        has_recent_scan = self.last_scan_time > 0.0 and (now - self.last_scan_time) <= drive_stale_sec
        has_recent_reactive = (
            self.last_reactive_time > 0.0 and (now - self.last_reactive_time) <= drive_stale_sec
        )
        has_recent_pursuit = (
            self.last_pursuit_time > 0.0 and (now - self.last_pursuit_time) <= drive_stale_sec
        )
        has_recent_tactical = (now - self.last_tactical_time) <= tactical_timeout

        # Freshness guard: stale sensor/control streams must not sustain high-speed commands.
        if self.mode in _ACTIVE_MODES:
            if not has_recent_scan and bool(self.get_parameter('halt_on_scan_stale').value):
                return DriveCommand()

            if self.mode in ('REACTIVE', 'MAPPING', 'LEARNING'):
                input_fresh = has_recent_reactive
            else:
                input_fresh = has_recent_pursuit or has_recent_reactive or (
                    tactical_enabled and has_recent_tactical
                )

            if not input_fresh and selected.linear_x > stale_speed_cap:
                selected = DriveCommand(linear_x=stale_speed_cap, angular_z=selected.angular_z)

        # Near-field depth obstacle guard.
        if self.mode in _ACTIVE_MODES and self.depth_obstacle and selected.linear_x > 0.0:
            selected = DriveCommand(linear_x=0.0, angular_z=selected.angular_z * 0.5)

        # Stuck detection: only trigger recovery when we are commanding forward
        # motion but measured speed indicates no progress.
        stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        cmd_speed_min = float(self.get_parameter('stuck_cmd_speed_min').value)
        speed_idle_max = float(self.get_parameter('stuck_actual_speed_max').value)
        stale_sec = float(self.get_parameter('stuck_sensor_stale_sec').value)

        speed_known, actual_speed = self._current_speed(now=now, stale_sec=stale_sec)
        requested_motion = selected.linear_x > cmd_speed_min
        stalled = speed_known and actual_speed < speed_idle_max
        has_recent_scan = self.last_scan_time > 0.0 and (now - self.last_scan_time) <= stale_sec
        has_recent_reactive = (
            self.last_reactive_time > 0.0 and (now - self.last_reactive_time) <= stale_sec
        )

        ws_known = self.wheel_speed_received and (now - self.last_wheel_speed_time) <= stale_sec
        is_spinning = ws_known and abs(self.imu_yaw_rate) > 1.5 and self.wheel_speed < 0.10
        is_impact = ws_known and self.imu_lon_accel < -3.0 and self.wheel_speed < 0.05

        # During cooldown after max recovery attempts, don't trigger more recoveries
        if self.recovery_cooldown_until > 0.0 and now < self.recovery_cooldown_until:
            self.stopped_since = 0.0
            return selected

        should_start_recovery = (
            self.mode in _ACTIVE_MODES
            and has_recent_scan
            and has_recent_reactive
            and (
                (requested_motion and (stalled or is_spinning or is_impact))
                or is_spinning
                or is_impact
            )
        )

        if should_start_recovery:
            if self.stopped_since == 0.0:
                self.stopped_since = now
            # Use shorter timeout when IMU confirms stuck condition
            effective_timeout = stuck_timeout * 0.5 if (is_spinning or is_impact) else stuck_timeout
            if (now - self.stopped_since) > effective_timeout:
                self._start_reverse(now)
                return self._run_reverse(now, 0.02)
        else:
            # Car is moving or in IDLE/STOPPED — reset stuck timer
            if self.stopped_since > 0.0:
                self.stopped_since = 0.0
                self.recovery_count = 0

        wrong_direction_active = self._wrong_direction_active()

        # Wrong-direction detection → U-turn trigger
        if self.mode in _ACTIVE_MODES and wrong_direction_active:
            self.wrong_dir_counter += 1
            early_dist = float(self.get_parameter('wrong_direction_trigger_distance_m').value)
            correction_ticks = max(
                1, int(self.get_parameter('wrong_direction_correction_trigger_ticks').value)
            )
            correction_dist = max(
                0.1, float(self.get_parameter('wrong_direction_correction_distance_m').value)
            )
            uturn_ticks = max(
                correction_ticks + 1,
                int(self.get_parameter('wrong_direction_uturn_trigger_ticks').value),
            )

            if (
                not self.is_wrong_direction_correcting
                and (
                    self.wrong_dir_counter >= correction_ticks
                    or self.wrong_dir_distance_m >= correction_dist
                )
            ):
                self._start_wrong_direction_correction(now=now, selected=selected)

            if self.is_wrong_direction_correcting:
                correction_max_sec = max(
                    0.2, float(self.get_parameter('wrong_direction_correction_max_sec').value)
                )
                if (
                    (now - self.wrong_direction_correction_since) >= correction_max_sec
                    or self.wrong_dir_counter >= uturn_ticks
                    or self.wrong_dir_distance_m >= max(0.2, early_dist)
                ):
                    self._clear_wrong_direction_correction()
                    self._start_uturn(now)
                    return DriveCommand()

                corr_speed = max(
                    0.0, float(self.get_parameter('wrong_direction_correction_speed_m_s').value)
                )
                corr_steer = max(
                    0.0, float(self.get_parameter('wrong_direction_correction_steer_rad').value)
                )
                return DriveCommand(
                    linear_x=corr_speed,
                    angular_z=corr_steer * self.wrong_direction_correction_sign,
                )

            if (
                self.wrong_dir_counter >= uturn_ticks
                or self.wrong_dir_distance_m >= max(0.2, early_dist)
            ):
                self._start_uturn(now)
                return DriveCommand()
        else:
            self.wrong_dir_counter = max(0, self.wrong_dir_counter - 2)
            self.wrong_dir_distance_m = max(0.0, self.wrong_dir_distance_m - 0.05)
            self._clear_wrong_direction_correction()

        return selected

    def _start_reverse(self, now: float):
        max_attempts = int(self.get_parameter('max_recovery_attempts').value)
        if self.recovery_count >= max_attempts:
            cooldown_sec = float(self.get_parameter('recovery_cooldown_sec').value)
            self.recovery_cooldown_until = now + cooldown_sec
            self.recovery_count = 0
            self.stopped_since = 0.0
            self.get_logger().warn(
                f'Max recovery attempts ({max_attempts}) reached; '
                f'cooling down for {cooldown_sec:.1f}s'
            )
            return
        self.is_reversing = True
        self.reverse_phase = 0
        self.reverse_phase_start = now
        self.reverse_distance = 0.0
        self.recovery_count += 1
        self.recovery_steer_sign *= -1.0
        self.get_logger().info(
            f'Recovery #{self.recovery_count}: starting reverse '
            f'(steer sign={self.recovery_steer_sign:+.0f})'
        )

    def _run_reverse(self, now: float, dt: float) -> DriveCommand:
        elapsed = now - self.reverse_phase_start

        # Phase transitions based on timing (durations are 0.0 in sim to skip ESC arming)
        phase0_dur = float(self.get_parameter('phase0_duration').value)
        phase1_dur = float(self.get_parameter('phase1_duration').value)
        if self.reverse_phase == 0 and elapsed >= phase0_dur:
            self.reverse_phase = 1
            self.reverse_phase_start = now
            elapsed = 0.0
        elif self.reverse_phase == 1 and elapsed >= phase1_dur:
            self.reverse_phase = 2
            self.reverse_phase_start = now
            self.reverse_distance = 0.0
            elapsed = 0.0

        # Phase 2 end conditions
        reverse_duration = float(self.get_parameter('reverse_duration').value)
        max_reverse_dist = float(self.get_parameter('max_reverse_distance').value)
        if self.reverse_phase == 2:
            speed_known, actual_speed = self._current_speed(now=now, stale_sec=0.60)
            if speed_known:
                self.reverse_distance += abs(actual_speed) * dt
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

    # ---- U-turn (wrong direction recovery) ----

    def _start_uturn(self, now: float):
        self._clear_wrong_direction_correction()
        self.is_uturn = True
        self.uturn_phase = 1  # brake
        self.uturn_phase_start = now
        # Alternate steer direction each U-turn for robustness
        self.uturn_steer_sign *= -1.0
        self.get_logger().info(
            f'U-turn triggered (wrong direction for '
            f'{self.wrong_dir_counter} ticks, steer_sign={self.uturn_steer_sign:+.0f})'
        )

    def _run_uturn(self, now: float) -> DriveCommand:
        elapsed = now - self.uturn_phase_start
        uturn_steer = max(0.0, min(0.55, float(self.get_parameter('uturn_steer').value)))

        if self.uturn_phase == 1:
            # Phase 1: brake
            if elapsed >= _UTURN_BRAKE_DURATION:
                if self.rear_blocked:
                    self.uturn_phase = 4
                    self.get_logger().warn(
                        'U-turn reverse blocked by rear obstacle; using forward-only correction'
                    )
                else:
                    self.uturn_phase = 2
                self.uturn_phase_start = now
                if self.uturn_phase == 2:
                    self.get_logger().info('U-turn phase 2: reverse with full lock')
            return DriveCommand()

        if self.uturn_phase == 2:
            # Phase 2: reverse with full lock steering
            if self.rear_blocked:
                self.uturn_phase = 4
                self.uturn_phase_start = now
                self.get_logger().warn(
                    'Rear obstacle during U-turn reverse; switching to forward-only correction'
                )
                return DriveCommand()
            if elapsed >= _UTURN_REVERSE_DURATION:
                self.uturn_phase = 3
                self.uturn_phase_start = now
                self.get_logger().info('U-turn phase 3: forward to complete turn')
            return DriveCommand(
                linear_x=_UTURN_REVERSE_SPEED,
                angular_z=uturn_steer * self.uturn_steer_sign,
            )

        if self.uturn_phase == 3:
            # Phase 3: forward with full lock to finish turn
            if elapsed >= _UTURN_FORWARD_DURATION:
                self._end_uturn()
                return DriveCommand()
            return DriveCommand(
                linear_x=_UTURN_FORWARD_SPEED,
                angular_z=uturn_steer * self.uturn_steer_sign,
            )

        if self.uturn_phase == 4:
            # Rear is blocked; do bounded low-speed forward-only correction.
            blocked_sec = max(0.2, float(self.get_parameter('uturn_blocked_forward_sec').value))
            if elapsed >= blocked_sec:
                self._end_uturn()
                return DriveCommand()
            blocked_speed = max(0.0, float(self.get_parameter('uturn_blocked_forward_speed').value))
            return DriveCommand(
                linear_x=blocked_speed,
                angular_z=uturn_steer * self.uturn_steer_sign,
            )

        # Shouldn't happen, but graceful fallback
        self._end_uturn()
        return DriveCommand()

    def _end_uturn(self):
        self.get_logger().info('U-turn complete, resuming normal driving')
        self.is_uturn = False
        self.uturn_phase = 0
        self.uturn_phase_start = 0.0
        self.wrong_dir_counter = 0
        self.wrong_dir_distance_m = 0.0

    def _tactical_context_allows(self, now: float) -> bool:
        mode = str(self.get_parameter('tactical_context_mode').value).strip().lower()
        if mode != 'auto':
            return True

        conf_thr = float(self.get_parameter('tactical_opp_conf_threshold').value)
        count_thr = float(self.get_parameter('tactical_opp_count_threshold').value)
        enter_sec = float(self.get_parameter('tactical_opp_persist_sec').value)
        clear_sec = float(self.get_parameter('tactical_clear_persist_sec').value)

        has_opp_now = (
            self.opponent_confidence >= max(0.0, min(1.0, conf_thr))
            or self.opponent_count >= max(0.0, count_thr)
        )
        if has_opp_now:
            if self._opp_candidate_since == 0.0:
                self._opp_candidate_since = now
            self._opp_last_seen = now
            if (now - self._opp_candidate_since) >= max(0.0, enter_sec):
                self._opp_context_active = True
        else:
            self._opp_candidate_since = 0.0
            if self._opp_context_active and (now - self._opp_last_seen) > max(0.0, clear_sec):
                self._opp_context_active = False

        return self._opp_context_active

    def _wrong_direction_signal_for_distance(self) -> bool:
        conf_exit = float(self.get_parameter('wrong_direction_conf_exit').value)
        return self.wrong_direction or self.wrong_direction_confidence >= max(0.0, conf_exit)

    def _wrong_direction_active(self) -> bool:
        conf_enter = float(self.get_parameter('wrong_direction_conf_enter').value)
        conf_exit = float(self.get_parameter('wrong_direction_conf_exit').value)
        confirm_ticks = max(1, int(self.get_parameter('wrong_direction_confirm_ticks').value))

        if self.wrong_direction_confidence >= conf_enter:
            self.wrong_direction_conf_ticks = min(
                confirm_ticks, self.wrong_direction_conf_ticks + 1
            )
        elif self.wrong_direction_confidence <= conf_exit:
            self.wrong_direction_conf_ticks = max(0, self.wrong_direction_conf_ticks - 1)

        if not self.wrong_direction_conf_active and self.wrong_direction_conf_ticks >= confirm_ticks:
            self.wrong_direction_conf_active = True
        elif (
            self.wrong_direction_conf_active
            and self.wrong_direction_confidence <= conf_exit
            and self.wrong_direction_conf_ticks == 0
        ):
            self.wrong_direction_conf_active = False

        distance_confirm = max(
            0.0, float(self.get_parameter('wrong_direction_distance_confirm_m').value)
        )
        distance_persistent = self.wrong_dir_distance_m >= distance_confirm

        # Backward compatibility: bool topic still works on its own.
        return self.wrong_direction or self.wrong_direction_conf_active or distance_persistent

    def _start_wrong_direction_correction(self, now: float, selected: DriveCommand) -> None:
        if self.is_wrong_direction_correcting:
            return
        self.is_wrong_direction_correcting = True
        self.wrong_direction_correction_since = now
        if abs(selected.angular_z) > 1e-3:
            self.wrong_direction_correction_sign = -1.0 if selected.angular_z >= 0.0 else 1.0
        else:
            self.wrong_direction_correction_sign *= -1.0
        self.get_logger().info(
            f'Wrong-direction corrective steering engaged (sign={self.wrong_direction_correction_sign:+.0f})'
        )

    def _clear_wrong_direction_correction(self) -> None:
        self.is_wrong_direction_correcting = False
        self.wrong_direction_correction_since = 0.0

    def _current_speed(self, now: float, stale_sec: float) -> tuple[bool, float]:
        if (
            self.wheel_speed_received
            and self.last_wheel_speed_time > 0.0
            and (now - self.last_wheel_speed_time) <= stale_sec
        ):
            return True, abs(float(self.wheel_speed))
        if (
            self.odom_speed_received
            and self.last_odom_time > 0.0
            and (now - self.last_odom_time) <= stale_sec
        ):
            return True, abs(float(self.odom_speed))
        return False, 0.0

    def _reset_recovery(self):
        self.is_reversing = False
        self.reverse_phase = 0
        self.reverse_phase_start = 0.0
        self.reverse_distance = 0.0
        self.stopped_since = 0.0
        self.recovery_cooldown_until = 0.0
        self.is_uturn = False
        self.uturn_phase = 0
        self.uturn_phase_start = 0.0
        self.wrong_dir_counter = 0
        self.wrong_dir_distance_m = 0.0
        self.wrong_direction_conf_ticks = 0
        self.wrong_direction_conf_active = False
        self._clear_wrong_direction_correction()


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
