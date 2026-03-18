#!/usr/bin/env python3
import json
import math

import rclpy
from covapsy_interfaces.msg import DriveCommand, RaceTelemetry, RecoveryState, TrackQuality
from covapsy_control.recovery_fsm import RecoveryFSM, RecoveryStage
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, String


class ModeControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('mode_controller_node')
        self.declare_parameter('start_mode', 'IDLE')
        self.declare_parameter('wrong_way_recovery_conf', 0.8)
        self.declare_parameter('progress_speed_thresh_mps', 0.05)
        self.declare_parameter('stuck_cycles', 20)
        self.declare_parameter('front_blocked_m', 0.22)
        self.declare_parameter('race_speed_cap_mps', 1.5)
        self.declare_parameter('learn_speed_cap_mps', 0.7)
        self.declare_parameter('race_entry_quality_threshold', 0.75)
        self.declare_parameter('auto_switch_learn_to_race', True)
        self.declare_parameter('safety_steer_veto_clearance_m', 0.3)
        self.declare_parameter('race_obstacle_avoidance_enabled', True)
        self.declare_parameter('race_reactive_blend_clearance_m', 0.32)
        self.declare_parameter('odom_progress_window_s', 1.0)
        self.declare_parameter('odom_min_displacement_m', 0.02)

        self.mode = str(self.get_parameter('start_mode').value)
        self.wrong_way_recovery_conf = float(self.get_parameter('wrong_way_recovery_conf').value)
        self.progress_speed_thresh = float(self.get_parameter('progress_speed_thresh_mps').value)
        self.stuck_cycles_limit = int(self.get_parameter('stuck_cycles').value)
        self.front_blocked_m = float(self.get_parameter('front_blocked_m').value)
        self.race_speed_cap = float(self.get_parameter('race_speed_cap_mps').value)
        self.learn_speed_cap = float(self.get_parameter('learn_speed_cap_mps').value)
        self.race_entry_quality = float(self.get_parameter('race_entry_quality_threshold').value)
        self.auto_switch_learn_to_race = bool(self.get_parameter('auto_switch_learn_to_race').value)
        self.safety_veto_clearance = float(self.get_parameter('safety_steer_veto_clearance_m').value)
        self.race_obstacle_avoid = bool(self.get_parameter('race_obstacle_avoidance_enabled').value)
        self.race_reactive_blend_clearance = float(self.get_parameter('race_reactive_blend_clearance_m').value)
        self.odom_window = float(self.get_parameter('odom_progress_window_s').value)
        self.odom_min_disp = float(self.get_parameter('odom_min_displacement_m').value)

        self.cmd_reactive = DriveCommand()
        self.cmd_pursuit = DriveCommand()
        self.wrong_direction = False
        self.wrong_conf = 0.0
        self.track_learned = False
        self.track_quality = 0.0
        self.last_scan = None
        self.wheel_speed = 0.0
        self.rear_obstacle = False
        self.stuck_cycles = 0
        self.last_imu = None
        self.last_recovery_trigger_reason = 'none'
        self.odom_history = []  # [(time_s, x, y)]

        self.recovery = RecoveryFSM(max_attempts=2, brake_steps=8, reverse_steps=15, reassess_steps=6)

        self.create_subscription(DriveCommand, '/cmd_drive_reactive', self.reactive_cb, 20)
        self.create_subscription(DriveCommand, '/cmd_drive_pursuit', self.pursuit_cb, 20)
        self.create_subscription(Bool, '/wrong_direction', self.wrong_cb, 10)
        self.create_subscription(Float32, '/wrong_direction_confidence', self.wrong_conf_cb, 10)
        self.create_subscription(Bool, '/track_learned', self.learned_cb, 5)
        self.create_subscription(TrackQuality, '/track_quality', self.quality_cb, 5)
        self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)
        self.create_subscription(Float32, '/wheel_speed', self.wheel_speed_cb, 20)
        self.create_subscription(Bool, '/rear_obstacle', self.rear_obstacle_cb, 20)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 20)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)

        self.cmd_pub = self.create_publisher(DriveCommand, '/cmd_drive', 20)
        self.mode_pub = self.create_publisher(String, '/car_mode', 10)
        self.recovery_pub = self.create_publisher(RecoveryState, '/recovery_state', 10)
        self.recovery_debug_pub = self.create_publisher(String, '/recovery_debug', 10)
        self.telemetry_pub = self.create_publisher(RaceTelemetry, '/race_telemetry', 10)

        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('mode_controller_node ready')

    def reactive_cb(self, msg: DriveCommand) -> None:
        self.cmd_reactive = msg

    def pursuit_cb(self, msg: DriveCommand) -> None:
        self.cmd_pursuit = msg

    def wrong_cb(self, msg: Bool) -> None:
        self.wrong_direction = msg.data

    def wrong_conf_cb(self, msg: Float32) -> None:
        self.wrong_conf = float(msg.data)

    def learned_cb(self, msg: Bool) -> None:
        self.track_learned = bool(msg.data)

    def quality_cb(self, msg: TrackQuality) -> None:
        self.track_quality = float(msg.score)

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def wheel_speed_cb(self, msg: Float32) -> None:
        self.wheel_speed = float(msg.data)

    def rear_obstacle_cb(self, msg: Bool) -> None:
        self.rear_obstacle = bool(msg.data)

    def imu_cb(self, msg: Imu) -> None:
        self.last_imu = msg

    def odom_cb(self, msg: Odometry) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_history.append((t, x, y))
        cutoff = t - self.odom_window * 2.0
        while self.odom_history and self.odom_history[0][0] < cutoff:
            self.odom_history.pop(0)

    def _odom_making_progress(self) -> bool:
        """Check actual displacement over a time window instead of relying on
        commanded wheel_speed, which does not reflect physical motion in sim."""
        if len(self.odom_history) < 2:
            return True  # assume progress while data is sparse
        now_t = self.odom_history[-1][0]
        cutoff = now_t - self.odom_window
        old = None
        for t, x, y in self.odom_history:
            if t >= cutoff:
                old = (x, y)
                break
        if old is None:
            return True
        cur = (self.odom_history[-1][1], self.odom_history[-1][2])
        return math.hypot(cur[0] - old[0], cur[1] - old[1]) > self.odom_min_disp

    def _front_clearance(self) -> float:
        if self.last_scan is None or not self.last_scan.ranges:
            return 10.0
        n = len(self.last_scan.ranges)
        mid = n // 2
        sector = self.last_scan.ranges[max(0, mid - n // 20):min(n, mid + n // 20)]
        return max(0.05, min(sector)) if sector else 10.0

    def _side_clearance(self):
        if self.last_scan is None or not self.last_scan.ranges:
            return 1.0, 1.0
        n = len(self.last_scan.ranges)
        mid = n // 2
        left = self.last_scan.ranges[mid + n // 8:min(n, mid + n // 3)]
        right = self.last_scan.ranges[max(0, mid - n // 3):max(1, mid - n // 8)]
        left_c = min(left) if left else 1.0
        right_c = min(right) if right else 1.0
        return left_c, right_c

    def _recovery_trigger_reason(self, nominal_cmd: DriveCommand, front_clear: float) -> str:
        forward_cmd = nominal_cmd.speed_mps > 0.1
        # Use odom-based progress detection (works in sim, unlike wheel_speed).
        moving = self._odom_making_progress()
        if forward_cmd and not moving:
            self.stuck_cycles += 1
        else:
            self.stuck_cycles = 0  # full reset when progress is detected

        left_c, right_c = self._side_clearance()
        side_asym = abs(left_c - right_c) > 0.18
        wedge_signature = front_clear < self.front_blocked_m and side_asym and forward_cmd and not moving

        wrong_way = self.wrong_direction and self.wrong_conf > self.wrong_way_recovery_conf

        if front_clear < 0.12:
            return 'front_critically_blocked'
        if wrong_way:
            return 'wrong_direction_high_confidence'
        if self.stuck_cycles >= self.stuck_cycles_limit:
            return 'no_progress_stuck'
        if wedge_signature:
            return 'wedge_signature'
        return 'none'

    def _can_forward_reorient(self, front_clear: float, left_clear: float, right_clear: float) -> bool:
        if not self.rear_obstacle:
            return False
        if front_clear < (self.front_blocked_m + 0.10):
            return False
        if max(left_clear, right_clear) < 0.22:
            return False
        return True

    def _safety_overlay(self, nominal: DriveCommand, front_clear: float) -> DriveCommand:
        out = DriveCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        out.steer_rad = nominal.steer_rad
        out.speed_mps = min(nominal.speed_mps, self.race_speed_cap)

        # Keep race tracking nominally on pursuit, but when an obstacle is close
        # in front, progressively blend toward reactive LiDAR steering/speed.
        if self.race_obstacle_avoid and front_clear < self.race_reactive_blend_clearance:
            denom = max(1e-3, self.race_reactive_blend_clearance - self.front_blocked_m)
            alpha = min(1.0, max(0.0, (self.race_reactive_blend_clearance - front_clear) / denom))
            reactive_steer = float(self.cmd_reactive.steer_rad)
            reactive_speed = max(0.0, float(self.cmd_reactive.speed_mps))
            out.steer_rad = (1.0 - alpha) * out.steer_rad + alpha * reactive_steer
            blended_speed = (1.0 - alpha) * out.speed_mps + alpha * reactive_speed
            out.speed_mps = min(out.speed_mps, blended_speed)

        # Near obstacles, reduce speed instead of zeroing steering, which can
        # destabilize cornering and produce straight-into-wall behavior.
        if front_clear < self.safety_veto_clearance and abs(out.steer_rad) > 0.3:
            out.speed_mps = min(out.speed_mps, 0.35)
        if front_clear < self.front_blocked_m:
            out.speed_mps = min(out.speed_mps, 0.15)
        if front_clear < 0.12:
            out.speed_mps = 0.0

        return out

    def _mode_logic(self):
        track_ready = self.track_learned and self.track_quality >= self.race_entry_quality

        if self.mode == 'IDLE':
            if track_ready:
                self.mode = 'RACE'
            else:
                self.mode = 'LEARN'

        if self.mode == 'LEARN' and track_ready and self.auto_switch_learn_to_race:
            self.mode = 'RACE'

        # If race was requested but no valid track is currently available,
        # fall back to LEARN instead of stalling with zero pursuit command.
        if self.mode == 'RACE' and not track_ready:
            self.mode = 'LEARN'

    def tick(self) -> None:
        self._mode_logic()

        front_clear = self._front_clearance()
        nominal = self.cmd_reactive if self.mode == 'LEARN' else self.cmd_pursuit
        safety_veto = False
        emergency_brake = False
        rec_reason = 'none'
        rec_stage = RecoveryStage.NONE
        left_c, right_c = self._side_clearance()

        if self.mode in ('IDLE', 'STOPPED'):
            out = DriveCommand()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.speed_mps = 0.0
            out.steer_rad = 0.0
            out.emergency_brake = True
            emergency_brake = True
        elif self.mode == 'LEARN':
            out = DriveCommand()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.steer_rad = self.cmd_reactive.steer_rad
            out.speed_mps = min(self.cmd_reactive.speed_mps, self.learn_speed_cap)
            out.emergency_brake = False
            # LEARN-mode safety: reduce speed when front is close.
            if front_clear < self.front_blocked_m:
                out.speed_mps = min(out.speed_mps, 0.15)
            if front_clear < 0.12:
                out.speed_mps = 0.0
                out.emergency_brake = True
                emergency_brake = True
            trigger_reason = self._recovery_trigger_reason(out, front_clear)
            if trigger_reason != 'none':
                self.mode = 'RECOVERY'
                self.last_recovery_trigger_reason = trigger_reason
                self.recovery.trigger()
        elif self.mode == 'RACE':
            out = self._safety_overlay(self.cmd_pursuit, front_clear)
            out.emergency_brake = front_clear < 0.12
            emergency_brake = out.emergency_brake
            safety_veto = (
                out.speed_mps < self.cmd_pursuit.speed_mps - 1e-6
                or abs(out.steer_rad - self.cmd_pursuit.steer_rad) > 1e-6
            )
            trigger_reason = self._recovery_trigger_reason(self.cmd_pursuit, front_clear)
            if trigger_reason != 'none':
                self.mode = 'RECOVERY'
                self.last_recovery_trigger_reason = trigger_reason
                self.recovery.trigger()
        else:
            open_sign = -1.0 if left_c > right_c else 1.0
            allow_forward_reorient = self._can_forward_reorient(front_clear, left_c, right_c)
            rec_cmd = self.recovery.step(open_sign, allow_forward_reorient=allow_forward_reorient)
            rec_stage = rec_cmd.stage

            out = DriveCommand()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.speed_mps = rec_cmd.speed_mps
            out.steer_rad = rec_cmd.steer_rad
            out.emergency_brake = rec_cmd.stage == RecoveryStage.BRAKE
            emergency_brake = out.emergency_brake
            rec_reason = rec_cmd.reason

            if rec_cmd.stage == RecoveryStage.REVERSE and self.rear_obstacle:
                boxed_in = front_clear < self.front_blocked_m
                self.recovery.mark_reverse_blocked(boxed_in=boxed_in)
                rec_stage = self.recovery.stage
                out.speed_mps = 0.0
                out.steer_rad = 0.0
                out.emergency_brake = True
                emergency_brake = True
                rec_reason = 'rear_obstacle_blocks_reverse'
                if boxed_in:
                    rec_reason = 'boxed_in_front_and_rear'
                    self.mode = 'STOPPED'

            if rec_cmd.stage == RecoveryStage.FAILSAFE_STOP:
                self.mode = 'STOPPED'
            if rec_cmd.stage == RecoveryStage.ESCALATE:
                self.last_recovery_trigger_reason = rec_reason
                if self.track_learned and self.track_quality >= self.race_entry_quality:
                    self.mode = 'RACE'
                else:
                    self.mode = 'LEARN'

            rs = RecoveryState()
            rs.header.stamp = out.header.stamp
            rs.state = int(rec_cmd.stage)
            rs.attempt = int(self.recovery.attempt)
            rs.reason = rec_reason
            self.recovery_pub.publish(rs)

        if self.mode != 'RECOVERY' and self._odom_making_progress():
            self.recovery.reset_runtime()

        self.cmd_pub.publish(out)

        telemetry = RaceTelemetry()
        telemetry.header.stamp = out.header.stamp
        telemetry.mode = self.mode
        telemetry.cmd_speed_mps = float(out.speed_mps)
        telemetry.cmd_steer_rad = float(out.steer_rad)
        telemetry.speed_cap_mps = float(self.learn_speed_cap if self.mode == 'LEARN' else self.race_speed_cap)
        telemetry.safety_veto = bool(safety_veto)
        telemetry.emergency_brake = bool(emergency_brake)
        telemetry.wrong_direction = bool(self.wrong_direction)
        telemetry.wrong_direction_confidence = float(self.wrong_conf)
        telemetry.front_clearance_m = float(front_clear)
        telemetry.track_quality_score = float(self.track_quality)
        self.telemetry_pub.publish(telemetry)

        recovery_debug = String()
        dbg = {
            'trigger_reason': self.last_recovery_trigger_reason,
            'recovery_state': int(rec_stage),
            'recovery_state_name': RecoveryStage(rec_stage).name,
            'reverse_blocked': bool(self.recovery.reverse_blocked),
            'front_clearance_m': float(front_clear),
            'left_clearance_m': float(left_c),
            'right_clearance_m': float(right_c),
            'attempt_count': int(self.recovery.attempt),
            'stuck_cycles': int(self.stuck_cycles),
            'odom_progress': bool(self._odom_making_progress()),
        }
        recovery_debug.data = json.dumps(dbg, separators=(',', ':'))
        self.recovery_debug_pub.publish(recovery_debug)

        m = String()
        m.data = self.mode
        self.mode_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ModeControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
