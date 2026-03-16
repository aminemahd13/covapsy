#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from covapsy_interfaces.msg import DriveCommand
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ReactiveDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('reactive_driver_node')

        self.declare_parameter('car_half_width_m', 0.11)
        self.declare_parameter('safety_margin_m', 0.06)
        self.declare_parameter('forward_fov_deg', 200.0)
        self.declare_parameter('safety_bubble_radius_idx', 8)
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('steer_slew_rad_s', 2.0)
        self.declare_parameter('min_speed_mps', 0.15)
        self.declare_parameter('max_speed_mps', 0.9)
        self.declare_parameter('ttc_limit_s', 1.0)
        self.declare_parameter('target_center_bias', 0.25)

        self.car_half_width_m = float(self.get_parameter('car_half_width_m').value)
        self.safety_margin_m = float(self.get_parameter('safety_margin_m').value)
        self.forward_fov_deg = float(self.get_parameter('forward_fov_deg').value)
        self.safety_bubble_radius_idx = int(self.get_parameter('safety_bubble_radius_idx').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.steer_slew = float(self.get_parameter('steer_slew_rad_s').value)
        self.min_speed = float(self.get_parameter('min_speed_mps').value)
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.ttc_limit_s = float(self.get_parameter('ttc_limit_s').value)
        self.center_bias = float(self.get_parameter('target_center_bias').value)

        self.last_steer = 0.0
        self.last_time = self.get_clock().now()

        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)
        self.cmd_pub = self.create_publisher(DriveCommand, '/cmd_drive_reactive', 20)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/reactive_debug', 20)

        self.get_logger().info('reactive_driver_node ready')

    def _forward_indices(self, msg: LaserScan) -> Tuple[int, int]:
        half = math.radians(self.forward_fov_deg * 0.5)
        n = len(msg.ranges)
        mid = n // 2
        span = int(half / msg.angle_increment)
        return max(0, mid - span), min(n - 1, mid + span)

    def _apply_safety_bubble(self, ranges: List[float], idx_min: int, idx_max: int) -> None:
        nearest_idx = idx_min
        nearest_val = ranges[idx_min]
        for i in range(idx_min + 1, idx_max + 1):
            if ranges[i] < nearest_val:
                nearest_val = ranges[i]
                nearest_idx = i
        s = max(idx_min, nearest_idx - self.safety_bubble_radius_idx)
        e = min(idx_max, nearest_idx + self.safety_bubble_radius_idx)
        for i in range(s, e + 1):
            ranges[i] = 0.0

    def _apply_disparity_extension(self, ranges: List[float], angle_increment: float, idx_min: int, idx_max: int) -> None:
        footprint = self.car_half_width_m + self.safety_margin_m
        i = idx_min
        while i < idx_max:
            a = ranges[i]
            b = ranges[i + 1]
            if a <= 0.0 or b <= 0.0:
                i += 1
                continue
            if abs(a - b) > 0.15:
                near = min(a, b)
                extend_angle = math.atan2(footprint, max(near, 0.05))
                extend_bins = int(extend_angle / max(angle_increment, 1e-6))
                if a < b:
                    for k in range(i + 1, min(idx_max + 1, i + 1 + extend_bins)):
                        ranges[k] = min(ranges[k], a)
                else:
                    for k in range(max(idx_min, i - extend_bins), i + 1):
                        ranges[k] = min(ranges[k], b)
            i += 1

    def _extract_gaps(self, ranges: List[float], idx_min: int, idx_max: int) -> List[Tuple[int, int]]:
        gaps = []
        start = None
        for i in range(idx_min, idx_max + 1):
            if ranges[i] > 0.05 and start is None:
                start = i
            if (ranges[i] <= 0.05 or i == idx_max) and start is not None:
                end = i if ranges[i] > 0.05 else i - 1
                if end >= start:
                    gaps.append((start, end))
                start = None
        return gaps

    def _select_target(self, ranges: List[float], gaps: List[Tuple[int, int]], msg: LaserScan) -> int:
        if not gaps:
            return len(ranges) // 2
        scored = []
        center = len(ranges) // 2
        for s, e in gaps:
            width = e - s + 1
            mean_r = sum(ranges[s:e + 1]) / float(width)
            gap_center = (s + e) // 2
            center_bonus = 1.0 - min(1.0, abs(gap_center - center) / max(1.0, center))
            score = mean_r + 0.35 * width * msg.angle_increment + self.center_bias * center_bonus
            scored.append((score, s, e))
        _, s, e = max(scored, key=lambda x: x[0])
        return (s + e) // 2

    def _steer_from_target(self, target_idx: int, msg: LaserScan) -> float:
        angle = msg.angle_min + target_idx * msg.angle_increment
        steer = max(-self.max_steer, min(self.max_steer, angle))
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        dt = max(dt, 1e-3)
        max_step = self.steer_slew * dt
        steer = max(self.last_steer - max_step, min(self.last_steer + max_step, steer))
        self.last_time = now
        self.last_steer = steer
        return steer

    def _speed_policy(self, front_clear: float, steer: float) -> float:
        steer_factor = 1.0 - min(1.0, abs(steer) / max(self.max_steer, 1e-3))
        clearance_factor = min(1.0, max(0.0, (front_clear - 0.25) / 1.5))
        speed = self.min_speed + (self.max_speed - self.min_speed) * (0.5 * clearance_factor + 0.5 * steer_factor)
        if speed < self.min_speed:
            speed = self.min_speed
        if speed > self.max_speed:
            speed = self.max_speed

        v_for_ttc = max(speed, 0.05)
        ttc = front_clear / v_for_ttc
        if ttc < self.ttc_limit_s:
            speed *= max(0.0, ttc / max(self.ttc_limit_s, 1e-3))
        return max(0.0, speed)

    def _sector_min_positive(self, ranges: List[float], center_idx: int, half_window: int) -> float:
        n = len(ranges)
        s = max(0, center_idx - half_window)
        e = min(n - 1, center_idx + half_window)
        vals = [r for r in ranges[s:e + 1] if r > 0.05]
        if not vals:
            return 0.05
        return min(vals)

    def scan_cb(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return

        raw_ranges = [max(0.0, min(msg.range_max, float(r))) for r in msg.ranges]
        ranges = list(raw_ranges)
        idx_min, idx_max = self._forward_indices(msg)

        self._apply_safety_bubble(ranges, idx_min, idx_max)
        self._apply_disparity_extension(ranges, msg.angle_increment, idx_min, idx_max)
        gaps = self._extract_gaps(ranges, idx_min, idx_max)
        target_idx = self._select_target(ranges, gaps, msg)

        steer = self._steer_from_target(target_idx, msg)
        front_idx = len(raw_ranges) // 2
        center_clear = self._sector_min_positive(raw_ranges, front_idx, max(1, len(raw_ranges) // 60))
        target_clear = self._sector_min_positive(raw_ranges, target_idx, max(1, len(raw_ranges) // 90))

        # When turning, allow the free-space in the steering direction to dominate speed gating.
        if abs(steer) > 0.12:
            front_clear = max(center_clear, 0.9 * target_clear)
        else:
            front_clear = center_clear

        speed = self._speed_policy(front_clear, steer)

        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.steer_rad = steer
        cmd.speed_mps = speed
        self.cmd_pub.publish(cmd)

        dbg = Float32MultiArray()
        dbg.data = [
            float(front_clear),
            float(target_idx),
            float(steer),
            float(speed),
            float(len(gaps)),
        ]
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
