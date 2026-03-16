#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from covapsy_interfaces.msg import DirectionState, TrackQuality
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TrackLearnerNode(Node):
    def __init__(self) -> None:
        super().__init__('track_learner_node')
        self.declare_parameter('sample_distance_m', 0.05)
        self.declare_parameter('min_samples', 300)
        self.declare_parameter('min_quality', 0.72)
        self.declare_parameter('lane_half_width_assumed_m', 0.45)
        self.declare_parameter('max_curvature', 2.5)

        self.sample_distance = float(self.get_parameter('sample_distance_m').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.min_quality = float(self.get_parameter('min_quality').value)
        self.lane_half_width = float(self.get_parameter('lane_half_width_assumed_m').value)
        self.max_curvature = float(self.get_parameter('max_curvature').value)

        self.last_scan = None
        self.last_dir = DirectionState.DIR_UNKNOWN
        self.last_pos = None
        self.samples: List[Tuple[float, float]] = []

        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 30)
        self.dir_sub = self.create_subscription(DirectionState, '/track_direction', self.direction_cb, 10)

        self.path_pub = self.create_publisher(Path, '/track_path', 5)
        self.quality_pub = self.create_publisher(TrackQuality, '/track_quality', 5)
        self.learned_pub = self.create_publisher(Bool, '/track_learned', 5)

        self.timer = self.create_timer(0.5, self.publish_outputs)
        self.get_logger().info('track_learner_node ready')

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def direction_cb(self, msg: DirectionState) -> None:
        self.last_dir = msg.state

    def _front_left_right_ranges(self, scan: LaserScan):
        n = len(scan.ranges)
        mid = n // 2
        left_idx = min(n - 1, mid + n // 4)
        right_idx = max(0, mid - n // 4)
        left = max(0.05, min(scan.range_max, scan.ranges[left_idx]))
        right = max(0.05, min(scan.range_max, scan.ranges[right_idx]))
        return left, right

    def odom_cb(self, msg: Odometry) -> None:
        if self.last_scan is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pos is not None:
            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            if math.hypot(dx, dy) < self.sample_distance:
                return

        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        left_d, right_d = self._front_left_right_ranges(self.last_scan)
        lateral_offset = 0.5 * (right_d - left_d)
        lateral_offset = max(-self.lane_half_width, min(self.lane_half_width, lateral_offset))

        mx = x + lateral_offset * -math.sin(yaw)
        my = y + lateral_offset * math.cos(yaw)

        if self.last_dir == DirectionState.DIR_WRONG:
            return

        self.samples.append((mx, my))
        self.last_pos = (x, y)

    def _resample(self, pts: List[Tuple[float, float]], step: float) -> List[Tuple[float, float]]:
        if len(pts) < 2:
            return pts
        out = [pts[0]]
        carry = 0.0
        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            ux = (x1 - x0) / seg
            uy = (y1 - y0) / seg
            d = carry
            while d + step <= seg:
                d += step
                out.append((x0 + ux * d, y0 + uy * d))
            carry = seg - d
        return out

    def _smooth(self, pts: List[Tuple[float, float]], k: int = 5) -> List[Tuple[float, float]]:
        if len(pts) < k:
            return pts
        half = k // 2
        sm = []
        for i in range(len(pts)):
            xs = []
            ys = []
            for j in range(i - half, i + half + 1):
                p = pts[j % len(pts)]
                xs.append(p[0])
                ys.append(p[1])
            sm.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return sm

    def _curvature_stats(self, pts: List[Tuple[float, float]]) -> float:
        if len(pts) < 3:
            return 999.0
        curv = []
        for i in range(1, len(pts) - 1):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            a = math.hypot(x1 - x0, y1 - y0)
            b = math.hypot(x2 - x1, y2 - y1)
            c = math.hypot(x2 - x0, y2 - y0)
            if a < 1e-4 or b < 1e-4:
                continue
            s = (a + b + c) / 2.0
            area_sq = max(0.0, s * (s - a) * (s - b) * (s - c))
            if area_sq < 1e-12:
                continue
            area = math.sqrt(area_sq)
            kappa = 4.0 * area / max(1e-6, a * b * c)
            curv.append(abs(kappa))
        if not curv:
            return 999.0
        return sum(curv) / len(curv)

    def publish_outputs(self) -> None:
        learned = Bool()
        quality = TrackQuality()
        quality.header.stamp = self.get_clock().now().to_msg()

        if len(self.samples) < self.min_samples:
            learned.data = False
            quality.score = 0.0
            quality.is_valid = False
            quality.reason = 'not_enough_samples'
            self.learned_pub.publish(learned)
            self.quality_pub.publish(quality)
            return

        pts = self._resample(self.samples, self.sample_distance)
        pts = self._smooth(pts, 7)
        if len(pts) > 20:
            pts.append(pts[0])

        closure = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        spacing = []
        for i in range(1, len(pts)):
            spacing.append(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        spacing_mean = sum(spacing) / max(1, len(spacing))
        spacing_std = math.sqrt(sum((v - spacing_mean) ** 2 for v in spacing) / max(1, len(spacing)))
        curv = self._curvature_stats(pts)

        closure_score = max(0.0, 1.0 - closure / 1.0)
        spacing_score = max(0.0, 1.0 - spacing_std / max(0.05, self.sample_distance))
        smoothness = max(0.0, 1.0 - curv / max(1e-3, self.max_curvature))
        lap_consistency = min(1.0, len(self.samples) / float(self.min_samples * 2))

        score = 0.35 * closure_score + 0.25 * spacing_score + 0.25 * smoothness + 0.15 * lap_consistency

        quality.score = float(score)
        quality.closure_error_m = float(closure)
        quality.smoothness = float(smoothness)
        quality.spacing_stddev = float(spacing_std)
        quality.lap_consistency = float(lap_consistency)
        quality.is_valid = bool(score >= self.min_quality)
        quality.reason = 'ok' if quality.is_valid else 'quality_below_threshold'

        path = Path()
        path.header.stamp = quality.header.stamp
        path.header.frame_id = 'odom'
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        learned.data = quality.is_valid
        self.path_pub.publish(path)
        self.quality_pub.publish(quality)
        self.learned_pub.publish(learned)


def main(args=None):
    rclpy.init(args=args)
    node = TrackLearnerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
