#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from covapsy_interfaces.msg import DriveCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_node')
        self.declare_parameter('wheelbase_m', 0.25)
        self.declare_parameter('lookahead_min_m', 0.35)
        self.declare_parameter('lookahead_max_m', 1.1)
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('steer_slew_rad_s', 2.0)
        self.declare_parameter('speed_min_mps', 0.2)
        self.declare_parameter('speed_max_mps', 1.6)
        self.declare_parameter('curvature_speed_gain', 0.9)
        self.declare_parameter('speed_hint_scale', 1.0)
        self.declare_parameter('speed_hint_influence', 1.0)
        self.declare_parameter('lat_accel_limit_mps2', 2.6)
        self.declare_parameter('speed_ttc_target_s', 1.1)
        self.declare_parameter('speed_accel_limit_mps2', 1.6)
        self.declare_parameter('speed_decel_limit_mps2', 3.0)
        self.declare_parameter('curvature_window_pts', 8)

        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.lh_min = float(self.get_parameter('lookahead_min_m').value)
        self.lh_max = float(self.get_parameter('lookahead_max_m').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.steer_slew = float(self.get_parameter('steer_slew_rad_s').value)
        self.speed_min = float(self.get_parameter('speed_min_mps').value)
        self.speed_max = float(self.get_parameter('speed_max_mps').value)
        self.curvature_gain = float(self.get_parameter('curvature_speed_gain').value)
        self.speed_hint_scale = float(self.get_parameter('speed_hint_scale').value)
        self.speed_hint_influence = float(self.get_parameter('speed_hint_influence').value)
        self.lat_accel_limit = float(self.get_parameter('lat_accel_limit_mps2').value)
        self.speed_ttc_target = float(self.get_parameter('speed_ttc_target_s').value)
        self.speed_accel_limit = float(self.get_parameter('speed_accel_limit_mps2').value)
        self.speed_decel_limit = float(self.get_parameter('speed_decel_limit_mps2').value)
        self.curvature_window_pts = int(self.get_parameter('curvature_window_pts').value)

        self.path: List[Tuple[float, float]] = []
        self.path_speed_hints: List[float] = []
        self.last_scan = None
        self.last_steer = 0.0
        self.last_time = self.get_clock().now()
        self.last_speed_cmd = 0.0

        self.path_sub = self.create_subscription(Path, '/track_path', self.path_cb, 5)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)
        self.cmd_pub = self.create_publisher(DriveCommand, '/cmd_drive_pursuit', 20)

        self.get_logger().info('pure_pursuit_node ready')

    def path_cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_speed_hints = []
        for p in msg.poses:
            speed_hint = float(p.pose.position.z)
            if not math.isfinite(speed_hint) or speed_hint < 0.05:
                speed_hint = self.speed_max
            self.path_speed_hints.append(speed_hint)

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _lookahead(self, speed: float) -> float:
        alpha = min(1.0, max(0.0, speed / max(self.speed_max, 1e-3)))
        return self.lh_min + (self.lh_max - self.lh_min) * alpha

    def _closest_idx(self, x: float, y: float) -> int:
        best = 0
        best_d = 1e9
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best = i
        return best

    def _target_point(self, start_idx: int, lookahead: float) -> Tuple[float, float]:
        if not self.path:
            return 0.0, 0.0
        i = max(0, min(start_idx, len(self.path) - 1))
        dsum = 0.0
        prev = self.path[i]
        for k in range(i + 1, i + len(self.path)):
            cur = self.path[k % len(self.path)]
            dseg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            dsum += dseg
            if dsum >= lookahead:
                return cur
            prev = cur
        return self.path[i]

    def _speed_hint(self, idx: int) -> float:
        if not self.path_speed_hints:
            return self.speed_max
        i = max(0, min(idx, len(self.path_speed_hints) - 1))
        hinted = self.path_speed_hints[i] * max(0.1, self.speed_hint_scale)
        return max(self.speed_min, min(self.speed_max, hinted))

    @staticmethod
    def _menger_curvature(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
        ax, ay = a
        bx, by = b
        cx, cy = c
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        if ab < 1e-6 or bc < 1e-6 or ac < 1e-6:
            return 0.0
        s = 0.5 * (ab + bc + ac)
        area_sq = max(0.0, s * (s - ab) * (s - bc) * (s - ac))
        if area_sq <= 1e-12:
            return 0.0
        return 4.0 * math.sqrt(area_sq) / max(1e-6, ab * bc * ac)

    def _path_curvature_ahead(self, start_idx: int, window_pts: int) -> float:
        if len(self.path) < 3:
            return 0.0
        n = len(self.path)
        w = max(1, min(window_pts, n - 2))
        k_max = 0.0
        for j in range(w):
            i = (start_idx + j) % n
            p0 = self.path[(i - 1) % n]
            p1 = self.path[i]
            p2 = self.path[(i + 1) % n]
            k = abs(self._menger_curvature(p0, p1, p2))
            if k > k_max:
                k_max = k
        return k_max

    def _front_clearance(self) -> float:
        if self.last_scan is None or not self.last_scan.ranges:
            return 1.0
        n = len(self.last_scan.ranges)
        mid = n // 2
        sector = self.last_scan.ranges[max(0, mid - n // 16): min(n, mid + n // 16)]
        return max(0.05, min(sector)) if sector else 1.0

    def odom_cb(self, msg: Odometry) -> None:
        if len(self.path) < 3:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x

        lookahead = self._lookahead(max(0.0, v))
        closest_i = self._closest_idx(x, y)
        tx, ty = self._target_point(closest_i, lookahead)

        dx = tx - x
        dy = ty - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if abs(local_x) < 1e-4:
            curvature = 0.0
        else:
            curvature = 2.0 * local_y / (lookahead * lookahead)

        steer = math.atan(self.wheelbase * curvature)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_time).nanoseconds * 1e-9)
        max_step = self.steer_slew * dt
        steer = max(self.last_steer - max_step, min(self.last_steer + max_step, steer))
        self.last_steer = steer

        front_clear = self._front_clearance()
        path_curv = self._path_curvature_ahead(closest_i, self.curvature_window_pts)
        kappa_for_speed = max(abs(curvature), path_curv)
        kappa_effective = kappa_for_speed * max(0.1, self.curvature_gain)
        if kappa_effective <= 1e-4:
            v_curve = self.speed_max
        else:
            v_curve = math.sqrt(max(0.0, self.lat_accel_limit) / kappa_effective)

        ttc_target = max(0.2, self.speed_ttc_target)
        v_clear = self.speed_max
        if min(v_curve, self.speed_max) > 0.05:
            ttc_pred = front_clear / max(0.05, min(v_curve, self.speed_max))
            if ttc_pred < ttc_target:
                v_clear = front_clear / ttc_target

        v_model = min(v_curve, v_clear, self.speed_max)
        hint_speed = self._speed_hint(closest_i)
        hint_influence = min(1.0, max(0.0, self.speed_hint_influence))
        v_hint_cap = min(v_model, hint_speed)
        speed_target = (1.0 - hint_influence) * v_model + hint_influence * v_hint_cap

        # Apply speed slew limits for smoother high-speed behavior.
        accel_step = max(0.0, self.speed_accel_limit) * dt
        decel_step = max(0.0, self.speed_decel_limit) * dt
        speed = max(self.last_speed_cmd - decel_step, min(self.last_speed_cmd + accel_step, speed_target))
        speed = max(self.speed_min, min(self.speed_max, speed))
        self.last_speed_cmd = speed
        self.last_time = now

        cmd = DriveCommand()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.steer_rad = steer
        cmd.speed_mps = speed
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
