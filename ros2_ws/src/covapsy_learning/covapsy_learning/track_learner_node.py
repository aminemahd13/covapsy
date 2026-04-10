#!/usr/bin/env python3
import hashlib
import json
import math
import os
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import tf2_ros
from covapsy_interfaces.msg import DirectionState, TrackQuality
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Import for tf2_ros.Buffer.transform() support on PoseStamped
import tf2_geometry_msgs.tf2_geometry_msgs  # noqa: F401


class TrackLearnerNode(Node):
    def __init__(self) -> None:
        super().__init__('track_learner_node')
        self.declare_parameter('sample_distance_m', 0.05)
        self.declare_parameter('min_samples', 300)
        self.declare_parameter('min_quality', 0.72)
        self.declare_parameter('lane_half_width_assumed_m', 0.45)
        self.declare_parameter('max_curvature', 2.5)
        self.declare_parameter('speed_profile_max_mps', 0.6)
        self.declare_parameter('speed_profile_curvature_gain', 0.7)
        self.declare_parameter('min_path_length_m', 2.0)
        self.declare_parameter('max_path_length_m', 50.0)
        self.declare_parameter('track_store_path', '')
        self.declare_parameter('auto_save_on_valid', True)
        self.declare_parameter('freeze_after_valid', False)

        self.sample_distance = float(self.get_parameter('sample_distance_m').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.min_quality = float(self.get_parameter('min_quality').value)
        self.lane_half_width = float(self.get_parameter('lane_half_width_assumed_m').value)
        self.max_curvature = float(self.get_parameter('max_curvature').value)
        self.speed_max = float(self.get_parameter('speed_profile_max_mps').value)
        self.speed_curv_gain = float(self.get_parameter('speed_profile_curvature_gain').value)
        self.min_path_len = float(self.get_parameter('min_path_length_m').value)
        self.max_path_len = float(self.get_parameter('max_path_length_m').value)
        self.track_store_path = str(self.get_parameter('track_store_path').value).strip()
        if self.track_store_path:
            self.track_store_path = os.path.abspath(os.path.expanduser(self.track_store_path))
        self.auto_save_on_valid = bool(self.get_parameter('auto_save_on_valid').value)
        self.freeze_after_valid = bool(self.get_parameter('freeze_after_valid').value)
        self.declare_parameter('max_backups', 5)
        self.max_backups = int(self.get_parameter('max_backups').value)

        self.last_scan = None
        self.last_dir = DirectionState.DIR_UNKNOWN
        self.last_pos = None
        self.samples: List[Tuple[float, float]] = []
        self.cached_track_pts: List[Tuple[float, float]] = []
        self.cached_track_speeds: List[float] = []
        self.cached_track_quality: Optional[Dict[str, Any]] = None
        self._last_saved_signature = ''
        self.track_frozen = False

        # TF listener for map-based persistence
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 30)
        self.dir_sub = self.create_subscription(DirectionState, '/track_direction', self.direction_cb, 10)

        self.path_pub = self.create_publisher(Path, '/track_path', 5)
        self.quality_pub = self.create_publisher(TrackQuality, '/track_quality', 5)
        self.learned_pub = self.create_publisher(Bool, '/track_learned', 5)

        self._load_cached_track_from_file()

        self.timer = self.create_timer(0.5, self.publish_outputs)
        self.get_logger().info('track_learner_node ready')

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def direction_cb(self, msg: DirectionState) -> None:
        self.last_dir = msg.state

    def _front_left_right_ranges(self, scan: LaserScan):
        """Estimate left and right wall distances using sector averages
        centered at +/-90 degrees.  Much more robust than single-ray."""
        n = len(scan.ranges)
        mid = n // 2
        sector_half = max(1, n // 24)  # ~7.5 deg half-width

        # Left side: sector around +90 deg (index mid + n//4)
        left_center = min(n - 1, mid + n // 4)
        left_start = max(0, left_center - sector_half)
        left_end = min(n, left_center + sector_half + 1)
        left_vals = [v for v in scan.ranges[left_start:left_end] if math.isfinite(v) and 0.05 < v <= scan.range_max]
        left = sum(left_vals) / max(1, len(left_vals)) if left_vals else scan.range_max

        # Right side: sector around -90 deg (index mid - n//4)
        right_center = max(0, mid - n // 4)
        right_start = max(0, right_center - sector_half)
        right_end = min(n, right_center + sector_half + 1)
        right_vals = [v for v in scan.ranges[right_start:right_end] if math.isfinite(v) and 0.05 < v <= scan.range_max]
        right = sum(right_vals) / max(1, len(right_vals)) if right_vals else scan.range_max

        return left, right

    def odom_cb(self, msg: Odometry) -> None:
        if self.track_frozen:
            return
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
        # Lateral offset: positive when left wall is further -> midpoint is to the left.
        # Transform direction (-sin, cos) is the left-perpendicular, so positive offset
        # correctly shifts the midpoint toward the more-open left side.
        lateral_offset = 0.5 * (left_d - right_d)
        lateral_offset = max(-self.lane_half_width, min(self.lane_half_width, lateral_offset))

        mx = x + lateral_offset * -math.sin(yaw)
        my = y + lateral_offset * math.cos(yaw)

        if self.last_dir == DirectionState.DIR_WRONG:
            return

        # Deduplicate: skip if too close to any recent sample (handles revisits after recovery).
        check_range = min(50, len(self.samples))
        too_close = False
        for sx, sy in self.samples[-check_range:]:
            if math.hypot(mx - sx, my - sy) < self.sample_distance * 0.5:
                too_close = True
                break
        if too_close:
            self.last_pos = (x, y)
            return

        self.samples.append((mx, my))
        self.last_pos = (x, y)

    def _resample(self, pts: List[Tuple[float, float]], step: float) -> List[Tuple[float, float]]:
        if len(pts) < 2 or step <= 1e-4:
            return pts

        out = [pts[0]]
        cumulative = 0.0
        next_sample = step

        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue

            while cumulative + seg >= next_sample:
                ratio = (next_sample - cumulative) / seg
                out.append((x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)))
                next_sample += step

            cumulative += seg

        # Keep the path endpoint for stable closure/length scoring.
        if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > step * 0.25:
            out.append(pts[-1])

        return out

    def _quality_to_dict(self, quality: TrackQuality) -> Dict[str, Any]:
        return {
            'score': float(quality.score),
            'closure_error_m': float(quality.closure_error_m),
            'smoothness': float(quality.smoothness),
            'spacing_stddev': float(quality.spacing_stddev),
            'lap_consistency': float(quality.lap_consistency),
            'is_valid': bool(quality.is_valid),
            'reason': str(quality.reason),
        }

    def _quality_from_dict(self, data: Dict[str, Any], default_reason: str) -> TrackQuality:
        quality = TrackQuality()
        quality.header.stamp = self.get_clock().now().to_msg()
        quality.score = float(data.get('score', 0.0))
        quality.closure_error_m = float(data.get('closure_error_m', 0.0))
        quality.smoothness = float(data.get('smoothness', 0.0))
        quality.spacing_stddev = float(data.get('spacing_stddev', 0.0))
        quality.lap_consistency = float(data.get('lap_consistency', 0.0))
        quality.is_valid = bool(data.get('is_valid', False))
        quality.reason = str(data.get('reason', default_reason))
        return quality

    def _transform_path(self, pts: List[Tuple[float, float]], from_frame: str, to_frame: str) -> Optional[List[Tuple[float, float]]]:
        if from_frame == to_frame:
            return pts
        try:
            # Wait a tiny bit for transform
            self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            transformed = []
            for x, y in pts:
                ps = PoseStamped()
                ps.header.frame_id = from_frame
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                ps_out = self.tf_buffer.transform(ps, to_frame)
                transformed.append((ps_out.pose.position.x, ps_out.pose.position.y))
            return transformed
        except Exception as exc:
            self.get_logger().warning(f'failed to transform path from {from_frame} to {to_frame}: {exc}')
            return None

    def _cache_valid_track(self, pts: List[Tuple[float, float]], speeds: List[float], quality: TrackQuality) -> None:
        if not quality.is_valid:
            return
        self.cached_track_pts = list(pts)
        self.cached_track_speeds = list(speeds)
        self.cached_track_quality = self._quality_to_dict(quality)

    def _load_cached_track_from_file(self) -> None:
        if not self.track_store_path:
            return
        if not os.path.exists(self.track_store_path):
            return

        try:
            with open(self.track_store_path, 'r', encoding='utf-8') as f:
                payload = json.load(f)
            version = payload.get('version', 1)
            frame_id = payload.get('frame_id', 'odom')
            points = payload.get('points', [])
            quality_data = payload.get('quality', {})
            pts: List[Tuple[float, float]] = []
            speeds: List[float] = []
            for p in points:
                px = float(p['x'])
                py = float(p['y'])
                speed = float(p.get('speed_mps', self.speed_max))
                pts.append((px, py))
                speeds.append(max(0.05, speed))

            # If track was saved in map but we are in a session without a map (or different map),
            # this might be tricky. For now, we load it as is and let the publisher handle the frame.
            self._loaded_frame_id = frame_id

            quality = self._quality_from_dict(quality_data, default_reason='loaded_from_file')
            if len(pts) >= 3 and quality.is_valid:
                self._cache_valid_track(pts, speeds, quality)
                if self.freeze_after_valid:
                    self.track_frozen = True
                self.get_logger().info(
                    f'loaded cached track v{version} ({len(pts)} points, score={quality.score:.3f}, frame={frame_id}) from {self.track_store_path}'
                )
            else:
                self.get_logger().warning(
                    f'ignoring cached track from {self.track_store_path}: insufficient points or invalid quality'
                )
        except Exception as exc:
            self.get_logger().warning(f'failed to load cached track from {self.track_store_path}: {exc}')

    def _save_track_to_file(self, pts: List[Tuple[float, float]], speeds: List[float], quality: TrackQuality) -> None:
        if not self.track_store_path or not self.auto_save_on_valid or not quality.is_valid:
            return

        # Try to save in 'map' frame if available, else fallback to 'odom'
        target_frame = 'odom'
        pts_to_save = pts
        if self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.1)):
            transformed = self._transform_path(pts, 'odom', 'map')
            if transformed:
                target_frame = 'map'
                pts_to_save = transformed

        signature = (
            f'{len(pts_to_save)}|{quality.score:.4f}|'
            f'{pts_to_save[0][0]:.3f},{pts_to_save[0][1]:.3f}|{pts_to_save[-1][0]:.3f},{pts_to_save[-1][1]:.3f}'
        )
        if signature == self._last_saved_signature:
            return

        # Track ID for metadata
        track_hash = hashlib.sha256(signature.encode()).hexdigest()[:8]

        payload = {
            'version': 2,
            'frame_id': target_frame,
            'metadata': {
                'track_id': track_hash,
                'created_at': datetime.utcnow().isoformat() + 'Z',
                'closed_loop': True,  # Track learner extracts closed loops
            },
            'sample_distance_m': float(self.sample_distance),
            'points': [
                {
                    'x': float(px),
                    'y': float(py),
                    'speed_mps': float(speeds[idx]) if idx < len(speeds) else float(self.speed_max),
                }
                for idx, (px, py) in enumerate(pts_to_save)
            ],
            'quality': self._quality_to_dict(quality),
        }

        try:
            folder = os.path.dirname(self.track_store_path)
            if folder:
                os.makedirs(folder, exist_ok=True)

            # Rotation/Backup before overwriting
            if os.path.exists(self.track_store_path) and self.max_backups > 0:
                for i in range(self.max_backups - 1, 0, -1):
                    src = f'{self.track_store_path}.{i}'
                    dst = f'{self.track_store_path}.{i+1}'
                    if os.path.exists(src):
                        os.replace(src, dst)
                os.replace(self.track_store_path, f'{self.track_store_path}.1')

            tmp_path = f'{self.track_store_path}.tmp'
            with open(tmp_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2)
            os.replace(tmp_path, self.track_store_path)
            self._last_saved_signature = signature
            self.get_logger().info(f'saved track (frame={target_frame}, id={track_hash}) to {self.track_store_path}')
        except Exception as exc:
            self.get_logger().warning(f'failed to save track cache to {self.track_store_path}: {exc}')

    def _publish_track(self, pts: List[Tuple[float, float]], speeds: List[float], quality: TrackQuality) -> None:
        learned = Bool()
        learned.data = bool(quality.is_valid)

        # If it's a cached track from map, use map frame
        frame_id = 'odom'
        pts_to_pub = pts
        if hasattr(self, '_loaded_frame_id') and self._loaded_frame_id == 'map' and pts == self.cached_track_pts:
            frame_id = 'map'

        path = Path()
        path.header.stamp = quality.header.stamp
        path.header.frame_id = frame_id
        for idx, (px, py) in enumerate(pts_to_pub):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.position.z = float(speeds[idx]) if idx < len(speeds) else float(self.speed_max)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.path_pub.publish(path)
        self.quality_pub.publish(quality)
        self.learned_pub.publish(learned)

    def _publish_cached_track(self, reason: str) -> bool:
        if not self.cached_track_pts or self.cached_track_quality is None:
            return False
        quality_data = dict(self.cached_track_quality)
        quality_data['reason'] = reason
        quality = self._quality_from_dict(quality_data, default_reason=reason)
        self._publish_track(self.cached_track_pts, self.cached_track_speeds, quality)
        return True

    def _smooth(self, pts: List[Tuple[float, float]], k: int = 5, closed: bool = False) -> List[Tuple[float, float]]:
        if len(pts) < k:
            return pts
        half = k // 2
        sm = []
        for i in range(len(pts)):
            xs = []
            ys = []
            for j in range(i - half, i + half + 1):
                if closed:
                    p = pts[j % len(pts)]
                else:
                    jj = min(max(j, 0), len(pts) - 1)
                    p = pts[jj]
                xs.append(p[0])
                ys.append(p[1])
            sm.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return sm

    def _curvature_at(self, pts: List[Tuple[float, float]], i: int) -> float:
        """Curvature at point i via Menger curvature (3-point circle)."""
        n = len(pts)
        x0, y0 = pts[(i - 1) % n]
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        if a < 1e-4 or b < 1e-4:
            return 0.0
        s = (a + b + c) / 2.0
        area_sq = max(0.0, s * (s - a) * (s - b) * (s - c))
        if area_sq < 1e-12:
            return 0.0
        area = math.sqrt(area_sq)
        return 4.0 * area / max(1e-6, a * b * c)

    def _curvature_stats(self, pts: List[Tuple[float, float]]) -> float:
        if len(pts) < 3:
            return 999.0
        curv = []
        for i in range(len(pts)):
            k = self._curvature_at(pts, i)
            if k > 0.0:
                curv.append(k)
        if not curv:
            return 999.0
        return sum(curv) / len(curv)

    def _path_length(self, pts: List[Tuple[float, float]]) -> float:
        total = 0.0
        for i in range(1, len(pts)):
            total += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        return total

    def _compute_speed_profile(self, pts: List[Tuple[float, float]]) -> List[float]:
        """Conservative speed profile: slow in tight curves, faster on straights."""
        if len(pts) < 3:
            return [self.speed_max] * len(pts)
        speeds = []
        for i in range(len(pts)):
            kappa = self._curvature_at(pts, i)
            penalty = min(1.0, kappa / max(1e-3, self.max_curvature))
            speed = self.speed_max * max(0.3, 1.0 - self.speed_curv_gain * penalty)
            speeds.append(speed)
        # Smooth the speed profile with a moving average.
        if len(speeds) > 5:
            smoothed = []
            for i in range(len(speeds)):
                window = [speeds[(i + j) % len(speeds)] for j in range(-2, 3)]
                smoothed.append(sum(window) / len(window))
            speeds = smoothed
        return speeds

    def _extract_closed_loop(self, pts: List[Tuple[float, float]], closure_thresh_m: float = 1.0) -> List[Tuple[float, float]]:
        """Return a near-closed recent loop when possible, otherwise keep open path.

        Samples accumulate over time (including revisits). For quality/scoring we prefer
        the most recent lap-like segment rather than the full history.
        """
        if len(pts) < 30:
            return pts

        direct_closure = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        if direct_closure < closure_thresh_m:
            return pts + [pts[0]]

        # Find best earlier point that closes with the latest point while keeping
        # enough arc length to represent a lap-like segment.
        min_gap = max(100, int(0.4 * self.min_samples))
        best_i = -1
        best_d = 1e9
        end = len(pts) - 1
        for i in range(0, end - min_gap):
            d = math.hypot(pts[i][0] - pts[end][0], pts[i][1] - pts[end][1])
            if d < best_d:
                best_d = d
                best_i = i

        if best_i >= 0 and best_d < closure_thresh_m:
            loop_pts = pts[best_i:]
            if len(loop_pts) > 20:
                return loop_pts + [loop_pts[0]]

        return pts

    def publish_outputs(self) -> None:
        if self.track_frozen and self._publish_cached_track(reason='frozen_cached_track'):
            return

        if len(self.samples) < self.min_samples:
            if self._publish_cached_track(reason='serving_cached_track'):
                return
            quality = TrackQuality()
            quality.header.stamp = self.get_clock().now().to_msg()
            quality.score = 0.0
            quality.is_valid = False
            quality.reason = 'not_enough_samples'
            learned = Bool()
            learned.data = False
            self.quality_pub.publish(quality)
            self.learned_pub.publish(learned)
            return

        pts = self._resample(self.samples, self.sample_distance)
        pts = self._smooth(pts, 7, closed=False)
        pts = self._extract_closed_loop(pts, closure_thresh_m=1.0)

        is_closed = len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6
        pts = self._smooth(pts, 7, closed=is_closed)

        closure = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        spacing = []
        for i in range(1, len(pts)):
            spacing.append(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        spacing_mean = sum(spacing) / max(1, len(spacing))
        spacing_std = math.sqrt(sum((v - spacing_mean) ** 2 for v in spacing) / max(1, len(spacing)))
        curv = self._curvature_stats(pts)
        path_len = self._path_length(pts)

        closure_score = max(0.0, 1.0 - closure / 1.0)
        spacing_score = max(0.0, 1.0 - spacing_std / max(0.05, self.sample_distance))
        smoothness = max(0.0, 1.0 - curv / max(1e-3, self.max_curvature))
        lap_consistency = min(1.0, len(self.samples) / float(self.min_samples * 2))

        # Path length sanity: penalise tracks that are too short or too long.
        if path_len < self.min_path_len:
            length_score = path_len / max(1e-3, self.min_path_len)
        elif path_len > self.max_path_len:
            length_score = max(0.0, 1.0 - (path_len - self.max_path_len) / self.max_path_len)
        else:
            length_score = 1.0

        score = (0.30 * closure_score + 0.20 * spacing_score
                 + 0.25 * smoothness + 0.10 * lap_consistency + 0.15 * length_score)

        quality = TrackQuality()
        quality.header.stamp = self.get_clock().now().to_msg()
        quality.score = float(score)
        quality.closure_error_m = float(closure)
        quality.smoothness = float(smoothness)
        quality.spacing_stddev = float(spacing_std)
        quality.lap_consistency = float(lap_consistency)
        quality.is_valid = bool(score >= self.min_quality)
        quality.reason = 'ok' if quality.is_valid else 'quality_below_threshold'

        # Compute conservative speed profile (stored in position.z for each waypoint).
        speeds = self._compute_speed_profile(pts)

        if quality.is_valid:
            self._cache_valid_track(pts, speeds, quality)
            self._save_track_to_file(pts, speeds, quality)
            if self.freeze_after_valid:
                self.track_frozen = True
            self._publish_track(pts, speeds, quality)
            return

        if self._publish_cached_track(reason='serving_cached_track'):
            return

        self._publish_track(pts, speeds, quality)


def main(args=None):
    rclpy.init(args=args)
    node = TrackLearnerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
