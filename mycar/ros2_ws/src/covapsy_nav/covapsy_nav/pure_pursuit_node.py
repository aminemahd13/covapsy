"""Pure Pursuit Trajectory Tracking for COVAPSY.

Follows a pre-recorded racing path using the Pure Pursuit algorithm.
Used in RACING mode after SLAM has built and optimized a map.

Subscribes: /racing_path (Path), /odom (Odometry)
Publishes:  /cmd_vel_pursuit (Twist)

TT-02 wheelbase: 0.257 m
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

from covapsy_nav.race_profiles import resolve_profile_speed_cap
from covapsy_nav.pure_pursuit_utils import choose_progress_index
from covapsy_nav.pure_pursuit_utils import find_forward_lookahead_index
from covapsy_nav.pure_pursuit_utils import find_lookahead_index
from covapsy_nav.pure_pursuit_utils import front_min_distance
from covapsy_nav.pure_pursuit_utils import path_is_looped
from covapsy_nav.pure_pursuit_utils import to_vehicle_frame

try:
    import tf_transformations
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('wheelbase', 0.257)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('lookahead_max', 1.5)
        self.declare_parameter('speed_factor', 0.3)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('max_steering', 0.4)
        self.declare_parameter('steering_slew_rate', 0.08)
        self.declare_parameter('ttc_target_sec', 1.3)
        self.declare_parameter('race_profile', 'RACE_STABLE')
        self.declare_parameter('deployment_mode', 'real')
        self.declare_parameter('max_speed_real_cap', 2.0)
        self.declare_parameter('max_speed_sim_cap', 2.5)
        self.declare_parameter('scan_front_half_angle_deg', 20.0)
        self.declare_parameter('direction_guard_enabled', True)
        self.declare_parameter('direction_guard_min_forward_x_m', 0.05)
        self.declare_parameter('direction_guard_max_backward_index_jump', 8)
        self.declare_parameter('direction_guard_relocalization_distance_m', 0.90)
        self.declare_parameter('direction_guard_fallback_speed_m_s', 0.20)

        self.path_sub = self.create_subscription(
            Path, '/racing_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_pursuit', 10)

        self.path = None
        self.current_pose = None
        self.min_front_dist = float('inf')
        self.prev_steering = 0.0
        self.progress_idx = None
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        if not TF_AVAILABLE:
            self.get_logger().warn(
                'tf_transformations not available. '
                'Using manual quaternion conversion.')

        self.get_logger().info('Pure pursuit node started')

    def path_cb(self, msg: Path):
        self.path = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]
        self.progress_idx = None
        self.get_logger().info(f'Received racing path with {len(self.path)} waypoints')

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def scan_cb(self, msg: LaserScan):
        half_angle = float(self.get_parameter('scan_front_half_angle_deg').value)
        self.min_front_dist = front_min_distance(
            ranges_in=msg.ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            half_angle_deg=half_angle,
        )

    def control_loop(self):
        if self.path is None or self.current_pose is None or len(self.path) < 2:
            return

        L = self.get_parameter('wheelbase').value
        ld_min = self.get_parameter('lookahead_min').value
        ld_max = self.get_parameter('lookahead_max').value
        speed_factor = self.get_parameter('speed_factor').value
        configured_max_speed = self.get_parameter('max_speed').value
        max_speed = min(
            configured_max_speed,
            resolve_profile_speed_cap(
                race_profile=str(self.get_parameter('race_profile').value),
                deployment_mode=str(self.get_parameter('deployment_mode').value),
                max_speed_real_cap=float(self.get_parameter('max_speed_real_cap').value),
                max_speed_sim_cap=float(self.get_parameter('max_speed_sim_cap').value),
            ),
        )
        max_steer = self.get_parameter('max_steering').value

        # Current state
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        q = self.current_pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

        # Adaptive lookahead
        ld = np.clip(max_speed * speed_factor, ld_min, ld_max)

        # Find nearest point on path
        path_arr = np.array(self.path)
        dists = np.hypot(path_arr[:, 0] - px, path_arr[:, 1] - py)
        nearest_idx = int(np.argmin(dists))
        nearest_dist = float(dists[nearest_idx])

        looped_path = path_is_looped(self.path, closure_tol=max(0.4, float(ld_min)))
        guard_enabled = bool(self.get_parameter('direction_guard_enabled').value)
        if guard_enabled:
            progress_idx = choose_progress_index(
                nearest_idx=nearest_idx,
                prev_progress_idx=self.progress_idx,
                path_length=len(self.path),
                looped=looped_path,
                nearest_dist=nearest_dist,
                relocalization_distance_m=float(
                    self.get_parameter('direction_guard_relocalization_distance_m').value
                ),
                max_backward_index_jump=int(
                    self.get_parameter('direction_guard_max_backward_index_jump').value
                ),
            )
        else:
            progress_idx = nearest_idx
        self.progress_idx = progress_idx

        if guard_enabled:
            target_idx = find_forward_lookahead_index(
                path=self.path,
                start_idx=progress_idx,
                lookahead_dist=float(ld),
                looped=looped_path,
                pose_x=float(px),
                pose_y=float(py),
                yaw_rad=float(yaw),
                min_forward_x_m=float(
                    self.get_parameter('direction_guard_min_forward_x_m').value
                ),
            )
        else:
            target_idx = find_lookahead_index(
                path=self.path,
                start_idx=progress_idx,
                lookahead_dist=float(ld),
                looped=looped_path,
            )
        if target_idx is None:
            fallback_speed = min(
                float(max_speed),
                max(0.0, float(self.get_parameter('direction_guard_fallback_speed_m_s').value)),
            )
            cmd = Twist()
            cmd.linear.x = float(fallback_speed)
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        target = self.path[target_idx]

        # Transform to vehicle frame
        local_x, local_y = to_vehicle_frame(
            target_x=float(target[0]),
            target_y=float(target[1]),
            pose_x=float(px),
            pose_y=float(py),
            yaw_rad=float(yaw),
        )

        dist_sq = local_x ** 2 + local_y ** 2
        if dist_sq < 0.001:
            return

        # Pure pursuit curvature
        curvature = 2.0 * local_y / dist_sq
        steering = math.atan(L * curvature)
        steering = max(-max_steer, min(max_steer, steering))
        slew_rate = float(self.get_parameter('steering_slew_rate').value)
        steering = max(self.prev_steering - slew_rate, min(self.prev_steering + slew_rate, steering))
        self.prev_steering = steering

        # Speed: curvature + front free-space + TTC constrained.
        curvature_factor = 1.0 - 0.7 * abs(steering) / max(max_steer, 1e-6)
        free_space_factor = min(1.0, self.min_front_dist / 2.0)
        speed = max_speed * curvature_factor * free_space_factor
        speed = max(0.0, speed)
        ttc_target = max(float(self.get_parameter('ttc_target_sec').value), 0.2)
        ttc = self.min_front_dist / max(speed, 0.05)
        if ttc < ttc_target:
            speed = min(speed, self.min_front_dist / ttc_target)
        if self.min_front_dist > 0.8:
            speed = max(0.3, speed)
        speed = min(max_speed, speed)

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        if TF_AVAILABLE:
            _, _, yaw = tf_transformations.euler_from_quaternion([x, y, z, w])
            return yaw
        # Manual conversion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
