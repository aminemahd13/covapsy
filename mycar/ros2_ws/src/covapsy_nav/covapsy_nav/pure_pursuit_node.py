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
import numpy as np

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

        self.path_sub = self.create_subscription(
            Path, '/racing_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_pursuit', 10)

        self.path = None
        self.current_pose = None
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
        self.get_logger().info(f'Received racing path with {len(self.path)} waypoints')

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.path is None or self.current_pose is None or len(self.path) < 2:
            return

        L = self.get_parameter('wheelbase').value
        ld_min = self.get_parameter('lookahead_min').value
        ld_max = self.get_parameter('lookahead_max').value
        speed_factor = self.get_parameter('speed_factor').value
        max_speed = self.get_parameter('max_speed').value
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

        # Find lookahead point
        target = None
        for i in range(nearest_idx, len(self.path)):
            if dists[i] >= ld:
                target = self.path[i]
                break
        # Wrap around for looped paths
        if target is None:
            for i in range(0, nearest_idx):
                if dists[i] >= ld:
                    target = self.path[i]
                    break
        if target is None:
            target = self.path[-1]

        # Transform to vehicle frame
        dx = target[0] - px
        dy = target[1] - py
        local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        dist_sq = local_x ** 2 + local_y ** 2
        if dist_sq < 0.001:
            return

        # Pure pursuit curvature
        curvature = 2.0 * local_y / dist_sq
        steering = math.atan(L * curvature)
        steering = max(-max_steer, min(max_steer, steering))

        # Speed: inversely proportional to curvature
        speed = max_speed * (1.0 - 0.7 * abs(steering) / max_steer)
        speed = max(0.3, speed)

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
