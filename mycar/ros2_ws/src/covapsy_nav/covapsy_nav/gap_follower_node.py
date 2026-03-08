"""Follow-the-Gap with Disparity Extension for COVAPSY.

Reactive obstacle avoidance algorithm that:
1. Extends obstacles at depth discontinuities (disparity extension)
2. Creates a safety bubble around the nearest obstacle
3. Finds the largest navigable gap
4. Steers toward the deepest point in that gap
5. Adapts speed to steering angle and obstacle proximity

Subscribes: /scan_filtered (LaserScan), /rear_obstacle (Bool)
Publishes:  /cmd_vel_reactive (Twist)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np


class GapFollowerNode(Node):

    def __init__(self):
        super().__init__('gap_follower')

        # Parameters
        self.declare_parameter('car_width', 0.30)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('safety_radius', 0.20)
        self.declare_parameter('disparity_threshold', 0.5)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('fov_degrees', 200.0)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.rear_sub = self.create_subscription(
            Bool, '/rear_obstacle', self.rear_cb, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_reactive', 10)

        self.rear_blocked = False
        self.get_logger().info('Gap follower node started')

    def rear_cb(self, msg: Bool):
        self.rear_blocked = msg.data

    def scan_cb(self, msg: LaserScan):
        car_width = self.get_parameter('car_width').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        max_range = self.get_parameter('max_range').value
        safety_r = self.get_parameter('safety_radius').value
        disp_thresh = self.get_parameter('disparity_threshold').value
        steer_gain = self.get_parameter('steering_gain').value
        fov = math.radians(self.get_parameter('fov_degrees').value)

        ranges = np.array(msg.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return

        # Replace invalid readings
        ranges = np.where(np.isfinite(ranges), ranges, max_range)
        ranges = np.clip(ranges, 0.0, max_range)

        angles = np.linspace(msg.angle_min, msg.angle_max, n)

        # Only consider front FOV
        half_fov = fov / 2.0
        front_mask = np.abs(angles) <= half_fov
        front_idx = np.where(front_mask)[0]

        if len(front_idx) == 0:
            self._publish_stop()
            return

        f_ranges = ranges[front_idx].copy()
        f_angles = angles[front_idx]
        fn = len(f_ranges)
        angle_inc = abs(msg.angle_increment) if msg.angle_increment != 0 else (2 * math.pi / n)

        # ── Step 1: Disparity extension ──
        half_width = car_width / 2.0
        for i in range(1, fn):
            diff = abs(f_ranges[i] - f_ranges[i - 1])
            if diff > disp_thresh:
                ci = i if f_ranges[i] < f_ranges[i - 1] else i - 1
                cr = f_ranges[ci]
                if cr > 0.05:
                    ext_angle = math.atan2(half_width, cr)
                    ext_count = int(ext_angle / angle_inc)
                    lo = max(0, ci - ext_count)
                    hi = min(fn, ci + ext_count + 1)
                    for j in range(lo, hi):
                        f_ranges[j] = min(f_ranges[j], cr)

        # ── Step 2: Safety bubble ──
        nearest_idx = np.argmin(f_ranges)
        nearest_dist = f_ranges[nearest_idx]
        if nearest_dist < safety_r * 5:
            for i in range(fn):
                arc_dist = nearest_dist * abs(f_angles[i] - f_angles[nearest_idx])
                if arc_dist < safety_r:
                    f_ranges[i] = 0.0

        # ── Step 3: Find largest gap ──
        nonzero = f_ranges > 0.05
        gaps = []
        start = None
        for i in range(fn):
            if nonzero[i] and start is None:
                start = i
            elif not nonzero[i] and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, fn - 1))

        if not gaps:
            self._publish_stop()
            return

        # ── Step 4: Best point in largest gap ──
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_ranges = f_ranges[best_gap[0]:best_gap[1] + 1]
        best_in_gap = int(np.argmax(gap_ranges))
        best_idx = best_gap[0] + best_in_gap
        target_angle = f_angles[best_idx]

        # ── Step 5: Compute steering and speed ──
        steering = steer_gain * target_angle
        steering = max(-0.5, min(0.5, steering))

        steer_factor = 1.0 - 0.7 * abs(steering) / 0.5
        dist_factor = min(1.0, nearest_dist / 1.5)
        speed = min_speed + (max_speed - min_speed) * steer_factor * dist_factor
        speed = max(min_speed, min(max_speed, speed))

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
