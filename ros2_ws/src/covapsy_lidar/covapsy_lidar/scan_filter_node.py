#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_filter_node')
        self.declare_parameter('min_range_m', 0.05)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('median_window', 5)

        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.window = int(self.get_parameter('median_window').value)
        if self.window < 3:
            self.window = 3
        if self.window % 2 == 0:
            self.window += 1

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 20)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 20)

        self.get_logger().info('scan_filter_node ready')

    def _clamp(self, value: float) -> float:
        if math.isnan(value) or math.isinf(value):
            return self.max_range_m
        return max(self.min_range_m, min(self.max_range_m, value))

    def _median_filter(self, values):
        half = self.window // 2
        out = []
        ring = deque(values, maxlen=len(values))
        n = len(values)
        for i in range(n):
            window_vals = []
            for k in range(i - half, i + half + 1):
                window_vals.append(ring[k % n])
            window_vals.sort()
            out.append(window_vals[len(window_vals) // 2])
        return out

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = [self._clamp(v) for v in msg.ranges]
        filtered = self._median_filter(ranges)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.min_range_m
        out.range_max = self.max_range_m
        out.ranges = filtered
        out.intensities = msg.intensities
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
