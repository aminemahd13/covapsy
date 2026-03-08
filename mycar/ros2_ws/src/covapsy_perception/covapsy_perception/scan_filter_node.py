"""LiDAR Scan Filter Node for COVAPSY.

Preprocesses raw LaserScan from /scan:
  - Replaces inf/nan with max_range
  - Clips to [min_range, max_range]
  - Applies sliding median filter to reduce noise
  - Publishes filtered scan on /scan_filtered
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter')

        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('median_window', 3)

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered', 10)

        self.get_logger().info('Scan filter node started')

    def scan_cb(self, msg: LaserScan):
        max_r = self.get_parameter('max_range').value
        min_r = self.get_parameter('min_range').value
        window = self.get_parameter('median_window').value

        ranges = np.array(msg.ranges, dtype=np.float32)

        # Replace inf/nan with max_range
        ranges = np.where(np.isfinite(ranges), ranges, max_r)

        # Clip to valid range
        ranges = np.clip(ranges, min_r, max_r)

        # Median filter to reduce salt-and-pepper noise
        if window > 1:
            pad = window // 2
            padded = np.pad(ranges, pad, mode='edge')
            # Vectorized sliding median using stride tricks
            shape = (len(ranges), window)
            strides = (padded.strides[0], padded.strides[0])
            windows = np.lib.stride_tricks.as_strided(padded, shape=shape, strides=strides)
            ranges = np.median(windows, axis=1).astype(np.float32)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = min_r
        out.range_max = max_r
        out.ranges = ranges.tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
