"""LiDAR Scan Filter Node for COVAPSY."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from covapsy_perception.scan_filter_utils import filter_scan_ranges


class ScanFilterNode(Node):
    """Preprocess raw LaserScan: sanitize, clip, and median-filter."""

    def __init__(self):
        super().__init__("scan_filter")

        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("median_window", 5)

        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, "/scan_filtered", 10)

        self.get_logger().info("Scan filter node started")

    def scan_cb(self, msg: LaserScan):
        max_r = float(self.get_parameter("max_range").value)
        min_r = float(self.get_parameter("min_range").value)
        window = int(self.get_parameter("median_window").value)

        filtered = filter_scan_ranges(msg.ranges, max_r, min_r, window)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = min_r
        out.range_max = max_r
        out.ranges = filtered.tolist()
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


if __name__ == "__main__":
    main()
