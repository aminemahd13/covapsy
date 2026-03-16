#!/usr/bin/env python3
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from covapsy_interfaces.msg import DirectionState
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32


class DirectionDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('direction_detector_node')
        self.declare_parameter('min_green_ratio', 0.03)
        self.declare_parameter('min_red_ratio', 0.03)
        self.declare_parameter('enter_wrong_threshold', 0.75)
        self.declare_parameter('exit_wrong_threshold', 0.45)
        self.declare_parameter('scan_wall_max_range_m', 1.0)

        self.min_green_ratio = float(self.get_parameter('min_green_ratio').value)
        self.min_red_ratio = float(self.get_parameter('min_red_ratio').value)
        self.enter_wrong = float(self.get_parameter('enter_wrong_threshold').value)
        self.exit_wrong = float(self.get_parameter('exit_wrong_threshold').value)
        self.scan_wall_max_range_m = float(self.get_parameter('scan_wall_max_range_m').value)

        self.bridge = CvBridge()
        self.last_scan = None
        self.latched_wrong = False

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 20)

        self.wrong_pub = self.create_publisher(Bool, '/wrong_direction', 10)
        self.conf_pub = self.create_publisher(Float32, '/wrong_direction_confidence', 10)
        self.dir_pub = self.create_publisher(DirectionState, '/track_direction', 10)

        self.get_logger().info('direction_detector_node ready')

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _scan_gate(self, width: int, height: int):
        if self.last_scan is None or not self.last_scan.ranges:
            return (slice(height // 3, height), slice(0, width // 2)), (slice(height // 3, height), slice(width // 2, width))

        n = len(self.last_scan.ranges)
        mid = n // 2
        left = self.last_scan.ranges[mid + n // 8: min(n, mid + n // 2)]
        right = self.last_scan.ranges[max(0, mid - n // 2): max(1, mid - n // 8)]

        left_wall = any((0.05 < r < self.scan_wall_max_range_m) for r in left)
        right_wall = any((0.05 < r < self.scan_wall_max_range_m) for r in right)

        y0 = int(height * 0.35)
        if left_wall:
            left_roi = (slice(y0, height), slice(0, width // 2))
        else:
            left_roi = (slice(y0, height), slice(width // 6, width // 2))

        if right_wall:
            right_roi = (slice(y0, height), slice(width // 2, width))
        else:
            right_roi = (slice(y0, height), slice(width // 2, width - width // 6))

        return left_roi, right_roi

    def _color_ratio(self, hsv, roi, low, high) -> float:
        patch = hsv[roi[0], roi[1]]
        if patch.size == 0:
            return 0.0
        mask = cv2.inRange(patch, low, high)
        return float(mask.mean() / 255.0)

    def image_cb(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w, _ = frame.shape
        left_roi, right_roi = self._scan_gate(w, h)

        green_ratio = self._color_ratio(hsv, right_roi, (40, 70, 40), (90, 255, 255))
        red_ratio_1 = self._color_ratio(hsv, left_roi, (0, 70, 40), (10, 255, 255))
        red_ratio_2 = self._color_ratio(hsv, left_roi, (160, 70, 40), (179, 255, 255))
        red_ratio = max(red_ratio_1, red_ratio_2)

        wrong_red_right = max(
            self._color_ratio(hsv, right_roi, (0, 70, 40), (10, 255, 255)),
            self._color_ratio(hsv, right_roi, (160, 70, 40), (179, 255, 255)),
        )
        wrong_green_left = self._color_ratio(hsv, left_roi, (40, 70, 40), (90, 255, 255))

        ok_score = 0.5 * min(1.0, green_ratio / max(self.min_green_ratio, 1e-3)) + 0.5 * min(1.0, red_ratio / max(self.min_red_ratio, 1e-3))
        wrong_score = 0.5 * min(1.0, wrong_red_right / max(self.min_red_ratio, 1e-3)) + 0.5 * min(1.0, wrong_green_left / max(self.min_green_ratio, 1e-3))

        confidence = max(0.0, min(1.0, wrong_score - 0.4 * ok_score))

        if not self.latched_wrong and confidence >= self.enter_wrong:
            self.latched_wrong = True
        elif self.latched_wrong and confidence <= self.exit_wrong:
            self.latched_wrong = False

        state = DirectionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.confidence = float(confidence)
        if self.latched_wrong:
            state.state = DirectionState.DIR_WRONG
        elif ok_score > 0.4:
            state.state = DirectionState.DIR_OK
        else:
            state.state = DirectionState.DIR_UNKNOWN

        self.dir_pub.publish(state)

        wrong_msg = Bool()
        wrong_msg.data = self.latched_wrong
        self.wrong_pub.publish(wrong_msg)

        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self.conf_pub.publish(conf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DirectionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
