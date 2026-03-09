"""Border Color Detection Node for COVAPSY.

Detects green (right) and red (left) track borders from the RealSense
color image using HSV color filtering. Publishes a steering offset
on /camera_steering_offset to help keep the car centered.

COVAPSY border colors:
  - Red (RAL 3020) = left border
  - Green (RAL 6037) = right border
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
import numpy as np

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class BorderDetectNode(Node):

    def __init__(self):
        super().__init__('border_detect')

        if not CV_AVAILABLE:
            self.get_logger().warn(
                'OpenCV/cv_bridge not available. Border detection disabled.')
            return

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_cb, 5)
        self.pub = self.create_publisher(
            Float32, '/camera_steering_offset', 10)
        self.wrong_dir_pub = self.create_publisher(
            Bool, '/wrong_direction', 10)

        # Expected border order: red on left, green on right (COVAPSY standard)
        self.expected_red_on_left = True

        # HSV ranges for COVAPSY border colors (indoor fluorescent lighting)
        # GREEN (RAL 6037)
        self.green_lower = np.array([35, 50, 50])
        self.green_upper = np.array([85, 255, 255])
        # RED (RAL 3020) - two ranges because red wraps around H=0/180
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 50, 50])
        self.red_upper2 = np.array([180, 255, 255])

        self.get_logger().info('Border detection node started')

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # Only look at the lower third of the image (track-level)
        roi = frame[2 * h // 3:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detect green (right border)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

        # Detect red (left border)
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Compute centroids
        green_cx = self._centroid_x(green_mask)
        red_cx = self._centroid_x(red_mask)

        # Compute steering offset: positive = steer right, negative = steer left
        offset = 0.0
        roi_w = roi.shape[1]
        center = roi_w / 2.0

        if green_cx is not None and red_cx is not None:
            lane_center = (green_cx + red_cx) / 2.0
            offset = (lane_center - center) / center  # Normalized [-1, 1]
        elif green_cx is not None:
            offset = -0.3  # Only right border visible: steer left
        elif red_cx is not None:
            offset = 0.3   # Only left border visible: steer right

        msg_out = Float32()
        msg_out.data = float(offset)
        self.pub.publish(msg_out)

        # Wrong-direction detection: if both borders are visible,
        # check that red is on the expected side (left).
        wrong_direction = False
        if green_cx is not None and red_cx is not None:
            separation = abs(green_cx - red_cx)
            if separation > roi_w * 0.05:  # meaningful separation
                red_is_left = red_cx < green_cx
                wrong_direction = (red_is_left != self.expected_red_on_left)

        wrong_msg = Bool()
        wrong_msg.data = wrong_direction
        self.wrong_dir_pub.publish(wrong_msg)

    @staticmethod
    def _centroid_x(mask):
        moments = cv2.moments(mask)
        if moments['m00'] > 500:
            return moments['m10'] / moments['m00']
        return None


def main(args=None):
    rclpy.init(args=args)
    node = BorderDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
