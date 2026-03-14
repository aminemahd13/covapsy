"""Border Color Detection Node for COVAPSY.

Detects green (right) and red (left) track borders from the RealSense
color image using HSV color filtering. Publishes a steering offset
on /camera_steering_offset to help keep the car centered, plus
wrong-direction confidence with hysteresis.

COVAPSY border colors:
  - Red (RAL 3020) = left border
  - Green (RAL 6037) = right border
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
import numpy as np

from covapsy_perception.border_detect_utils import update_wrong_way_hysteresis
from covapsy_perception.border_detect_utils import update_track_direction_hysteresis

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

_WRONG_WAY_CALIB_MIN_CONF = 0.45
_CONF_PIXEL_REF = 7000.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


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
        self.wrong_dir_conf_pub = self.create_publisher(
            Float32, '/wrong_direction_confidence', 10)
        self.track_dir_pub = self.create_publisher(
            String, '/track_direction', 10)

        self.declare_parameter("wrong_way_enter_conf", 0.55)
        self.declare_parameter("wrong_way_exit_conf", 0.35)
        self.declare_parameter("wrong_way_confirm_frames", 6)
        self.declare_parameter("track_direction_confirm_frames", 4)

        # Expected border order: red on left, green on right (COVAPSY standard)
        self.expected_red_on_left = None
        self._wrong_way_active = False
        self._wrong_way_confirm_count = 0
        self._track_direction = "unknown"
        self._track_direction_candidate = "unknown"
        self._track_direction_candidate_count = 0

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
        red_pixels = int(cv2.countNonZero(red_mask))
        green_pixels = int(cv2.countNonZero(green_mask))

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

        # Wrong-direction confidence + hysteresis:
        # use side ordering when both borders are visible with sufficient confidence.
        conf = _clamp((red_pixels + green_pixels) / _CONF_PIXEL_REF, 0.0, 1.0)
        observed_direction = "unknown"
        if green_cx is not None and red_cx is not None:
            conf = _clamp(conf + 0.25, 0.0, 1.0)
            separation = abs(green_cx - red_cx)
            if separation > roi_w * 0.05:  # meaningful separation
                red_is_left = red_cx < green_cx
                observed_direction = "red_left" if red_is_left else "red_right"
                if self.expected_red_on_left is None and conf >= _WRONG_WAY_CALIB_MIN_CONF:
                    self.expected_red_on_left = bool(red_is_left)
                    self.get_logger().info(
                        "Camera direction calibration locked: expected red_on_left=%s"
                        % ("true" if self.expected_red_on_left else "false")
                    )
                wrong_conf = 0.0
                if self.expected_red_on_left is not None and (
                    red_is_left != self.expected_red_on_left
                ):
                    wrong_conf = float(conf)
            else:
                wrong_conf = 0.0
        else:
            conf = min(conf, 0.55)
            wrong_conf = 0.0

        track_confirm_frames = max(
            1, int(self.get_parameter("track_direction_confirm_frames").value)
        )
        prev_track_direction = self._track_direction
        (
            self._track_direction,
            self._track_direction_candidate,
            self._track_direction_candidate_count,
        ) = update_track_direction_hysteresis(
            observed_direction=observed_direction,
            active_direction=self._track_direction,
            candidate_direction=self._track_direction_candidate,
            candidate_count=self._track_direction_candidate_count,
            confirm_frames=track_confirm_frames,
        )
        if prev_track_direction != self._track_direction and self._track_direction != "unknown":
            self.get_logger().info(f"Track direction updated: {self._track_direction}")

        enter_conf = float(self.get_parameter("wrong_way_enter_conf").value)
        exit_conf = float(self.get_parameter("wrong_way_exit_conf").value)
        confirm_frames = max(1, int(self.get_parameter("wrong_way_confirm_frames").value))
        self._wrong_way_active, self._wrong_way_confirm_count = update_wrong_way_hysteresis(
            wrong_conf=wrong_conf,
            active=self._wrong_way_active,
            confirm_count=self._wrong_way_confirm_count,
            enter_conf=enter_conf,
            exit_conf=exit_conf,
            confirm_frames=confirm_frames,
        )

        wrong_msg = Bool()
        wrong_msg.data = bool(self._wrong_way_active)
        self.wrong_dir_pub.publish(wrong_msg)

        conf_msg = Float32()
        conf_msg.data = float(_clamp(wrong_conf, 0.0, 1.0))
        self.wrong_dir_conf_pub.publish(conf_msg)

        direction_msg = String()
        direction_msg.data = self._track_direction
        self.track_dir_pub.publish(direction_msg)

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
