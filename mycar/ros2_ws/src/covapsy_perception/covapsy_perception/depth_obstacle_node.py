"""RealSense Depth Obstacle Detection Node.

Processes depth images from the Intel RealSense D435 to detect
close-range obstacles that complement the 2D LiDAR.

Publishes:
  /depth_front_dist (Float32)  — minimum distance in front depth FOV
  /depth_obstacle   (Bool)     — True if obstacle very close (<0.35 m)

Subscribes:
  /camera/camera/depth/image_rect_raw (Image, 16UC1 encoding)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

import numpy as np

try:
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class DepthObstacleNode(Node):

    def __init__(self):
        super().__init__("depth_obstacle")

        if not CV_AVAILABLE:
            self.get_logger().warn("cv_bridge not available; depth obstacle node disabled")
            return

        self.declare_parameter("min_valid_m", 0.15)
        self.declare_parameter("max_valid_m", 4.0)
        self.declare_parameter("obstacle_threshold_m", 0.35)
        self.declare_parameter("roi_top_frac", 0.30)
        self.declare_parameter("roi_bottom_frac", 0.85)
        self.declare_parameter("roi_left_frac", 0.15)
        self.declare_parameter("roi_right_frac", 0.85)

        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_cb,
            5,
        )
        self.dist_pub = self.create_publisher(Float32, "/depth_front_dist", 10)
        self.obstacle_pub = self.create_publisher(Bool, "/depth_obstacle", 10)

        self.get_logger().info("Depth obstacle node started")

    def depth_cb(self, msg: Image):
        # RealSense publishes 16UC1 in millimetres
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]

        top = int(h * float(self.get_parameter("roi_top_frac").value))
        bot = int(h * float(self.get_parameter("roi_bottom_frac").value))
        left = int(w * float(self.get_parameter("roi_left_frac").value))
        right = int(w * float(self.get_parameter("roi_right_frac").value))

        roi = depth_img[top:bot, left:right].astype(np.float32)

        # Convert mm → m (RealSense 16UC1 convention)
        if msg.encoding == "16UC1":
            roi = roi * 0.001

        min_valid = float(self.get_parameter("min_valid_m").value)
        max_valid = float(self.get_parameter("max_valid_m").value)

        valid_mask = (roi > min_valid) & (roi < max_valid) & np.isfinite(roi)
        valid_depths = roi[valid_mask]

        if valid_depths.size == 0:
            front_dist = max_valid
        else:
            # Use 5th percentile to be robust against noise
            front_dist = float(np.percentile(valid_depths, 5))

        dist_msg = Float32()
        dist_msg.data = front_dist
        self.dist_pub.publish(dist_msg)

        thresh = float(self.get_parameter("obstacle_threshold_m").value)
        obs_msg = Bool()
        obs_msg.data = front_dist < thresh
        self.obstacle_pub.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
