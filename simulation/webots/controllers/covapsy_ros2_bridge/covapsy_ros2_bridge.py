"""ROS2 bridge controller for Webots TT-02 simulation.

This controller connects one Webots car to the ROS2 stack:
- Publishes /scan, /odom, /wheel_speed, /rear_obstacle, /mcu_status,
  /depth_front_dist, /depth_obstacle
- Subscribes to /cmd_vel
- Applies command timeout watchdog
"""

import math
import time
from typing import List, Optional

from controller import GPS
from controller import InertialUnit
from controller import Lidar

try:
    from controller import Camera, RangeFinder
    CAM_AVAILABLE = True
except Exception:  # pragma: no cover
    CAM_AVAILABLE = False

from vehicle import Driver

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu
    from std_msgs.msg import Bool
    from std_msgs.msg import Float32
    from std_msgs.msg import String
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except Exception:  # pragma: no cover
    ROS2_AVAILABLE = False


def _yaw_to_quaternion(yaw: float) -> List[float]:
    """Return quaternion [x, y, z, w] from planar yaw."""
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def _build_scan_ranges(raw_ranges: List[float], max_range: float) -> List[float]:
    """Convert Webots lidar ordering to ROS LaserScan angle ordering [-pi, pi]."""
    # Build a 360 array with index 0=front, +angle=left, matching professor convention.
    lidar_m = [max_range] * 360
    n = len(raw_ranges)
    if n == 0:
        return lidar_m

    for i in range(360):
        distance = raw_ranges[-i % n]
        mapped_idx = (i - 180) % 360
        if 0.05 <= distance <= max_range:
            lidar_m[mapped_idx] = float(distance)
        else:
            lidar_m[mapped_idx] = max_range

    # Re-index to LaserScan order angle=-180..+179 deg.
    out = []
    for angle_deg in range(-180, 180):
        out.append(lidar_m[angle_deg % 360])
    return out


def _ackermann_safe_center_limit(inner_limit_rad, wheelbase_m, track_front_m, margin_rad):
    """Return max center steering so Ackermann inner wheel stays below inner_limit_rad."""
    tan_inner = math.tan(float(inner_limit_rad))
    if abs(tan_inner) < 1e-9:
        return 0.0
    center_limit = math.atan(
        float(wheelbase_m) / ((float(wheelbase_m) / tan_inner) + (0.5 * float(track_front_m)))
    )
    return max(0.0, center_limit - float(margin_rad))


_WEBOTS_WHEELBASE_M = 0.257
_WEBOTS_TRACK_FRONT_M = 0.15
_WEBOTS_STEERING_INNER_LIMIT_RAD = 0.35
_WEBOTS_STEERING_MARGIN_RAD = 5e-4
_SAFE_MAX_STEERING_RAD = _ackermann_safe_center_limit(
    inner_limit_rad=_WEBOTS_STEERING_INNER_LIMIT_RAD,
    wheelbase_m=_WEBOTS_WHEELBASE_M,
    track_front_m=_WEBOTS_TRACK_FRONT_M,
    margin_rad=_WEBOTS_STEERING_MARGIN_RAD,
)


# --------------- camera border detection (compact) ----------------
_RGB_NAMES = ("RealSenseRGB", "RGB_camera", "camera")
_DEPTH_NAMES = ("RealSenseDepth", "Depth_camera", "range-finder")
_ROI_START = 0.58
_STRIDE = 3
_MIN_BRIGHT = 36
_MIN_SAT = 0.22
_RED_DOM = 1.30
_GREEN_DOM = 1.25
_MIN_PX = 80
_CONF_PX = 850
_LANE_GAIN = math.radians(12.0)
_SINGLE_BORDER = math.radians(8.0)
_MAX_STEER = math.radians(18.0)
_WRONG_WAY_CALIB_MIN_CONF = 0.45
_DEPTH_ROI_TOP_FRAC = 0.20
_DEPTH_ROI_BOTTOM_FRAC = 0.65
_DEPTH_ROI_LEFT_FRAC = 0.28
_DEPTH_ROI_RIGHT_FRAC = 0.72


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _compute_camera_border_state(rgb_camera):
    """Return camera border state: (steer, conf, red_cx, green_cx)."""
    if not CAM_AVAILABLE or rgb_camera is None:
        return 0.0, 0.0, None, None

    image = rgb_camera.getImage()
    w = int(rgb_camera.getWidth())
    h = int(rgb_camera.getHeight())
    if image is None or w <= 0 or h <= 0:
        return 0.0, 0.0, None, None

    roi_y0 = int(h * _ROI_START)
    cx = 0.5 * w
    rc = gc = 0
    rsx = gsx = 0.0

    for y in range(roi_y0, h, _STRIDE):
        for x in range(0, w, _STRIDE):
            r = Camera.imageGetRed(image, w, x, y)
            g = Camera.imageGetGreen(image, w, x, y)
            b = Camera.imageGetBlue(image, w, x, y)
            cmax = max(r, g, b)
            if cmax < _MIN_BRIGHT:
                continue
            cmin = min(r, g, b)
            if (cmax - cmin) / float(max(cmax, 1)) < _MIN_SAT:
                continue
            if r > _RED_DOM * g and r > _RED_DOM * b:
                rc += 1
                rsx += x
            elif g > _GREEN_DOM * r and g > _GREEN_DOM * b:
                gc += 1
                gsx += x

    steer = 0.0
    red_cx = rsx / rc if rc >= _MIN_PX else None
    green_cx = gsx / gc if gc >= _MIN_PX else None
    if red_cx is not None and green_cx is not None:
        lane_c = 0.5 * (red_cx + green_cx)
        steer = -_LANE_GAIN * (lane_c - cx) / max(cx, 1.0)
    elif green_cx is not None:
        steer = _SINGLE_BORDER
    elif red_cx is not None:
        steer = -_SINGLE_BORDER

    steer = _clamp(steer, -_MAX_STEER, _MAX_STEER)
    conf = _clamp((rc + gc) / _CONF_PX, 0.0, 1.0)
    if red_cx is not None and green_cx is not None:
        conf = _clamp(conf + 0.25, 0.0, 1.0)
    else:
        conf = min(conf, 0.55)
    return steer, conf, red_cx, green_cx


class WebotsRosBridge(Node):
    """ROS2 node embedded in a Webots controller process."""

    def __init__(
        self,
        driver: Driver,
        lidar: Lidar,
        gps: GPS,
        imu: InertialUnit,
        step_time_s: float,
        rgb_camera=None,
        depth_sensor=None,
    ):
        super().__init__("covapsy_webots_bridge")
        self.driver = driver
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.rgb_camera = rgb_camera
        self.depth_sensor = depth_sensor
        self.step_time_s = max(float(step_time_s), 1e-3)

        self.declare_parameter("max_speed_m_s", 2.5)
        self.declare_parameter("max_steering_rad", _SAFE_MAX_STEERING_RAD)
        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("wheelbase_m", 0.257)
        self.declare_parameter("scan_frame", "laser")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wrong_way_enter_conf", 0.55)
        self.declare_parameter("wrong_way_exit_conf", 0.35)
        self.declare_parameter("wrong_way_confirm_frames", 6)
        self.declare_parameter("camera_border_period_sec", 0.08)
        self.declare_parameter("depth_obstacle_threshold_m", 0.28)
        self.declare_parameter("depth_valid_min_m", 0.18)
        self.declare_parameter("depth_valid_max_m", 6.0)
        self.declare_parameter("depth_roi_top_frac", _DEPTH_ROI_TOP_FRAC)
        self.declare_parameter("depth_roi_bottom_frac", _DEPTH_ROI_BOTTOM_FRAC)
        self.declare_parameter("depth_roi_left_frac", _DEPTH_ROI_LEFT_FRAC)
        self.declare_parameter("depth_roi_right_frac", _DEPTH_ROI_RIGHT_FRAC)

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.speed_pub = self.create_publisher(Float32, "/wheel_speed", 10)
        self.rear_pub = self.create_publisher(Bool, "/rear_obstacle", 10)
        self.status_pub = self.create_publisher(String, "/mcu_status", 10)
        self.depth_front_pub = self.create_publisher(Float32, "/depth_front_dist", 10)
        self.depth_obstacle_pub = self.create_publisher(Bool, "/depth_obstacle", 10)
        self.camera_steer_pub = self.create_publisher(Float32, "/camera_steering_offset", 10)
        self.wrong_dir_pub = self.create_publisher(Bool, "/wrong_direction", 10)
        self.wrong_dir_conf_pub = self.create_publisher(Float32, "/wrong_direction_confidence", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)

        self.last_cmd_time = time.monotonic()
        self.steering_cmd = 0.0
        self.speed_cmd = 0.0
        self.last_status_time = 0.0
        self.safe_max_steering_rad = _SAFE_MAX_STEERING_RAD
        self._steering_cap_warned = False
        self._prev_speed = 0.0
        self._prev_yaw = 0.0
        self._prev_time = time.monotonic()
        self._cam_next_eval_time = 0.0
        self._cam_steer_cached = 0.0
        self._cam_conf_cached = 0.0
        self._cam_expected_red_on_left: Optional[bool] = None
        self._depth_next_eval_time = 0.0
        self._depth_front_dist_cached = float("inf")
        self._depth_obstacle_cached = False
        self._wrong_way_active = False
        self._wrong_way_confirm_count = 0
        self._wrong_way_confidence = 0.0
        self.get_logger().info("Webots ROS2 bridge started (IMU publisher enabled)")

    def _effective_max_steering(self) -> float:
        configured = float(self.get_parameter("max_steering_rad").value)
        effective = min(configured, self.safe_max_steering_rad)
        if configured > self.safe_max_steering_rad and not self._steering_cap_warned:
            self.get_logger().warn(
                "max_steering_rad=%.4f exceeds safe Webots cap %.4f; clamping to safe cap."
                % (configured, self.safe_max_steering_rad)
            )
            self._steering_cap_warned = True
        return effective

    def _cmd_cb(self, msg: Twist) -> None:
        max_speed = float(self.get_parameter("max_speed_m_s").value)
        max_steer = self._effective_max_steering()
        self.speed_cmd = max(-max_speed, min(max_speed, float(msg.linear.x)))
        self.steering_cmd = max(-max_steer, min(max_steer, float(msg.angular.z)))
        self.last_cmd_time = time.monotonic()

    def apply_drive(self) -> None:
        timeout = float(self.get_parameter("cmd_timeout_sec").value)
        max_speed = float(self.get_parameter("max_speed_m_s").value)
        max_steer = self._effective_max_steering()

        cmd_speed = self.speed_cmd
        cmd_steer = self.steering_cmd

        if (time.monotonic() - self.last_cmd_time) > timeout:
            cmd_speed = 0.0
            cmd_steer = 0.0

        cmd_speed = max(-max_speed, min(max_speed, cmd_speed))
        cmd_steer = max(-max_steer, min(max_steer, cmd_steer))

        # Webots car API uses km/h, and steering sign is inverted versus ROS command here.
        self.driver.setCruisingSpeed(cmd_speed * 3.6)
        self.driver.setSteeringAngle(-cmd_steer)

    def publish_state(self) -> None:
        now_mono = time.monotonic()
        now = self.get_clock().now().to_msg()
        scan_frame = str(self.get_parameter("scan_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        max_range = float(self.lidar.getMaxRange())

        # LaserScan
        raw_ranges = list(self.lidar.getRangeImage())
        ranges = _build_scan_ranges(raw_ranges, max_range)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = scan_frame
        scan.angle_min = -math.pi
        scan.angle_increment = (2.0 * math.pi) / max(len(ranges), 1)
        scan.angle_max = scan.angle_min + (len(ranges) - 1) * scan.angle_increment
        scan.scan_time = self.step_time_s
        scan.time_increment = self.step_time_s / max(len(ranges), 1)
        scan.range_min = 0.05
        scan.range_max = max_range
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        # Wheel speed
        speed_ms = float(self.driver.getCurrentSpeed()) / 3.6
        speed_msg = Float32()
        speed_msg.data = speed_ms
        self.speed_pub.publish(speed_msg)

        # Rear sensor (not simulated)
        rear = Bool()
        rear.data = False
        self.rear_pub.publish(rear)

        # Odometry from GPS + IMU
        gps_val = self.gps.getValues()
        rpy = self.imu.getRollPitchYaw()
        yaw = float(rpy[2])
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = float(gps_val[0])
        odom.pose.pose.position.y = float(gps_val[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = speed_ms
        odom.twist.twist.angular.z = (speed_ms / wheelbase) * math.tan(self.steering_cmd)
        self.odom_pub.publish(odom)

        if (now_mono - self.last_status_time) > 1.0:
            status = String()
            status.data = "backend=webots_ros2;ok=1"
            self.status_pub.publish(status)
            self.last_status_time = now_mono

        # Depth sensing from Webots RealSenseDepth rangefinder.
        self._update_depth_state(now_mono)
        depth_msg = Float32()
        depth_msg.data = float(self._depth_front_dist_cached)
        self.depth_front_pub.publish(depth_msg)
        depth_obs_msg = Bool()
        depth_obs_msg.data = bool(self._depth_obstacle_cached)
        self.depth_obstacle_pub.publish(depth_obs_msg)

        # Camera steering and wrong-way detection, decimated to reduce Webots load.
        self._update_camera_direction_state(now_mono)

        steer_offset = self._cam_steer_cached
        cam_msg = Float32()
        cam_msg.data = steer_offset
        self.camera_steer_pub.publish(cam_msg)

        wrong_msg = Bool()
        wrong_msg.data = self._wrong_way_active
        self.wrong_dir_pub.publish(wrong_msg)
        wrong_conf_msg = Float32()
        wrong_conf_msg.data = float(self._wrong_way_confidence)
        self.wrong_dir_conf_pub.publish(wrong_conf_msg)

        # IMU data derived from Webots InertialUnit + speed delta
        cur_time = time.monotonic()
        dt = max(cur_time - self._prev_time, 0.001)
        yaw_rate = (yaw - self._prev_yaw) / dt
        # Wrap yaw_rate for discontinuities at ±π
        if yaw_rate > math.pi / dt:
            yaw_rate -= 2.0 * math.pi / dt
        elif yaw_rate < -math.pi / dt:
            yaw_rate += 2.0 * math.pi / dt

        lon_accel = (speed_ms - self._prev_speed) / dt
        # Lateral acceleration ≈ speed × yaw_rate (centripetal)
        lat_accel = speed_ms * yaw_rate

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = base_frame
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.z = yaw_rate
        imu_msg.linear_acceleration.x = lon_accel
        imu_msg.linear_acceleration.y = lat_accel
        imu_msg.linear_acceleration.z = 9.81  # gravity
        self.imu_pub.publish(imu_msg)

        self._prev_speed = speed_ms
        self._prev_yaw = yaw
        self._prev_time = cur_time

    def _update_camera_direction_state(self, now_mono: float) -> None:
        period = max(0.02, float(self.get_parameter("camera_border_period_sec").value))
        if now_mono < self._cam_next_eval_time:
            return
        self._cam_next_eval_time = now_mono + period

        steer, conf, red_cx, green_cx = _compute_camera_border_state(self.rgb_camera)
        self._cam_steer_cached = float(steer)
        self._cam_conf_cached = float(conf)

        both_seen = red_cx is not None and green_cx is not None
        if (
            self._cam_expected_red_on_left is None
            and both_seen
            and conf >= _WRONG_WAY_CALIB_MIN_CONF
        ):
            self._cam_expected_red_on_left = bool(red_cx < green_cx)
            self.get_logger().info(
                "Camera direction calibration locked: expected red_on_left=%s"
                % ("true" if self._cam_expected_red_on_left else "false")
            )

        wrong_conf = 0.0
        if self._cam_expected_red_on_left is not None and both_seen:
            red_on_left = bool(red_cx < green_cx)
            if red_on_left != self._cam_expected_red_on_left:
                wrong_conf = float(conf)
        self._wrong_way_confidence = _clamp(wrong_conf, 0.0, 1.0)

        enter_conf = float(self.get_parameter("wrong_way_enter_conf").value)
        exit_conf = float(self.get_parameter("wrong_way_exit_conf").value)
        confirm_frames = max(1, int(self.get_parameter("wrong_way_confirm_frames").value))
        if self._wrong_way_confidence >= enter_conf:
            self._wrong_way_confirm_count = min(confirm_frames, self._wrong_way_confirm_count + 1)
        elif self._wrong_way_confidence <= exit_conf:
            self._wrong_way_confirm_count = max(0, self._wrong_way_confirm_count - 1)

        if not self._wrong_way_active and self._wrong_way_confirm_count >= confirm_frames:
            self._wrong_way_active = True
        elif (
            self._wrong_way_active
            and self._wrong_way_confidence <= exit_conf
            and self._wrong_way_confirm_count == 0
        ):
            self._wrong_way_active = False

    def _update_depth_state(self, now_mono: float) -> None:
        period = max(0.02, float(self.get_parameter("camera_border_period_sec").value))
        if now_mono < self._depth_next_eval_time:
            return
        self._depth_next_eval_time = now_mono + period

        if not CAM_AVAILABLE or self.depth_sensor is None:
            self._depth_front_dist_cached = float("inf")
            self._depth_obstacle_cached = False
            return

        try:
            raw = self.depth_sensor.getRangeImage()
            width = int(self.depth_sensor.getWidth())
            height = int(self.depth_sensor.getHeight())
        except Exception:
            self._depth_front_dist_cached = float("inf")
            self._depth_obstacle_cached = False
            return

        if raw is None or width <= 0 or height <= 0:
            self._depth_front_dist_cached = float("inf")
            self._depth_obstacle_cached = False
            return

        values = list(raw)
        if len(values) < width * height:
            self._depth_front_dist_cached = float("inf")
            self._depth_obstacle_cached = False
            return

        top_frac = _clamp(float(self.get_parameter("depth_roi_top_frac").value), 0.0, 0.95)
        bottom_frac = _clamp(
            float(self.get_parameter("depth_roi_bottom_frac").value),
            top_frac + 0.02,
            1.0,
        )
        left_frac = _clamp(float(self.get_parameter("depth_roi_left_frac").value), 0.0, 0.95)
        right_frac = _clamp(
            float(self.get_parameter("depth_roi_right_frac").value),
            left_frac + 0.02,
            1.0,
        )

        y0 = int(height * top_frac)
        y1 = int(height * bottom_frac)
        x0 = int(width * left_frac)
        x1 = int(width * right_frac)
        min_valid = max(0.01, float(self.get_parameter("depth_valid_min_m").value))
        max_valid = max(min_valid + 0.01, float(self.get_parameter("depth_valid_max_m").value))

        valid: list[float] = []
        for y in range(y0, y1, 2):
            row = y * width
            for x in range(x0, x1, 2):
                d = float(values[row + x])
                if math.isfinite(d) and min_valid < d < max_valid:
                    valid.append(d)

        if not valid:
            front_dist = max_valid
        else:
            valid.sort()
            pct_idx = int(max(0, min(len(valid) - 1, round(0.05 * (len(valid) - 1)))))
            front_dist = valid[pct_idx]

        self._depth_front_dist_cached = float(front_dist)
        threshold = max(0.05, float(self.get_parameter("depth_obstacle_threshold_m").value))
        self._depth_obstacle_cached = bool(front_dist < threshold)


def main() -> None:
    driver = Driver()
    basic_time_step = int(driver.getBasicTimeStep())
    sensor_time_step = max(basic_time_step, 32)

    lidar = Lidar("RpLidarA2")
    lidar.enable(sensor_time_step)
    lidar.enablePointCloud()

    gps = GPS("gps")
    gps.enable(sensor_time_step)

    imu = InertialUnit("imu")
    imu.enable(sensor_time_step)

    # Try to enable RGB camera for border detection
    rgb_camera = None
    depth_sensor = None
    if CAM_AVAILABLE:
        for name in _RGB_NAMES:
            try:
                cam = Camera(name)
                cam.enable(sensor_time_step)
                rgb_camera = cam
                break
            except Exception:
                continue
        for name in _DEPTH_NAMES:
            try:
                depth = RangeFinder(name)
                depth.enable(sensor_time_step)
                depth_sensor = depth
                break
            except Exception:
                continue

    if not ROS2_AVAILABLE:  # pragma: no cover
        print("[covapsy_ros2_bridge] ROS2 not available; controller will hold stop.")
        while driver.step() != -1:
            driver.setCruisingSpeed(0.0)
            driver.setSteeringAngle(0.0)
        return

    rclpy.init()
    node = WebotsRosBridge(
        driver=driver,
        lidar=lidar,
        gps=gps,
        imu=imu,
        step_time_s=sensor_time_step / 1000.0,
        rgb_camera=rgb_camera,
        depth_sensor=depth_sensor,
    )
    try:
        while driver.step() != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.apply_drive()
            node.publish_state()
    finally:
        driver.setCruisingSpeed(0.0)
        driver.setSteeringAngle(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
