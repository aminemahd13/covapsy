"""ROS2 bridge controller for Webots TT-02 simulation.

This controller connects one Webots car to the ROS2 stack:
- Publishes /scan, /odom, /wheel_speed, /rear_obstacle, /mcu_status
- Subscribes to /cmd_vel
- Applies command timeout watchdog
"""

import math
import time
from typing import List

from controller import GPS
from controller import InertialUnit
from controller import Lidar
from vehicle import Driver

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
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


class WebotsRosBridge(Node):
    """ROS2 node embedded in a Webots controller process."""

    def __init__(
        self,
        driver: Driver,
        lidar: Lidar,
        gps: GPS,
        imu: InertialUnit,
        step_time_s: float,
    ):
        super().__init__("covapsy_webots_bridge")
        self.driver = driver
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.step_time_s = max(float(step_time_s), 1e-3)

        self.declare_parameter("max_speed_m_s", 2.5)
        self.declare_parameter("max_steering_rad", 0.35)
        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("wheelbase_m", 0.257)
        self.declare_parameter("scan_frame", "laser")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.speed_pub = self.create_publisher(Float32, "/wheel_speed", 10)
        self.rear_pub = self.create_publisher(Bool, "/rear_obstacle", 10)
        self.status_pub = self.create_publisher(String, "/mcu_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)

        self.last_cmd_time = time.monotonic()
        self.steering_cmd = 0.0
        self.speed_cmd = 0.0
        self.last_status_time = 0.0
        self.get_logger().info("Webots ROS2 bridge started")

    def _cmd_cb(self, msg: Twist) -> None:
        max_speed = float(self.get_parameter("max_speed_m_s").value)
        max_steer = float(self.get_parameter("max_steering_rad").value)
        self.speed_cmd = max(-max_speed, min(max_speed, float(msg.linear.x)))
        self.steering_cmd = max(-max_steer, min(max_steer, float(msg.angular.z)))
        self.last_cmd_time = time.monotonic()

    def apply_drive(self) -> None:
        timeout = float(self.get_parameter("cmd_timeout_sec").value)
        max_speed = float(self.get_parameter("max_speed_m_s").value)
        max_steer = float(self.get_parameter("max_steering_rad").value)

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

        if (time.monotonic() - self.last_status_time) > 1.0:
            status = String()
            status.data = "backend=webots_ros2;ok=1"
            self.status_pub.publish(status)
            self.last_status_time = time.monotonic()


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
