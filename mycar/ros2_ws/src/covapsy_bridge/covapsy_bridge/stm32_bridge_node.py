"""STM32 HAT Bridge Node for COVAPSY autonomous racing car.

Bridges ROS2 topics to/from the STM32 HAT PCB via UART.
The STM32 controls: ESC PWM, steering servo PWM, reads encoder + rear IR.

Protocol adapted from professor's CoVAPSy_DemoSimple_STM32 firmware:
  - Direction: set_direction_degres(float) -> range [-18, +18] degrees
  - Speed: set_vitesse_m_s(float) -> range [-8.0, +2.0] m/s (soft limit 2.0 fwd)
  - LiDAR data: 360 x uint16 mm array from STM32 (if STM32 handles LiDAR)

Communication uses a simple binary protocol over /dev/ttyAMA0 at 115200 baud.

Frame format (Pi -> STM32):
  [0xAA] [0x55] [CMD] [payload...] [CRC8]
  CMD 0x01: Drive command  -> payload: steering_float(4B) + speed_float(4B)
  CMD 0x04: TFT update     -> payload: mode(1B) + speed_float(4B)

Frame format (STM32 -> Pi):
  [0xAA] [0x55] [CMD] [payload...] [CRC8]
  CMD 0x02: Speed telemetry -> payload: speed_float(4B)
  CMD 0x03: Rear IR         -> payload: state(1B)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import struct


class STM32BridgeNode(Node):

    def __init__(self):
        super().__init__('stm32_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('watchdog_timeout', 0.25)
        self.declare_parameter('max_steering_deg', 18.0)
        self.declare_parameter('max_speed_fwd', 2.0)
        self.declare_parameter('max_speed_rev', -8.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # Try to open serial port
        self.ser = None
        try:
            import serial
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'Serial port opened: {port} @ {baud}')
        except Exception as e:
            self.get_logger().warn(
                f'Cannot open serial port {port}: {e}. '
                'Running in dry-run mode (no hardware).'
            )

        # Subscribers: commands FROM ROS2 TO STM32
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.tft_sub = self.create_subscription(
            String, '/tft_command', self.tft_callback, 10)

        # Publishers: data FROM STM32 TO ROS2
        self.speed_pub = self.create_publisher(Float32, '/wheel_speed', 10)
        self.rear_ir_pub = self.create_publisher(Bool, '/rear_obstacle', 10)

        # Timer to read STM32 data at 100 Hz
        self.create_timer(0.01, self.read_stm32)

        # Watchdog: if no command in 250ms, send zero
        self.last_cmd_time = self.get_clock().now()
        timeout = self.get_parameter('watchdog_timeout').value
        self.create_timer(0.05, self.watchdog_check)

        self.get_logger().info('STM32 bridge node started')

    def cmd_callback(self, msg: Twist):
        """Convert Twist to steering angle (degrees) + speed (m/s) and send to STM32."""
        self.last_cmd_time = self.get_clock().now()

        max_steer = self.get_parameter('max_steering_deg').value
        max_fwd = self.get_parameter('max_speed_fwd').value
        max_rev = self.get_parameter('max_speed_rev').value

        # angular.z is steering in radians -> convert to degrees
        # Clip to STM32 limits
        import math
        steering_deg = math.degrees(msg.angular.z)
        steering_deg = max(-max_steer, min(max_steer, steering_deg))

        # linear.x is speed in m/s
        speed = msg.linear.x
        speed = max(max_rev, min(max_fwd, speed))

        self._send_drive(steering_deg, speed)

    def tft_callback(self, msg: String):
        """Forward TFT display command to STM32."""
        if self.ser is None:
            return
        # Send as raw string terminated with newline
        try:
            self.ser.write((msg.data + '\n').encode('ascii'))
        except Exception:
            pass

    def _send_drive(self, steering_deg: float, speed_m_s: float):
        """Send drive command frame to STM32."""
        if self.ser is None:
            return

        payload = struct.pack('<ff', steering_deg, speed_m_s)
        frame = bytes([0xAA, 0x55, 0x01]) + payload
        frame += bytes([self._crc8(frame)])
        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def read_stm32(self):
        """Read telemetry frames from STM32."""
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting >= 8:
                b = self.ser.read(1)
                if b != b'\xAA':
                    continue
                b = self.ser.read(1)
                if b != b'\x55':
                    continue

                msg_type_raw = self.ser.read(1)
                if len(msg_type_raw) < 1:
                    continue
                msg_type = msg_type_raw[0]

                if msg_type == 0x02:  # Speed telemetry
                    data = self.ser.read(5)  # 4B float + 1B CRC
                    if len(data) >= 4:
                        speed = struct.unpack('<f', data[:4])[0]
                        msg = Float32()
                        msg.data = speed
                        self.speed_pub.publish(msg)

                elif msg_type == 0x03:  # Rear IR
                    data = self.ser.read(2)  # 1B bool + 1B CRC
                    if len(data) >= 1:
                        msg = Bool()
                        msg.data = bool(data[0])
                        self.rear_ir_pub.publish(msg)
        except Exception:
            pass

    def watchdog_check(self):
        """Send zero command if no cmd received recently."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        timeout = self.get_parameter('watchdog_timeout').value
        if elapsed > timeout:
            self._send_drive(0.0, 0.0)

    @staticmethod
    def _crc8(data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def destroy_node(self):
        if self.ser is not None:
            self._send_drive(0.0, 0.0)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
