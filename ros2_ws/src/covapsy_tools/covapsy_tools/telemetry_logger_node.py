#!/usr/bin/env python3

import csv
import pathlib

import rclpy
from covapsy_interfaces.msg import RaceTelemetry
from rclpy.node import Node


class TelemetryLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_logger_node')
        self.declare_parameter('csv_path', '/tmp/covapsy_telemetry.csv')
        self.csv_path = pathlib.Path(str(self.get_parameter('csv_path').value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

        self.fp = self.csv_path.open('w', newline='')
        self.writer = csv.writer(self.fp)
        self.writer.writerow([
            'stamp', 'mode', 'cmd_speed_mps', 'cmd_steer_rad', 'speed_cap_mps',
            'safety_veto', 'emergency_brake', 'wrong_direction',
            'wrong_direction_confidence', 'front_clearance_m', 'track_quality_score',
        ])

        self.sub = self.create_subscription(RaceTelemetry, '/race_telemetry', self.cb, 20)
        self.get_logger().info(f'telemetry_logger_node writing {self.csv_path}')

    def cb(self, msg: RaceTelemetry) -> None:
        sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.writer.writerow([
            sec,
            msg.mode,
            msg.cmd_speed_mps,
            msg.cmd_steer_rad,
            msg.speed_cap_mps,
            int(msg.safety_veto),
            int(msg.emergency_brake),
            int(msg.wrong_direction),
            msg.wrong_direction_confidence,
            msg.front_clearance_m,
            msg.track_quality_score,
        ])
        self.fp.flush()

    def destroy_node(self):
        self.fp.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
