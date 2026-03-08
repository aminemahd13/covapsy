"""Launch RPLidar A2M12 driver."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
                'scan_frequency': 10.0,
            }],
            output='screen',
        ),
    ])
