"""Full car bringup: STM32 bridge + LiDAR + perception + navigation + TF."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('covapsy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='reactive',
                              description='Start mode: reactive, mapping, racing'),
        DeclareLaunchArgument('max_speed', default_value='2.0'),

        # ── STM32 Bridge ──
        Node(
            package='covapsy_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baudrate': 115200,
                'watchdog_timeout': 0.25,
            }],
            output='screen',
        ),

        # ── TFT Status Display ──
        Node(
            package='covapsy_bridge',
            executable='tft_status_node',
            name='tft_status',
            output='screen',
        ),

        # ── LiDAR Driver ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'lidar.launch.py'))),

        # ── Navigation Stack ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'nav.launch.py')),
            launch_arguments={
                'max_speed': LaunchConfiguration('max_speed'),
            }.items()),

        # ── Static TF: base_link -> laser ──
        # LiDAR mounted on top of car, 15cm above base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        ),

        # ── Static TF: base_link -> camera_link ──
        # Camera mounted front, 10cm forward, 12cm up, 0.1 rad tilt down
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.10', '0', '0.12', '0', '0.1', '0', 'base_link', 'camera_link'],
        ),
    ])
