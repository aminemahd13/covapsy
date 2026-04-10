from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('covapsy_bringup'), 'config', 'race_real.yaml')
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar.launch.py',
    )
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                'serial_port': '/dev/rplidar',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
        ),
        Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg]),
        Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg]),
        Node(package='covapsy_direction', executable='direction_detector_node', parameters=[cfg]),
        Node(package='covapsy_learning', executable='track_learner_node', parameters=[cfg]),
        Node(package='covapsy_control', executable='pure_pursuit_node', parameters=[cfg]),
        Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg]),
        Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg]),
        Node(package='covapsy_bridge', executable='dynamixel_steering_node', parameters=[cfg]),
    ])
