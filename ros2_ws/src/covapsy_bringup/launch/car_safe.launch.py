from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('covapsy_bringup'), 'config', 'learn_real.yaml')
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )
    enable_realsense = LaunchConfiguration('enable_realsense')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='false',
            description='Launch RealSense camera for car_safe mode',
        ),
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': False,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(enable_realsense),
        ),
        Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg]),
        Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg]),
        Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg]),
        Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg]),
        Node(package='covapsy_bridge', executable='dynamixel_steering_node', parameters=[cfg]),
    ])
