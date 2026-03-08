"""Launch SLAM Toolbox for mapping or localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('covapsy_bringup')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            description='SLAM mode: mapping or localization'),

        DeclareLaunchArgument(
            'map', default_value='',
            description='Path to map file for localization mode'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params,
                {'mode': LaunchConfiguration('mode')},
            ],
            output='screen',
        ),
    ])
