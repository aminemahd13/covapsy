"""Bringup for ROS2-in-the-loop Webots simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("covapsy_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed", default_value="1.5"),
            DeclareLaunchArgument("initial_mode", default_value="REACTIVE"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "nav.launch.py")
                ),
                launch_arguments={
                    "max_speed": LaunchConfiguration("max_speed"),
                    "initial_mode": LaunchConfiguration("initial_mode"),
                    "enable_pure_pursuit": "false",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.21", "0", "0.079", "0", "0", "0", "base_link", "laser"],
            ),
        ]
    )

