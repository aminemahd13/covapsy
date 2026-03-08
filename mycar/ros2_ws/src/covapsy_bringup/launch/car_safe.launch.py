"""Minimal safe bringup for first real-car track tests."""

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
    default_backend_cfg = os.path.join(bringup_dir, "config", "backend_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="spi"),
            DeclareLaunchArgument("backend_config", default_value=default_backend_cfg),
            DeclareLaunchArgument("watchdog_timeout", default_value="0.25"),
            DeclareLaunchArgument("max_speed", default_value="0.6"),
            DeclareLaunchArgument("initial_mode", default_value="REACTIVE"),
            Node(
                package="covapsy_bridge",
                executable="stm32_bridge_node",
                name="stm32_bridge",
                parameters=[
                    LaunchConfiguration("backend_config"),
                    {
                        "backend": LaunchConfiguration("backend"),
                        "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                    },
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "lidar.launch.py")
                )
            ),
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
        ]
    )

