"""Full real-car bringup: bridge backend + lidar + nav + TF."""

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
            DeclareLaunchArgument(
                "backend",
                default_value="spi",
                description="Actuator backend: spi, uart, pi_pwm",
            ),
            DeclareLaunchArgument(
                "backend_config",
                default_value=default_backend_cfg,
                description="YAML file with backend calibration and interface params",
            ),
            DeclareLaunchArgument(
                "initial_mode",
                default_value="IDLE",
                description="Initial mode for mode controller",
            ),
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument("watchdog_timeout", default_value="0.25"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel_autonomy"),
            Node(
                package="covapsy_bridge",
                executable="stm32_bridge_node",
                name="stm32_bridge",
                parameters=[
                    LaunchConfiguration("backend_config"),
                    {
                        "backend": LaunchConfiguration("backend"),
                        "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                        "cmd_topic": LaunchConfiguration("command_topic"),
                    },
                ],
                output="screen",
            ),
            Node(
                package="covapsy_bridge",
                executable="tft_status_node",
                name="tft_status",
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
                    "command_topic": LaunchConfiguration("command_topic"),
                    "allow_runtime_mode_switch": "false",
                    "enable_pure_pursuit": "true",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0.15", "0", "0", "0", "base_link", "laser"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.10",
                    "0",
                    "0.12",
                    "0",
                    "0.1",
                    "0",
                    "base_link",
                    "camera_link",
                ],
            ),
        ]
    )
