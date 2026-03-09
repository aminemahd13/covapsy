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
            DeclareLaunchArgument("race_profile", default_value="HOMOLOGATION"),
            DeclareLaunchArgument("traffic_mode", default_value="conservative"),
            DeclareLaunchArgument("max_speed_real_cap", default_value="0.8"),
            DeclareLaunchArgument("max_speed_sim_cap", default_value="0.8"),
            DeclareLaunchArgument("initial_mode", default_value="IDLE"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel_autonomy"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
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
                        "race_profile": LaunchConfiguration("race_profile"),
                        "deployment_mode": "real",
                        "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                        "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
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
                    "race_profile": LaunchConfiguration("race_profile"),
                    "deployment_mode": "real",
                    "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                    "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
                    "traffic_mode": LaunchConfiguration("traffic_mode"),
                    "initial_mode": LaunchConfiguration("initial_mode"),
                    "command_topic": LaunchConfiguration("command_topic"),
                    "allow_runtime_mode_switch": "false",
                    "enable_pure_pursuit": "false",
                    "enable_runtime_logs": LaunchConfiguration("enable_runtime_logs"),
                    "runtime_log_period_s": LaunchConfiguration("runtime_log_period_s"),
                    "enable_tactical_ai": "false",
                }.items(),
            ),
        ]
    )
