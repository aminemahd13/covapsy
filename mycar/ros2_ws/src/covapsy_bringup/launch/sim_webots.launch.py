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
            DeclareLaunchArgument("max_speed", default_value="2.6"),
            DeclareLaunchArgument("race_profile", default_value="RACE_STABLE"),
            DeclareLaunchArgument("traffic_mode", default_value="balanced"),
            DeclareLaunchArgument("max_speed_real_cap", default_value="2.0"),
            DeclareLaunchArgument("max_speed_sim_cap", default_value="2.8"),
            DeclareLaunchArgument("initial_mode", default_value="REACTIVE"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
            DeclareLaunchArgument("enable_track_learning", default_value="false"),
            DeclareLaunchArgument("enable_depth_obstacle", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "nav.launch.py")
                ),
                launch_arguments={
                    "max_speed": LaunchConfiguration("max_speed"),
                    "race_profile": LaunchConfiguration("race_profile"),
                    "deployment_mode": "sim",
                    "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                    "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
                    "enable_tactical_ai": "true",
                    "traffic_mode": LaunchConfiguration("traffic_mode"),
                    "initial_mode": LaunchConfiguration("initial_mode"),
                    "enable_pure_pursuit": "false",
                    "enable_runtime_logs": LaunchConfiguration("enable_runtime_logs"),
                    "runtime_log_period_s": LaunchConfiguration("runtime_log_period_s"),
                    "enable_track_learning": LaunchConfiguration("enable_track_learning"),
                    "enable_depth_obstacle": LaunchConfiguration("enable_depth_obstacle"),
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.21", "0", "0.079", "0", "0", "0", "base_link", "laser"],
            ),
        ]
    )
