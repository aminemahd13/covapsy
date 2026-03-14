"""Full real-car bringup: bridge backend + lidar + nav + TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
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
            DeclareLaunchArgument(
                "start_mode",
                default_value="LEARNING",
                description="Mode selected after /race_start",
            ),
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument("race_profile", default_value="RACE_STABLE"),
            DeclareLaunchArgument("traffic_mode", default_value="balanced"),
            DeclareLaunchArgument("max_speed_real_cap", default_value="2.0"),
            DeclareLaunchArgument("max_speed_sim_cap", default_value="2.5"),
            DeclareLaunchArgument("vehicle_max_steering_rad", default_value="0.32"),
            DeclareLaunchArgument("watchdog_timeout", default_value="0.25"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel_autonomy"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
            DeclareLaunchArgument("enable_track_learning", default_value="true"),
            DeclareLaunchArgument("enable_tactical_ai", default_value="true"),
            DeclareLaunchArgument("enable_border_detect", default_value="true"),
            DeclareLaunchArgument("enable_depth_obstacle", default_value="true"),
            DeclareLaunchArgument("enable_realsense", default_value="true"),
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
                    os.path.join(bringup_dir, "launch", "realsense.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("enable_realsense")),
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
                    "vehicle_max_steering_rad": LaunchConfiguration("vehicle_max_steering_rad"),
                    "enable_tactical_ai": LaunchConfiguration("enable_tactical_ai"),
                    "tactical_context_mode": "auto",
                    "traffic_mode": LaunchConfiguration("traffic_mode"),
                    "initial_mode": LaunchConfiguration("initial_mode"),
                    "start_mode": LaunchConfiguration("start_mode"),
                    "command_topic": LaunchConfiguration("command_topic"),
                    "allow_runtime_mode_switch": "false",
                    "enable_pure_pursuit": "true",
                    "enable_runtime_logs": LaunchConfiguration("enable_runtime_logs"),
                    "runtime_log_period_s": LaunchConfiguration("runtime_log_period_s"),
                    "enable_track_learning": LaunchConfiguration("enable_track_learning"),
                    "enable_border_detect": LaunchConfiguration("enable_border_detect"),
                    "enable_depth_obstacle": LaunchConfiguration("enable_depth_obstacle"),
                    "reactive_odom_speed_timeout_sec": "0.50",
                    "reactive_depth_timeout_sec": "0.30",
                    "reactive_camera_timeout_sec": "0.25",
                    "drive_input_stale_sec": "0.40",
                    "stale_speed_cap": "0.15",
                    "halt_on_scan_stale": "true",
                    "track_learning_required_laps": "2",
                    "track_learned_handoff_confirm_sec": "0.80",
                    "wrong_direction_conf_enter": "0.52",
                    "wrong_direction_conf_exit": "0.32",
                    "wrong_direction_confirm_ticks": "5",
                    "wrong_direction_correction_trigger_ticks": "8",
                    "wrong_direction_correction_distance_m": "0.40",
                    "wrong_direction_correction_speed_m_s": "0.16",
                    "wrong_direction_correction_steer_rad": "0.22",
                    "wrong_direction_correction_max_sec": "0.65",
                    "wrong_direction_uturn_trigger_ticks": "18",
                    "pursuit_turn_entry_curvature_ref": "0.62",
                    "pursuit_turn_entry_curvature_brake_gain": "0.54",
                    "pursuit_turn_entry_ttc_extra_sec": "0.45",
                    "tactical_input_stale_sec": "0.35",
                    "tactical_opponent_stale_sec": "0.45",
                    "tactical_camera_stale_sec": "0.35",
                    "wrong_direction_trigger_distance_m": "1.0",
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
