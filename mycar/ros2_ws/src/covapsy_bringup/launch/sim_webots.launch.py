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
            DeclareLaunchArgument("max_speed", default_value="2.2"),
            DeclareLaunchArgument("race_profile", default_value="RACE_STABLE"),
            DeclareLaunchArgument("traffic_mode", default_value="balanced"),
            DeclareLaunchArgument("max_speed_real_cap", default_value="2.0"),
            DeclareLaunchArgument("max_speed_sim_cap", default_value="2.4"),
            DeclareLaunchArgument("vehicle_max_steering_rad", default_value="0.32"),
            DeclareLaunchArgument("initial_mode", default_value="LEARNING"),
            DeclareLaunchArgument("start_mode", default_value="LEARNING"),
            DeclareLaunchArgument("enable_tactical_ai", default_value="false"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
            DeclareLaunchArgument("enable_track_learning", default_value="true"),
            DeclareLaunchArgument("enable_depth_obstacle", default_value="true"),
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
                    "vehicle_max_steering_rad": LaunchConfiguration("vehicle_max_steering_rad"),
                    "enable_tactical_ai": LaunchConfiguration("enable_tactical_ai"),
                    "tactical_context_mode": "auto",
                    "traffic_mode": LaunchConfiguration("traffic_mode"),
                    "initial_mode": LaunchConfiguration("initial_mode"),
                    "start_mode": LaunchConfiguration("start_mode"),
                    "enable_pure_pursuit": "true",
                    "enable_runtime_logs": LaunchConfiguration("enable_runtime_logs"),
                    "runtime_log_period_s": LaunchConfiguration("runtime_log_period_s"),
                    "enable_track_learning": LaunchConfiguration("enable_track_learning"),
                    "enable_depth_obstacle": LaunchConfiguration("enable_depth_obstacle"),
                    # Sim-specific recovery: skip ESC brake/neutral arming phases
                    "phase0_duration": "0.0",
                    "phase1_duration": "0.0",
                    # Allow 5 consecutive recoveries before a 5s cooldown pause
                    "max_recovery_attempts": "5",
                    "recovery_cooldown_sec": "5.0",
                    "stuck_cmd_speed_min": "0.12",
                    "stuck_actual_speed_max": "0.05",
                    "stuck_sensor_stale_sec": "0.50",
                    # Reduce far-centering and smooth reactive steering in sim.
                    "reactive_far_center_gain": "0.15",
                    "reactive_camera_center_gain": "0.15",
                    "reactive_far_weight_max": "0.35",
                    "reactive_steering_low_pass_alpha_min": "0.72",
                    "reactive_steering_low_pass_alpha_max": "0.94",
                    "reactive_steering_jerk_limit_rad": "0.025",
                    "reactive_steering_sign_hysteresis_rad": "0.035",
                    "reactive_steering_center_hold_speed_m_s": "0.95",
                    "reactive_steering_slew_speed_scale": "0.55",
                    "reactive_steering_low_pass_alpha": "0.82",
                    "reactive_speed_slew_up": "0.06",
                    "reactive_speed_slew_down": "0.10",
                    "reactive_use_odom_speed_fallback": "true",
                    "reactive_wheel_speed_timeout_sec": "0.35",
                    "reactive_odom_speed_timeout_sec": "0.45",
                    "reactive_depth_timeout_sec": "0.35",
                    "reactive_camera_timeout_sec": "0.30",
                    "reactive_camera_offset_conf_ref_rad": "0.18",
                    "reactive_camera_offset_jitter_ref_rad": "0.06",
                    "reactive_camera_offset_min_confidence": "0.08",
                    "drive_input_stale_sec": "0.45",
                    "stale_speed_cap": "0.18",
                    "halt_on_scan_stale": "true",
                    "wrong_direction_conf_enter": "0.50",
                    "wrong_direction_conf_exit": "0.30",
                    "wrong_direction_confirm_ticks": "5",
                    "wrong_direction_correction_trigger_ticks": "6",
                    "wrong_direction_correction_distance_m": "0.35",
                    "wrong_direction_correction_speed_m_s": "0.16",
                    "wrong_direction_correction_steer_rad": "0.24",
                    "wrong_direction_correction_max_sec": "0.60",
                    "wrong_direction_uturn_trigger_ticks": "16",
                    "pursuit_direction_guard_enabled": "true",
                    "pursuit_direction_guard_min_forward_x_m": "0.04",
                    "pursuit_direction_guard_max_backward_index_jump": "6",
                    "pursuit_direction_guard_relocalization_distance_m": "0.85",
                    "pursuit_direction_guard_fallback_speed_m_s": "0.16",
                    "pursuit_turn_entry_curvature_ref": "0.60",
                    "pursuit_turn_entry_curvature_brake_gain": "0.58",
                    "pursuit_turn_entry_ttc_extra_sec": "0.50",
                    "tactical_input_stale_sec": "0.30",
                    "tactical_opponent_stale_sec": "0.40",
                    "tactical_camera_stale_sec": "0.30",
                    "track_learning_required_laps": "1",
                    "track_learned_handoff_confirm_sec": "0.60",
                    "wrong_direction_trigger_distance_m": "0.8",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.21",
                    "--y",
                    "0.0",
                    "--z",
                    "0.079",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "laser",
                ],
            ),
        ]
    )
