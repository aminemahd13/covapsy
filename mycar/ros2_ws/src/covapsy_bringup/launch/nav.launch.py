"""Launch navigation nodes used in both simulation and real-car modes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument("safety_radius", default_value="0.20"),
            DeclareLaunchArgument("race_profile", default_value="RACE_STABLE"),
            DeclareLaunchArgument("deployment_mode", default_value="real"),
            DeclareLaunchArgument("max_speed_real_cap", default_value="2.0"),
            DeclareLaunchArgument("max_speed_sim_cap", default_value="2.5"),
            DeclareLaunchArgument("vehicle_max_steering_rad", default_value="0.32"),
            DeclareLaunchArgument("enable_tactical_ai", default_value="false"),
            DeclareLaunchArgument("traffic_mode", default_value="balanced"),
            DeclareLaunchArgument("initial_mode", default_value="IDLE"),
            DeclareLaunchArgument("start_mode", default_value="REACTIVE"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("allow_runtime_mode_switch", default_value="false"),
            DeclareLaunchArgument("enable_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
            DeclareLaunchArgument("runtime_log_stale_timeout_s", default_value="1.5"),
            DeclareLaunchArgument("enable_track_learning", default_value="false"),
            DeclareLaunchArgument("enable_depth_obstacle", default_value="false"),
            DeclareLaunchArgument("enable_border_detect", default_value="false"),
            DeclareLaunchArgument("use_close_far_fusion", default_value="true"),
            DeclareLaunchArgument("phase0_duration", default_value="0.20"),
            DeclareLaunchArgument("phase1_duration", default_value="0.10"),
            DeclareLaunchArgument("max_recovery_attempts", default_value="30"),
            DeclareLaunchArgument("recovery_cooldown_sec", default_value="10.0"),
            DeclareLaunchArgument("stuck_cmd_speed_min", default_value="0.08"),
            DeclareLaunchArgument("stuck_actual_speed_max", default_value="0.06"),
            DeclareLaunchArgument("stuck_sensor_stale_sec", default_value="0.40"),
            DeclareLaunchArgument("stuck_front_blocked_dist", default_value="0.22"),
            DeclareLaunchArgument("stuck_front_blocked_steer_min", default_value="0.08"),
            DeclareLaunchArgument("stuck_front_blocked_timeout_factor", default_value="0.65"),
            DeclareLaunchArgument("stuck_side_blocked_dist", default_value="0.20"),
            DeclareLaunchArgument("stuck_side_asymmetry_min", default_value="0.12"),
            DeclareLaunchArgument(
                "stuck_deadlock_unknown_speed_timeout_factor", default_value="1.20"
            ),
            DeclareLaunchArgument("stuck_deadlock_clear_grace_sec", default_value="0.18"),
            DeclareLaunchArgument("wrong_direction_trigger_distance_m", default_value="1.20"),
            DeclareLaunchArgument("wrong_direction_conf_enter", default_value="0.55"),
            DeclareLaunchArgument("wrong_direction_conf_exit", default_value="0.35"),
            DeclareLaunchArgument("wrong_direction_confirm_ticks", default_value="6"),
            DeclareLaunchArgument("wrong_direction_uturn_trigger_ticks", default_value="15"),
            DeclareLaunchArgument("track_learned_handoff_confirm_sec", default_value="0.50"),
            DeclareLaunchArgument("tactical_context_mode", default_value="manual"),
            DeclareLaunchArgument("tactical_opp_conf_threshold", default_value="0.45"),
            DeclareLaunchArgument("tactical_opp_count_threshold", default_value="0.5"),
            DeclareLaunchArgument("tactical_opp_persist_sec", default_value="0.60"),
            DeclareLaunchArgument("tactical_clear_persist_sec", default_value="1.20"),
            DeclareLaunchArgument("tactical_input_stale_sec", default_value="0.35"),
            DeclareLaunchArgument("tactical_opponent_stale_sec", default_value="0.45"),
            DeclareLaunchArgument("tactical_camera_stale_sec", default_value="0.35"),
            DeclareLaunchArgument("reactive_far_center_gain", default_value="0.35"),
            DeclareLaunchArgument("reactive_camera_center_gain", default_value="0.25"),
            DeclareLaunchArgument("reactive_far_weight_min", default_value="0.10"),
            DeclareLaunchArgument("reactive_far_weight_max", default_value="0.55"),
            DeclareLaunchArgument("reactive_fusion_clearance_ref_m", default_value="1.8"),
            DeclareLaunchArgument("reactive_speed_slew_up", default_value="0.25"),
            DeclareLaunchArgument("reactive_speed_slew_down", default_value="0.35"),
            DeclareLaunchArgument("reactive_use_odom_speed_fallback", default_value="true"),
            DeclareLaunchArgument("reactive_wheel_speed_timeout_sec", default_value="0.30"),
            DeclareLaunchArgument("reactive_odom_speed_timeout_sec", default_value="0.45"),
            DeclareLaunchArgument("reactive_depth_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("reactive_camera_timeout_sec", default_value="0.30"),
            DeclareLaunchArgument("drive_input_stale_sec", default_value="0.45"),
            DeclareLaunchArgument("stale_speed_cap", default_value="0.20"),
            DeclareLaunchArgument("halt_on_scan_stale", default_value="true"),
            DeclareLaunchArgument("track_learning_required_laps", default_value="1"),
            DeclareLaunchArgument("track_learning_closure_tolerance", default_value="0.60"),
            DeclareLaunchArgument("track_learning_smoothing_window", default_value="7"),
            DeclareLaunchArgument("track_learning_apex_iterations", default_value="5"),
            DeclareLaunchArgument("track_learning_auto_publish", default_value="true"),
            DeclareLaunchArgument("pursuit_direction_guard_enabled", default_value="true"),
            DeclareLaunchArgument("pursuit_direction_guard_min_forward_x_m", default_value="0.05"),
            DeclareLaunchArgument("pursuit_direction_guard_max_backward_index_jump", default_value="8"),
            DeclareLaunchArgument("pursuit_direction_guard_relocalization_distance_m", default_value="0.90"),
            DeclareLaunchArgument("pursuit_direction_guard_fallback_speed_m_s", default_value="0.20"),
            DeclareLaunchArgument("pursuit_turn_entry_curvature_ref", default_value="0.65"),
            DeclareLaunchArgument("pursuit_turn_entry_curvature_brake_gain", default_value="0.50"),
            DeclareLaunchArgument("pursuit_turn_entry_ttc_extra_sec", default_value="0.45"),
            DeclareLaunchArgument("border_wrong_way_conf_enter", default_value="0.55"),
            DeclareLaunchArgument("border_wrong_way_conf_exit", default_value="0.35"),
            DeclareLaunchArgument("border_wrong_way_confirm_frames", default_value="6"),
            DeclareLaunchArgument("tactical_near_weight_base", default_value="0.35"),
            DeclareLaunchArgument("tactical_near_weight_min", default_value="0.20"),
            DeclareLaunchArgument("tactical_near_weight_max", default_value="0.90"),
            DeclareLaunchArgument("tactical_near_weight_clear_ref_dist", default_value="1.8"),
            DeclareLaunchArgument("tactical_near_weight_traffic_boost", default_value="0.35"),
            DeclareLaunchArgument("tactical_near_weight_clearance_boost", default_value="0.30"),
            DeclareLaunchArgument(
                "tactical_near_weight_steer_disagreement_boost",
                default_value="0.20",
            ),
            # Scan filter
            Node(
                package="covapsy_perception",
                executable="scan_filter_node",
                name="scan_filter",
                parameters=[
                    {
                        "max_range": 5.0,
                        "min_range": 0.15,
                        "median_window": 5,
                    }
                ],
                output="screen",
            ),
            # Gap follower (reactive, always running)
            Node(
                package="covapsy_nav",
                executable="gap_follower_node",
                name="gap_follower",
                parameters=[
                    {
                        "car_width": 0.30,
                        "max_speed": LaunchConfiguration("max_speed"),
                        "min_speed": 0.35,
                        "safety_radius": LaunchConfiguration("safety_radius"),
                        "disparity_threshold": 0.5,
                        "steering_gain": 1.0,
                        "race_profile": LaunchConfiguration("race_profile"),
                        "deployment_mode": LaunchConfiguration("deployment_mode"),
                        "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                        "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
                        "steering_slew_rate": 0.25,
                        "max_steering": LaunchConfiguration("vehicle_max_steering_rad"),
                        "ttc_target_sec": 0.70,
                        "use_ai_speed": True,
                        "use_imu_fusion": True,
                        "use_close_far_fusion": LaunchConfiguration("use_close_far_fusion"),
                        "far_center_gain": LaunchConfiguration("reactive_far_center_gain"),
                        "camera_center_gain": LaunchConfiguration("reactive_camera_center_gain"),
                        "far_weight_min": LaunchConfiguration("reactive_far_weight_min"),
                        "far_weight_max": LaunchConfiguration("reactive_far_weight_max"),
                        "fusion_clearance_ref_m": LaunchConfiguration(
                            "reactive_fusion_clearance_ref_m"
                        ),
                        "speed_slew_up": LaunchConfiguration("reactive_speed_slew_up"),
                        "speed_slew_down": LaunchConfiguration("reactive_speed_slew_down"),
                        "use_odom_speed_fallback": LaunchConfiguration(
                            "reactive_use_odom_speed_fallback"
                        ),
                        "wheel_speed_timeout_sec": LaunchConfiguration(
                            "reactive_wheel_speed_timeout_sec"
                        ),
                        "odom_speed_timeout_sec": LaunchConfiguration(
                            "reactive_odom_speed_timeout_sec"
                        ),
                        "depth_timeout_sec": LaunchConfiguration("reactive_depth_timeout_sec"),
                        "camera_timeout_sec": LaunchConfiguration("reactive_camera_timeout_sec"),
                    }
                ],
                output="screen",
            ),
            # Pure pursuit (optional in simulation)
            Node(
                package="covapsy_nav",
                executable="pure_pursuit_node",
                name="pure_pursuit",
                condition=IfCondition(LaunchConfiguration("enable_pure_pursuit")),
                parameters=[
                    {
                        "wheelbase": 0.257,
                        "lookahead_min": 0.6,
                        "lookahead_max": 1.8,
                        "max_speed": LaunchConfiguration("max_speed"),
                        "race_profile": LaunchConfiguration("race_profile"),
                        "deployment_mode": LaunchConfiguration("deployment_mode"),
                        "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                        "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
                        "max_steering": LaunchConfiguration("vehicle_max_steering_rad"),
                        "steering_slew_rate": 0.06,
                        "scan_front_half_angle_deg": 20.0,
                        "ttc_target_sec": 1.2,
                        "direction_guard_enabled": LaunchConfiguration("pursuit_direction_guard_enabled"),
                        "direction_guard_min_forward_x_m": LaunchConfiguration(
                            "pursuit_direction_guard_min_forward_x_m"
                        ),
                        "direction_guard_max_backward_index_jump": LaunchConfiguration(
                            "pursuit_direction_guard_max_backward_index_jump"
                        ),
                        "direction_guard_relocalization_distance_m": LaunchConfiguration(
                            "pursuit_direction_guard_relocalization_distance_m"
                        ),
                        "direction_guard_fallback_speed_m_s": LaunchConfiguration(
                            "pursuit_direction_guard_fallback_speed_m_s"
                        ),
                        "turn_entry_curvature_ref": LaunchConfiguration(
                            "pursuit_turn_entry_curvature_ref"
                        ),
                        "turn_entry_curvature_brake_gain": LaunchConfiguration(
                            "pursuit_turn_entry_curvature_brake_gain"
                        ),
                        "turn_entry_ttc_extra_sec": LaunchConfiguration(
                            "pursuit_turn_entry_ttc_extra_sec"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="covapsy_perception",
                executable="border_detect_node",
                name="border_detect",
                condition=IfCondition(LaunchConfiguration("enable_border_detect")),
                parameters=[
                    {
                        "wrong_way_enter_conf": LaunchConfiguration("border_wrong_way_conf_enter"),
                        "wrong_way_exit_conf": LaunchConfiguration("border_wrong_way_conf_exit"),
                        "wrong_way_confirm_frames": LaunchConfiguration(
                            "border_wrong_way_confirm_frames"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="covapsy_perception",
                executable="opponent_detect_node",
                name="opponent_detect",
                condition=IfCondition(LaunchConfiguration("enable_tactical_ai")),
                parameters=[
                    {
                        "traffic_mode": LaunchConfiguration("traffic_mode"),
                        "enable_tracking": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="covapsy_nav",
                executable="tactical_race_node",
                name="tactical_race",
                condition=IfCondition(LaunchConfiguration("enable_tactical_ai")),
                parameters=[
                    {
                        "race_profile": LaunchConfiguration("race_profile"),
                        "deployment_mode": LaunchConfiguration("deployment_mode"),
                        "max_speed_real_cap": LaunchConfiguration("max_speed_real_cap"),
                        "max_speed_sim_cap": LaunchConfiguration("max_speed_sim_cap"),
                        "traffic_mode": LaunchConfiguration("traffic_mode"),
                        "max_steering": LaunchConfiguration("vehicle_max_steering_rad"),
                        "enable_predictive_tracking": True,
                        "near_weight_base": LaunchConfiguration("tactical_near_weight_base"),
                        "near_weight_min": LaunchConfiguration("tactical_near_weight_min"),
                        "near_weight_max": LaunchConfiguration("tactical_near_weight_max"),
                        "near_weight_clear_ref_dist": LaunchConfiguration(
                            "tactical_near_weight_clear_ref_dist"
                        ),
                        "near_weight_traffic_boost": LaunchConfiguration(
                            "tactical_near_weight_traffic_boost"
                        ),
                        "near_weight_clearance_boost": LaunchConfiguration(
                            "tactical_near_weight_clearance_boost"
                        ),
                        "near_weight_steer_disagreement_boost": LaunchConfiguration(
                            "tactical_near_weight_steer_disagreement_boost"
                        ),
                        "input_stale_sec": LaunchConfiguration("tactical_input_stale_sec"),
                        "opponent_stale_sec": LaunchConfiguration("tactical_opponent_stale_sec"),
                        "camera_stale_sec": LaunchConfiguration("tactical_camera_stale_sec"),
                    }
                ],
                output="screen",
            ),
            # Track learner (records setup laps and builds racing line)
            Node(
                package="covapsy_nav",
                executable="track_learner_node",
                name="track_learner",
                condition=IfCondition(LaunchConfiguration("enable_track_learning")),
                parameters=[
                    {
                        "required_laps": LaunchConfiguration("track_learning_required_laps"),
                        "closure_tolerance": LaunchConfiguration("track_learning_closure_tolerance"),
                        "smoothing_window": LaunchConfiguration("track_learning_smoothing_window"),
                        "apex_iterations": LaunchConfiguration("track_learning_apex_iterations"),
                        "auto_publish": LaunchConfiguration("track_learning_auto_publish"),
                    }
                ],
                output="screen",
            ),
            # Depth obstacle (RealSense depth for near-field obstacles)
            Node(
                package="covapsy_perception",
                executable="depth_obstacle_node",
                name="depth_obstacle",
                condition=IfCondition(LaunchConfiguration("enable_depth_obstacle")),
                parameters=[
                    {
                        "obstacle_threshold_m": 0.35,
                        "roi_top_frac": 0.30,
                        "roi_bottom_frac": 0.85,
                    }
                ],
                output="screen",
            ),
            # Mode controller (top-level state machine)
            Node(
                package="covapsy_nav",
                executable="mode_controller_node",
                name="mode_controller",
                parameters=[
                    {
                        "initial_mode": LaunchConfiguration("initial_mode"),
                        "start_mode": LaunchConfiguration("start_mode"),
                        "stuck_timeout": 0.8,
                        "reverse_speed": -0.6,
                        "reverse_duration": 0.8,
                        "reverse_steer": 0.3,
                        "max_reverse_distance": 0.8,
                        "command_topic": LaunchConfiguration("command_topic"),
                        "allow_runtime_mode_switch": LaunchConfiguration(
                            "allow_runtime_mode_switch"
                        ),
                        "enable_tactical_ai": LaunchConfiguration("enable_tactical_ai"),
                        "tactical_timeout": 0.25,
                        "lock_mode_after_start": True,
                        "phase0_duration": LaunchConfiguration("phase0_duration"),
                        "phase1_duration": LaunchConfiguration("phase1_duration"),
                        "max_recovery_attempts": LaunchConfiguration("max_recovery_attempts"),
                        "recovery_cooldown_sec": LaunchConfiguration("recovery_cooldown_sec"),
                        "stuck_cmd_speed_min": LaunchConfiguration("stuck_cmd_speed_min"),
                        "stuck_actual_speed_max": LaunchConfiguration("stuck_actual_speed_max"),
                        "stuck_sensor_stale_sec": LaunchConfiguration("stuck_sensor_stale_sec"),
                        "stuck_front_blocked_dist": LaunchConfiguration(
                            "stuck_front_blocked_dist"
                        ),
                        "stuck_front_blocked_steer_min": LaunchConfiguration(
                            "stuck_front_blocked_steer_min"
                        ),
                        "stuck_front_blocked_timeout_factor": LaunchConfiguration(
                            "stuck_front_blocked_timeout_factor"
                        ),
                        "stuck_side_blocked_dist": LaunchConfiguration("stuck_side_blocked_dist"),
                        "stuck_side_asymmetry_min": LaunchConfiguration(
                            "stuck_side_asymmetry_min"
                        ),
                        "stuck_deadlock_unknown_speed_timeout_factor": LaunchConfiguration(
                            "stuck_deadlock_unknown_speed_timeout_factor"
                        ),
                        "stuck_deadlock_clear_grace_sec": LaunchConfiguration(
                            "stuck_deadlock_clear_grace_sec"
                        ),
                        "drive_input_stale_sec": LaunchConfiguration("drive_input_stale_sec"),
                        "stale_speed_cap": LaunchConfiguration("stale_speed_cap"),
                        "halt_on_scan_stale": LaunchConfiguration("halt_on_scan_stale"),
                        "wrong_direction_trigger_distance_m": LaunchConfiguration(
                            "wrong_direction_trigger_distance_m"
                        ),
                        "wrong_direction_conf_enter": LaunchConfiguration("wrong_direction_conf_enter"),
                        "wrong_direction_conf_exit": LaunchConfiguration("wrong_direction_conf_exit"),
                        "wrong_direction_confirm_ticks": LaunchConfiguration(
                            "wrong_direction_confirm_ticks"
                        ),
                        "wrong_direction_uturn_trigger_ticks": LaunchConfiguration(
                            "wrong_direction_uturn_trigger_ticks"
                        ),
                        "uturn_steer": LaunchConfiguration("vehicle_max_steering_rad"),
                        "track_learned_handoff_confirm_sec": LaunchConfiguration(
                            "track_learned_handoff_confirm_sec"
                        ),
                        "tactical_context_mode": LaunchConfiguration("tactical_context_mode"),
                        "tactical_opp_conf_threshold": LaunchConfiguration(
                            "tactical_opp_conf_threshold"
                        ),
                        "tactical_opp_count_threshold": LaunchConfiguration(
                            "tactical_opp_count_threshold"
                        ),
                        "tactical_opp_persist_sec": LaunchConfiguration(
                            "tactical_opp_persist_sec"
                        ),
                        "tactical_clear_persist_sec": LaunchConfiguration(
                            "tactical_clear_persist_sec"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="covapsy_nav",
                executable="runtime_monitor_node",
                name="runtime_monitor",
                parameters=[
                    {
                        "enable_logs": LaunchConfiguration("enable_runtime_logs"),
                        "log_period_s": LaunchConfiguration("runtime_log_period_s"),
                        "stale_timeout_s": LaunchConfiguration("runtime_log_stale_timeout_s"),
                        "command_topic": LaunchConfiguration("command_topic"),
                    }
                ],
                output="screen",
            ),
        ]
    )
