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
            DeclareLaunchArgument("enable_tactical_ai", default_value="false"),
            DeclareLaunchArgument("traffic_mode", default_value="balanced"),
            DeclareLaunchArgument("initial_mode", default_value="IDLE"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("allow_runtime_mode_switch", default_value="false"),
            DeclareLaunchArgument("enable_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("enable_runtime_logs", default_value="false"),
            DeclareLaunchArgument("runtime_log_period_s", default_value="1.0"),
            DeclareLaunchArgument("runtime_log_stale_timeout_s", default_value="1.5"),
            DeclareLaunchArgument("enable_track_learning", default_value="false"),
            DeclareLaunchArgument("enable_depth_obstacle", default_value="false"),
            # Scan filter
            Node(
                package="covapsy_perception",
                executable="scan_filter_node",
                name="scan_filter",
                parameters=[
                    {
                        "max_range": 5.0,
                        "min_range": 0.15,
                        "median_window": 3,
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
                        "steering_slew_rate": 0.07,
                        "ttc_target_sec": 1.2,
                        "use_ai_speed": True,
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
                        "steering_slew_rate": 0.06,
                        "scan_front_half_angle_deg": 20.0,
                        "ttc_target_sec": 1.2,
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
                        "enable_predictive_tracking": True,
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
                        "required_laps": 1,
                        "closure_tolerance": 0.60,
                        "smoothing_window": 7,
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
                        "stuck_timeout": 2.0,
                        "reverse_speed": -0.4,
                        "reverse_duration": 1.0,
                        "reverse_steer": 0.3,
                        "max_reverse_distance": 0.8,
                        "command_topic": LaunchConfiguration("command_topic"),
                        "allow_runtime_mode_switch": LaunchConfiguration(
                            "allow_runtime_mode_switch"
                        ),
                        "enable_tactical_ai": LaunchConfiguration("enable_tactical_ai"),
                        "tactical_timeout": 0.25,
                        "lock_mode_after_start": True,
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
