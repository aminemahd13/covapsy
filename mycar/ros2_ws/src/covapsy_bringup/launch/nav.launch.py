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
            DeclareLaunchArgument("initial_mode", default_value="IDLE"),
            DeclareLaunchArgument("command_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("allow_runtime_mode_switch", default_value="false"),
            DeclareLaunchArgument("enable_pure_pursuit", default_value="true"),
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
                        "min_speed": 0.5,
                        "safety_radius": LaunchConfiguration("safety_radius"),
                        "disparity_threshold": 0.5,
                        "steering_gain": 1.0,
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
                        "lookahead_min": 0.5,
                        "lookahead_max": 1.5,
                        "max_speed": 2.5,
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
                        "lock_mode_after_start": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
