"""Launch navigation nodes: scan filter + gap follower + pure pursuit + mode controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_speed', default_value='2.0'),
        DeclareLaunchArgument('safety_radius', default_value='0.20'),

        # Scan filter
        Node(
            package='covapsy_perception',
            executable='scan_filter_node',
            name='scan_filter',
            parameters=[{
                'max_range': 5.0,
                'min_range': 0.15,
                'median_window': 3,
            }],
            output='screen',
        ),

        # Gap follower (reactive, always running)
        Node(
            package='covapsy_nav',
            executable='gap_follower_node',
            name='gap_follower',
            parameters=[{
                'car_width': 0.30,
                'max_speed': LaunchConfiguration('max_speed'),
                'min_speed': 0.5,
                'safety_radius': LaunchConfiguration('safety_radius'),
                'disparity_threshold': 0.5,
                'steering_gain': 1.0,
            }],
            output='screen',
        ),

        # Pure pursuit (publishes only when path is available)
        Node(
            package='covapsy_nav',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            parameters=[{
                'wheelbase': 0.257,
                'lookahead_min': 0.5,
                'lookahead_max': 1.5,
                'max_speed': 2.5,
            }],
            output='screen',
        ),

        # Mode controller (state machine)
        Node(
            package='covapsy_nav',
            executable='mode_controller_node',
            name='mode_controller',
            output='screen',
        ),
    ])
