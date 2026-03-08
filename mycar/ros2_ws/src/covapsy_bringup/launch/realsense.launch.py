"""Launch Intel RealSense at reduced resolution for Pi 5 2GB."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '424x240x15',
                'depth_module.depth_profile': '424x240x15',
                'align_depth.enable': True,
                'pointcloud.enable': False,
                'decimation_filter.enable': True,
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
            }],
            output='screen',
        ),
    ])
