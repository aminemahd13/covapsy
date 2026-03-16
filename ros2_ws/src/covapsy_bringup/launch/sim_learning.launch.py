from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('covapsy_bringup'), 'config', 'learn_sim.yaml')
    return LaunchDescription([
        Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg]),
        Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg]),
        Node(package='covapsy_direction', executable='direction_detector_node', parameters=[cfg]),
        Node(package='covapsy_learning', executable='track_learner_node', parameters=[cfg]),
        Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg]),
        Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg]),
    ])
