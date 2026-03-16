import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _bool_from_text(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _sim_nodes_for_mode(cfg_path: str, mode: str):
    mode = mode.lower()

    if mode == 'reactive':
        return [
            Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg_path]),
            Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg_path]),
            Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg_path]),
            Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg_path]),
        ]

    if mode == 'learning':
        return [
            Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg_path]),
            Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg_path]),
            Node(package='covapsy_direction', executable='direction_detector_node', parameters=[cfg_path]),
            Node(package='covapsy_learning', executable='track_learner_node', parameters=[cfg_path]),
            Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg_path]),
            Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg_path]),
        ]

    if mode == 'race':
        return [
            Node(package='covapsy_lidar', executable='scan_filter_node', parameters=[cfg_path]),
            Node(package='covapsy_lidar', executable='reactive_driver_node', parameters=[cfg_path]),
            Node(package='covapsy_direction', executable='direction_detector_node', parameters=[cfg_path]),
            Node(package='covapsy_learning', executable='track_learner_node', parameters=[cfg_path]),
            Node(package='covapsy_control', executable='pure_pursuit_node', parameters=[cfg_path]),
            Node(package='covapsy_control', executable='mode_controller_node', parameters=[cfg_path]),
            Node(package='covapsy_bridge', executable='stm32_bridge_node', parameters=[cfg_path]),
        ]

    raise ValueError(f"Unsupported mode '{mode}'. Expected one of: reactive, learning, race.")


def _build_runtime_actions(context):
    mode = LaunchConfiguration('mode').perform(context)
    project_root = LaunchConfiguration('project_root').perform(context)
    world = LaunchConfiguration('world').perform(context)
    webots_executable = LaunchConfiguration('webots_executable').perform(context)
    headless = _bool_from_text(LaunchConfiguration('headless').perform(context))
    startup_delay = float(LaunchConfiguration('startup_delay_sec').perform(context))

    bringup_share = get_package_share_directory('covapsy_bringup')
    cfg_name = 'race_sim.yaml' if mode.lower() == 'race' else 'learn_sim.yaml'
    cfg_path = os.path.join(bringup_share, 'config', cfg_name)

    world_path = os.path.join(project_root, 'simulation', 'webots', 'worlds', world)

    webots_cmd = [webots_executable]
    if headless:
        webots_cmd.extend(['--batch', '--no-rendering'])
    webots_cmd.append(world_path)

    runtime_actions = [
        LogInfo(msg=f'Launching Webots world: {world_path}'),
        ExecuteProcess(cmd=webots_cmd, cwd=project_root, output='screen'),
    ]

    if not os.path.exists(world_path):
        runtime_actions.append(
            LogInfo(msg=f'Warning: world file does not exist yet at {world_path}')
        )

    sim_nodes = _sim_nodes_for_mode(cfg_path, mode)
    sim_nodes.append(
        Node(
            package='covapsy_tools',
            executable='drive_cmd_to_twist_adapter',
            parameters=[
                {
                    'input_topic': '/cmd_drive',
                    'output_topic': '/cmd_vel',
                }
            ],
        )
    )

    runtime_actions.append(
        TimerAction(
            period=max(0.0, startup_delay),
            actions=[LogInfo(msg=f'Starting ROS sim nodes after {startup_delay:.1f}s delay')] + sim_nodes,
        )
    )

    return runtime_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode',
                default_value='race',
                description='Simulation mode: reactive, learning, or race.',
            ),
            DeclareLaunchArgument(
                'world',
                default_value='Piste_CoVAPSy_2025a_ros2.wbt',
                description='World filename from simulation/webots/worlds/.',
            ),
            DeclareLaunchArgument(
                'project_root',
                default_value=EnvironmentVariable(
                    'COVAPSY_PROJECT_ROOT',
                    default_value=os.path.expanduser('~/Desktop/covapsy'),
                ),
                description='Repository root containing simulation/webots/.',
            ),
            DeclareLaunchArgument(
                'webots_executable',
                default_value='webots',
                description='Webots executable name or absolute path.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Use true to launch Webots without rendering.',
            ),
            DeclareLaunchArgument(
                'startup_delay_sec',
                default_value='2.0',
                description='Delay before starting ROS nodes after Webots launch.',
            ),
            OpaqueFunction(function=_build_runtime_actions),
        ]
    )
