"""HanMole state estimator launch based on robot_localization EKF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params_path(robot_version: str) -> Path:
    if robot_version not in ('v0_1', 'v0_2'):
        raise RuntimeError(f'robot_version must be v0_1 or v0_2, got {robot_version}')
    share = Path(get_package_share_directory('hanmole_navigation'))
    path = share / 'config' / robot_version / 'ekf.yaml'
    if not path.exists():
        raise RuntimeError(f'ekf params file not found: {path}')
    return path


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot_version = LaunchConfiguration('robot_version').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    base_controller_name = LaunchConfiguration('base_controller_name').perform(context)

    resolved_params_file = params_file or str(_default_params_path(robot_version))
    odom_input_topic = '/odom'
    if robot_version == 'v0_1':
        odom_input_topic = f'/{base_controller_name}/odometry'

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                resolved_params_file,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'odom0': odom_input_topic,
                },
            ],
        )
    ]


def generate_launch_description():
    """Generate the HanMole EKF launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_version', default_value='v0_2'),
        DeclareLaunchArgument('base_controller_name', default_value='base_controller'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
