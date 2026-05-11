"""Bring up Hanmole AGV controller and Nav2 by robot_version."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import tomllib


def _as_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _default_robot_version() -> str:
    pkg_share = get_package_share_directory('hanmole_navigation')
    settings_path = os.path.join(pkg_share, 'config', 'settings.toml')
    with open(settings_path, 'rb') as settings_file:
        settings = tomllib.load(settings_file)

    robot_version = settings.get('robot_settings', {}).get(
        'robot_version', 'v0_1')
    if robot_version not in ('v0_1', 'v0_2'):
        raise RuntimeError(
            f'Invalid robot_version in {settings_path}: {robot_version}')
    return robot_version


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot_version = LaunchConfiguration('robot_version').perform(context)
    if robot_version not in ('v0_1', 'v0_2'):
        raise RuntimeError('robot_version must be one of: v0_1, v0_2')
    if _as_bool(LaunchConfiguration('use_nav2').perform(context)):
        map_yaml = LaunchConfiguration('map').perform(context)
        if not map_yaml:
            raise RuntimeError('localization requires a non-empty map yaml file')

    pkg_share = get_package_share_directory('hanmole_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    controller_params = os.path.join(
        pkg_share, 'config', robot_version, 'ros2_controllers.yaml')
    default_nav2_params = os.path.join(
        pkg_share, 'config', robot_version, 'nav2_params.yaml')
    default_ekf_params = os.path.join(
        pkg_share, 'config', robot_version, 'ekf.yaml')

    nav2_params = (
        LaunchConfiguration('params_file').perform(context) or default_nav2_params)
    ekf_params = (
        LaunchConfiguration('state_estimator_params_file').perform(context) or default_ekf_params)

    base_controller_name = LaunchConfiguration(
        'base_controller_name').perform(context)
    controller_manager_name = LaunchConfiguration(
        'controller_manager_name').perform(context)

    actions = []

    if _as_bool(LaunchConfiguration('use_controller_spawners').perform(context)):
        actions.extend([
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_joint_state_broadcaster',
                output='screen',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    controller_manager_name,
                    '--param-file',
                    controller_params,
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_base_controller',
                output='screen',
                arguments=[
                    base_controller_name,
                    '--controller-manager',
                    controller_manager_name,
                    '--param-file',
                    controller_params,
                ],
            ),
        ])

    if _as_bool(LaunchConfiguration('use_state_estimator').perform(context)):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'ekf.launch.py')),
                launch_arguments={
                    'robot_version': robot_version,
                    'base_controller_name': base_controller_name,
                    'params_file': ekf_params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        )

    if robot_version == 'v0_1' and not _as_bool(
        LaunchConfiguration('use_state_estimator').perform(context)
    ):
        actions.extend([
            Node(
                package='topic_tools',
                executable='relay',
                name='relay_mecanum_odom',
                output='screen',
                parameters=[{
                    'input_topic': f'/{base_controller_name}/odometry',
                    'output_topic': '/odom',
                }],
            ),
            Node(
                package='topic_tools',
                executable='relay',
                name='relay_mecanum_tf',
                output='screen',
                parameters=[{
                    'input_topic': f'/{base_controller_name}/tf_odometry',
                    'output_topic': '/tf',
                }],
            ),
        ])

    if _as_bool(LaunchConfiguration('use_nav2').perform(context)):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'slam': LaunchConfiguration('slam'),
                    'map': LaunchConfiguration('map'),
                    'params_file': nav2_params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_collision_monitor': 'False',
                    'use_smoother': 'False',
                    'use_waypoint_follower': 'False',
                    'use_velocity_smoother': 'False',
                    'use_docking_server': 'False',
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    default_robot_version = _default_robot_version()

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_version',
            default_value=default_robot_version,
            description=(
                'AGV version. Default comes from config/settings.toml; '
                'use robot_version:=v0_1|v0_2 to override.'
            ),
        ),
        DeclareLaunchArgument(
            'base_controller_name',
            default_value='base_controller',
            description='Base controller name loaded by controller_manager.',
        ),
        DeclareLaunchArgument(
            'controller_manager_name',
            default_value='/controller_manager',
            description='controller_manager service namespace.',
        ),
        DeclareLaunchArgument(
            'use_controller_spawners',
            default_value='true',
            description='Spawn joint_state_broadcaster and base controller.',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Include nav2_bringup bringup_launch.py.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description=(
                'Optional Nav2 params file. Empty uses '
                'config/<robot_version>/nav2_params.yaml.'
            ),
        ),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        DeclareLaunchArgument(
            'use_state_estimator',
            default_value='true',
            description='Start robot_localization EKF',
        ),
        DeclareLaunchArgument(
            'state_estimator_params_file',
            default_value='',
            description='Optional EKF params file override',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
