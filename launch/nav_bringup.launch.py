"""Bring up Hanmole AGV controller and Nav2 by robot_version."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
import tomllib


def _as_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _launch_config_as_bool(context, name: str, default: str = 'false') -> bool:
    return _as_bool(context.launch_configurations.get(name, default))


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


def _navigation_remappings() -> list[tuple[str, str]]:
    return [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def _navigation_lifecycle_nodes() -> list[str]:
    return [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]


def _build_configured_nav2_params(
    params_file,
    namespace,
    use_namespace,
    autostart,
    map_yaml='',
):
    use_namespace_value = use_namespace
    if isinstance(use_namespace, str):
        use_namespace_value = use_namespace
    rewritten_source = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace_value),
    )
    param_rewrites = {'autostart': autostart}
    if map_yaml:
        param_rewrites['map_server.ros__parameters.yaml_filename'] = map_yaml
    return ParameterFile(
        RewrittenYaml(
            source_file=rewritten_source,
            root_key=namespace,
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )


def _create_navigation_node_actions(
    configured_params,
    use_sim_time,
    use_respawn,
    log_level,
):
    remappings = _navigation_remappings()
    lifecycle_node_names = _navigation_lifecycle_nodes()
    controller_action = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    return GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            controller_action,
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': LaunchConfiguration('autostart')},
                    {'node_names': lifecycle_node_names},
                ],
            ),
        ],
    )


def _create_navigation_composable_actions(
    configured_params,
    namespace,
    container_name,
    use_sim_time,
    log_level,
):
    remappings = _navigation_remappings()
    composable_nodes = [
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_smoother',
            plugin='nav2_smoother::SmootherServer',
            name='smoother_server',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_planner',
            plugin='nav2_planner::PlannerServer',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_waypoint_follower',
            plugin='nav2_waypoint_follower::WaypointFollower',
            name='waypoint_follower',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        ComposableNode(
            package='nav2_collision_monitor',
            plugin='nav2_collision_monitor::CollisionMonitor',
            name='collision_monitor',
            parameters=[configured_params],
            remappings=remappings,
        ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[
                {
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': _navigation_lifecycle_nodes(),
                }
            ],
        ),
    ]

    actions = [
        SetParameter('use_sim_time', use_sim_time),
        LoadComposableNodes(
            target_container=(namespace, '/', container_name),
            composable_node_descriptions=composable_nodes,
        ),
    ]
    return GroupAction(
        condition=IfCondition(LaunchConfiguration('use_composition')),
        actions=actions,
    )


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

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    namespace_value = context.launch_configurations.get('namespace', '')
    use_namespace_value = context.launch_configurations.get('use_namespace', 'False')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    container_name = LaunchConfiguration('container_name')
    use_sim_time_value = use_sim_time.perform(context)
    autostart_value = autostart.perform(context)
    use_composition_value = context.launch_configurations.get(
        'use_composition', 'True')
    use_respawn_value = context.launch_configurations.get(
        'use_respawn', 'False')
    container_name_value = context.launch_configurations.get(
        'container_name', 'nav2_container')
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
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )

    if _as_bool(LaunchConfiguration('use_nav2').perform(context)):
        configured_params = _build_configured_nav2_params(
            nav2_params,
            namespace_value,
            use_namespace_value,
            autostart,
            map_yaml,
        )
        configured_params_path = str(configured_params.evaluate(context))
        nav2_launch_dir = os.path.join(nav2_share, 'launch')
        actions.extend([
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            GroupAction(
                [
                    PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                    Node(
                        condition=IfCondition(use_composition),
                        name='nav2_container',
                        package='rclcpp_components',
                        executable='component_container_isolated',
                        parameters=[configured_params, {'autostart': autostart}],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=_navigation_remappings(),
                        output='screen',
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_launch_dir, 'slam_launch.py')
                        ),
                        condition=IfCondition(LaunchConfiguration('slam')),
                        launch_arguments={
                            'namespace': namespace,
                            'use_sim_time': use_sim_time_value,
                            'autostart': autostart_value,
                            'use_respawn': use_respawn_value,
                            'params_file': configured_params_path,
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_launch_dir, 'localization_launch.py')
                        ),
                        condition=UnlessCondition(LaunchConfiguration('slam')),
                        launch_arguments={
                            'namespace': namespace,
                            'map': map_yaml,
                            'use_sim_time': use_sim_time_value,
                            'autostart': autostart_value,
                            'params_file': configured_params_path,
                            'use_composition': use_composition_value,
                            'use_respawn': use_respawn_value,
                            'container_name': container_name_value,
                        }.items(),
                    ),
                    _create_navigation_node_actions(
                        configured_params,
                        use_sim_time,
                        use_respawn,
                        log_level,
                    ),
                    _create_navigation_composable_actions(
                        configured_params,
                        namespace,
                        container_name,
                        use_sim_time,
                        log_level,
                    ),
                ]
            ),
        ])

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
            description='Include Nav2 localization and navigation actions.',
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
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='False'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),

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
