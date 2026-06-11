from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_version = LaunchConfiguration('robot_version')
    use_safety_filter = LaunchConfiguration('use_safety_filter')
    use_velocity_smoother = LaunchConfiguration('use_velocity_smoother')
    use_nav2_recovery = LaunchConfiguration('use_nav2_recovery')
    use_safety_with_smoother = PythonExpression([
        '"', use_safety_filter, '" == "true" and "', use_velocity_smoother, '" == "true"'
    ])
    use_safety_without_smoother = PythonExpression([
        '"', use_safety_filter, '" == "true" and "', use_velocity_smoother, '" != "true"'
    ])

    follower_params = PathJoinSubstitution([
        FindPackageShare('hanmole_navigation'),
        'config',
        robot_version,
        'scan_path_follower.yaml',
    ])
    safety_params = PathJoinSubstitution([
        FindPackageShare('hanmole_navigation'),
        'config',
        robot_version,
        'cmd_vel_safety_filter.yaml',
    ])
    velocity_smoother_params = PathJoinSubstitution([
        FindPackageShare('hanmole_navigation'),
        'config',
        robot_version,
        'velocity_smoother.yaml',
    ])
    recovery_params = PathJoinSubstitution([
        FindPackageShare('hanmole_navigation'),
        'config',
        robot_version,
        'recovery_behaviors.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_version', default_value='v0_1'),
        DeclareLaunchArgument('use_safety_filter', default_value='true'),
        DeclareLaunchArgument('use_velocity_smoother', default_value='true'),
        DeclareLaunchArgument('use_nav2_recovery', default_value='true'),
        Node(
            package='hanmole_navigation',
            executable='scan_path_follower_node',
            name='scan_path_follower',
            output='screen',
            parameters=[follower_params, {
                'enable_recovery': ParameterValue(use_nav2_recovery, value_type=bool),
            }],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[velocity_smoother_params],
            remappings=[
                ('cmd_vel', '/cmd_vel_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
            condition=IfCondition(use_velocity_smoother),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_scan_velocity_smoother',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['velocity_smoother'],
            }],
            condition=IfCondition(use_velocity_smoother),
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_params],
            remappings=[
                ('cmd_vel', '/cmd_vel_recovery'),
            ],
            condition=IfCondition(use_nav2_recovery),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_scan_recovery',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['behavior_server'],
            }],
            condition=IfCondition(use_nav2_recovery),
        ),
        Node(
            package='hanmole_navigation',
            executable='cmd_vel_safety_filter_node',
            name='cmd_vel_safety_filter',
            output='screen',
            parameters=[safety_params],
            condition=IfCondition(use_safety_with_smoother),
        ),
        Node(
            package='hanmole_navigation',
            executable='cmd_vel_safety_filter_node',
            name='cmd_vel_safety_filter',
            output='screen',
            parameters=[safety_params, {'input_cmd_topic': '/cmd_vel_raw'}],
            condition=IfCondition(use_safety_without_smoother),
        ),
    ])
