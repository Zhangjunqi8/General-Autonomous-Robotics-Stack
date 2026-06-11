"""Public navigation bringup entrypoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the public navigation bringup description."""
    package_share = FindPackageShare('hanmole_navigation')

    declared_arguments = [
        DeclareLaunchArgument('robot_version', default_value='v0_2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav_mode', default_value='ekf_odom'),
        DeclareLaunchArgument('map_yaml_file', default_value=''),
        DeclareLaunchArgument('nav_map_yaml_file', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=''),
        DeclareLaunchArgument('base_controller_name', default_value='base_controller'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_scan_local_navigation', default_value='true'),
        DeclareLaunchArgument('use_scan_velocity_smoother', default_value='true'),
        DeclareLaunchArgument('use_nav2_recovery', default_value='true'),
        DeclareLaunchArgument('use_nav2_controller', default_value='false'),
        DeclareLaunchArgument('use_smoother_server', default_value='false'),
        DeclareLaunchArgument('use_behavior_server', default_value='false'),
        DeclareLaunchArgument('use_velocity_smoother', default_value='false'),
        DeclareLaunchArgument('use_collision_monitor', default_value='false'),
        DeclareLaunchArgument('use_bt_navigator', default_value='false'),
        DeclareLaunchArgument('use_waypoint_follower', default_value='false'),
    ]

    ekf_odom_mode = PythonExpression([
        '"',
        LaunchConfiguration('nav_mode'),
        '" == "ekf_odom"',
    ])
    wheel_odom_mode = PythonExpression([
        '"',
        LaunchConfiguration('nav_mode'),
        '" == "wheel_odom"',
    ])
    supported_nav_mode = PythonExpression([
        '"',
        LaunchConfiguration('nav_mode'),
        '" == "ekf_odom" or "',
        LaunchConfiguration('nav_mode'),
        '" == "wheel_odom"',
    ])
    map_yaml_file = PythonExpression([
        "'",
        LaunchConfiguration('map_yaml_file'),
        "' or '",
        LaunchConfiguration('nav_map_yaml_file'),
        "'",
    ])

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'localization_launch.py'])
        ),
        condition=IfCondition(ekf_odom_mode),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'params_file': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    scan_local_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'scan_local_navigation.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('use_scan_local_navigation')),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'use_velocity_smoother': LaunchConfiguration('use_scan_velocity_smoother'),
            'use_nav2_recovery': LaunchConfiguration('use_nav2_recovery'),
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'navigation_launch.py'])
        ),
        condition=IfCondition(supported_nav_mode),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'nav_mode': LaunchConfiguration('nav_mode'),
            'map': map_yaml_file,
            'params_file': LaunchConfiguration('nav_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_nav2_controller': LaunchConfiguration('use_nav2_controller'),
            'use_smoother_server': LaunchConfiguration('use_smoother_server'),
            'use_behavior_server': LaunchConfiguration('use_behavior_server'),
            'use_velocity_smoother': LaunchConfiguration('use_velocity_smoother'),
            'use_collision_monitor': LaunchConfiguration('use_collision_monitor'),
            'use_bt_navigator': LaunchConfiguration('use_bt_navigator'),
            'use_waypoint_follower': LaunchConfiguration('use_waypoint_follower'),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            ekf_launch,
            nav_launch,
            scan_local_navigation_launch,
        ]
    )
