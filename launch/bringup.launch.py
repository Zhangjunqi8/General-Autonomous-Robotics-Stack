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
        DeclareLaunchArgument('nav_map_yaml_file', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=''),
        DeclareLaunchArgument('base_controller_name', default_value='base_controller'),
        DeclareLaunchArgument('use_composition', default_value='False'),
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

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'navigation_launch.py'])
        ),
        condition=IfCondition(supported_nav_mode),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'nav_mode': LaunchConfiguration('nav_mode'),
            'map': LaunchConfiguration('nav_map_yaml_file'),
            'params_file': LaunchConfiguration('nav_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            ekf_launch,
            nav_launch,
        ]
    )
