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
        DeclareLaunchArgument('nav_mode', default_value='localization'),
        DeclareLaunchArgument('map_yaml_file', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=''),
        DeclareLaunchArgument('base_controller_name', default_value='base_controller'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
    ]

    localization_mode = PythonExpression([
        '"',
        LaunchConfiguration('nav_mode'),
        '" == "localization"',
    ])

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'nav_bringup.launch.py'])
        ),
        condition=IfCondition(localization_mode),
        launch_arguments={
            'robot_version': LaunchConfiguration('robot_version'),
            'map': LaunchConfiguration('map_yaml_file'),
            'params_file': LaunchConfiguration('nav_params_file'),
            'base_controller_name': LaunchConfiguration('base_controller_name'),
            'use_controller_spawners': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [nav_launch])
