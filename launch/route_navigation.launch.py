from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('hanmole_navigation')
    default_params = PathJoinSubstitution([package_share, 'config', 'route_navigation.yaml'])
    default_ekf_params = PathJoinSubstitution(
        [package_share, 'config', 'v0_1', 'ekf.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[default_ekf_params],
            condition=IfCondition(LaunchConfiguration('use_ekf')),
        ),
        Node(
            package='hanmole_navigation',
            executable='route_follower_node',
            name='route_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='hanmole_navigation',
            executable='route_recorder_node',
            name='route_recorder',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
