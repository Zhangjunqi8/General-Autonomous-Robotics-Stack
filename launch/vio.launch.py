"""Launch ORB-SLAM3 stereo-inertial VIO without mutating global environments."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_path(robot_version: str) -> Path:
    if robot_version not in ('v0_1', 'v0_2'):
        raise RuntimeError(f'robot_version must be v0_1 or v0_2, got {robot_version}')
    share = Path(get_package_share_directory('hanmole_navigation'))
    path = share / 'config' / robot_version / 'vio.yaml'
    if not path.exists():
        raise RuntimeError(f'vio config file not found: {path}')
    return path


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot_version = LaunchConfiguration('robot_version').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    resolved_config_file = config_file or str(_default_config_path(robot_version))

    return [
        Node(
            package='hanmole_navigation',
            executable='orbslam3_stereo_inertial_node',
            name='orbslam3_stereo_inertial_node',
            output='screen',
            parameters=[
                {
                    'config_file': resolved_config_file,
                    'left_image_topic': LaunchConfiguration('vo_left_topic'),
                    'right_image_topic': LaunchConfiguration('vo_right_topic'),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_version', default_value='v0_1'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument(
            'vo_left_topic',
            default_value='/hanmole/sensor/camera_head_left/image_raw',
        ),
        DeclareLaunchArgument(
            'vo_right_topic',
            default_value='/hanmole/sensor/camera_head_right/image_raw',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
