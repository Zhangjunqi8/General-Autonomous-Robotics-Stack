"""HanMole state estimator launch based on robot_localization EKF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _package_share() -> Path:
    return Path(get_package_share_directory('hanmole_navigation'))


def _default_params_path(robot_version: str) -> Path:
    if robot_version not in ('v0_1', 'v0_2'):
        raise RuntimeError(f'robot_version must be v0_1 or v0_2, got {robot_version}')
    path = _package_share() / 'config' / robot_version / 'ekf.yaml'
    if not path.exists():
        raise RuntimeError(f'ekf params file not found: {path}')
    return path


def _default_vio_config_path(robot_version: str) -> Path:
    return _package_share() / 'config' / robot_version / 'vio.yaml'


def _vio_runtime_ready(config_path: Path) -> bool:
    if not config_path.exists():
        return False

    try:
        config = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError:
        return False

    orbslam3 = config.get('orbslam3') or {}
    if not orbslam3.get('calibration_ready', False):
        return False

    library_root = Path(str(orbslam3.get('library_root', '')))
    vocabulary = Path(str(orbslam3.get('vocabulary', '')))
    required_files = [
        vocabulary,
        library_root / 'lib' / 'libORB_SLAM3.so',
        library_root / 'Thirdparty' / 'DBoW2' / 'lib' / 'libDBoW2.so',
        library_root / 'Thirdparty' / 'g2o' / 'lib' / 'libg2o.so',
    ]
    return all(path.exists() for path in required_files)


def _vio_ekf_parameters() -> dict:
    return {
        'odom1': '/vio/odom',
        'odom1_config': [
            True, True, False,
            False, False, True,
            False, False, False,
            False, False, False,
            False, False, False,
        ],
        'odom1_queue_size': 10,
        'odom1_differential': False,
        'odom1_relative': True,
    }


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot_version = LaunchConfiguration('robot_version').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    resolved_params_file = params_file or str(_default_params_path(robot_version))

    vio_config_file = _default_vio_config_path(robot_version)
    use_vio = _vio_runtime_ready(vio_config_file)

    ekf_overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'odom0': '/odom',
    }
    if use_vio:
        ekf_overrides.update(_vio_ekf_parameters())

    actions = []
    if use_vio:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(_package_share() / 'launch' / 'vio.launch.py')
                ),
                launch_arguments={
                    'robot_version': robot_version,
                    'config_file': str(vio_config_file),
                }.items(),
            )
        )

    actions.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                resolved_params_file,
                ekf_overrides,
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_version', default_value='v0_2'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
