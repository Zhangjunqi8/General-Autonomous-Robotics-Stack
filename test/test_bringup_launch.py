"""Launch contract tests for hanmole_navigation."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node


def _load_launch_module(relative_path: str, module_name: str):
    module_path = Path(__file__).resolve().parents[1] / relative_path
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _node_private_value(node: Node, key: str):
    return getattr(node, key)


def _node_parameters_text(node: Node) -> str:
    return str(_node_private_value(node, '_Node__parameters'))


def test_navigation_public_bringup_exists():
    bringup = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    assert bringup.exists()


def test_navigation_launch_directory_only_keeps_three_entrypoints():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    launch_files = sorted(path.name for path in launch_dir.glob('*.py'))

    assert launch_files == [
        'bringup.launch.py',
        'localization_launch.py',
        'navigation_launch.py',
    ]


def test_v0_1_state_estimator_path_does_not_relay_mecanum_tf():
    module = _load_launch_module(
        'launch/navigation_launch.py',
        'hanmole_navigation_navigation_launch',
    )

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'params_file': '',
        'slam': 'False',
        'map': '/tmp/mock_map.yaml',
        'use_sim_time': 'false',
        'autostart': 'true',
        'use_state_estimator': 'true',
        'state_estimator_params_file': '',
    })

    actions = module._launch_setup(context)

    relay_nodes = [
        action for action in actions
        if isinstance(action, Node)
        and _node_private_value(action, '_Node__package') == 'topic_tools'
    ]
    relay_parameter_text = '\n'.join(_node_parameters_text(action) for action in relay_nodes)

    assert '/base_controller/tf_odometry' not in relay_parameter_text


def test_v0_1_state_estimator_path_does_not_launch_cmd_vel_adapter():
    module = _load_launch_module(
        'launch/navigation_launch.py',
        'hanmole_navigation_navigation_launch_cmd_vel',
    )

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'params_file': '',
        'slam': 'False',
        'map': '/tmp/mock_map.yaml',
        'use_sim_time': 'false',
        'autostart': 'true',
        'use_state_estimator': 'true',
        'state_estimator_params_file': '',
    })

    actions = module._launch_setup(context)
    adapter_nodes = [
        action for action in actions
        if isinstance(action, Node)
        and _node_private_value(action, '_Node__package') == 'hanmole_navigation'
    ]
    executable_text = '\n'.join(
        _node_private_value(action, '_Node__node_executable') for action in adapter_nodes
    )
    assert 'cmd_vel_to_twist_stamped' not in executable_text


def test_v0_1_localization_requires_non_empty_map_argument():
    module = _load_launch_module(
        'launch/navigation_launch.py',
        'hanmole_navigation_navigation_launch_requires_map',
    )

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'params_file': '',
        'slam': 'False',
        'map': '',
        'use_sim_time': 'false',
        'autostart': 'true',
        'use_state_estimator': 'true',
        'state_estimator_params_file': '',
    })

    try:
        module._launch_setup(context)
    except RuntimeError as exc:
        assert 'map yaml file' in str(exc)
    else:
        raise AssertionError('expected localization launch to reject an empty map argument')


def test_public_bringup_switches_to_ekf_odom_mode_by_default():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('nav_mode', default_value='ekf_odom')" in source
    assert "localization_launch.py" in source
    assert "navigation_launch.py" in source
    assert '" == "ekf_odom"' in source
    assert '" == "wheel_odom"' in source


def test_public_bringup_only_starts_ekf_for_ekf_odom():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert "condition=IfCondition(ekf_odom_mode)" in source


def test_navigation_launch_turns_off_nav2_lifecycle_autostart():
    source = (
        Path(__file__).resolve().parents[1] / 'launch' / 'navigation_launch.py'
    ).read_text(encoding='utf-8')

    assert "'autostart': False" in source


def test_navigation_launch_passes_nav_mode_to_gateway_node():
    source = (
        Path(__file__).resolve().parents[1] / 'launch' / 'navigation_launch.py'
    ).read_text(encoding='utf-8')

    assert "'nav_mode': LaunchConfiguration('nav_mode')" in source


def test_v0_1_navigation_launch_uses_accelerator_mux_before_collision_monitor():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'navigation_launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert "executable='cmd_vel_nav_accelerator_mux_node'" in source
    assert "name='cmd_vel_nav_accelerator_mux'" in source
    assert "package='nav2_collision_monitor'" in source
    assert "name='collision_monitor'" in source
    assert "use_cmd_vel_accelerator = robot_version == 'v0_1'" in source


def test_public_bringup_no_longer_declares_cmd_vel_adapter_toggle():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert 'use_cmd_vel_adapter' not in source
    assert 'use_nav2' not in source


def test_public_bringup_no_longer_references_legacy_launch_files():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert 'ekf.launch.py' not in source
    assert 'nav_bringup.launch.py' not in source
    assert 'ekf_target_navigation.launch.py' not in source
    assert 'route_navigation.launch.py' not in source
    assert 'rviz.launch.py' not in source


def test_navigation_launch_starts_gateway_and_catalog_nodes():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'navigation_launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert "executable='nav2_target_gateway_node'" in source
    assert "executable='target_catalog_publisher_node'" in source
