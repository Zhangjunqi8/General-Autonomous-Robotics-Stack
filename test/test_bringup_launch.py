"""Launch contract tests for hanmole_navigation."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
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


def _expanded_node_parameter_map(node: Node, context: LaunchContext) -> dict[str, str]:
    parameters = _node_private_value(node, '_Node__parameters')
    expanded = {}
    for parameter_dict in parameters:
        for key, value in parameter_dict.items():
            expanded[perform_substitutions(context, list(key))] = perform_substitutions(
                context, list(value)
            )
    return expanded


def test_navigation_public_bringup_exists():
    bringup = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    assert bringup.exists()


def test_v0_1_state_estimator_path_does_not_relay_mecanum_tf():
    module = _load_launch_module('launch/nav_bringup.launch.py', 'hanmole_navigation_nav_bringup')

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'use_nav2': 'false',
        'params_file': '',
        'slam': 'False',
        'map': '',
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
        'launch/nav_bringup.launch.py',
        'hanmole_navigation_nav_bringup_cmd_vel',
    )

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'use_nav2': 'false',
        'params_file': '',
        'slam': 'False',
        'map': '',
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
        'launch/nav_bringup.launch.py',
        'hanmole_navigation_nav_bringup_requires_map',
    )

    context = LaunchContext()
    context.launch_configurations.update({
        'robot_version': 'v0_1',
        'base_controller_name': 'base_controller',
        'controller_manager_name': '/controller_manager_agv',
        'use_controller_spawners': 'false',
        'use_nav2': 'true',
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


def test_v0_1_public_bringup_forwards_localization_arguments():
    module = _load_launch_module('launch/bringup.launch.py', 'hanmole_navigation_public_bringup')

    launch_description = module.generate_launch_description()
    include_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert include_actions


def test_public_bringup_no_longer_declares_cmd_vel_adapter_toggle():
    bringup_path = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    source = bringup_path.read_text(encoding='utf-8')

    assert 'use_cmd_vel_adapter' not in source
