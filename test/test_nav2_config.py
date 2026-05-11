"""Configuration contract tests for HanMole Nav2 profiles."""

from pathlib import Path

import yaml


def test_v0_1_nav2_profile_uses_filtered_odom_and_fused_scan_contracts():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'v0_1'
        / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    velocity_smoother = config['velocity_smoother']['ros__parameters']
    collision_monitor = config['collision_monitor']['ros__parameters']
    amcl = config['amcl']['ros__parameters']
    controller_server = config['controller_server']['ros__parameters']

    assert velocity_smoother['odom_topic'] == '/odometry/filtered'
    assert velocity_smoother['enable_stamped_cmd_vel'] is True
    assert collision_monitor['base_frame_id'] == 'base_footprint'
    assert collision_monitor['enable_stamped_cmd_vel'] is True
    assert collision_monitor['observation_sources'] == ['scan']
    assert collision_monitor['scan']['topic'] == '/scan'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == '/scan'
    assert controller_server['enable_stamped_cmd_vel'] is True


def test_v0_2_nav2_profile_publishes_stamped_cmd_vel():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'v0_2'
        / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    controller_server = config['controller_server']['ros__parameters']
    velocity_smoother = config['velocity_smoother']['ros__parameters']

    assert controller_server['enable_stamped_cmd_vel'] is True
    assert velocity_smoother['enable_stamped_cmd_vel'] is True
