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

    assert velocity_smoother['odom_topic'] == '/odometry/filtered'
    assert collision_monitor['base_frame_id'] == 'base_footprint'
    assert collision_monitor['observation_sources'] == ['scan']
    assert collision_monitor['scan']['topic'] == '/scan'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == '/scan'
