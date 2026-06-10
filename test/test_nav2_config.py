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
    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']
    local_costmap = config['local_costmap']['local_costmap']['ros__parameters']

    assert velocity_smoother['odom_topic'] == '/odometry/filtered'
    assert velocity_smoother['enable_stamped_cmd_vel'] is True
    assert collision_monitor['base_frame_id'] == 'base_footprint'
    assert collision_monitor['enable_stamped_cmd_vel'] is True
    assert collision_monitor['source_timeout'] == 1.0
    assert collision_monitor['observation_sources'] == ['scan']
    assert collision_monitor['scan']['topic'] == '/scan'
    assert collision_monitor['scan']['source_timeout'] == 1.0
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == '/scan'
    assert controller_server['enable_stamped_cmd_vel'] is True
    assert global_costmap['update_frequency'] == 5.0
    assert global_costmap['publish_frequency'] == 5.0
    expected_costmap_footprint = '[[0.24, 0.17], [0.24, -0.17], [-0.24, -0.17], [-0.24, 0.17]]'
    expected_stop_polygon = '[[0.40, 0.18], [0.40, -0.18], [-0.25, -0.18], [-0.25, 0.18]]'
    assert global_costmap['footprint'] == expected_costmap_footprint
    assert local_costmap['footprint'] == expected_costmap_footprint
    assert collision_monitor['FootprintStop']['points'] == expected_stop_polygon


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
    collision_monitor = config['collision_monitor']['ros__parameters']

    assert controller_server['enable_stamped_cmd_vel'] is True
    assert velocity_smoother['enable_stamped_cmd_vel'] is True
    assert collision_monitor['enable_stamped_cmd_vel'] is True
    assert controller_server['cmd_vel_out_topic'] == 'cmd_vel_nav'
    assert velocity_smoother['cmd_vel_in_topic'] == 'cmd_vel_nav'
    assert velocity_smoother['cmd_vel_out_topic'] == 'cmd_vel_smoothed'
    assert collision_monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert collision_monitor['cmd_vel_out_topic'] == 'cmd_vel'


def test_ekf_target_navigation_map_profile_exists_and_uses_map_frame():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'target_map.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    assert config['frame'] == 'map'
    assert config['default_group'] == 'default'
    assert config['initial_pose_target'] == 'HOME'
    assert 'target_groups' in config
    assert 'poses' in config['target_groups']['default']['HOME']


def test_ekf_target_navigation_launch_uses_map_target_config():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'ekf_target_navigation.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    gateway = config['nav2_target_gateway']['ros__parameters']
    catalog = config['target_catalog_publisher']['ros__parameters']

    assert gateway['target_file'].endswith('/target_map.yaml')
    assert catalog['target_file'].endswith('/target_map.yaml')
    assert gateway['nav_status_topic'] == '/nav_status'
    assert gateway['amcl_pose_topic'] == '/amcl_pose'


def test_v0_1_ekf_profile_predicts_filtered_odom_to_current_time():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'v0_1'
        / 'ekf.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    ekf = config['ekf_filter_node']['ros__parameters']

    assert ekf['frequency'] == 20.0
    assert ekf['sensor_timeout'] == 0.1
    assert ekf['predict_to_current_time'] is True
    assert ekf['transform_time_offset'] == 0.02
