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
    boost_source = config['cmd_vel_boost_source']['ros__parameters']
    mux = config['cmd_vel_mux_stamped']['ros__parameters']
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
    assert controller_server['cmd_vel_out_topic'] == 'cmd_vel_nav'
    assert boost_source['input_topic'] == 'cmd_vel_nav'
    assert boost_source['output_topic'] == 'cmd_vel_nav_fast'
    assert boost_source['scan_topic'] == '/scan'
    assert boost_source['boost_start_linear_x'] == 0.60
    assert boost_source['boost_gain'] == 4.00
    assert boost_source['max_linear_x'] == 1.20
    assert boost_source['max_abs_linear_y'] == 0.10
    assert boost_source['max_abs_angular_z'] == 0.15
    assert boost_source['corridor_lookahead_m'] == 3.0
    assert 'corridor_half_width' not in boost_source
    assert mux['output_topic'] == 'cmd_vel_nav_muxed'
    assert mux['input_names'] == ['nav_fast', 'nav']
    assert mux['inputs']['nav_fast']['topic'] == 'cmd_vel_nav_fast'
    assert mux['inputs']['nav_fast']['priority'] == 100
    assert mux['inputs']['nav_fast']['timeout'] == 0.25
    assert mux['inputs']['nav']['topic'] == 'cmd_vel_nav'
    assert mux['inputs']['nav']['priority'] == 10
    assert mux['inputs']['nav']['timeout'] == 0.20
    assert velocity_smoother['cmd_vel_in_topic'] == 'cmd_vel_nav_muxed'
    assert velocity_smoother['cmd_vel_out_topic'] == 'cmd_vel_smoothed'
    assert global_costmap['update_frequency'] == 1.0
    assert global_costmap['publish_frequency'] == 1.0
    expected_costmap_footprint = '[[0.31, 0.30], [0.31, -0.30], [-0.33, -0.30], [-0.33, 0.30]]'
    expected_stop_polygon = '[[0.31, 0.30], [0.31, -0.30], [-0.33, -0.30], [-0.33, 0.30]]'
    assert global_costmap['footprint'] == expected_costmap_footprint
    assert local_costmap['footprint'] == expected_costmap_footprint
    assert global_costmap['footprint_padding'] == 0.02
    assert local_costmap['footprint_padding'] == 0.02
    assert collision_monitor['FootprintStop']['points'] == expected_stop_polygon


def test_v0_1_nav2_profile_matches_current_mppi_omni_behavior():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'v0_1'
        / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    controller_server = config['controller_server']['ros__parameters']
    progress_checker = controller_server['progress_checker']
    goal_checker = controller_server['general_goal_checker']
    follow_path = controller_server['FollowPath']
    planner = config['planner_server']['ros__parameters']['GridBased']
    behavior_server = config['behavior_server']['ros__parameters']
    velocity_smoother = config['velocity_smoother']['ros__parameters']
    collision_monitor = config['collision_monitor']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']
    local_costmap = config['local_costmap']['local_costmap']['ros__parameters']
    local_scan = local_costmap['obstacle_layer']['scan']

    assert controller_server['min_y_velocity_threshold'] == 0.02
    assert progress_checker['required_movement_radius'] == 0.05
    assert progress_checker['movement_time_allowance'] == 5.0
    assert goal_checker['plugin'] == 'nav2_controller::StoppedGoalChecker'
    assert goal_checker['xy_goal_tolerance'] == 0.20
    assert goal_checker['yaw_goal_tolerance'] == 0.15
    assert goal_checker['trans_stopped_velocity'] == 0.05
    assert goal_checker['rot_stopped_velocity'] == 0.05
    assert follow_path['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow_path['motion_model'] == 'Omni'
    assert follow_path['time_steps'] == 70
    assert follow_path['model_dt'] == 0.05
    assert follow_path['batch_size'] == 2000
    assert follow_path['vx_max'] == 1.05
    assert follow_path['vx_min'] == -0.15
    assert follow_path['vy_max'] == 0.28
    assert follow_path['vy_min'] == -0.28
    assert follow_path['wz_max'] == 1.2
    assert follow_path['wz_min'] == -1.0
    assert follow_path['use_polygon_footprint'] is True

    assert planner['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert planner['tolerance'] == 0.5
    assert planner['allow_unknown'] is True
    assert planner['cost_travel_multiplier'] == 4.0

    assert behavior_server['max_rotational_vel'] == 4.5
    assert behavior_server['min_rotational_vel'] == 1.0
    assert behavior_server['rotational_acc_lim'] == 5.0

    assert velocity_smoother['feedback'] == 'OPEN_LOOP'
    assert velocity_smoother['stamp_smoothed_velocity_with_smoothing_time'] is True
    assert velocity_smoother['max_velocity'] == [1.20, 0.28, 1.4]
    assert velocity_smoother['min_velocity'] == [-0.15, -0.28, -1.0]
    assert velocity_smoother['max_accel'] == [6.0, 0.45, 1.0]
    assert velocity_smoother['max_decel'] == [-6.0, -0.9, -2.0]
    assert velocity_smoother['deadband_velocity'] == [0.0, 0.01, 0.001]
    assert velocity_smoother['velocity_timeout'] == 1.0

    assert collision_monitor['polygons'] == ['FootprintStop', 'FootprintApproach']

    assert global_costmap['update_frequency'] == 1.0
    assert global_costmap['publish_frequency'] == 1.0
    assert global_costmap['inflation_layer']['inflation_radius'] == 0.9
    assert local_costmap['update_frequency'] == 10.0
    assert local_costmap['publish_frequency'] == 5.0
    assert local_costmap['width'] == 6
    assert local_costmap['height'] == 6
    assert local_scan['obstacle_max_range'] == 2.0
    assert local_scan['raytrace_max_range'] == 2.5
    assert local_scan['observation_persistence'] == 0.0
    assert local_costmap['inflation_layer']['cost_scaling_factor'] == 5.0
    assert local_costmap['inflation_layer']['inflation_radius'] == 0.5


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


def test_v0_2_nav2_profile_matches_old_project_like_navigation_behavior():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'v0_2'
        / 'nav2_params.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    controller_server = config['controller_server']['ros__parameters']
    goal_checker = controller_server['general_goal_checker']
    follow_path = controller_server['FollowPath']
    planner = config['planner_server']['ros__parameters']['GridBased']
    velocity_smoother = config['velocity_smoother']['ros__parameters']
    behavior_server = config['behavior_server']['ros__parameters']

    assert controller_server['min_y_velocity_threshold'] == 0.5
    assert goal_checker['stateful'] is True
    assert goal_checker['xy_goal_tolerance'] == 0.10
    assert goal_checker['yaw_goal_tolerance'] == 0.10

    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert follow_path['desired_linear_vel'] == 0.35
    assert follow_path['lookahead_dist'] == 0.6
    assert follow_path['min_lookahead_dist'] == 0.15
    assert follow_path['max_lookahead_dist'] == 0.8
    assert follow_path['lookahead_time'] == 1.0
    assert follow_path['rotate_to_heading_angular_vel'] == 0.8
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['rotate_to_heading_min_angle'] == 0.35
    assert follow_path['allow_reversing'] is True
    assert follow_path['min_approach_linear_velocity'] == 0.05
    assert follow_path['approach_velocity_scaling_dist'] == 0.5
    assert follow_path['max_angular_accel'] == 0.8
    assert follow_path['stateful'] is True

    assert planner['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert planner['tolerance'] == 0.5
    assert planner['use_astar'] is True
    assert planner['allow_unknown'] is True

    assert velocity_smoother['max_velocity'] == [0.30, 0.0, 0.35]
    assert velocity_smoother['min_velocity'] == [-0.25, 0.0, -0.35]
    assert velocity_smoother['max_accel'] == [0.80, 0.0, 0.80]
    assert velocity_smoother['max_decel'] == [-0.80, 0.0, -0.80]
    assert velocity_smoother['deadband_velocity'] == [0.002, 0.0, 0.02]

    assert behavior_server['max_rotational_vel'] == 0.50
    assert behavior_server['min_rotational_vel'] == 0.10
    assert behavior_server['rotational_acc_lim'] == 0.80


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
    assert gateway['goal_xy_tolerance'] == 0.05
    assert gateway['goal_yaw_tolerance'] == 0.03


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
