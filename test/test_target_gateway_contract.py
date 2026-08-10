"""Source-level contracts for nav2 target gateway runtime behavior."""

from pathlib import Path


def test_target_gateway_uses_multithreaded_executor():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'MultiThreadedExecutor' in source


def test_target_gateway_stamps_initial_pose_from_latest_odom_tf():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'lookup_latest_transform_stamp(odom_frame_, robot_base_frame_)' in source
    assert 'message.header.stamp = *latest_odom_tf_stamp' in source


def test_target_gateway_uses_reentrant_callback_group():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'CallbackGroupType::Reentrant' in source


def test_target_gateway_supports_already_at_target_short_circuit():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'goal_xy_tolerance' in source
    assert 'goal_yaw_tolerance' in source
    assert 'already at target' in source


def test_target_gateway_supports_post_nav_fine_tune_direct_cmd_vel():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'fine_tune_enabled' in source
    assert 'fine_tuning' in source
    assert 'fine_tune_cmd_vel_topic' in source
    assert 'ActionStatusRunning_fine_tune' in source
    assert 'create_publisher<geometry_msgs::msg::TwistStamped>' in source


def test_target_gateway_defers_gateway_terminal_status_to_result_callback():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'gatewayGoalHandleIsActive' in source
    assert 'result callback owns gateway terminal state' in source


def test_target_gateway_declares_startup_orchestration_parameters():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'startup_delay_ms' in source
    assert 'input_freshness_timeout_ms' in source
    assert 'stable_tf_success_count' in source
    assert 'lifecycle_request_timeout_ms' in source
    assert 'nav_mode' in source


def test_target_gateway_selects_filtered_or_raw_odom_by_nav_mode():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert '/odometry/filtered' in source
    assert '/odom' in source
    assert 'wheel_odom' in source
    assert 'ekf_odom' in source
    assert 'nav_mode_ == "wheel_odom"' in source


def test_target_gateway_uses_lifecycle_manage_nodes_service():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'nav2_msgs/srv/manage_lifecycle_nodes.hpp' in source
    assert 'STARTUP' in source
    assert 'lifecycle_manager_localization' in source
    assert 'lifecycle_manager_navigation' in source
    assert 'lifecycle_msgs/srv/get_state.hpp' in source


def test_target_gateway_guards_startup_tick_reentry_and_checks_active_states():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'startup_tick_running_' in source
    assert 'map_server/get_state' in source
    assert 'amcl/get_state' in source
    assert 'planner_server/get_state' in source
    assert 'controller_server/get_state' in source
    assert 'bt_navigator/get_state' in source


def test_target_gateway_rejects_navigation_before_stack_ready():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'navigation stack not ready' in source


def test_target_gateway_has_startup_state_machine_contract():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'boot_delay' in source
    assert 'waiting_for_inputs' in source
    assert 'waiting_for_map_tf' in source
    assert 'starting_navigation' in source
    assert 'waiting_for_navigation_ready' in source


def test_target_gateway_waits_for_fresh_odom_tf_before_initial_pose():
    source = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'nav2_target_gateway_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'tf_freshness_threshold_ms' in source
    assert 'lookup_latest_transform_stamp' in source
    assert 'transform_is_fresh' in source
