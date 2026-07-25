#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "hanmole_msgs/srv/set_string.hpp"
#include "hanmole_navigation/target_repository.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/srv/toggle.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }

  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2 != 0) {
    return upper;
  }

  const auto lower = std::max_element(values.begin(), middle);
  return (*lower + upper) * 0.5;
}

geometry_msgs::msg::Quaternion quaternion_from_target_pose(
  const hanmole_navigation::TargetPose & pose)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = pose.qx;
  quaternion.y = pose.qy;
  quaternion.z = pose.qz;
  quaternion.w = pose.qw;
  return quaternion;
}

double yaw_from_target_pose(const hanmole_navigation::TargetPose & pose)
{
  return yaw_from_quaternion(quaternion_from_target_pose(pose));
}

enum class StartupState
{
  BootDelay,
  WaitingForInputs,
  WaitingForOdomTf,
  StartingLocalization,
  WaitingForLocalizationReady,
  PublishingInitialPose,
  WaitingForMapTf,
  StartingNavigation,
  WaitingForNavigationReady,
  Ready,
  Error,
};

enum class DockState
{
  Free,
  Docking,
  Docked,
  Undocking,
};

struct ChargingHubDetection
{
  geometry_msgs::msg::PoseStamped dock_pose;
  geometry_msgs::msg::Point left_feature;
  geometry_msgs::msg::Point right_feature;
  double confidence{0.0};
};

const char * startup_state_name(StartupState state)
{
  switch (state) {
    case StartupState::BootDelay:
      return "boot_delay";
    case StartupState::WaitingForInputs:
      return "waiting_for_inputs";
    case StartupState::WaitingForOdomTf:
      return "waiting_for_odom_tf";
    case StartupState::StartingLocalization:
      return "starting_localization";
    case StartupState::WaitingForLocalizationReady:
      return "waiting_for_localization_ready";
    case StartupState::PublishingInitialPose:
      return "publishing_initial_pose";
    case StartupState::WaitingForMapTf:
      return "waiting_for_map_tf";
    case StartupState::StartingNavigation:
      return "starting_navigation";
    case StartupState::WaitingForNavigationReady:
      return "waiting_for_navigation_ready";
    case StartupState::Ready:
      return "ready";
    case StartupState::Error:
      return "error";
  }
  return "error";
}

const char * dock_state_name(DockState state)
{
  switch (state) {
    case DockState::Free:
      return "free";
    case DockState::Docking:
      return "docking";
    case DockState::Docked:
      return "docked";
    case DockState::Undocking:
      return "undocking";
  }
  return "free";
}

class Nav2TargetGatewayNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using GetState = lifecycle_msgs::srv::GetState;
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using SetString = hanmole_msgs::srv::SetString;
  using Toggle = nav2_msgs::srv::Toggle;
  using Trigger = std_srvs::srv::Trigger;
  using GoalStatus = action_msgs::msg::GoalStatus;
  using GoalStatusArray = action_msgs::msg::GoalStatusArray;

  Nav2TargetGatewayNode()
  : Node("nav2_target_gateway")
  {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    const std::string target_file = declare_parameter<std::string>("target_file", "");
    target_group_ = declare_parameter<std::string>("target_group", "");
    nav_mode_ = declare_parameter<std::string>("nav_mode", "ekf_odom");
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    dock_scan_topic_ = declare_parameter<std::string>("dock_scan_topic", "/scan/rear");
    filtered_odom_topic_ = declare_parameter<std::string>(
      "filtered_odom_topic", "/odometry/filtered");
    raw_odom_topic_ = declare_parameter<std::string>("raw_odom_topic", "/odom");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    localization_manager_service_ = declare_parameter<std::string>(
      "localization_manager_service", "/lifecycle_manager_localization/manage_nodes");
    navigation_manager_service_ = declare_parameter<std::string>(
      "navigation_manager_service", "/lifecycle_manager_navigation/manage_nodes");
    map_server_state_service_ = declare_parameter<std::string>(
      "map_server_state_service", "/map_server/get_state");
    amcl_state_service_ = declare_parameter<std::string>(
      "amcl_state_service", "/amcl/get_state");
    planner_server_state_service_ = declare_parameter<std::string>(
      "planner_server_state_service", "/planner_server/get_state");
    controller_server_state_service_ = declare_parameter<std::string>(
      "controller_server_state_service", "/controller_server/get_state");
    bt_navigator_state_service_ = declare_parameter<std::string>(
      "bt_navigator_state_service", "/bt_navigator/get_state");
    navigate_action_name_ = declare_parameter<std::string>(
      "navigate_action_name", "/navigate_to_pose");
    navigate_action_status_topic_ = declare_parameter<std::string>(
      "navigate_action_status_topic", action_status_topic_for_action(navigate_action_name_));
    navigate_service_name_ = declare_parameter<std::string>(
      "navigate_service_name", "/hanmole/agv/navigate_to_pose");
    cancel_service_name_ = declare_parameter<std::string>(
      "cancel_service_name", "/hanmole/agv/navigate_cancel");
    auto_dock_service_name_ = declare_parameter<std::string>(
      "auto_dock_service_name", "/hanmole/agv/auto_dock");
    undock_service_name_ = declare_parameter<std::string>(
      "undock_service_name", "/hanmole/agv/undock");
    set_initial_pose_service_name_ = declare_parameter<std::string>(
      "set_initial_pose_service_name", "/hanmole/agv/set_initial_pose");
    nav_status_topic_ = declare_parameter<std::string>("nav_status_topic", "/nav_status");
    nav_feedback_topic_ = declare_parameter<std::string>(
      "nav_feedback_topic", "/hanmole/agv/nav_feedback");
    target_state_topic_ = declare_parameter<std::string>(
      "target_state_topic", "/hanmole_navigation/target_state");
    dock_state_topic_ = declare_parameter<std::string>(
      "dock_state_topic", "/hanmole_navigation/dock_state");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    follow_path_action_name_ = declare_parameter<std::string>(
      "follow_path_action_name", "/follow_path");
    collision_monitor_toggle_service_ = declare_parameter<std::string>(
      "collision_monitor_toggle_service", "/collision_monitor/toggle");
    dock_staging_target_ = declare_parameter<std::string>("dock_staging_target", "DOCK_STAGING");
    dock_charge_target_ = declare_parameter<std::string>("dock_charge_target", "DOCK_CHARGE");
    charge_marker_topic_ = declare_parameter<std::string>("charge_marker_topic", "/charge_marker");
    charge_marker_enabled_ = declare_parameter<bool>("charge_marker_enabled", true);
    const int charge_marker_lifetime_ms = declare_parameter<int>(
      "charge_marker_lifetime_ms", 0);
    docking_controller_id_ = declare_parameter<std::string>("docking_controller_id", "DockMppi");
    docking_goal_checker_id_ = declare_parameter<std::string>(
      "docking_goal_checker_id", "dock_goal_checker");
    undock_goal_checker_id_ = declare_parameter<std::string>(
      "undock_goal_checker_id", "general_goal_checker");
    docking_progress_checker_id_ = declare_parameter<std::string>(
      "docking_progress_checker_id", "progress_checker");
    auto_dock_staging_behavior_tree_ = declare_parameter<std::string>(
      "auto_dock_staging_behavior_tree", "");
    const int dock_detection_timeout_ms = declare_parameter<int>(
      "dock_detection_timeout_ms", 3000);
    const int dock_detection_scan_freshness_ms = declare_parameter<int>(
      "dock_detection_scan_freshness_ms", 500);
    dock_detection_min_samples_ = declare_parameter<int>(
      "dock_detection_min_samples", 5);
    dock_detection_max_samples_ = declare_parameter<int>(
      "dock_detection_max_samples", 12);
    dock_detection_max_range_ = declare_parameter<double>(
      "dock_detection_max_range", 2.0);
    dock_scan_min_angle_ = declare_parameter<double>(
      "dock_scan_min_angle", -3.141592653589793);
    dock_scan_max_angle_ = declare_parameter<double>(
      "dock_scan_max_angle", 3.141592653589793);
    dock_interpolation_max_difference_ = declare_parameter<double>(
      "dock_interpolation_max_difference", 0.02);
    dock_mutation_min_jump_ = declare_parameter<double>(
      "dock_mutation_min_jump", 0.025);
    dock_mutation_max_jump_ = declare_parameter<double>(
      "dock_mutation_max_jump", 0.065);
    dock_feature_pair_max_angle_ = declare_parameter<double>(
      "dock_feature_pair_max_angle", 0.12217304763960307);
    dock_feature_outer_check_angle_ = declare_parameter<double>(
      "dock_feature_outer_check_angle", 0.08726646259971647);
    dock_feature_pair_max_range_difference_ = declare_parameter<double>(
      "dock_feature_pair_max_range_difference", 0.08);
    dock_feature_min_width_ = declare_parameter<double>(
      "dock_feature_min_width", 0.10);
    dock_feature_max_width_ = declare_parameter<double>(
      "dock_feature_max_width", 1.00);
    dock_flat_plate_detection_enabled_ = declare_parameter<bool>(
      "dock_flat_plate_detection_enabled", true);
    dock_flat_plate_center_angle_ = declare_parameter<double>(
      "dock_flat_plate_center_angle", 0.0);
    dock_flat_plate_half_angle_ = declare_parameter<double>(
      "dock_flat_plate_half_angle", 0.20);
    dock_flat_plate_min_points_ = declare_parameter<int>(
      "dock_flat_plate_min_points", 8);
    dock_flat_plate_max_rms_error_ = declare_parameter<double>(
      "dock_flat_plate_max_rms_error", 0.03);
    dock_flat_plate_lock_normal_to_center_angle_ = declare_parameter<bool>(
      "dock_flat_plate_lock_normal_to_center_angle", true);
    dock_detection_max_position_spread_ = declare_parameter<double>(
      "dock_detection_max_position_spread", 0.05);
    dock_detection_max_yaw_spread_ = declare_parameter<double>(
      "dock_detection_max_yaw_spread", 0.08726646259971647);
    dock_dynamic_max_position_correction_ = declare_parameter<double>(
      "dock_dynamic_max_position_correction", 0.30);
    dock_dynamic_max_yaw_correction_ = declare_parameter<double>(
      "dock_dynamic_max_yaw_correction", 0.2617993877991494);
    dock_nominal_hub_max_distance_ = declare_parameter<double>(
      "dock_nominal_hub_max_distance", 1.0);
    dock_base_to_hub_offset_min_ = declare_parameter<double>(
      "dock_base_to_hub_offset_min", 0.05);
    dock_base_to_hub_offset_max_ = declare_parameter<double>(
      "dock_base_to_hub_offset_max", 1.0);
    dock_base_to_hub_offset_ = declare_parameter<double>(
      "dock_base_to_hub_offset", -1.0);
    const int action_timeout_ms = declare_parameter<int>("action_timeout_ms", 3000);
    const int dock_motion_timeout_ms = declare_parameter<int>("dock_motion_timeout_ms", 30000);
    const int startup_delay_ms = declare_parameter<int>("startup_delay_ms", 3000);
    const int input_freshness_timeout_ms = declare_parameter<int>(
      "input_freshness_timeout_ms", 500);
    const int startup_poll_period_ms = declare_parameter<int>("startup_poll_period_ms", 5);
    const int tf_freshness_threshold_ms = declare_parameter<int>("tf_freshness_threshold_ms", 2);
    const int lifecycle_request_timeout_ms = declare_parameter<int>(
      "lifecycle_request_timeout_ms", 12000);
    stable_tf_success_count_ = declare_parameter<int>("stable_tf_success_count", 3);
    goal_xy_tolerance_ = declare_parameter<double>("goal_xy_tolerance", 0.10);
    goal_yaw_tolerance_ = declare_parameter<double>("goal_yaw_tolerance", 0.20);

    if (target_file.empty()) {
      throw std::runtime_error("target_file cannot be empty");
    }
    if (nav_mode_ != "ekf_odom" && nav_mode_ != "wheel_odom") {
      throw std::runtime_error("nav_mode must be ekf_odom or wheel_odom");
    }

    if (odom_topic_.empty()) {
      odom_topic_ = nav_mode_ == "wheel_odom" ? raw_odom_topic_ : filtered_odom_topic_;
    }
    if (stable_tf_success_count_ < 1) {
      stable_tf_success_count_ = 1;
    }
    if (dock_detection_min_samples_ < 1) {
      dock_detection_min_samples_ = 1;
    }
    if (dock_detection_max_samples_ < dock_detection_min_samples_) {
      dock_detection_max_samples_ = dock_detection_min_samples_;
    }
    if (dock_scan_min_angle_ >= dock_scan_max_angle_) {
      throw std::runtime_error("dock_scan_min_angle must be less than dock_scan_max_angle");
    }
    if (dock_mutation_min_jump_ <= 0.0 ||
      dock_mutation_min_jump_ >= dock_mutation_max_jump_)
    {
      throw std::runtime_error("dock mutation jump thresholds are invalid");
    }
    if (dock_detection_timeout_ms <= 0 || dock_detection_scan_freshness_ms <= 0) {
      throw std::runtime_error("dock detection timeouts must be positive");
    }
    if (!std::isfinite(dock_detection_max_range_) || dock_detection_max_range_ <= 0.0 ||
      !std::isfinite(dock_detection_max_position_spread_) ||
      dock_detection_max_position_spread_ <= 0.0 ||
      !std::isfinite(dock_detection_max_yaw_spread_) || dock_detection_max_yaw_spread_ <= 0.0 ||
      !std::isfinite(dock_dynamic_max_position_correction_) ||
      dock_dynamic_max_position_correction_ <= 0.0 ||
      !std::isfinite(dock_dynamic_max_yaw_correction_) ||
      dock_dynamic_max_yaw_correction_ <= 0.0 ||
      !std::isfinite(dock_nominal_hub_max_distance_) || dock_nominal_hub_max_distance_ <= 0.0)
    {
      throw std::runtime_error("dock detection limits must be finite and positive");
    }
    if (!std::isfinite(dock_feature_min_width_) || !std::isfinite(dock_feature_max_width_) ||
      dock_feature_min_width_ <= 0.0 || dock_feature_min_width_ >= dock_feature_max_width_)
    {
      throw std::runtime_error("dock feature width limits are invalid");
    }
    if (dock_flat_plate_min_points_ < 3) {
      dock_flat_plate_min_points_ = 3;
    }
    if (!std::isfinite(dock_flat_plate_center_angle_) ||
      !std::isfinite(dock_flat_plate_half_angle_) ||
      dock_flat_plate_half_angle_ <= 0.0 || dock_flat_plate_half_angle_ > M_PI ||
      !std::isfinite(dock_flat_plate_max_rms_error_) ||
      dock_flat_plate_max_rms_error_ <= 0.0)
    {
      throw std::runtime_error("dock flat plate detection parameters are invalid");
    }
    if (!std::isfinite(dock_base_to_hub_offset_min_) ||
      !std::isfinite(dock_base_to_hub_offset_max_) || dock_base_to_hub_offset_min_ < 0.0 ||
      dock_base_to_hub_offset_min_ >= dock_base_to_hub_offset_max_)
    {
      throw std::runtime_error("dock base-to-hub offset limits are invalid");
    }
    if (dock_base_to_hub_offset_ > 0.0 &&
      (dock_base_to_hub_offset_ < dock_base_to_hub_offset_min_ ||
      dock_base_to_hub_offset_ > dock_base_to_hub_offset_max_))
    {
      throw std::runtime_error("dock_base_to_hub_offset is outside the configured limits");
    }
    if (dock_base_to_hub_offset_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "dock_base_to_hub_offset is not calibrated; dynamic docking will only correct lateral error and yaw");
    }

    repository_.load_from_file(target_file);
    if (target_group_.empty()) {
      target_group_ = repository_.default_group();
    }
    if (!repository_.has_group(target_group_)) {
      throw std::runtime_error("target_group not found: " + target_group_);
    }

    action_wait_timeout_ = std::chrono::milliseconds(action_timeout_ms);
    dock_motion_timeout_ = std::chrono::milliseconds(dock_motion_timeout_ms);
    dock_detection_timeout_ = std::chrono::milliseconds(dock_detection_timeout_ms);
    dock_detection_scan_freshness_ =
      std::chrono::milliseconds(dock_detection_scan_freshness_ms);
    charge_marker_lifetime_ =
      std::chrono::milliseconds(std::max(0, charge_marker_lifetime_ms));
    startup_delay_ = std::chrono::milliseconds(startup_delay_ms);
    input_freshness_timeout_ = std::chrono::milliseconds(input_freshness_timeout_ms);
    startup_poll_period_ = std::chrono::milliseconds(startup_poll_period_ms);
    tf_freshness_threshold_ = std::chrono::milliseconds(tf_freshness_threshold_ms);
    lifecycle_request_timeout_ = std::chrono::milliseconds(lifecycle_request_timeout_ms);
    startup_deadline_ = std::chrono::steady_clock::now() + startup_delay_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      navigate_action_name_,
      callback_group_);
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
      this,
      follow_path_action_name_,
      callback_group_);
    collision_toggle_client_ = create_client<Toggle>(
      collision_monitor_toggle_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    localization_manager_client_ = create_client<ManageLifecycleNodes>(
      localization_manager_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    navigation_manager_client_ = create_client<ManageLifecycleNodes>(
      navigation_manager_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    map_server_state_client_ = create_client<GetState>(
      map_server_state_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    amcl_state_client_ = create_client<GetState>(
      amcl_state_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    planner_server_state_client_ = create_client<GetState>(
      planner_server_state_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    controller_server_state_client_ = create_client<GetState>(
      controller_server_state_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    bt_navigator_state_client_ = create_client<GetState>(
      bt_navigator_state_service_,
      rclcpp::ServicesQoS(),
      callback_group_);

    const auto state_qos = rclcpp::QoS(10).reliable().transient_local();
    nav_status_publisher_ = create_publisher<std_msgs::msg::String>(nav_status_topic_, state_qos);
    nav_feedback_publisher_ = create_publisher<NavigateToPose::Impl::FeedbackMessage>(
      nav_feedback_topic_,
      rclcpp::QoS(10).reliable());
    target_state_publisher_ = create_publisher<std_msgs::msg::String>(target_state_topic_, state_qos);
    dock_state_publisher_ = create_publisher<std_msgs::msg::String>(dock_state_topic_, state_qos);
    initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_,
      rclcpp::QoS(1).transient_local().reliable());
    charge_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      charge_marker_topic_,
      rclcpp::QoS(10).transient_local().reliable());

    status_publish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { republish_state(); },
      callback_group_);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr) {last_scan_time_ = now();},
      subscription_options);
    dock_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      dock_scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {handle_dock_scan(*message);},
      subscription_options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr) {
        last_odom_time_ = now();
      },
      subscription_options);
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
        last_map_time_ = now();
        map_received_ = true;
      },
      subscription_options);
    amcl_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_,
      10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        handle_amcl_pose(*message);
      },
      subscription_options);
    action_status_subscription_ = create_subscription<GoalStatusArray>(
      navigate_action_status_topic_,
      rclcpp_action::DefaultActionStatusQoS(),
      [this](const GoalStatusArray::SharedPtr message) {
        handle_action_status(*message);
      },
      subscription_options);

    navigate_service_ = create_service<SetString>(
      navigate_service_name_,
      [this](
        const std::shared_ptr<SetString::Request> request,
        std::shared_ptr<SetString::Response> response)
      {
        handle_navigate_request(request, response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);
    cancel_service_ = create_service<Trigger>(
      cancel_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        handle_cancel_request(response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);
    auto_dock_service_ = create_service<SetString>(
      auto_dock_service_name_,
      [this](
        const std::shared_ptr<SetString::Request> request,
        std::shared_ptr<SetString::Response> response)
      {
        handle_auto_dock_request(request, response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);
    undock_service_ = create_service<Trigger>(
      undock_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        handle_undock_request(response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);
    set_initial_pose_service_ = create_service<Trigger>(
      set_initial_pose_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        handle_set_initial_pose_request(response);
      },
      rclcpp::ServicesQoS(),
      callback_group_);

    startup_timer_ = create_wall_timer(
      startup_poll_period_,
      [this]() { tick_startup_state_machine(); },
      callback_group_);

    publish_state("idle", "idle");
    publish_dock_state();
    RCLCPP_INFO(
      get_logger(),
      "startup orchestration waiting for %s and %s in %s mode",
      scan_topic_.c_str(),
      odom_topic_.c_str(),
      nav_mode_.c_str());
  }

private:
  static std::string action_status_topic_for_action(const std::string & action_name)
  {
    if (action_name.empty()) {
      return "/navigate_to_pose/_action/status";
    }
    return action_name + "/_action/status";
  }

  void handle_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    update_current_pose_cache(message.pose.pose);
  }

  void update_current_pose_cache(const geometry_msgs::msg::PoseStamped & pose)
  {
    update_current_pose_cache(pose.pose);
  }

  void update_current_pose_cache(const geometry_msgs::msg::Pose & pose)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pose_xy_ = std::make_pair(pose.position.x, pose.position.y);
    current_pose_yaw_ = yaw_from_quaternion(pose.orientation);
  }

  void handle_action_status(const GoalStatusArray & message)
  {
    bool has_accepted = false;
    bool has_canceling = false;
    bool has_executing = false;
    bool has_succeeded = false;
    bool has_canceled = false;
    bool has_failed = false;

    for (const auto & status : message.status_list) {
      switch (status.status) {
        case GoalStatus::STATUS_ACCEPTED:
          has_accepted = true;
          break;
        case GoalStatus::STATUS_EXECUTING:
          has_executing = true;
          break;
        case GoalStatus::STATUS_CANCELING:
          has_canceling = true;
          break;
        case GoalStatus::STATUS_SUCCEEDED:
          has_succeeded = true;
          break;
        case GoalStatus::STATUS_CANCELED:
          has_canceled = true;
          break;
        case GoalStatus::STATUS_ABORTED:
          has_failed = true;
          break;
        default:
          break;
      }
    }

    if (has_executing) {
      mark_action_status_active();
      publish_action_state("navigating");
      return;
    }
    if (has_canceling) {
      mark_action_status_active();
      publish_action_state("canceling");
      return;
    }
    if (has_accepted) {
      mark_action_status_active();
      publish_action_state("waiting_for_nav2");
      return;
    }
    if (has_succeeded) {
      publish_action_terminal_state("succeeded");
      return;
    }
    if (has_failed) {
      publish_action_terminal_state("failed");
      return;
    }
    if (has_canceled) {
      publish_action_terminal_state("canceled");
      return;
    }

    mark_action_status_inactive();
    publish_action_state("idle");
  }

  void handle_navigate_request(
    const std::shared_ptr<SetString::Request> request,
    std::shared_ptr<SetString::Response> response)
  {
    std::optional<std::pair<double, double>> current_pose_xy;
    std::shared_ptr<GoalHandleNavigateToPose> active_goal_handle;
    std::string active_target_name;
    DockState dock_state = DockState::Free;
    bool switch_in_progress = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!navigation_ready_) {
        response->success = false;
        response->result = "navigation stack not ready";
        RCLCPP_WARN(
          get_logger(),
          "navigate request rejected: target=%s reason=%s",
          request->data.c_str(), response->result.c_str());
        return;
      }
      current_pose_xy = current_pose_xy_;
      active_goal_handle = active_goal_handle_;
      active_target_name = active_target_name_;
      dock_state = dock_state_;
      switch_in_progress = switching_to_pending_target_;
    }

    RCLCPP_INFO(
      get_logger(),
      "navigate request received: target=%s dock_state=%s active_goal=%s switch_in_progress=%s",
      request->data.c_str(), dock_state_name(dock_state),
      active_goal_handle ? "true" : "false",
      switch_in_progress ? "true" : "false");

    if (active_goal_handle) {
      const auto target_pose = repository_.resolve_target(target_group_, request->data, current_pose_xy);
      if (!target_pose.has_value()) {
        response->success = false;
        response->result =
          "target not found in group '" + target_group_ + "': " + request->data;
        RCLCPP_WARN(get_logger(), "navigate switch rejected: %s", response->result.c_str());
        return;
      }

      bool request_cancel = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pending_target_name_ = request->data;
        if (!switching_to_pending_target_) {
          switching_to_pending_target_ = true;
          request_cancel = true;
        }
      }

      if (request_cancel || !switch_in_progress) {
        action_client_->async_cancel_goal(active_goal_handle);
      }
      const std::string current_target = active_target_name.empty() ? "idle" : active_target_name;
      publish_state("canceling", current_target);
      RCLCPP_INFO(
        get_logger(),
        "navigate switch requested: current_target=%s next_target=%s request_cancel=%s",
        current_target.c_str(), request->data.c_str(), request_cancel ? "true" : "false");
      response->success = true;
      response->result = "switch requested: " + current_target + " -> " + request->data;
      RCLCPP_INFO(
        get_logger(),
        "business navigate response: target=%s success=%s result=%s",
        request->data.c_str(), response->success ? "true" : "false", response->result.c_str());
      return;
    }

    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->success = false;
      response->result = "navigation operation already in progress";
      RCLCPP_WARN(
        get_logger(),
        "business navigate rejected before dock handling: target=%s reason=%s",
        request->data.c_str(), response->result.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "business navigate dock check: target=%s dock_state=%s",
      request->data.c_str(), dock_state_name(dock_state));
    if (!leave_dock_if_needed(response->result)) {
      RCLCPP_WARN(
        get_logger(),
        "navigate request failed during dock handling: target=%s reason=%s",
        request->data.c_str(), response->result.c_str());
      response->success = false;
      RCLCPP_WARN(
        get_logger(),
        "business navigate response: target=%s success=%s result=%s",
        request->data.c_str(), response->success ? "true" : "false", response->result.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "navigate request proceeding after dock handling: target=%s",
      request->data.c_str());
    response->success = start_navigation_to_target(request->data, response->result);
    if (response->success) {
      RCLCPP_INFO(
        get_logger(),
        "business navigate response: target=%s success=%s result=%s",
        request->data.c_str(), response->success ? "true" : "false", response->result.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "business navigate response: target=%s success=%s result=%s",
        request->data.c_str(), response->success ? "true" : "false", response->result.c_str());
    }
  }

  bool start_navigation_to_target(const std::string & target_name, std::string & result_message)
  {
    RCLCPP_INFO(get_logger(), "start navigation to target: target=%s", target_name.c_str());

    std::optional<std::pair<double, double>> current_pose_xy;
    std::optional<double> current_pose_yaw;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!navigation_ready_) {
        result_message = "navigation stack not ready";
        RCLCPP_WARN(
          get_logger(),
          "navigation start rejected: target=%s reason=%s",
          target_name.c_str(), result_message.c_str());
        return false;
      }
      current_pose_xy = current_pose_xy_;
      current_pose_yaw = current_pose_yaw_;
    }

    const auto target_pose = repository_.resolve_target(target_group_, target_name, current_pose_xy);
    if (!target_pose.has_value()) {
      result_message =
        "target not found in group '" + target_group_ + "': " + target_name;
      RCLCPP_WARN(get_logger(), "navigation target lookup failed: %s", result_message.c_str());
      return false;
    }

    if (current_pose_xy.has_value() && current_pose_yaw.has_value()) {
      const double dx = target_pose->x - current_pose_xy->first;
      const double dy = target_pose->y - current_pose_xy->second;
      const double xy_error = std::hypot(dx, dy);
      const double yaw_error = std::abs(
        normalize_angle(yaw_from_target_pose(*target_pose) - *current_pose_yaw));
      if (xy_error <= goal_xy_tolerance_ && yaw_error <= goal_yaw_tolerance_) {
        publish_terminal_state("succeeded", target_name);
        result_message = "already at target: " + target_name;
        RCLCPP_INFO(
          get_logger(),
          "navigation target already reached: target=%s xy_error=%.3f yaw_error=%.3f",
          target_name.c_str(), xy_error, yaw_error);
        return true;
      }
    }

    publish_state("waiting_for_nav2", target_name);
    if (!action_client_->wait_for_action_server(action_wait_timeout_)) {
      publish_terminal_state("failed");
      result_message = "navigate_to_pose action server is unavailable";
      RCLCPP_WARN(
        get_logger(),
        "navigation start failed: target=%s reason=%s",
        target_name.c_str(), result_message.c_str());
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = repository_.frame();
    goal.pose.pose.position.x = target_pose->x;
    goal.pose.pose.position.y = target_pose->y;
    goal.pose.pose.orientation = quaternion_from_target_pose(*target_pose);

    auto options = typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.feedback_callback =
      [this](
        GoalHandleNavigateToPose::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        update_current_pose_cache(feedback->current_pose);
        NavigateToPose::Impl::FeedbackMessage feedback_message;
        feedback_message.goal_id.uuid = goal_handle->get_goal_id();
        feedback_message.feedback = *feedback;
        nav_feedback_publisher_->publish(feedback_message);
      };
    options.result_callback =
      [this, target_name](const GoalHandleNavigateToPose::WrappedResult & result) {
        handle_result(target_name, result);
      };

    RCLCPP_INFO(
      get_logger(),
      "sending navigate_to_pose goal: target=%s frame=%s x=%.3f y=%.3f yaw=%.3f",
      target_name.c_str(), goal.pose.header.frame_id.c_str(),
      goal.pose.pose.position.x, goal.pose.pose.position.y,
      yaw_from_quaternion(goal.pose.pose.orientation));
    auto future = action_client_->async_send_goal(goal, options);
    if (future.wait_for(action_wait_timeout_) != std::future_status::ready) {
      publish_terminal_state("failed");
      result_message = "timed out waiting for goal acceptance";
      RCLCPP_WARN(
        get_logger(),
        "navigation goal acceptance timed out: target=%s reason=%s",
        target_name.c_str(), result_message.c_str());
      return false;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
      publish_terminal_state("failed");
      result_message = "goal rejected by action server";
      RCLCPP_WARN(
        get_logger(),
        "navigation goal rejected: target=%s reason=%s",
        target_name.c_str(), result_message.c_str());
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_handle_ = goal_handle;
      active_target_name_ = target_name;
    }
    publish_state("navigating", target_name);
    RCLCPP_INFO(get_logger(), "navigate_to_pose goal accepted: target=%s", target_name.c_str());

    result_message = "goal accepted: " + target_name;
    return true;
  }

  void handle_cancel_request(std::shared_ptr<Trigger::Response> response)
  {
    std::shared_ptr<GoalHandleNavigateToPose> goal_handle;
    std::string active_target_name = "idle";
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_handle = active_goal_handle_;
      pending_target_name_.reset();
      switching_to_pending_target_ = false;
      if (!active_target_name_.empty()) {
        active_target_name = active_target_name_;
      }
    }

    if (!goal_handle) {
      publish_state("idle", "idle");
      response->success = true;
      response->message = "no active navigation goal";
      return;
    }

    action_client_->async_cancel_goal(goal_handle);
    publish_state("canceling", active_target_name);
    response->success = true;
    response->message = "cancel requested";
  }

  void handle_dock_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!dock_detection_enabled_.load(std::memory_order_relaxed)) {
      return;
    }
    dock_detection_received_scans_.fetch_add(1, std::memory_order_relaxed);

    const rclcpp::Time scan_stamp(scan.header.stamp);
    if (scan_stamp.nanoseconds() <= 0) {
      return;
    }
    const int64_t scan_age = (now() - scan_stamp).nanoseconds();
    const int64_t maximum_age = std::chrono::duration_cast<std::chrono::nanoseconds>(
      dock_detection_scan_freshness_).count();
    if (scan_age < -100000000LL || scan_age > maximum_age) {
      return;
    }
    dock_detection_fresh_scans_.fetch_add(1, std::memory_order_relaxed);

    const auto detection = detect_charging_hub(scan);
    if (detection.has_value()) {
      dock_detection_pattern_matches_.fetch_add(1, std::memory_order_relaxed);
      add_dock_detection_sample(*detection);
      publish_charge_hub_marker(*detection);
    }
  }

  bool get_dock_scan_window(
    const sensor_msgs::msg::LaserScan & scan,
    size_t & begin,
    size_t & end) const
  {
    if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
      std::abs(scan.angle_increment) <= std::numeric_limits<float>::epsilon())
    {
      return false;
    }

    begin = scan.ranges.size();
    end = 0;
    for (size_t index = 0; index < scan.ranges.size(); ++index) {
      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      if (angle >= dock_scan_min_angle_ && angle <= dock_scan_max_angle_) {
        begin = std::min(begin, index);
        end = index + 1;
      }
    }

    return begin < end && end - begin >= 12;
  }

  std::vector<float> filter_and_interpolate_dock_ranges(
    const sensor_msgs::msg::LaserScan & scan,
    size_t begin,
    size_t end) const
  {
    std::vector<float> ranges(end - begin, 0.0F);
    const double sensor_min = std::isfinite(scan.range_min) ? scan.range_min : 0.0;
    const double sensor_max =
      std::isfinite(scan.range_max) && scan.range_max > 0.0 ?
      scan.range_max : dock_detection_max_range_;
    const double maximum_range = std::min(sensor_max, dock_detection_max_range_);

    for (size_t index = begin; index < end; ++index) {
      const float range = scan.ranges[index];
      if (std::isfinite(range) && range >= sensor_min && range <= maximum_range && range > 0.0F) {
        ranges[index - begin] = range;
      }
    }

    constexpr float zero_tolerance = 1.0e-7F;
    for (size_t index = 1; index + 1 < ranges.size(); ++index) {
      if (ranges[index] > zero_tolerance || ranges[index - 1] <= zero_tolerance) {
        continue;
      }

      if (ranges[index + 1] > zero_tolerance &&
        std::abs(ranges[index + 1] - ranges[index - 1]) < dock_interpolation_max_difference_)
      {
        ranges[index] = (ranges[index - 1] + ranges[index + 1]) * 0.5F;
        continue;
      }

      if (index + 2 < ranges.size() && ranges[index + 1] <= zero_tolerance &&
        ranges[index + 2] > zero_tolerance &&
        std::abs(ranges[index + 2] - ranges[index - 1]) < dock_interpolation_max_difference_)
      {
        const float interpolated = (ranges[index - 1] + ranges[index + 2]) * 0.5F;
        ranges[index] = interpolated;
        ranges[index + 1] = interpolated;
        ++index;
      }
    }

    return ranges;
  }

  std::vector<size_t> find_dock_mutation_points(const std::vector<float> & ranges) const
  {
    std::vector<size_t> mutation_indices;
    for (size_t index = 0; index + 1 < ranges.size(); ++index) {
      if (ranges[index] <= 0.0F || ranges[index + 1] <= 0.0F) {
        continue;
      }

      const double jump = std::abs(ranges[index] - ranges[index + 1]);
      if (jump > dock_mutation_min_jump_ && jump < dock_mutation_max_jump_) {
        mutation_indices.push_back(index);
      }
    }
    return mutation_indices;
  }

  std::optional<std::array<size_t, 4>> match_charging_hub_pattern(
    const std::vector<float> & ranges,
    const std::vector<size_t> & mutation_indices,
    double angle_increment) const
  {
    if (mutation_indices.size() < 4 || angle_increment <= 0.0) {
      return std::nullopt;
    }

    const size_t maximum_pair_gap = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(dock_feature_pair_max_angle_ / angle_increment)));
    const size_t outside_offset = std::max<size_t>(
      1, static_cast<size_t>(std::lround(dock_feature_outer_check_angle_ / angle_increment)));

    std::optional<std::array<size_t, 4>> best_match;
    double best_score = std::numeric_limits<double>::max();
    for (size_t start = 0; start + 3 < mutation_indices.size(); ++start) {
      const std::array<size_t, 4> points{
        mutation_indices[start], mutation_indices[start + 1],
        mutation_indices[start + 2], mutation_indices[start + 3]};
      const size_t p0 = points[0];
      const size_t p1 = points[1];
      const size_t p2 = points[2];
      const size_t p3 = points[3];

      if (p0 < outside_offset || p3 + outside_offset >= ranges.size()) {
        continue;
      }
      if (!(ranges[p0] < ranges[p1] && ranges[p2] < ranges[p3])) {
        continue;
      }
      if (!(ranges[p0] < ranges[p0 - outside_offset] &&
        ranges[p3] < ranges[p3 + outside_offset]))
      {
        continue;
      }
      if (p1 - p0 > maximum_pair_gap || p3 - p2 > maximum_pair_gap) {
        continue;
      }
      if (p2 - p1 <= maximum_pair_gap) {
        continue;
      }

      const double right_pair_error = std::abs(ranges[p0] - ranges[p1]);
      const double left_pair_error = std::abs(ranges[p2] - ranges[p3]);
      if (right_pair_error >= dock_feature_pair_max_range_difference_ ||
        left_pair_error >= dock_feature_pair_max_range_difference_)
      {
        continue;
      }

      const double angular_balance =
        std::abs(static_cast<double>(p1 - p0) - static_cast<double>(p3 - p2)) *
        angle_increment;
      const double score = right_pair_error + left_pair_error + angular_balance;
      if (score < best_score) {
        best_score = score;
        best_match = points;
      }
    }

    return best_match;
  }

  geometry_msgs::msg::Point dock_laser_point(
    const sensor_msgs::msg::LaserScan & scan,
    size_t scan_index,
    float range) const
  {
    const double angle =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(scan_index) * static_cast<double>(scan.angle_increment);
    geometry_msgs::msg::Point point;
    point.x = static_cast<double>(range) * std::cos(angle);
    point.y = static_cast<double>(range) * std::sin(angle);
    return point;
  }

  std::optional<ChargingHubDetection> calculate_charging_hub_detection(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<float> & ranges,
    size_t scan_begin,
    const std::array<size_t, 4> & feature_indices) const
  {
    const size_t right_index = feature_indices[0];
    const size_t left_index = feature_indices[3];
    const auto right = dock_laser_point(scan, scan_begin + right_index, ranges[right_index]);
    const auto left = dock_laser_point(scan, scan_begin + left_index, ranges[left_index]);

    const double tangent_x = left.x - right.x;
    const double tangent_y = left.y - right.y;
    const double feature_width = std::hypot(tangent_x, tangent_y);
    if (feature_width < dock_feature_min_width_ || feature_width > dock_feature_max_width_) {
      return std::nullopt;
    }

    const double center_x = (left.x + right.x) * 0.5;
    const double center_y = (left.y + right.y) * 0.5;
    double normal_x = -tangent_y / feature_width;
    double normal_y = tangent_x / feature_width;
    if (normal_x * -center_x + normal_y * -center_y < 0.0) {
      normal_x = -normal_x;
      normal_y = -normal_y;
    }

    ChargingHubDetection detection;
    detection.dock_pose.header = scan.header;
    detection.dock_pose.pose.position.x = center_x;
    detection.dock_pose.pose.position.y = center_y;
    detection.dock_pose.pose.orientation = quaternion_from_yaw(std::atan2(normal_y, normal_x));
    detection.left_feature = left;
    detection.right_feature = right;

    const double pair_error =
      std::abs(ranges[feature_indices[0]] - ranges[feature_indices[1]]) +
      std::abs(ranges[feature_indices[2]] - ranges[feature_indices[3]]);
    detection.confidence = std::clamp(1.0 - pair_error / 0.16, 0.0, 1.0);
    return detection;
  }

  std::optional<ChargingHubDetection> detect_flat_charging_face(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<float> & ranges,
    size_t scan_begin) const
  {
    if (!dock_flat_plate_detection_enabled_ || ranges.empty()) {
      return std::nullopt;
    }

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(ranges.size());
    for (size_t index = 0; index < ranges.size(); ++index) {
      const float range = ranges[index];
      if (range <= 0.0F) {
        continue;
      }

      const size_t scan_index = scan_begin + index;
      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(scan_index) * static_cast<double>(scan.angle_increment);
      if (std::abs(normalize_angle(angle - dock_flat_plate_center_angle_)) >
        dock_flat_plate_half_angle_)
      {
        continue;
      }

      points.push_back(dock_laser_point(scan, scan_index, range));
    }

    if (points.size() < static_cast<size_t>(dock_flat_plate_min_points_)) {
      return std::nullopt;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & point : points) {
      center_x += point.x;
      center_y += point.y;
    }
    center_x /= static_cast<double>(points.size());
    center_y /= static_cast<double>(points.size());

    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    for (const auto & point : points) {
      const double dx = point.x - center_x;
      const double dy = point.y - center_y;
      covariance_xx += dx * dx;
      covariance_xy += dx * dy;
      covariance_yy += dy * dy;
    }
    covariance_xx /= static_cast<double>(points.size());
    covariance_xy /= static_cast<double>(points.size());
    covariance_yy /= static_cast<double>(points.size());

    if (covariance_xx + covariance_yy <= std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }

    const double tangent_yaw = 0.5 * std::atan2(
      2.0 * covariance_xy, covariance_xx - covariance_yy);
    const double tangent_x = std::cos(tangent_yaw);
    const double tangent_y = std::sin(tangent_yaw);
    double normal_x = -tangent_y;
    double normal_y = tangent_x;
    if (normal_x * -center_x + normal_y * -center_y < 0.0) {
      normal_x = -normal_x;
      normal_y = -normal_y;
    }

    double minimum_projection = std::numeric_limits<double>::max();
    double maximum_projection = std::numeric_limits<double>::lowest();
    double squared_normal_error_sum = 0.0;
    for (const auto & point : points) {
      const double dx = point.x - center_x;
      const double dy = point.y - center_y;
      const double tangent_projection = dx * tangent_x + dy * tangent_y;
      minimum_projection = std::min(minimum_projection, tangent_projection);
      maximum_projection = std::max(maximum_projection, tangent_projection);
      const double normal_error = dx * normal_x + dy * normal_y;
      squared_normal_error_sum += normal_error * normal_error;
    }

    const double feature_width = maximum_projection - minimum_projection;
    if (feature_width < dock_feature_min_width_ || feature_width > dock_feature_max_width_) {
      return std::nullopt;
    }

    const double rms_error = std::sqrt(
      squared_normal_error_sum / static_cast<double>(points.size()));
    if (rms_error > dock_flat_plate_max_rms_error_) {
      return std::nullopt;
    }

    double pose_center_x = center_x;
    double pose_center_y = center_y;
    const double ray_x = std::cos(dock_flat_plate_center_angle_);
    const double ray_y = std::sin(dock_flat_plate_center_angle_);
    const double ray_cross_tangent = ray_x * tangent_y - ray_y * tangent_x;
    if (std::abs(ray_cross_tangent) > std::numeric_limits<double>::epsilon()) {
      const double ray_distance =
        (center_x * tangent_y - center_y * tangent_x) / ray_cross_tangent;
      const double intersection_x = ray_distance * ray_x;
      const double intersection_y = ray_distance * ray_y;
      const double intersection_projection =
        (intersection_x - center_x) * tangent_x + (intersection_y - center_y) * tangent_y;
      constexpr double projection_margin = 0.05;
      if (std::isfinite(ray_distance) && ray_distance > 0.0 &&
        ray_distance <= dock_detection_max_range_ &&
        intersection_projection >= minimum_projection - projection_margin &&
        intersection_projection <= maximum_projection + projection_margin)
      {
        pose_center_x = intersection_x;
        pose_center_y = intersection_y;
      }
    }
    if (dock_flat_plate_lock_normal_to_center_angle_) {
      normal_x = -ray_x;
      normal_y = -ray_y;
    } else if (normal_x * -pose_center_x + normal_y * -pose_center_y < 0.0) {
      normal_x = -normal_x;
      normal_y = -normal_y;
    }

    geometry_msgs::msg::Point right;
    right.x = center_x + tangent_x * minimum_projection;
    right.y = center_y + tangent_y * minimum_projection;
    geometry_msgs::msg::Point left;
    left.x = center_x + tangent_x * maximum_projection;
    left.y = center_y + tangent_y * maximum_projection;

    ChargingHubDetection detection;
    detection.dock_pose.header = scan.header;
    detection.dock_pose.pose.position.x = pose_center_x;
    detection.dock_pose.pose.position.y = pose_center_y;
    detection.dock_pose.pose.orientation = quaternion_from_yaw(std::atan2(normal_y, normal_x));
    detection.left_feature = left;
    detection.right_feature = right;
    detection.confidence = std::clamp(
      1.0 - rms_error / dock_flat_plate_max_rms_error_, 0.0, 1.0);
    return detection;
  }

  std::optional<ChargingHubDetection> detect_charging_hub(
    const sensor_msgs::msg::LaserScan & scan) const
  {
    if (scan.header.frame_id.empty()) {
      return std::nullopt;
    }

    size_t begin = 0;
    size_t end = 0;
    if (!get_dock_scan_window(scan, begin, end)) {
      return std::nullopt;
    }

    const auto ranges = filter_and_interpolate_dock_ranges(scan, begin, end);
    const auto mutation_indices = find_dock_mutation_points(ranges);
    const auto match = match_charging_hub_pattern(
      ranges, mutation_indices, std::abs(static_cast<double>(scan.angle_increment)));
    if (match.has_value()) {
      const auto pattern_detection = calculate_charging_hub_detection(scan, ranges, begin, *match);
      if (pattern_detection.has_value()) {
        return pattern_detection;
      }
    }

    return detect_flat_charging_face(scan, ranges, begin);
  }

  void start_dock_detection()
  {
    clear_charge_markers();

    std::lock_guard<std::mutex> lock(dock_detection_mutex_);
    dock_detection_samples_.clear();
    dock_detection_received_scans_.store(0, std::memory_order_relaxed);
    dock_detection_fresh_scans_.store(0, std::memory_order_relaxed);
    dock_detection_pattern_matches_.store(0, std::memory_order_relaxed);
    ++dock_detection_sequence_;
    dock_detection_enabled_.store(true, std::memory_order_relaxed);
  }

  void stop_dock_detection()
  {
    dock_detection_enabled_.store(false, std::memory_order_relaxed);
    dock_detection_cv_.notify_all();
  }

  void apply_charge_marker_lifetime(visualization_msgs::msg::Marker & marker) const
  {
    const int64_t lifetime_ms = charge_marker_lifetime_.count();
    if (lifetime_ms <= 0) {
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 0;
      return;
    }

    marker.lifetime.sec = static_cast<int32_t>(lifetime_ms / 1000);
    marker.lifetime.nanosec =
      static_cast<uint32_t>((lifetime_ms % 1000) * 1000000LL);
  }

  void clear_charge_markers()
  {
    if (!charge_marker_enabled_ || !charge_marker_publisher_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.action = visualization_msgs::msg::Marker::DELETE;

    marker.ns = "charge_hub_points";
    marker.id = 0;
    charge_marker_publisher_->publish(marker);

    marker.ns = "dynamic_charge_pose";
    marker.id = 1;
    charge_marker_publisher_->publish(marker);
  }

  void publish_charge_hub_marker(const ChargingHubDetection & detection)
  {
    if (!charge_marker_enabled_ || !charge_marker_publisher_) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header = detection.dock_pose.header;
    marker.ns = "charge_hub_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.color.r = 154.0F / 255.0F;
    marker.color.g = 50.0F / 255.0F;
    marker.color.b = 205.0F / 255.0F;
    marker.color.a = 1.0F;
    apply_charge_marker_lifetime(marker);

    marker.points.reserve(3);
    marker.points.push_back(detection.left_feature);
    marker.points.push_back(detection.right_feature);
    marker.points.push_back(detection.dock_pose.pose.position);

    charge_marker_publisher_->publish(marker);
  }

  void publish_dynamic_charge_pose_marker(
    const geometry_msgs::msg::PoseStamped & charge_pose)
  {
    if (!charge_marker_enabled_ || !charge_marker_publisher_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = charge_pose.header;
    if (rclcpp::Time(marker.header.stamp).nanoseconds() <= 0) {
      marker.header.stamp = now();
    }
    marker.ns = "dynamic_charge_pose";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = charge_pose.pose;
    marker.scale.x = 0.35;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.r = 50.0F / 255.0F;
    marker.color.g = 200.0F / 255.0F;
    marker.color.b = 150.0F / 255.0F;
    marker.color.a = 1.0F;
    apply_charge_marker_lifetime(marker);

    charge_marker_publisher_->publish(marker);
  }

  void add_dock_detection_sample(const ChargingHubDetection & detection)
  {
    {
      std::lock_guard<std::mutex> lock(dock_detection_mutex_);
      if (!dock_detection_enabled_.load(std::memory_order_relaxed)) {
        return;
      }
      if (!dock_detection_samples_.empty() &&
        dock_detection_samples_.back().dock_pose.header.frame_id !=
        detection.dock_pose.header.frame_id)
      {
        dock_detection_samples_.clear();
      }
      if (!dock_detection_samples_.empty()) {
        const rclcpp::Time latest_stamp(
          dock_detection_samples_.back().dock_pose.header.stamp);
        const rclcpp::Time incoming_stamp(detection.dock_pose.header.stamp);
        if (incoming_stamp <= latest_stamp) {
          return;
        }
      }
      dock_detection_samples_.push_back(detection);
      while (dock_detection_samples_.size() >
        static_cast<size_t>(dock_detection_max_samples_))
      {
        dock_detection_samples_.pop_front();
      }
      ++dock_detection_sequence_;
    }
    dock_detection_cv_.notify_all();
  }

  bool stable_dock_pose_locked(geometry_msgs::msg::PoseStamped & pose) const
  {
    if (dock_detection_samples_.size() < static_cast<size_t>(dock_detection_min_samples_)) {
      return false;
    }

    std::vector<double> x_values;
    std::vector<double> y_values;
    x_values.reserve(dock_detection_samples_.size());
    y_values.reserve(dock_detection_samples_.size());
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & sample : dock_detection_samples_) {
      x_values.push_back(sample.dock_pose.pose.position.x);
      y_values.push_back(sample.dock_pose.pose.position.y);
      const double yaw = yaw_from_quaternion(sample.dock_pose.pose.orientation);
      sin_sum += std::sin(yaw);
      cos_sum += std::cos(yaw);
    }

    const double x = median(x_values);
    const double y = median(y_values);
    const double yaw = std::atan2(sin_sum, cos_sum);
    for (const auto & sample : dock_detection_samples_) {
      const double position_spread = std::hypot(
        sample.dock_pose.pose.position.x - x,
        sample.dock_pose.pose.position.y - y);
      const double yaw_spread = std::abs(normalize_angle(
        yaw_from_quaternion(sample.dock_pose.pose.orientation) - yaw));
      if (position_spread > dock_detection_max_position_spread_ ||
        yaw_spread > dock_detection_max_yaw_spread_)
      {
        return false;
      }
    }

    pose = dock_detection_samples_.back().dock_pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = quaternion_from_yaw(yaw);
    return true;
  }

  bool wait_for_stable_dock_pose(
    geometry_msgs::msg::PoseStamped & pose,
    std::string & result_message)
  {
    const auto deadline = std::chrono::steady_clock::now() + dock_detection_timeout_;
    std::unique_lock<std::mutex> lock(dock_detection_mutex_);

    while (dock_detection_enabled_.load(std::memory_order_relaxed)) {
      if (stable_dock_pose_locked(pose)) {
        result_message =
          "stable charging hub detected with " +
          std::to_string(dock_detection_samples_.size()) + " samples";
        return true;
      }

      if (!dock_detection_cv_.wait_until(
          lock, deadline,
          [this, sequence = dock_detection_sequence_]() {
            return dock_detection_sequence_ != sequence ||
                   !dock_detection_enabled_.load(std::memory_order_relaxed);
          }))
      {
        break;
      }
    }

    result_message =
      "charging hub detection timed out; topic=" + dock_scan_topic_ +
      " received_scans=" +
      std::to_string(dock_detection_received_scans_.load(std::memory_order_relaxed)) +
      " fresh_scans=" +
      std::to_string(dock_detection_fresh_scans_.load(std::memory_order_relaxed)) +
      " pattern_matches=" +
      std::to_string(dock_detection_pattern_matches_.load(std::memory_order_relaxed)) +
      " valid_samples=" + std::to_string(dock_detection_samples_.size());
    return false;
  }

  bool transform_dock_pose_to_map(
    const geometry_msgs::msg::PoseStamped & dock_pose,
    geometry_msgs::msg::PoseStamped & map_pose,
    std::string & result_message)
  {
    if (dock_pose.header.frame_id == map_frame_) {
      map_pose = dock_pose;
      return true;
    }

    const auto apply_transform = [this, &dock_pose, &map_pose](
      const geometry_msgs::msg::TransformStamped & transform) {
        const double transform_yaw = yaw_from_quaternion(transform.transform.rotation);
        const double cos_yaw = std::cos(transform_yaw);
        const double sin_yaw = std::sin(transform_yaw);
        const double source_x = dock_pose.pose.position.x;
        const double source_y = dock_pose.pose.position.y;

        map_pose = dock_pose;
        map_pose.header.frame_id = map_frame_;
        map_pose.header.stamp = transform.header.stamp;
        map_pose.pose.position.x =
          transform.transform.translation.x + cos_yaw * source_x - sin_yaw * source_y;
        map_pose.pose.position.y =
          transform.transform.translation.y + sin_yaw * source_x + cos_yaw * source_y;
        map_pose.pose.orientation = quaternion_from_yaw(normalize_angle(
          transform_yaw + yaw_from_quaternion(dock_pose.pose.orientation)));
      };

    std::string stamped_lookup_error;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, dock_pose.header.frame_id, rclcpp::Time(dock_pose.header.stamp),
        rclcpp::Duration::from_nanoseconds(200000000));
      apply_transform(transform);
      return true;
    } catch (const tf2::TransformException & exception) {
      stamped_lookup_error = exception.what();
    }

    try {
      const auto latest_transform = tf_buffer_->lookupTransform(
        map_frame_, dock_pose.header.frame_id, rclcpp::Time(0),
        rclcpp::Duration::from_nanoseconds(200000000));
      RCLCPP_WARN(
        get_logger(),
        "charging hub pose timestamped transform failed, using latest transform: %s",
        stamped_lookup_error.c_str());
      apply_transform(latest_transform);
      return true;
    } catch (const tf2::TransformException & exception) {
      result_message =
        "failed to transform charging hub pose from " + dock_pose.header.frame_id +
        " to " + map_frame_ + ": " + stamped_lookup_error +
        "; latest transform fallback failed: " + exception.what();
      return false;
    }
  }

  bool make_dynamic_charge_pose(
    const geometry_msgs::msg::PoseStamped & dock_pose,
    const geometry_msgs::msg::PoseStamped & nominal_charge_pose,
    geometry_msgs::msg::PoseStamped & dynamic_charge_pose,
    std::string & result_message) const
  {
    double outward_yaw = yaw_from_quaternion(dock_pose.pose.orientation);
    double normal_x = std::cos(outward_yaw);
    double normal_y = std::sin(outward_yaw);
    const double nominal_dx =
      nominal_charge_pose.pose.position.x - dock_pose.pose.position.x;
    const double nominal_dy =
      nominal_charge_pose.pose.position.y - dock_pose.pose.position.y;
    const double nominal_hub_distance = std::hypot(nominal_dx, nominal_dy);
    if (nominal_hub_distance > dock_nominal_hub_max_distance_) {
      result_message =
        "detected charging hub is too far from nominal charge target: " +
        std::to_string(nominal_hub_distance);
      return false;
    }
    double nominal_offset = nominal_dx * normal_x + nominal_dy * normal_y;
    if (nominal_offset < 0.0) {
      outward_yaw = normalize_angle(outward_yaw + M_PI);
      normal_x = -normal_x;
      normal_y = -normal_y;
      nominal_offset = -nominal_offset;
    }

    const double base_to_hub_offset =
      dock_base_to_hub_offset_ > 0.0 ? dock_base_to_hub_offset_ : nominal_offset;
    if (!std::isfinite(base_to_hub_offset) ||
      base_to_hub_offset < dock_base_to_hub_offset_min_ ||
      base_to_hub_offset > dock_base_to_hub_offset_max_)
    {
      result_message =
        "charging hub offset is outside configured limits: " +
        std::to_string(base_to_hub_offset);
      return false;
    }

    dynamic_charge_pose = nominal_charge_pose;
    dynamic_charge_pose.header.stamp = now();
    dynamic_charge_pose.pose.position.x =
      dock_pose.pose.position.x + normal_x * base_to_hub_offset;
    dynamic_charge_pose.pose.position.y =
      dock_pose.pose.position.y + normal_y * base_to_hub_offset;

    const double nominal_yaw = yaw_from_quaternion(nominal_charge_pose.pose.orientation);
    const double outward_error = std::abs(normalize_angle(outward_yaw - nominal_yaw));
    const double inward_yaw = normalize_angle(outward_yaw + M_PI);
    const double inward_error = std::abs(normalize_angle(inward_yaw - nominal_yaw));
    const double target_yaw = outward_error <= inward_error ? outward_yaw : inward_yaw;
    dynamic_charge_pose.pose.orientation = quaternion_from_yaw(target_yaw);

    const double position_correction = std::hypot(
      dynamic_charge_pose.pose.position.x - nominal_charge_pose.pose.position.x,
      dynamic_charge_pose.pose.position.y - nominal_charge_pose.pose.position.y);
    const double yaw_correction = std::abs(normalize_angle(target_yaw - nominal_yaw));
    if (position_correction > dock_dynamic_max_position_correction_) {
      result_message =
        "detected charging pose exceeds position correction limit: " +
        std::to_string(position_correction);
      return false;
    }
    if (yaw_correction > dock_dynamic_max_yaw_correction_) {
      result_message =
        "detected charging pose exceeds yaw correction limit: " +
        std::to_string(yaw_correction);
      return false;
    }

    result_message =
      "dynamic charging pose accepted: position_correction=" +
      std::to_string(position_correction) +
      " yaw_correction=" + std::to_string(yaw_correction);
    return true;
  }

  void handle_auto_dock_request(
    const std::shared_ptr<SetString::Request> request,
    std::shared_ptr<SetString::Response> response)
  {
    (void)request;
    RCLCPP_INFO(
      get_logger(),
      "auto_dock request received: staging_target=%s charge_target=%s",
      dock_staging_target_.c_str(), dock_charge_target_.c_str());
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->success = false;
      response->result = "navigation operation already in progress";
      RCLCPP_WARN(get_logger(), "auto_dock request rejected: reason=%s", response->result.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      RCLCPP_INFO(
        get_logger(),
        "auto_dock precheck: navigation_ready=%s active_goal=%s dock_state=%s",
        navigation_ready_ ? "true" : "false",
        active_goal_handle_ ? "true" : "false",
        dock_state_name(dock_state_));
      if (!navigation_ready_) {
        response->success = false;
        response->result = "navigation stack not ready";
        RCLCPP_WARN(get_logger(), "auto_dock request rejected: reason=%s", response->result.c_str());
        return;
      }
      if (active_goal_handle_) {
        response->success = false;
        response->result = "cannot dock while a navigation goal is active";
        RCLCPP_WARN(get_logger(), "auto_dock request rejected: reason=%s", response->result.c_str());
        return;
      }
      if (dock_state_ == DockState::Docked) {
        response->success = true;
        response->result = "already docked";
        RCLCPP_INFO(
          get_logger(),
          "auto_dock request skipped: dock_state=%s result=%s",
          dock_state_name(dock_state_), response->result.c_str());
        return;
      }
    }

    set_dock_state(DockState::Docking);
    RCLCPP_INFO(
      get_logger(),
      "auto_dock starting staging navigation: target=%s",
      dock_staging_target_.c_str());

    publish_state("docking_to_staging", dock_staging_target_);
    if (!run_navigation_to_target_blocking(
        dock_staging_target_, response->result, auto_dock_staging_behavior_tree_))
    {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock staging navigation failed: target=%s reason=%s",
        dock_staging_target_.c_str(), failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }

    geometry_msgs::msg::PoseStamped nominal_charge_pose;
    if (!resolve_target_pose(dock_charge_target_, nominal_charge_pose, response->result)) {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock charge target lookup failed: target=%s reason=%s",
        dock_charge_target_.c_str(), failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }

    publish_state("docking_recognition", dock_charge_target_);
    RCLCPP_INFO(
      get_logger(), "auto_dock charging hub recognition started: topic=%s",
      dock_scan_topic_.c_str());
    start_dock_detection();
    geometry_msgs::msg::PoseStamped detected_dock_pose;
    const bool detected = wait_for_stable_dock_pose(detected_dock_pose, response->result);
    stop_dock_detection();
    if (!detected) {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock charging hub recognition failed: reason=%s",
        failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }

    geometry_msgs::msg::PoseStamped map_dock_pose;
    if (!transform_dock_pose_to_map(detected_dock_pose, map_dock_pose, response->result)) {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock charging hub transform failed: reason=%s",
        failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }

    geometry_msgs::msg::PoseStamped charge_pose;
    if (!make_dynamic_charge_pose(
        map_dock_pose, nominal_charge_pose, charge_pose, response->result))
    {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock dynamic charge pose rejected: reason=%s",
        failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }
    publish_dynamic_charge_pose_marker(charge_pose);

    RCLCPP_INFO(
      get_logger(),
      "auto_dock charging hub recognized: hub_x=%.3f hub_y=%.3f hub_yaw=%.3f "
      "target_x=%.3f target_y=%.3f target_yaw=%.3f",
      map_dock_pose.pose.position.x, map_dock_pose.pose.position.y,
      yaw_from_quaternion(map_dock_pose.pose.orientation),
      charge_pose.pose.position.x, charge_pose.pose.position.y,
      yaw_from_quaternion(charge_pose.pose.orientation));

    publish_state("docking_approach", dock_charge_target_);
    if (!follow_path_to_pose_with_collision_monitor_disabled(
        charge_pose, response->result, true, docking_goal_checker_id_,
        docking_progress_checker_id_, "dock_final_approach"))
    {
      const std::string failure_message = response->result;
      RCLCPP_WARN(
        get_logger(),
        "auto_dock final approach failed: target=%s reason=%s",
        dock_charge_target_.c_str(), failure_message.c_str());
      set_dock_state(DockState::Free);
      response->result = failure_message;
      response->success = false;
      return;
    }

    set_dock_state(DockState::Docked);
    publish_state("docked", dock_charge_target_);
    RCLCPP_INFO(get_logger(), "auto_dock completed: charge_target=%s", dock_charge_target_.c_str());
    response->success = true;
    response->result = "docked at target: " + dock_charge_target_;
  }

  void handle_undock_request(std::shared_ptr<Trigger::Response> response)
  {
    DockState dock_state = DockState::Free;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dock_state = dock_state_;
    }
    RCLCPP_INFO(
      get_logger(),
      "undock request received: dock_state=%s staging_target=%s",
      dock_state_name(dock_state), dock_staging_target_.c_str());

    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->success = false;
      response->message = "navigation operation already in progress";
      RCLCPP_WARN(get_logger(), "undock request rejected: reason=%s", response->message.c_str());
      return;
    }

    response->success = leave_dock_if_needed(response->message);
    if (response->success && response->message.empty()) {
      response->message = "robot is not docked";
    }

    if (response->success) {
      RCLCPP_INFO(get_logger(), "undock request completed: %s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "undock request failed: %s", response->message.c_str());
    }
  }

  void handle_set_initial_pose_request(std::shared_ptr<Trigger::Response> response)
  {
    if (!publish_initial_pose_internal()) {
      response->success = false;
      response->message = "initial_pose_target not found";
      return;
    }

    response->success = true;
    response->message = "published initial pose: " + repository_.initial_pose_target();
  }

  bool leave_dock_if_needed(std::string & result_message)
  {
    DockState dock_state = DockState::Free;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dock_state = dock_state_;
      if (dock_state != DockState::Docked) {
        RCLCPP_INFO(
          get_logger(),
          "business navigate undock decision: skip because dock_state=%s is not docked",
          dock_state_name(dock_state));
        return true;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "business navigate undock decision: execute undock because dock_state=docked staging_target=%s",
      dock_staging_target_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "undock starting: staging_target=%s",
      dock_staging_target_.c_str());
    set_dock_state(DockState::Undocking);
    publish_state("undocking_to_staging", dock_staging_target_);

    geometry_msgs::msg::PoseStamped staging_pose;
    if (!resolve_target_pose(dock_staging_target_, staging_pose, result_message)) {
      RCLCPP_WARN(
        get_logger(),
        "undock staging target lookup failed: target=%s reason=%s",
        dock_staging_target_.c_str(), result_message.c_str());
      set_dock_state(DockState::Docked);
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "undock staging target resolved: target=%s frame=%s x=%.3f y=%.3f yaw=%.3f",
      dock_staging_target_.c_str(), staging_pose.header.frame_id.c_str(),
      staging_pose.pose.position.x, staging_pose.pose.position.y,
      yaw_from_quaternion(staging_pose.pose.orientation));
    if (!follow_path_to_pose_with_collision_monitor_disabled(
        staging_pose, result_message, true, undock_goal_checker_id_,
        docking_progress_checker_id_, "undock_staging"))
    {
      RCLCPP_WARN(
        get_logger(),
        "undock follow_path failed: staging_target=%s reason=%s",
        dock_staging_target_.c_str(), result_message.c_str());
      set_dock_state(DockState::Docked);
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "undock reached staging target; re-enabling collision monitor");
    if (!set_collision_monitor_enabled(true, result_message)) {
      RCLCPP_WARN(
        get_logger(),
        "undock reached staging but collision monitor re-enable failed: reason=%s",
        result_message.c_str());
      set_dock_state(DockState::Free);
      return false;
    }

    set_dock_state(DockState::Free);
    publish_state("idle", "idle");
    result_message = "undocked to staging: " + dock_staging_target_;
    RCLCPP_INFO(get_logger(), "undock completed: %s", result_message.c_str());
    return true;
  }

  bool run_navigation_to_target_blocking(
    const std::string & target_name,
    std::string & result_message,
    const std::string & behavior_tree = "")
  {
    std::optional<std::pair<double, double>> current_pose_xy;
    std::optional<double> current_pose_yaw;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!navigation_ready_) {
        result_message = "navigation stack not ready";
        return false;
      }
      current_pose_xy = current_pose_xy_;
      current_pose_yaw = current_pose_yaw_;
    }

    const auto target_pose = repository_.resolve_target(target_group_, target_name, current_pose_xy);
    if (!target_pose.has_value()) {
      result_message = "target not found: " + target_name;
      return false;
    }

    if (current_pose_xy.has_value() && current_pose_yaw.has_value()) {
      const double dx = target_pose->x - current_pose_xy->first;
      const double dy = target_pose->y - current_pose_xy->second;
      const double xy_error = std::hypot(dx, dy);
      const double yaw_error = std::abs(
        normalize_angle(yaw_from_target_pose(*target_pose) - *current_pose_yaw));
      if (xy_error <= goal_xy_tolerance_ && yaw_error <= goal_yaw_tolerance_) {
        result_message = "already at target: " + target_name;
        return true;
      }
    }

    if (!action_client_->wait_for_action_server(action_wait_timeout_)) {
      result_message = "navigate_to_pose action server is unavailable";
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose = target_pose_stamped(*target_pose);
    goal.behavior_tree = behavior_tree;

    RCLCPP_INFO(
      get_logger(),
      "sending blocking navigate_to_pose goal: target=%s frame=%s x=%.3f y=%.3f yaw=%.3f "
      "behavior_tree=%s",
      target_name.c_str(), goal.pose.header.frame_id.c_str(),
      goal.pose.pose.position.x, goal.pose.pose.position.y,
      yaw_from_quaternion(goal.pose.pose.orientation),
      goal.behavior_tree.empty() ? "default" : goal.behavior_tree.c_str());

    auto options = typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.feedback_callback =
      [this](
        GoalHandleNavigateToPose::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        update_current_pose_cache(feedback->current_pose);
        NavigateToPose::Impl::FeedbackMessage feedback_message;
        feedback_message.goal_id.uuid = goal_handle->get_goal_id();
        feedback_message.feedback = *feedback;
        nav_feedback_publisher_->publish(feedback_message);
      };

    auto future = action_client_->async_send_goal(goal, options);
    if (future.wait_for(action_wait_timeout_) != std::future_status::ready) {
      result_message = "timed out waiting for staging goal acceptance";
      return false;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
      result_message = "staging goal rejected by action server";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_handle_ = goal_handle;
      active_target_name_ = target_name;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(dock_motion_timeout_) != std::future_status::ready) {
      action_client_->async_cancel_goal(goal_handle);
      clear_active_goal();
      result_message = "timed out navigating to staging target: " + target_name;
      return false;
    }

    const auto result = result_future.get();
    clear_active_goal();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->error_code == NavigateToPose::Result::NONE)
    {
      const auto reached_pose = target_pose_stamped(*target_pose);
      update_current_pose_cache(reached_pose);
      result_message = "reached target: " + target_name;
      RCLCPP_INFO(get_logger(), "blocking navigate_to_pose succeeded: target=%s", target_name.c_str());
      return true;
    }

    const uint16_t error_code = result.result ? result.result->error_code : 0;
    const std::string error_msg = result.result ? result.result->error_msg : "empty result";
    result_message =
      "failed to reach target: " + target_name +
      " result_code=" + std::to_string(static_cast<int>(result.code)) +
      " error_code=" + std::to_string(error_code) +
      " error_msg=" + error_msg;
    RCLCPP_WARN(
      get_logger(),
      "blocking navigate_to_pose failed: target=%s result_code=%d error_code=%u error_msg=%s",
      target_name.c_str(), static_cast<int>(result.code), error_code, error_msg.c_str());
    return false;
  }

  bool resolve_target_pose(
    const std::string & target_name,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & result_message)
  {
    std::optional<std::pair<double, double>> current_pose_xy;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_pose_xy = current_pose_xy_;
    }

    const auto target_pose = repository_.resolve_target(target_group_, target_name, current_pose_xy);
    if (!target_pose.has_value()) {
      result_message =
        "target not found in group '" + target_group_ + "': " + target_name;
      RCLCPP_WARN(get_logger(), "target pose lookup failed: %s", result_message.c_str());
      return false;
    }

    pose = target_pose_stamped(*target_pose);
    return true;
  }

  geometry_msgs::msg::PoseStamped target_pose_stamped(
    const hanmole_navigation::TargetPose & target_pose)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = repository_.frame();
    pose.pose.position.x = target_pose.x;
    pose.pose.position.y = target_pose.y;
    pose.pose.orientation = quaternion_from_target_pose(target_pose);
    return pose;
  }

  nav_msgs::msg::Path make_short_path_to_pose(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = target_pose.header.frame_id;

    std::optional<std::pair<double, double>> current_pose_xy;
    std::optional<double> current_pose_yaw;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_pose_xy = current_pose_xy_;
      current_pose_yaw = current_pose_yaw_;
    }

    if (current_pose_xy.has_value()) {
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose.header = path.header;
      current_pose.pose.position.x = current_pose_xy->first;
      current_pose.pose.position.y = current_pose_xy->second;
      if (current_pose_yaw.has_value()) {
        current_pose.pose.orientation.z = std::sin(*current_pose_yaw * 0.5);
        current_pose.pose.orientation.w = std::cos(*current_pose_yaw * 0.5);
      } else {
        current_pose.pose.orientation.w = 1.0;
      }
      path.poses.push_back(current_pose);
    }

    path.poses.push_back(target_pose);
    return path;
  }

  bool follow_path_to_pose_blocking(
    const geometry_msgs::msg::PoseStamped & target_pose,
    std::string & result_message,
    const std::string & goal_checker_id,
    const std::string & progress_checker_id,
    const std::string & phase)
  {
    if (!follow_path_client_->wait_for_action_server(action_wait_timeout_)) {
      result_message = "follow_path action server is unavailable";
      RCLCPP_WARN(get_logger(), "follow_path unavailable: reason=%s", result_message.c_str());
      return false;
    }

    FollowPath::Goal goal;
    goal.path = make_short_path_to_pose(target_pose);
    goal.controller_id = docking_controller_id_;
    goal.goal_checker_id = goal_checker_id;
    goal.progress_checker_id = progress_checker_id;

    if (!goal.path.poses.empty()) {
      const auto & start_pose = goal.path.poses.front();
      const double dx = target_pose.pose.position.x - start_pose.pose.position.x;
      const double dy = target_pose.pose.position.y - start_pose.pose.position.y;
      RCLCPP_INFO(
        get_logger(),
        "follow_path endpoints: phase=%s start_x=%.3f start_y=%.3f start_yaw=%.3f target_x=%.3f "
        "target_y=%.3f target_yaw=%.3f distance=%.3f",
        phase.c_str(), start_pose.pose.position.x, start_pose.pose.position.y,
        yaw_from_quaternion(start_pose.pose.orientation),
        target_pose.pose.position.x, target_pose.pose.position.y,
        yaw_from_quaternion(target_pose.pose.orientation), std::hypot(dx, dy));
    }

    RCLCPP_INFO(
      get_logger(),
      "sending follow_path goal: phase=%s frame=%s x=%.3f y=%.3f yaw=%.3f poses=%zu controller=%s "
      "goal_checker=%s progress_checker=%s",
      phase.c_str(), target_pose.header.frame_id.c_str(), target_pose.pose.position.x, target_pose.pose.position.y,
      yaw_from_quaternion(target_pose.pose.orientation), goal.path.poses.size(),
      goal.controller_id.c_str(), goal.goal_checker_id.c_str(), goal.progress_checker_id.c_str());
    auto last_feedback = std::make_shared<std::optional<FollowPath::Feedback>>();
    auto last_feedback_log_time = std::make_shared<std::chrono::steady_clock::time_point>(
      std::chrono::steady_clock::now());
    auto options = typename rclcpp_action::Client<FollowPath>::SendGoalOptions();
    options.feedback_callback =
      [this, last_feedback, last_feedback_log_time](
        GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
      {
        *last_feedback = *feedback;
        const auto now_time = std::chrono::steady_clock::now();
        if (now_time - *last_feedback_log_time >= std::chrono::seconds(1)) {
          *last_feedback_log_time = now_time;
          RCLCPP_INFO(
            get_logger(),
            "follow_path feedback: distance_to_goal=%.3f speed=%.3f",
            feedback->distance_to_goal, feedback->speed);
        }
      };

    auto future = follow_path_client_->async_send_goal(goal, options);
    if (future.wait_for(action_wait_timeout_) != std::future_status::ready) {
      result_message = "timed out waiting for follow_path goal acceptance";
      RCLCPP_WARN(get_logger(), "follow_path goal acceptance timed out: reason=%s", result_message.c_str());
      return false;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
      result_message = "follow_path goal rejected by controller_server";
      RCLCPP_WARN(get_logger(), "follow_path goal rejected: reason=%s", result_message.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "follow_path goal accepted: phase=%s", phase.c_str());
    auto result_future = follow_path_client_->async_get_result(goal_handle);
    if (result_future.wait_for(dock_motion_timeout_) != std::future_status::ready) {
      follow_path_client_->async_cancel_goal(goal_handle);
      result_message = "timed out executing " + phase + " MPPI path";
      if (last_feedback->has_value()) {
        result_message +=
          " last_distance=" + std::to_string((*last_feedback)->distance_to_goal) +
          " last_speed=" + std::to_string((*last_feedback)->speed);
      }
      RCLCPP_WARN(
        get_logger(), "follow_path execution timed out: phase=%s reason=%s",
        phase.c_str(), result_message.c_str());
      return false;
    }

    const auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->error_code == FollowPath::Result::NONE)
    {
      result_message = phase + " MPPI path succeeded";
      RCLCPP_INFO(get_logger(), "follow_path succeeded: phase=%s", phase.c_str());
      return true;
    }

    result_message =
      phase + " MPPI path failed: error_code=" + std::to_string(result.result->error_code) +
      " error_msg=" + result.result->error_msg;
    if (last_feedback->has_value()) {
      result_message +=
        " last_distance=" + std::to_string((*last_feedback)->distance_to_goal) +
        " last_speed=" + std::to_string((*last_feedback)->speed);
    }
    RCLCPP_WARN(
      get_logger(), "follow_path failed: phase=%s reason=%s", phase.c_str(), result_message.c_str());
    return false;
  }

  bool follow_path_to_pose_with_collision_monitor_disabled(
    const geometry_msgs::msg::PoseStamped & target_pose,
    std::string & result_message,
    bool restore_on_failure,
    const std::string & goal_checker_id,
    const std::string & progress_checker_id,
    const std::string & phase)
  {
    RCLCPP_INFO(
      get_logger(),
      "disabling collision monitor for follow_path: phase=%s restore_on_failure=%s",
      phase.c_str(), restore_on_failure ? "true" : "false");
    if (!set_collision_monitor_enabled(false, result_message)) {
      RCLCPP_WARN(
        get_logger(),
        "failed to disable collision monitor before follow_path: reason=%s",
        result_message.c_str());
      return false;
    }

    if (follow_path_to_pose_blocking(
        target_pose, result_message, goal_checker_id, progress_checker_id, phase))
    {
      RCLCPP_INFO(
        get_logger(),
        "follow_path completed while collision monitor is disabled; caller is responsible for re-enabling it");
      return true;
    }

    if (restore_on_failure) {
      const std::string failure_message = result_message;
      std::string cleanup_message;
      if (set_collision_monitor_enabled(true, cleanup_message)) {
        RCLCPP_INFO(get_logger(), "collision monitor restored after follow_path failure");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "failed to restore collision monitor after follow_path failure: reason=%s",
          cleanup_message.c_str());
      }
      result_message = failure_message;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "follow_path failed while collision monitor remains disabled because restore_on_failure=false");
    }
    return false;
  }

  bool set_collision_monitor_enabled(bool enabled, std::string & result_message)
  {
    const char * requested_state = enabled ? "enabled" : "disabled";
    RCLCPP_INFO(
      get_logger(),
      "collision monitor toggle request: service=%s target_state=%s",
      collision_monitor_toggle_service_.c_str(), requested_state);

    if (!collision_toggle_client_->wait_for_service(action_wait_timeout_)) {
      result_message =
        "collision monitor toggle service unavailable: " + collision_monitor_toggle_service_;
      RCLCPP_ERROR(get_logger(), "%s", result_message.c_str());
      return false;
    }

    auto request = std::make_shared<Toggle::Request>();
    request->enable = enabled;
    auto future = collision_toggle_client_->async_send_request(request);
    if (future.wait_for(action_wait_timeout_) != std::future_status::ready) {
      result_message =
        "timed out toggling collision monitor to " + std::string(requested_state) +
        " via " + collision_monitor_toggle_service_;
      RCLCPP_ERROR(get_logger(), "%s", result_message.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
      const std::string response_message =
        response ? response->message : "empty toggle service response";
      result_message =
        "collision monitor toggle to " + std::string(requested_state) + " failed: " +
        response_message;
      RCLCPP_ERROR(get_logger(), "%s", result_message.c_str());
      return false;
    }

    result_message = "collision monitor " + std::string(requested_state);
    RCLCPP_INFO(get_logger(), "collision monitor toggle succeeded: state=%s", requested_state);
    return true;
  }

  void clear_active_goal()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_goal_handle_.reset();
    active_target_name_.clear();
  }

  void handle_result(
    const std::string & target_name,
    const GoalHandleNavigateToPose::WrappedResult & result)
  {
    std::string nav_status = "failed";
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->error_code == NavigateToPose::Result::NONE)
    {
      nav_status = "succeeded";
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      nav_status = "canceled";
    }

    std::optional<std::string> pending_target_name;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_handle_.reset();
      active_target_name_.clear();
      if (switching_to_pending_target_ && pending_target_name_.has_value()) {
        pending_target_name = pending_target_name_;
        pending_target_name_.reset();
        switching_to_pending_target_ = false;
      }
    }

    if (pending_target_name.has_value()) {
      RCLCPP_INFO(
        get_logger(),
        "navigation result for %s while switching target: %s",
        target_name.c_str(),
        nav_status.c_str());
      std::string result_message;
      if (!start_navigation_to_target(*pending_target_name, result_message)) {
        publish_terminal_state("failed");
        RCLCPP_WARN(
          get_logger(),
          "failed to start pending navigation target %s: %s",
          pending_target_name->c_str(),
          result_message.c_str());
      }
      return;
    }

    publish_terminal_state(nav_status);
    RCLCPP_INFO(get_logger(), "navigation result for %s: %s", target_name.c_str(), nav_status.c_str());
  }

  void publish_terminal_state(
    const std::string & nav_status,
    const std::string & target_state = "idle")
  {
    publish_state(nav_status, target_state);
    publish_state("idle", "idle");
  }

  void mark_action_status_active()
  {
    std::lock_guard<std::mutex> lock(action_status_mutex_);
    action_status_active_goal_ = true;
  }

  void mark_action_status_inactive()
  {
    std::lock_guard<std::mutex> lock(action_status_mutex_);
    action_status_active_goal_ = false;
  }

  void publish_action_terminal_state(const std::string & nav_status)
  {
    bool should_publish_terminal = false;
    {
      std::lock_guard<std::mutex> lock(action_status_mutex_);
      should_publish_terminal = action_status_active_goal_;
      action_status_active_goal_ = false;
    }

    if (should_publish_terminal) {
      publish_terminal_state(nav_status);
      return;
    }
    publish_action_state("idle");
  }

  void publish_action_state(const std::string & nav_status)
  {
    std::string target_state = "idle";
    if (nav_status != "idle") {
      std::lock_guard<std::mutex> lock(published_state_mutex_);
      target_state = last_target_state_;
    }
    publish_state(nav_status, target_state);
  }

  void publish_state(const std::string & nav_status, const std::string & target_state)
  {
    {
      std::lock_guard<std::mutex> lock(published_state_mutex_);
      last_nav_status_ = nav_status;
      last_target_state_ = target_state;
    }

    publish_state_message(nav_status, target_state);
  }

  void republish_state()
  {
    std::string nav_status;
    std::string target_state;
    {
      std::lock_guard<std::mutex> lock(published_state_mutex_);
      nav_status = last_nav_status_;
      target_state = last_target_state_;
    }

    publish_state_message(nav_status, target_state);
  }

  void publish_state_message(const std::string & nav_status, const std::string & target_state)
  {
    std_msgs::msg::String nav_status_message;
    nav_status_message.data = nav_status;
    nav_status_publisher_->publish(nav_status_message);

    std_msgs::msg::String target_state_message;
    target_state_message.data = target_state;
    target_state_publisher_->publish(target_state_message);
  }

  void set_dock_state(DockState dock_state)
  {
    DockState previous_dock_state = DockState::Free;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous_dock_state = dock_state_;
      dock_state_ = dock_state;
    }
    RCLCPP_INFO(
      get_logger(),
      "dock state transition: %s -> %s",
      dock_state_name(previous_dock_state), dock_state_name(dock_state));
    publish_dock_state();
  }

  void publish_dock_state()
  {
    DockState dock_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dock_state = dock_state_;
    }

    std_msgs::msg::String message;
    message.data = dock_state_name(dock_state);
    dock_state_publisher_->publish(message);
  }

  bool publish_initial_pose_internal()
  {
    const auto pose = repository_.resolve_target(
      target_group_,
      repository_.initial_pose_target(),
      std::nullopt);
    if (!pose.has_value()) {
      return false;
    }

    const auto latest_odom_tf_stamp = lookup_latest_transform_stamp(odom_frame_, robot_base_frame_);
    if (!latest_odom_tf_stamp.has_value()) {
      return false;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = *latest_odom_tf_stamp;
    message.header.frame_id = repository_.frame();
    message.pose.pose.position.x = pose->x;
    message.pose.pose.position.y = pose->y;
    message.pose.pose.orientation = quaternion_from_target_pose(*pose);
    message.pose.covariance[0] = 0.25;
    message.pose.covariance[7] = 0.25;
    message.pose.covariance[35] = 0.0685;
    initial_pose_publisher_->publish(message);

    initial_pose_published_ = true;
    map_tf_success_count_ = 0;
    const auto tf_age_ms = (now() - *latest_odom_tf_stamp).seconds() * 1000.0;
    RCLCPP_INFO(
      get_logger(),
      "published initial pose: %s (odom tf age %.1f ms)",
      repository_.initial_pose_target().c_str(),
      tf_age_ms);
    return true;
  }

  void tick_startup_state_machine()
  {
    if (startup_tick_running_.exchange(true)) {
      return;
    }

    struct StartupTickGuard
    {
      explicit StartupTickGuard(std::atomic_bool & flag_ref)
      : flag(flag_ref)
      {
      }

      ~StartupTickGuard()
      {
        flag.store(false);
      }

      std::atomic_bool & flag;
    } guard(startup_tick_running_);

    switch (startup_state_) {
      case StartupState::BootDelay:
        if (std::chrono::steady_clock::now() >= startup_deadline_) {
          transition_startup_state(StartupState::WaitingForInputs);
        }
        break;
      case StartupState::WaitingForInputs:
        if (message_is_fresh(last_scan_time_) && message_is_fresh(last_odom_time_)) {
          transition_startup_state(StartupState::WaitingForOdomTf);
        }
        break;
      case StartupState::WaitingForOdomTf:
        if (can_transform(odom_frame_, robot_base_frame_)) {
          transition_startup_state(StartupState::StartingLocalization);
        }
        break;
      case StartupState::StartingLocalization:
        if (localization_nodes_are_active()) {
          transition_startup_state(StartupState::WaitingForLocalizationReady);
        } else if (request_lifecycle_startup(localization_manager_client_, "localization")) {
          transition_startup_state(StartupState::WaitingForLocalizationReady);
        } else if (localization_nodes_are_active()) {
          transition_startup_state(StartupState::WaitingForLocalizationReady);
        }
        break;
      case StartupState::WaitingForLocalizationReady:
        if (localization_nodes_are_active()) {
          transition_startup_state(StartupState::PublishingInitialPose);
        }
        break;
      case StartupState::PublishingInitialPose:
        if (transform_is_fresh(odom_frame_, robot_base_frame_)) {
          if (publish_initial_pose_internal()) {
            transition_startup_state(StartupState::WaitingForMapTf);
          } else {
            transition_startup_state(StartupState::Error);
          }
        }
        break;
      case StartupState::WaitingForMapTf:
        if (map_transform_is_stable()) {
          transition_startup_state(StartupState::StartingNavigation);
        }
        break;
      case StartupState::StartingNavigation:
        if (navigation_nodes_are_active()) {
          transition_startup_state(StartupState::WaitingForNavigationReady);
        } else if (request_lifecycle_startup(navigation_manager_client_, "navigation")) {
          transition_startup_state(StartupState::WaitingForNavigationReady);
        } else if (navigation_nodes_are_active()) {
          transition_startup_state(StartupState::WaitingForNavigationReady);
        }
        break;
      case StartupState::WaitingForNavigationReady:
        if (navigation_nodes_are_active()) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          navigation_ready_ = true;
          transition_startup_state(StartupState::Ready);
        }
        break;
      case StartupState::Ready:
      case StartupState::Error:
        break;
    }

  }

  bool request_lifecycle_startup(
    const rclcpp::Client<ManageLifecycleNodes>::SharedPtr & client,
    const std::string & name)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(0))) {
      return false;
    }

    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = ManageLifecycleNodes::Request::STARTUP;
    auto future = client->async_send_request(request);
    if (future.wait_for(lifecycle_request_timeout_) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "timed out waiting for %s lifecycle startup response", name.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
      RCLCPP_WARN(get_logger(), "%s lifecycle startup request failed", name.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "%s lifecycle startup succeeded", name.c_str());
    return true;
  }

  bool lifecycle_node_is_active(const rclcpp::Client<GetState>::SharedPtr & client)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(0))) {
      return false;
    }

    auto request = std::make_shared<GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) {
      return false;
    }

    const auto response = future.get();
    return response &&
           response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  bool localization_nodes_are_active()
  {
    return lifecycle_node_is_active(map_server_state_client_) &&
           lifecycle_node_is_active(amcl_state_client_);
  }

  bool navigation_nodes_are_active()
  {
    return lifecycle_node_is_active(planner_server_state_client_) &&
           lifecycle_node_is_active(controller_server_state_client_) &&
           lifecycle_node_is_active(bt_navigator_state_client_);
  }

  bool message_is_fresh(const std::optional<rclcpp::Time> & stamp) const
  {
    if (!stamp.has_value()) {
      return false;
    }

    const auto freshness_limit = rclcpp::Duration::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(input_freshness_timeout_).count());
    return (now() - *stamp) <= freshness_limit;
  }

  std::optional<rclcpp::Time> lookup_latest_transform_stamp(
    const std::string & target_frame,
    const std::string & source_frame) const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        rclcpp::Time(0),
        rclcpp::Duration::from_nanoseconds(0));
      return rclcpp::Time(transform.header.stamp);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  bool transform_is_fresh(const std::string & target_frame, const std::string & source_frame) const
  {
    const auto stamp = lookup_latest_transform_stamp(target_frame, source_frame);
    if (!stamp.has_value()) {
      return false;
    }

    const auto freshness_limit = rclcpp::Duration::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tf_freshness_threshold_).count());
    return (now() - *stamp) <= freshness_limit;
  }

  bool can_transform(const std::string & target_frame, const std::string & source_frame) const
  {
    return tf_buffer_->canTransform(
      target_frame,
      source_frame,
      rclcpp::Time(0),
      rclcpp::Duration::from_nanoseconds(0));
  }

  bool map_transform_is_stable()
  {
    if (can_transform(map_frame_, robot_base_frame_)) {
      ++map_tf_success_count_;
    } else {
      map_tf_success_count_ = 0;
    }
    return map_tf_success_count_ >= stable_tf_success_count_;
  }

  void transition_startup_state(StartupState next_state)
  {
    if (startup_state_ == next_state) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "startup state: %s -> %s",
      startup_state_name(startup_state_),
      startup_state_name(next_state));
    startup_state_ = next_state;
  }

  hanmole_navigation::TargetRepository repository_;
  std::string target_group_;
  std::string nav_mode_;
  std::string amcl_pose_topic_;
  std::string scan_topic_;
  std::string dock_scan_topic_;
  std::string filtered_odom_topic_;
  std::string raw_odom_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string robot_base_frame_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string localization_manager_service_;
  std::string navigation_manager_service_;
  std::string map_server_state_service_;
  std::string amcl_state_service_;
  std::string planner_server_state_service_;
  std::string controller_server_state_service_;
  std::string bt_navigator_state_service_;
  std::string navigate_action_name_;
  std::string navigate_action_status_topic_;
  std::string navigate_service_name_;
  std::string cancel_service_name_;
  std::string auto_dock_service_name_;
  std::string undock_service_name_;
  std::string set_initial_pose_service_name_;
  std::string nav_status_topic_;
  std::string nav_feedback_topic_;
  std::string target_state_topic_;
  std::string dock_state_topic_;
  std::string initial_pose_topic_;
  std::string follow_path_action_name_;
  std::string collision_monitor_toggle_service_;
  std::string dock_staging_target_;
  std::string dock_charge_target_;
  std::string charge_marker_topic_;
  bool charge_marker_enabled_{true};
  std::string docking_controller_id_;
  std::string docking_goal_checker_id_;
  std::string undock_goal_checker_id_;
  std::string docking_progress_checker_id_;
  std::string auto_dock_staging_behavior_tree_;
  int dock_detection_min_samples_{5};
  int dock_detection_max_samples_{12};
  double dock_detection_max_range_{2.0};
  double dock_scan_min_angle_{-3.141592653589793};
  double dock_scan_max_angle_{3.141592653589793};
  double dock_interpolation_max_difference_{0.02};
  double dock_mutation_min_jump_{0.025};
  double dock_mutation_max_jump_{0.065};
  double dock_feature_pair_max_angle_{0.12217304763960307};
  double dock_feature_outer_check_angle_{0.08726646259971647};
  double dock_feature_pair_max_range_difference_{0.08};
  double dock_feature_min_width_{0.10};
  double dock_feature_max_width_{1.00};
  bool dock_flat_plate_detection_enabled_{true};
  double dock_flat_plate_center_angle_{0.0};
  double dock_flat_plate_half_angle_{0.20};
  int dock_flat_plate_min_points_{8};
  double dock_flat_plate_max_rms_error_{0.03};
  bool dock_flat_plate_lock_normal_to_center_angle_{true};
  double dock_detection_max_position_spread_{0.05};
  double dock_detection_max_yaw_spread_{0.08726646259971647};
  double dock_dynamic_max_position_correction_{0.30};
  double dock_dynamic_max_yaw_correction_{0.2617993877991494};
  double dock_nominal_hub_max_distance_{1.0};
  double dock_base_to_hub_offset_min_{0.05};
  double dock_base_to_hub_offset_max_{1.0};
  double dock_base_to_hub_offset_{-1.0};
  std::chrono::milliseconds action_wait_timeout_{3000};
  std::chrono::milliseconds dock_motion_timeout_{30000};
  std::chrono::milliseconds dock_detection_timeout_{3000};
  std::chrono::milliseconds dock_detection_scan_freshness_{500};
  std::chrono::milliseconds charge_marker_lifetime_{0};
  std::chrono::milliseconds startup_delay_{3000};
  std::chrono::milliseconds input_freshness_timeout_{500};
  std::chrono::milliseconds startup_poll_period_{5};
  std::chrono::milliseconds tf_freshness_threshold_{2};
  std::chrono::milliseconds lifecycle_request_timeout_{12000};

  mutable std::mutex state_mutex_;
  std::optional<std::pair<double, double>> current_pose_xy_;
  std::optional<double> current_pose_yaw_;
  std::shared_ptr<GoalHandleNavigateToPose> active_goal_handle_;
  std::string active_target_name_;
  std::optional<std::string> pending_target_name_;
  bool switching_to_pending_target_{false};
  bool navigation_ready_{false};
  DockState dock_state_{DockState::Free};
  std::mutex operation_mutex_;
  std::atomic_bool dock_detection_enabled_{false};
  mutable std::mutex dock_detection_mutex_;
  std::condition_variable dock_detection_cv_;
  std::deque<ChargingHubDetection> dock_detection_samples_;
  uint64_t dock_detection_sequence_{0};
  std::atomic<uint64_t> dock_detection_received_scans_{0};
  std::atomic<uint64_t> dock_detection_fresh_scans_{0};
  std::atomic<uint64_t> dock_detection_pattern_matches_{0};

  StartupState startup_state_{StartupState::BootDelay};
  std::chrono::steady_clock::time_point startup_deadline_;
  int stable_tf_success_count_{3};
  int map_tf_success_count_{0};
  bool map_received_{false};
  bool initial_pose_published_{false};
  std::atomic_bool startup_tick_running_{false};
  std::optional<rclcpp::Time> last_scan_time_;
  std::optional<rclcpp::Time> last_odom_time_;
  std::optional<rclcpp::Time> last_map_time_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  double goal_xy_tolerance_{0.10};
  double goal_yaw_tolerance_{0.20};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp::Client<Toggle>::SharedPtr collision_toggle_client_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr localization_manager_client_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr navigation_manager_client_;
  rclcpp::Client<GetState>::SharedPtr map_server_state_client_;
  rclcpp::Client<GetState>::SharedPtr amcl_state_client_;
  rclcpp::Client<GetState>::SharedPtr planner_server_state_client_;
  rclcpp::Client<GetState>::SharedPtr controller_server_state_client_;
  rclcpp::Client<GetState>::SharedPtr bt_navigator_state_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_publisher_;
  rclcpp::Publisher<NavigateToPose::Impl::FeedbackMessage>::SharedPtr nav_feedback_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dock_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr charge_marker_publisher_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr action_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr dock_scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_subscription_;
  rclcpp::Service<SetString>::SharedPtr navigate_service_;
  rclcpp::Service<SetString>::SharedPtr auto_dock_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr undock_service_;
  rclcpp::Service<Trigger>::SharedPtr set_initial_pose_service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;
  std::mutex published_state_mutex_;
  std::string last_nav_status_{"idle"};
  std::string last_target_state_{"idle"};
  std::mutex action_status_mutex_;
  bool action_status_active_goal_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2TargetGatewayNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
