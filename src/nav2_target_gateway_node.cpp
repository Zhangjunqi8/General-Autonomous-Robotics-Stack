#include <chrono>
#include <cmath>
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
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
    docking_controller_id_ = declare_parameter<std::string>("docking_controller_id", "DockMppi");
    docking_goal_checker_id_ = declare_parameter<std::string>(
      "docking_goal_checker_id", "dock_goal_checker");
    undock_goal_checker_id_ = declare_parameter<std::string>(
      "undock_goal_checker_id", "general_goal_checker");
    docking_progress_checker_id_ = declare_parameter<std::string>(
      "docking_progress_checker_id", "progress_checker");
    auto_dock_staging_behavior_tree_ = declare_parameter<std::string>(
      "auto_dock_staging_behavior_tree", "");
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

    repository_.load_from_file(target_file);
    if (target_group_.empty()) {
      target_group_ = repository_.default_group();
    }
    if (!repository_.has_group(target_group_)) {
      throw std::runtime_error("target_group not found: " + target_group_);
    }

    action_wait_timeout_ = std::chrono::milliseconds(action_timeout_ms);
    dock_motion_timeout_ = std::chrono::milliseconds(dock_motion_timeout_ms);
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

    status_publish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { republish_state(); },
      callback_group_);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
        last_scan_time_ = now();
      },
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

    geometry_msgs::msg::PoseStamped charge_pose;
    if (!resolve_target_pose(dock_charge_target_, charge_pose, response->result)) {
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
  std::string docking_controller_id_;
  std::string docking_goal_checker_id_;
  std::string undock_goal_checker_id_;
  std::string docking_progress_checker_id_;
  std::string auto_dock_staging_behavior_tree_;
  std::chrono::milliseconds action_wait_timeout_{3000};
  std::chrono::milliseconds dock_motion_timeout_{30000};
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
  rclcpp::Subscription<GoalStatusArray>::SharedPtr action_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
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
