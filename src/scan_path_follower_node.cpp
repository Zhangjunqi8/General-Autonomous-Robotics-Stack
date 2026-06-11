#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/wait.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace hanmole_navigation
{

struct Footprint
{
  double front{0.24};
  double rear{0.24};
  double left{0.17};
  double right{0.17};
};

struct ObstaclePoint
{
  double x{0.0};
  double y{0.0};
};

class ScanPathFollowerNode : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using BackUp = nav2_msgs::action::BackUp;
  using Wait = nav2_msgs::action::Wait;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;
  using GoalHandleWait = rclcpp_action::ClientGoalHandle<Wait>;

  ScanPathFollowerNode()
  : Node("scan_path_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/scan_follower/plan");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/scan_follower/cancel");
    nav_status_topic_ = declare_parameter<std::string>("nav_status_topic", "/nav_status");
    target_state_topic_ = declare_parameter<std::string>(
      "target_state_topic", "/hanmole_navigation/target_state");
    safety_state_topic_ = declare_parameter<std::string>(
      "safety_state_topic", "/cmd_vel_safety_filter/state");
    compute_path_action_ = declare_parameter<std::string>("compute_path_action", "compute_path_to_pose");
    backup_action_ = declare_parameter<std::string>("backup_action", "backup");
    wait_action_ = declare_parameter<std::string>("wait_action", "wait");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    footprint_.front = declare_parameter<double>("footprint.front", 0.24);
    footprint_.rear = declare_parameter<double>("footprint.rear", 0.24);
    footprint_.left = declare_parameter<double>("footprint.left", 0.17);
    footprint_.right = declare_parameter<double>("footprint.right", 0.17);

    local_goal_distance_ = declare_parameter<double>("local_goal_distance", 0.7);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.12);
    obstacle_clearance_ = declare_parameter<double>("obstacle_clearance", 0.10);
    prediction_time_ = declare_parameter<double>("prediction_time", 0.8);
    simulation_dt_ = declare_parameter<double>("simulation_dt", 0.1);
    control_frequency_ = declare_parameter<double>("control_frequency", 15.0);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.4);
    replan_period_ = declare_parameter<double>("replan_period", 2.0);

    max_vel_x_ = declare_parameter<double>("max_vel_x", 0.25);
    max_vel_y_ = declare_parameter<double>("max_vel_y", 0.15);
    max_vel_theta_ = declare_parameter<double>("max_vel_theta", 0.7);
    min_cmd_speed_ = declare_parameter<double>("min_cmd_speed", 0.04);
    goal_gain_x_ = declare_parameter<double>("goal_gain_x", 0.75);
    goal_gain_y_ = declare_parameter<double>("goal_gain_y", 0.75);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 0.9);
    heading_weight_ = declare_parameter<double>("heading_weight", 2.0);
    velocity_weight_ = declare_parameter<double>("velocity_weight", 0.15);
    turn_weight_ = declare_parameter<double>("turn_weight", 0.25);
    allow_strafe_ = declare_parameter<bool>("allow_strafe", true);
    allow_reverse_ = declare_parameter<bool>("allow_reverse", true);
    stop_on_stale_scan_ = declare_parameter<bool>("stop_on_stale_scan", true);
    enable_recovery_ = declare_parameter<bool>("enable_recovery", true);
    blocked_timeout_ = declare_parameter<double>("blocked_timeout", 2.0);
    recovery_wait_time_ = declare_parameter<double>("recovery_wait_time", 0.5);
    backup_distance_ = declare_parameter<double>("backup_distance", 0.15);
    backup_speed_ = declare_parameter<double>("backup_speed", 0.05);
    backup_time_allowance_ = declare_parameter<double>("backup_time_allowance", 8.0);
    max_recovery_attempts_ = declare_parameter<int>("max_recovery_attempts", 3);

    path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, compute_path_action_);
    backup_client_ = rclcpp_action::create_client<BackUp>(this, backup_action_);
    wait_client_ = rclcpp_action::create_client<Wait>(this, wait_action_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanPathFollowerNode::onScan, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&ScanPathFollowerNode::onGoal, this, std::placeholders::_1));
    safety_state_sub_ = create_subscription<std_msgs::msg::String>(
      safety_state_topic_, 10,
      std::bind(&ScanPathFollowerNode::onSafetyState, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, 1);
    nav_status_pub_ = create_publisher<std_msgs::msg::String>(nav_status_topic_, 10);
    target_state_pub_ = create_publisher<std_msgs::msg::String>(target_state_topic_, 10);
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      cancel_service_name_,
      std::bind(
        &ScanPathFollowerNode::onCancel, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ScanPathFollowerNode::onTimer, this));

    publishState("idle", "idle");
    RCLCPP_INFO(get_logger(), "scan_path_follower ready: %s -> %s", goal_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_ = *goal;
    has_active_goal_ = true;
    current_path_.poses.clear();
    last_replan_time_ = now() - rclcpp::Duration::from_seconds(replan_period_ + 1.0);
    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    safety_blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    safety_blocked_ = false;
    recovery_attempts_ = 0;
    recovery_in_progress_ = false;
    RCLCPP_INFO(get_logger(), "received goal %.2f %.2f", goal->pose.position.x, goal->pose.position.y);
    publishNavStatus("navigating");
  }

  void onCancel(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_active_goal_ = false;
      current_path_.poses.clear();
      path_request_in_flight_ = false;
      blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      safety_blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      safety_blocked_ = false;
      recovery_attempts_ = 0;
      recovery_in_progress_ = false;
    }
    if (backup_client_) {
      backup_client_->async_cancel_all_goals();
    }
    if (wait_client_) {
      wait_client_->async_cancel_all_goals();
    }
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    publishState("idle", "idle");
    response->success = true;
    response->message = "scan follower navigation canceled";
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::vector<ObstaclePoint> points;
    points.reserve(scan->ranges.size());

    geometry_msgs::msg::TransformStamped transform;
    bool have_transform = false;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, scan->header.frame_id, tf2::TimePointZero, 50ms);
      have_transform = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "scan transform unavailable, assuming scan frame is base: %s", ex.what());
    }

    double angle = scan->angle_min;
    for (const auto range : scan->ranges) {
      if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
        const double x = range * std::cos(angle);
        const double y = range * std::sin(angle);
        if (have_transform) {
          geometry_msgs::msg::PointStamped in;
          geometry_msgs::msg::PointStamped out;
          in.header = scan->header;
          in.point.x = x;
          in.point.y = y;
          in.point.z = 0.0;
          tf2::doTransform(in, out, transform);
          points.push_back({out.point.x, out.point.y});
        } else {
          points.push_back({x, y});
        }
      }
      angle += scan->angle_increment;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = std::move(points);
    last_scan_time_ = now();
    has_scan_ = true;
  }

  void onSafetyState(const std_msgs::msg::String::SharedPtr state)
  {
    const auto current_time = now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (state->data == "blocked") {
      if (!safety_blocked_) {
        safety_blocked_since_ = current_time;
        if (blocked_since_.nanoseconds() == 0) {
          blocked_since_ = current_time;
        }
      }
      safety_blocked_ = true;
      return;
    }
    safety_blocked_ = false;
    safety_blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void onTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_active_goal_) {
        goal = active_goal_;
      }
    }

    if (recoveryInProgress()) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    if (safetyBlockedTooLong()) {
      handleBlockedPath();
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    if (goal) {
      maybeRequestPath(*goal);
    }

    cmd_pub_->publish(computeCommand());
  }

  void maybeRequestPath(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (path_request_in_flight_) {
      return;
    }
    if ((now() - last_replan_time_).seconds() < replan_period_) {
      return;
    }
    if (!path_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "planner action server %s unavailable", compute_path_action_.c_str());
      publishState("failed", "idle");
      clearActiveGoal();
      return;
    }

    ComputePathToPose::Goal action_goal;
    action_goal.goal = goal;
    action_goal.planner_id = planner_id_;
    action_goal.use_start = false;

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandleComputePath::SharedPtr & handle) {
      if (!handle) {
        path_request_in_flight_ = false;
        RCLCPP_WARN(get_logger(), "planner rejected goal");
        publishState("failed", "idle");
        clearActiveGoal();
      }
    };
    options.result_callback = [this](const GoalHandleComputePath::WrappedResult & result) {
      path_request_in_flight_ = false;
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(get_logger(), "planner failed with result code %d", static_cast<int>(result.code));
        publishState("failed", "idle");
        clearActiveGoal();
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      current_path_ = result.result->path;
      current_path_.header.stamp = now();
      plan_pub_->publish(current_path_);
      RCLCPP_INFO(get_logger(), "received path with %zu poses", current_path_.poses.size());
    };

    path_request_in_flight_ = true;
    last_replan_time_ = now();
    path_client_->async_send_goal(action_goal, options);
  }

  geometry_msgs::msg::Twist computeCommand()
  {
    std::vector<ObstaclePoint> obstacles;
    nav_msgs::msg::Path path;
    bool has_goal = false;
    bool has_scan = false;
    rclcpp::Time last_scan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      obstacles = obstacles_;
      path = current_path_;
      has_goal = has_active_goal_;
      has_scan = has_scan_;
      last_scan = last_scan_time_;
    }

    geometry_msgs::msg::Twist zero;
    if (!has_goal || path.poses.empty()) {
      return zero;
    }
    if (stop_on_stale_scan_ && (!has_scan || (now() - last_scan).seconds() > scan_timeout_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "stale or missing scan, stopping");
      return zero;
    }

    const auto local_goal = selectLocalGoal(path);
    if (!local_goal) {
      clearGoalIfReached(path);
      return zero;
    }

    const double dist = std::hypot(local_goal->first, local_goal->second);
    if (dist < goal_tolerance_) {
      clearGoalIfReached(path);
      return zero;
    }

    const auto candidates = generateCandidates(*local_goal);
    double best_score = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Twist best_cmd;
    bool found = false;

    for (const auto & candidate : candidates) {
      if (isTrajectoryCollisionFree(candidate, obstacles)) {
        const double end_x = candidate.linear.x * prediction_time_;
        const double end_y = candidate.linear.y * prediction_time_;
        const double goal_error = std::hypot(local_goal->first - end_x, local_goal->second - end_y);
        const double speed = std::hypot(candidate.linear.x, candidate.linear.y);
        const double turn = std::abs(candidate.angular.z);
        const double score = heading_weight_ * goal_error - velocity_weight_ * speed + turn_weight_ * turn;
        if (score < best_score) {
          best_score = score;
          best_cmd = candidate;
          found = true;
        }
      }
    }

    if (!found) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "no collision-free candidate velocity, stopping");
      handleBlockedPath();
      return zero;
    }
    if (!safetyBlocked()) {
      clearBlockedState();
    }
    return best_cmd;
  }

  bool recoveryInProgress()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return recovery_in_progress_;
  }

  bool safetyBlocked()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return safety_blocked_;
  }

  bool safetyBlockedTooLong()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_active_goal_ && safety_blocked_ && !recovery_in_progress_ &&
      safety_blocked_since_.nanoseconds() != 0 &&
      (now() - safety_blocked_since_).seconds() >= blocked_timeout_;
  }

  void handleBlockedPath()
  {
    if (!enable_recovery_) {
      return;
    }

    const auto current_time = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_active_goal_ || recovery_in_progress_) {
        return;
      }
      if (blocked_since_.nanoseconds() == 0) {
        blocked_since_ = current_time;
        return;
      }
      if ((current_time - blocked_since_).seconds() < blocked_timeout_) {
        return;
      }
      if (recovery_attempts_ >= max_recovery_attempts_) {
        RCLCPP_WARN(get_logger(), "recovery attempts exceeded");
        has_active_goal_ = false;
        current_path_.poses.clear();
        path_request_in_flight_ = false;
        recovery_in_progress_ = false;
        blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        publishState("failed", "idle");
        return;
      }
      recovery_in_progress_ = true;
      ++recovery_attempts_;
    }

    startBackupRecovery();
  }

  void clearBlockedState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    safety_blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    safety_blocked_ = false;
    if (!recovery_in_progress_) {
      recovery_attempts_ = 0;
    }
  }

  builtin_interfaces::msg::Duration toDuration(double seconds) const
  {
    builtin_interfaces::msg::Duration duration;
    const auto whole_seconds = static_cast<int32_t>(std::floor(std::max(0.0, seconds)));
    duration.sec = whole_seconds;
    duration.nanosec = static_cast<uint32_t>(
      std::round((std::max(0.0, seconds) - whole_seconds) * 1e9));
    return duration;
  }

  void startBackupRecovery()
  {
    if (!backup_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "backup action server %s unavailable", backup_action_.c_str());
      finishRecovery(false);
      return;
    }

    BackUp::Goal goal;
    goal.target.x = -std::abs(backup_distance_);
    goal.target.y = 0.0;
    goal.target.z = 0.0;
    goal.speed = static_cast<float>(std::abs(backup_speed_));
    goal.time_allowance = toDuration(backup_time_allowance_);

    publishTargetState("recovering");
    RCLCPP_WARN(
      get_logger(), "starting recovery backup %.2fm attempt %d/%d",
      backup_distance_, recovery_attempts_, max_recovery_attempts_);

    rclcpp_action::Client<BackUp>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandleBackUp::SharedPtr & handle) {
      if (!handle) {
        RCLCPP_WARN(get_logger(), "backup recovery rejected");
        finishRecovery(false);
      }
    };
    options.result_callback = [this](const GoalHandleBackUp::WrappedResult & result) {
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->error_code == 0;
      if (!ok) {
        RCLCPP_WARN(get_logger(), "backup recovery failed");
        finishRecovery(false);
        return;
      }
      startWaitRecovery();
    };
    backup_client_->async_send_goal(goal, options);
  }

  void startWaitRecovery()
  {
    if (!wait_client_->wait_for_action_server(0s)) {
      finishRecovery(true);
      return;
    }

    Wait::Goal goal;
    goal.time = toDuration(recovery_wait_time_);

    rclcpp_action::Client<Wait>::SendGoalOptions options;
    options.goal_response_callback = [this](const GoalHandleWait::SharedPtr & handle) {
      if (!handle) {
        finishRecovery(true);
      }
    };
    options.result_callback = [this](const GoalHandleWait::WrappedResult &) {
      finishRecovery(true);
    };
    wait_client_->async_send_goal(goal, options);
  }

  void finishRecovery(bool success)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      recovery_in_progress_ = false;
      blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      safety_blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      safety_blocked_ = false;
      current_path_.poses.clear();
      path_request_in_flight_ = false;
      last_replan_time_ = now() - rclcpp::Duration::from_seconds(replan_period_ + 1.0);
      if (!success && recovery_attempts_ >= max_recovery_attempts_) {
        has_active_goal_ = false;
      }
    }

    if (!success && recovery_attempts_ >= max_recovery_attempts_) {
      RCLCPP_WARN(get_logger(), "recovery failed after maximum attempts");
      publishState("failed", "idle");
      return;
    }

    publishTargetState(success ? "replanned_after_recovery" : "retrying_recovery");
  }

  std::optional<std::pair<double, double>> selectLocalGoal(const nav_msgs::msg::Path & path)
  {
    for (const auto & pose : path.poses) {
      try {
        const auto transformed = tf_buffer_.transform(pose, base_frame_, 50ms);
        const double x = transformed.pose.position.x;
        const double y = transformed.pose.position.y;
        if (std::hypot(x, y) >= local_goal_distance_) {
          return std::make_pair(x, y);
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "path transform unavailable: %s", ex.what());
        return std::nullopt;
      }
    }

    if (!path.poses.empty()) {
      try {
        const auto transformed = tf_buffer_.transform(path.poses.back(), base_frame_, 50ms);
        return std::make_pair(transformed.pose.position.x, transformed.pose.position.y);
      } catch (const tf2::TransformException &) {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  void clearGoalIfReached(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      return;
    }
    try {
      const auto transformed = tf_buffer_.transform(path.poses.back(), base_frame_, 50ms);
      if (std::hypot(transformed.pose.position.x, transformed.pose.position.y) < goal_tolerance_) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          has_active_goal_ = false;
          current_path_.poses.clear();
        }
        RCLCPP_INFO(get_logger(), "goal reached");
        publishState("succeeded", "idle");
        publishState("idle", "idle");
      }
    } catch (const tf2::TransformException &) {
    }
  }

  void clearActiveGoal()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_active_goal_ = false;
    current_path_.poses.clear();
    path_request_in_flight_ = false;
  }

  void publishNavStatus(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    nav_status_pub_->publish(message);
  }

  void publishTargetState(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    target_state_pub_->publish(message);
  }

  void publishState(const std::string & nav_status, const std::string & target_state)
  {
    publishNavStatus(nav_status);
    publishTargetState(target_state);
  }

  std::vector<geometry_msgs::msg::Twist> generateCandidates(const std::pair<double, double> & local_goal) const
  {
    std::vector<geometry_msgs::msg::Twist> candidates;
    const double target_vx = std::clamp(goal_gain_x_ * local_goal.first, -max_vel_x_, max_vel_x_);
    const double target_vy = allow_strafe_ ? std::clamp(goal_gain_y_ * local_goal.second, -max_vel_y_, max_vel_y_) : 0.0;
    const double target_wz = std::clamp(yaw_gain_ * std::atan2(local_goal.second, std::max(0.05, local_goal.first)), -max_vel_theta_, max_vel_theta_);

    auto add = [&](double vx, double vy, double wz) {
      if (!allow_reverse_ && vx < -1e-6) {
        return;
      }
      if (!allow_strafe_) {
        vy = 0.0;
      }
      if (std::hypot(vx, vy) < min_cmd_speed_ && std::abs(wz) < min_cmd_speed_) {
        return;
      }
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::clamp(vx, -max_vel_x_, max_vel_x_);
      cmd.linear.y = std::clamp(vy, -max_vel_y_, max_vel_y_);
      cmd.angular.z = std::clamp(wz, -max_vel_theta_, max_vel_theta_);
      candidates.push_back(cmd);
    };

    add(target_vx, target_vy, target_wz);
    add(target_vx, 0.0, target_wz);
    add(0.7 * target_vx, target_vy, 0.5 * target_wz);
    add(0.5 * target_vx, 0.5 * target_vy, target_wz);
    add(0.0, target_vy, target_wz);
    add(0.0, 0.0, target_wz);
    add(max_vel_x_ * 0.5, 0.0, 0.0);
    add(max_vel_x_ * 0.35, max_vel_y_ * 0.6, 0.0);
    add(max_vel_x_ * 0.35, -max_vel_y_ * 0.6, 0.0);
    add(0.0, max_vel_y_ * 0.8, 0.0);
    add(0.0, -max_vel_y_ * 0.8, 0.0);
    add(-max_vel_x_ * 0.25, 0.0, 0.0);
    add(0.0, 0.0, max_vel_theta_ * 0.6);
    add(0.0, 0.0, -max_vel_theta_ * 0.6);
    return candidates;
  }

  bool isTrajectoryCollisionFree(const geometry_msgs::msg::Twist & cmd, const std::vector<ObstaclePoint> & obstacles) const
  {
    const int steps = std::max(1, static_cast<int>(std::ceil(prediction_time_ / std::max(0.02, simulation_dt_))));
    for (int step = 1; step <= steps; ++step) {
      const double t = step * prediction_time_ / steps;
      if (poseCollides(cmd.linear.x * t, cmd.linear.y * t, cmd.angular.z * t, obstacles)) {
        return false;
      }
    }
    return true;
  }

  bool poseCollides(double pose_x, double pose_y, double pose_yaw, const std::vector<ObstaclePoint> & obstacles) const
  {
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double front = footprint_.front + obstacle_clearance_;
    const double rear = footprint_.rear + obstacle_clearance_;
    const double left = footprint_.left + obstacle_clearance_;
    const double right = footprint_.right + obstacle_clearance_;

    for (const auto & obstacle : obstacles) {
      const double dx = obstacle.x - pose_x;
      const double dy = obstacle.y - pose_y;
      const double local_x = cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;
      if (local_x <= front && local_x >= -rear && local_y <= left && local_y >= -right) {
        return true;
      }
    }
    return false;
  }

  std::string scan_topic_;
  std::string goal_topic_;
  std::string cmd_vel_topic_;
  std::string plan_topic_;
  std::string cancel_service_name_;
  std::string nav_status_topic_;
  std::string target_state_topic_;
  std::string safety_state_topic_;
  std::string compute_path_action_;
  std::string backup_action_;
  std::string wait_action_;
  std::string planner_id_;
  std::string base_frame_;
  Footprint footprint_;
  double local_goal_distance_{};
  double goal_tolerance_{};
  double obstacle_clearance_{};
  double prediction_time_{};
  double simulation_dt_{};
  double control_frequency_{};
  double scan_timeout_{};
  double replan_period_{};
  double max_vel_x_{};
  double max_vel_y_{};
  double max_vel_theta_{};
  double min_cmd_speed_{};
  double goal_gain_x_{};
  double goal_gain_y_{};
  double yaw_gain_{};
  double heading_weight_{};
  double velocity_weight_{};
  double turn_weight_{};
  bool allow_strafe_{};
  bool allow_reverse_{};
  bool stop_on_stale_scan_{};
  bool enable_recovery_{};
  double blocked_timeout_{};
  double recovery_wait_time_{};
  double backup_distance_{};
  double backup_speed_{};
  double backup_time_allowance_{};
  int max_recovery_attempts_{};

  std::mutex mutex_;
  std::vector<ObstaclePoint> obstacles_;
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::PoseStamped active_goal_;
  bool has_active_goal_{false};
  bool has_scan_{false};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time safety_blocked_since_{0, 0, RCL_ROS_TIME};
  bool path_request_in_flight_{false};
  bool recovery_in_progress_{false};
  bool safety_blocked_{false};
  int recovery_attempts_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<Wait>::SharedPtr wait_client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hanmole_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::ScanPathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
