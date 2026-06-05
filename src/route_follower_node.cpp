#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hanmole_msgs/action/navigate_to_named_target.hpp>
#include <hanmole_msgs/srv/follow_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RoutePoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RouteTarget
{
  std::size_t index{0};
  RoutePoint pose;
};

struct Route
{
  std::vector<RoutePoint> points;
  std::map<std::string, RouteTarget> targets;
};

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

enum class PrimitiveAction
{
  Rotate,
  MoveX,
  MoveY,
};

struct PrimitiveStep
{
  PrimitiveAction action{PrimitiveAction::Rotate};
  RoutePoint target;
  double distance{0.0};
};

struct PrimitiveCandidate
{
  double cost{std::numeric_limits<double>::infinity()};
  std::vector<PrimitiveStep> steps;
};

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

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double limit_delta(double target, double current, double max_delta)
{
  return current + clamp(target - current, -max_delta, max_delta);
}

std::string json_escape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const char c : input) {
    if (c == '\\' || c == '"') {
      output.push_back('\\');
    }
    output.push_back(c);
  }
  return output;
}
}  // namespace

class RouteFollowerNode : public rclcpp::Node
{
public:
  RouteFollowerNode()
  : Node("route_follower")
  {
    declare_parameters();
    load_parameters();
    load_routes();
    parse_footprint();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&RouteFollowerNode::on_odom, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RouteFollowerNode::on_scan, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    set_origin_srv_ = create_service<std_srvs::srv::Trigger>(
      "/hanmole_navigation/set_origin_here",
      std::bind(
        &RouteFollowerNode::handle_set_origin, this, std::placeholders::_1,
        std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "/hanmole_navigation/cancel_route",
      std::bind(
        &RouteFollowerNode::handle_cancel_route, this, std::placeholders::_1,
        std::placeholders::_2));
    follow_route_srv_ = create_service<hanmole_msgs::srv::FollowRoute>(
      "/hanmole_navigation/follow_route",
      std::bind(
        &RouteFollowerNode::handle_follow_route, this, std::placeholders::_1,
        std::placeholders::_2));

    navigate_action_server_ = rclcpp_action::create_server<NavigateToNamedTarget>(
      this,
      "/hanmole_navigation/navigate_to_target",
      std::bind(
        &RouteFollowerNode::handle_navigate_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&RouteFollowerNode::handle_navigate_cancel, this, std::placeholders::_1),
      std::bind(&RouteFollowerNode::handle_navigate_accepted, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RouteFollowerNode::on_control_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "route_follower ready: routes=%zu odom=%s scan=%s cmd_vel=%s safety_rect=[%.3f, %.3f]",
      routes_.size(), odom_topic_.c_str(), scan_topic_.c_str(), cmd_vel_topic_.c_str(),
      stop_x_half_, stop_y_half_);
  }

private:
  using NavigateToNamedTarget = hanmole_msgs::action::NavigateToNamedTarget;
  using GoalHandleNavigateToNamedTarget = rclcpp_action::ServerGoalHandle<NavigateToNamedTarget>;

  enum class State
  {
    Idle,
    FollowRoute,
    Blocked,
    Succeeded,
    Failed,
    Canceled,
  };

  void declare_parameters()
  {
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("state_topic", "/hanmole_navigation/route_state");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>(
      "routes_file", "/home/hanmole/ros2_ws/src/hanmole_navigation/config/routes.yaml");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<std::string>(
      "footprint", "[[0.24, 0.17], [0.24, -0.17], [-0.24, -0.17], [-0.24, 0.17]]");
    declare_parameter<double>("safety_margin_m", 0.30);
    declare_parameter<int>("safety_min_points", 3);
    declare_parameter<double>("safety_clear_duration_sec", 1.0);
    declare_parameter<double>("scan_timeout_sec", 0.5);
    declare_parameter<double>("final_xy_tolerance_m", 0.03);
    declare_parameter<double>("final_yaw_tolerance_deg", 3.0);
    declare_parameter<double>("max_route_start_distance_m", 0.50);
    declare_parameter<bool>("require_final_yaw", true);
    declare_parameter<double>("max_vx_mps", 0.15);
    declare_parameter<double>("max_vy_mps", 0.12);
    declare_parameter<double>("max_wz_radps", 0.35);
    declare_parameter<double>("max_ax_mps2", 0.30);
    declare_parameter<double>("max_ay_mps2", 0.30);
    declare_parameter<double>("max_awz_radps2", 0.80);
    declare_parameter<bool>("primitive_snap_yaw_to_90deg", true);
    declare_parameter<double>("primitive_yaw_snap_tolerance_deg", 15.0);
    declare_parameter<double>("primitive_rotate_90_cost_m", 0.4);
    declare_parameter<double>("primitive_min_segment_m", 0.03);
    declare_parameter<double>("primitive_max_strafe_m", 0.05);
    declare_parameter<double>("primitive_xy_tolerance_m", 0.03);
    declare_parameter<double>("primitive_yaw_tolerance_deg", 3.0);
    declare_parameter<double>("primitive_move_yaw_gate_deg", 8.0);
    declare_parameter<double>("primitive_kx", 0.9);
    declare_parameter<double>("primitive_ky", 0.55);
    declare_parameter<double>("primitive_trim_max_vy_mps", 0.03);
    declare_parameter<double>("primitive_kyaw_hold", 1.0);
    declare_parameter<double>("primitive_kyaw_rotate", 1.0);
    declare_parameter<std::string>("primitive_target_lateral_biases", "");
  }

  void load_parameters()
  {
    odom_topic_ = get_parameter("odom_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    routes_file_ = get_parameter("routes_file").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    footprint_text_ = get_parameter("footprint").as_string();
    safety_margin_m_ = get_parameter("safety_margin_m").as_double();
    safety_min_points_ = get_parameter("safety_min_points").as_int();
    safety_clear_duration_sec_ = get_parameter("safety_clear_duration_sec").as_double();
    scan_timeout_sec_ = get_parameter("scan_timeout_sec").as_double();
    final_xy_tolerance_m_ = get_parameter("final_xy_tolerance_m").as_double();
    final_yaw_tolerance_rad_ = get_parameter("final_yaw_tolerance_deg").as_double() * M_PI / 180.0;
    max_route_start_distance_m_ = get_parameter("max_route_start_distance_m").as_double();
    require_final_yaw_ = get_parameter("require_final_yaw").as_bool();
    max_vx_mps_ = get_parameter("max_vx_mps").as_double();
    max_vy_mps_ = get_parameter("max_vy_mps").as_double();
    max_wz_radps_ = get_parameter("max_wz_radps").as_double();
    max_ax_mps2_ = get_parameter("max_ax_mps2").as_double();
    max_ay_mps2_ = get_parameter("max_ay_mps2").as_double();
    max_awz_radps2_ = get_parameter("max_awz_radps2").as_double();
    primitive_snap_yaw_to_90deg_ = get_parameter("primitive_snap_yaw_to_90deg").as_bool();
    primitive_yaw_snap_tolerance_rad_ =
      get_parameter("primitive_yaw_snap_tolerance_deg").as_double() * M_PI / 180.0;
    primitive_rotate_90_cost_m_ = get_parameter("primitive_rotate_90_cost_m").as_double();
    primitive_min_segment_m_ = get_parameter("primitive_min_segment_m").as_double();
    primitive_max_strafe_m_ = get_parameter("primitive_max_strafe_m").as_double();
    primitive_xy_tolerance_m_ = get_parameter("primitive_xy_tolerance_m").as_double();
    primitive_yaw_tolerance_rad_ = get_parameter("primitive_yaw_tolerance_deg").as_double() * M_PI / 180.0;
    primitive_move_yaw_gate_rad_ =
      get_parameter("primitive_move_yaw_gate_deg").as_double() * M_PI / 180.0;
    primitive_kx_ = get_parameter("primitive_kx").as_double();
    primitive_ky_ = get_parameter("primitive_ky").as_double();
    primitive_trim_max_vy_mps_ = get_parameter("primitive_trim_max_vy_mps").as_double();
    primitive_kyaw_hold_ = get_parameter("primitive_kyaw_hold").as_double();
    primitive_kyaw_rotate_ = get_parameter("primitive_kyaw_rotate").as_double();
    parse_primitive_target_lateral_biases(
      get_parameter("primitive_target_lateral_biases").as_string());
  }

  void parse_primitive_target_lateral_biases(const std::string & text)
  {
    primitive_target_lateral_biases_.clear();
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
      const auto separator = item.find(':');
      if (separator == std::string::npos) {
        continue;
      }
      const std::string name = trim_copy(item.substr(0, separator));
      const std::string value = trim_copy(item.substr(separator + 1));
      if (name.empty() || value.empty()) {
        continue;
      }
      primitive_target_lateral_biases_[name] = std::stod(value);
    }
  }

  std::string trim_copy(const std::string & text) const
  {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
  }

  void parse_footprint()
  {
    YAML::Node node = YAML::Load(footprint_text_);
    if (!node.IsSequence() || node.size() < 3) {
      throw std::runtime_error("footprint must be a sequence with at least 3 points");
    }

    body_x_half_ = 0.0;
    body_y_half_ = 0.0;
    for (const auto & point : node) {
      if (!point.IsSequence() || point.size() < 2) {
        throw std::runtime_error("each footprint point must be [x, y]");
      }
      body_x_half_ = std::max(body_x_half_, std::abs(point[0].as<double>()));
      body_y_half_ = std::max(body_y_half_, std::abs(point[1].as<double>()));
    }
    stop_x_half_ = body_x_half_ + safety_margin_m_;
    stop_y_half_ = body_y_half_ + safety_margin_m_;
  }

  void load_routes()
  {
    routes_.clear();
    YAML::Node root = YAML::LoadFile(routes_file_);
    YAML::Node routes = root["routes"];
    if (!routes || !routes.IsMap()) {
      throw std::runtime_error("routes.yaml must contain a routes map");
    }

    for (const auto & entry : routes) {
      const std::string name = entry.first.as<std::string>();
      YAML::Node points_node = entry.second["points"];
      if (!points_node || !points_node.IsSequence()) {
        RCLCPP_WARN(get_logger(), "route '%s' has no points sequence, ignored", name.c_str());
        continue;
      }

      Route route;
      for (const auto & point : points_node) {
        RoutePoint route_point;
        route_point.x = point["x"].as<double>();
        route_point.y = point["y"].as<double>();
        route_point.yaw = point["yaw"].as<double>();
        route.points.push_back(route_point);
      }
      if (route.points.size() < 2) {
        RCLCPP_WARN(get_logger(), "route '%s' has fewer than 2 points, ignored", name.c_str());
        continue;
      }

      YAML::Node targets_node = entry.second["targets"];
      if (targets_node && targets_node.IsMap()) {
        for (const auto & target_entry : targets_node) {
          const std::string target_name = target_entry.first.as<std::string>();
          const YAML::Node target_node = target_entry.second;
          if (!target_node["index"]) {
            RCLCPP_WARN(
              get_logger(), "route '%s' target '%s' has no index, ignored", name.c_str(),
              target_name.c_str());
            continue;
          }
          RouteTarget target;
          target.index = target_node["index"].as<std::size_t>();
          if (target.index >= route.points.size()) {
            RCLCPP_WARN(
              get_logger(), "route '%s' target '%s' index %zu out of range, ignored",
              name.c_str(), target_name.c_str(), target.index);
            continue;
          }
          if (target_node["x"] && target_node["y"] && target_node["yaw"]) {
            target.pose.x = target_node["x"].as<double>();
            target.pose.y = target_node["y"].as<double>();
            target.pose.yaw = target_node["yaw"].as<double>();
          } else {
            target.pose = route.points[target.index];
          }
          route.targets[target_name] = target;
        }
      }

      routes_[name] = route;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_odom_pose_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yaw_from_quaternion(msg->pose.pose.orientation)};
    have_odom_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_scan_time_ = now();
    have_scan_ = true;

    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "scan frame is '%s', first version assumes scan points are already in '%s'",
        msg->header.frame_id.c_str(), base_frame_.c_str());
    }

    int blocked_points = 0;
    double angle = msg->angle_min;
    for (const auto range : msg->ranges) {
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
        const double x = static_cast<double>(range) * std::cos(angle);
        const double y = static_cast<double>(range) * std::sin(angle);
        if (point_in_stop_zone(x, y)) {
          ++blocked_points;
          if (blocked_points >= safety_min_points_) {
            break;
          }
        }
      }
      angle += msg->angle_increment;
    }

    const bool blocked_now = blocked_points >= safety_min_points_;
    if (blocked_now) {
      safety_blocked_ = true;
      last_blocked_time_ = last_scan_time_;
      safety_blocked_points_ = blocked_points;
    } else if (safety_blocked_) {
      const double clear_seconds = (last_scan_time_ - last_blocked_time_).seconds();
      if (clear_seconds >= safety_clear_duration_sec_) {
        safety_blocked_ = false;
        safety_blocked_points_ = 0;
      }
    } else {
      safety_blocked_points_ = 0;
    }
  }

  bool point_in_stop_zone(double x, double y) const
  {
    const bool inside_stop_rect = std::abs(x) <= stop_x_half_ && std::abs(y) <= stop_y_half_;
    const bool inside_body_rect = std::abs(x) <= body_x_half_ && std::abs(y) <= body_y_half_;
    return inside_stop_rect && !inside_body_rect;
  }

  void handle_set_origin(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_odom_) {
      response->success = false;
      response->message = "No odometry received yet";
      return;
    }
    origin_pose_ = current_odom_pose_;
    have_origin_ = true;
    response->success = true;
    response->message = "Origin set from current odometry";
  }

  void handle_cancel_route(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_active_route_locked("GOAL_CANCELED: route canceled");
    complete_action_canceled_locked(last_message_);
    response->success = true;
    response->message = last_message_;
  }

  void handle_follow_route(
    const std::shared_ptr<hanmole_msgs::srv::FollowRoute::Request> request,
    std::shared_ptr<hanmole_msgs::srv::FollowRoute::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string message;
    const bool started = start_route_goal_locked(
      request->route_name, request->target_name, true, message);
    response->accepted = started;
    response->success = started;
    response->message = started ?
      (request->wait_result ?
      "Route started; wait_result is not blocking, monitor /hanmole_navigation/route_state" :
      "Route started") :
      message;
  }

  rclcpp_action::GoalResponse handle_navigate_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToNamedTarget::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal || trim_copy(goal->target_name).empty()) {
      RCLCPP_WARN(get_logger(), "Rejected empty navigation target goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_) {
      RCLCPP_WARN(
        get_logger(), "Rejected navigation target '%s': another route is active",
        goal->target_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Accepted navigation target goal: %s", goal->target_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_navigate_cancel(
    const std::shared_ptr<GoalHandleNavigateToNamedTarget> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_handle_ && active_goal_handle_ == goal_handle) {
      cancel_active_route_locked("GOAL_CANCELED: navigation goal canceled");
      complete_action_canceled_locked(last_message_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_navigate_accepted(
    const std::shared_ptr<GoalHandleNavigateToNamedTarget> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto goal = goal_handle->get_goal();
    std::string message;
    if (!start_route_goal_locked(default_route_name_, goal->target_name, goal->reload_routes, message)) {
      auto result = std::make_shared<NavigateToNamedTarget::Result>();
      result->success = false;
      result->status = "GOAL_FAILED";
      result->message = message;
      goal_handle->abort(result);
      return;
    }
    active_goal_handle_ = goal_handle;
    last_message_ = "GOAL_ACCEPTED: target " + goal->target_name;
    publish_state_locked();
    publish_action_feedback_locked();
  }

  bool start_route_goal_locked(
    const std::string & route_name, const std::string & target_name, bool reload_routes,
    std::string & message)
  {
    const std::string resolved_route_name = route_name.empty() ? default_route_name_ : route_name;
    const std::string requested_target_name = trim_copy(target_name);
    std::string resolved_target_name = requested_target_name;
    if (!have_origin_) {
      message = "Origin is not set; call /hanmole_navigation/set_origin_here first";
      return false;
    }
    if (reload_routes) {
      try {
        load_routes();
      } catch (const std::exception & error) {
        message = std::string("Failed to reload routes: ") + error.what();
        return false;
      }
    }

    const auto route_it = routes_.find(resolved_route_name);
    if (route_it == routes_.end()) {
      message = "Route not found: " + resolved_route_name;
      return false;
    }
    if (!have_odom_) {
      message = "No odometry received yet";
      return false;
    }

    const Route & route = route_it->second;
    std::size_t goal_index = route.points.size() - 1;
    RoutePoint goal_pose = route.points[goal_index];
    if (!resolved_target_name.empty()) {
      auto target_it = route.targets.find(resolved_target_name);
      if (target_it == route.targets.end() && requested_target_name.rfind("target_", 0) == 0) {
        const std::string stripped_target_name = requested_target_name.substr(7);
        target_it = route.targets.find(stripped_target_name);
        if (target_it != route.targets.end()) {
          resolved_target_name = stripped_target_name;
        }
      }
      if (target_it == route.targets.end()) {
        message = "Target not found in route '" + resolved_route_name + "': " + requested_target_name;
        return false;
      }
      goal_index = target_it->second.index;
      goal_pose = target_it->second.pose;
    }

    active_route_name_ = resolved_route_name;
    active_target_name_ = resolved_target_name;
    active_route_ = route.points;
    active_goal_index_ = goal_index;
    route_index_ = 0;
    const Pose2D current_origin_pose = odom_to_origin(current_odom_pose_);
    const std::size_t nearest_index = find_nearest_route_index(current_origin_pose);
    const auto & nearest_point = active_route_[nearest_index];
    const double start_distance =
      std::hypot(nearest_point.x - current_origin_pose.x, nearest_point.y - current_origin_pose.y);
    if (start_distance > max_route_start_distance_m_) {
      clear_active_route_locked();
      message = "Current pose is too far from route: " + std::to_string(start_distance) + " m";
      return false;
    }
    active_start_index_ = nearest_index;
    route_index_ = nearest_index;
    primitive_steps_.clear();
    primitive_step_index_ = 0;
    std::string plan_error;
    if (!build_primitive_plan(current_origin_pose, goal_pose, primitive_steps_, plan_error)) {
      clear_active_route_locked();
      message = plan_error;
      return false;
    }
    double snapped_goal_yaw = goal_pose.yaw;
    snap_yaw_to_quadrant(goal_pose.yaw, snapped_goal_yaw);
    active_route_[active_goal_index_].x = goal_pose.x;
    active_route_[active_goal_index_].y = goal_pose.y;
    active_route_[active_goal_index_].yaw = snapped_goal_yaw;
    active_route_[active_goal_index_] =
      append_final_lateral_bias_steps(active_target_name_, active_route_[active_goal_index_], primitive_steps_);
    active_ = true;
    state_ = State::FollowRoute;
    last_result_success_ = false;
    last_message_ = active_target_name_.empty() ?
      "GOAL_EXECUTING: route started " + active_route_name_ :
      "GOAL_EXECUTING: route started " + active_route_name_ + " -> target " + active_target_name_;
    last_message_ += " primitive_steps=" + std::to_string(primitive_steps_.size());
    last_cmd_ = VelocityCommand{};
    last_control_time_ = now();
    message = last_message_;
    publish_state_locked();
    return true;
  }

  void clear_active_route_locked()
  {
    active_route_.clear();
    active_route_name_.clear();
    active_target_name_.clear();
    active_goal_index_ = 0;
    active_start_index_ = 0;
    route_index_ = 0;
    primitive_steps_.clear();
    primitive_step_index_ = 0;
    active_ = false;
  }

  void cancel_active_route_locked(const std::string & message)
  {
    clear_active_route_locked();
    state_ = State::Canceled;
    last_result_success_ = false;
    last_message_ = message;
    publish_stop_locked();
    publish_state_locked();
  }

  void on_control_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      return;
    }

    const auto current_time = now();
    double dt = (current_time - last_control_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / std::max(control_rate_hz_, 1.0);
    }
    last_control_time_ = current_time;

    if (!have_odom_) {
      fail_locked("No odometry received");
      return;
    }
    if (!have_scan_ || (current_time - last_scan_time_).seconds() > scan_timeout_sec_) {
      state_ = State::Blocked;
      last_message_ = "GOAL_BLOCKED: waiting for fresh scan";
      publish_stop_locked();
      publish_state_locked();
      publish_action_feedback_locked();
      return;
    }
    if (safety_blocked_) {
      state_ = State::Blocked;
      last_message_ = "GOAL_BLOCKED: obstacle in footprint safety margin";
      publish_stop_locked();
      publish_state_locked();
      publish_action_feedback_locked();
      return;
    }

    state_ = State::FollowRoute;
    const Pose2D current_origin_pose = odom_to_origin(current_odom_pose_);
    update_primitive_plan(current_origin_pose, dt);
    publish_state_locked();
    publish_action_feedback_locked();
  }

  Pose2D odom_to_origin(const Pose2D & odom_pose) const
  {
    const double dx = odom_pose.x - origin_pose_.x;
    const double dy = odom_pose.y - origin_pose_.y;
    const double theta = origin_pose_.yaw;
    return Pose2D{
      std::cos(theta) * dx + std::sin(theta) * dy,
      -std::sin(theta) * dx + std::cos(theta) * dy,
      normalize_angle(odom_pose.yaw - theta)};
  }

  bool route_finished(const Pose2D & current) const
  {
    if (active_route_.empty()) {
      return true;
    }
    const auto & goal = active_route_[std::min(active_goal_index_, active_route_.size() - 1)];
    const double distance = std::hypot(goal.x - current.x, goal.y - current.y);
    const bool xy_reached = distance <= final_xy_tolerance_m_;
    if (!require_final_yaw_) {
      return xy_reached;
    }
    const double yaw_error = std::abs(normalize_angle(goal.yaw - current.yaw));
    return xy_reached && yaw_error <= final_yaw_tolerance_rad_;
  }

  std::size_t find_nearest_route_index(const Pose2D & current) const
  {
    if (active_route_.empty()) {
      return 0;
    }
    std::size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < active_route_.size(); ++index) {
      const auto & point = active_route_[index];
      const double distance = std::hypot(point.x - current.x, point.y - current.y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    return best_index;
  }

  bool build_primitive_plan(
    const Pose2D & current, const RoutePoint & raw_goal, std::vector<PrimitiveStep> & output,
    std::string & error) const
  {
    double start_yaw = 0.0;
    double goal_yaw = 0.0;
    if (!snap_yaw_to_quadrant(current.yaw, start_yaw)) {
      error = "Current yaw is not close to a 90deg heading";
      return false;
    }
    if (!snap_yaw_to_quadrant(raw_goal.yaw, goal_yaw)) {
      error = "Target yaw is not close to a 90deg heading";
      return false;
    }

    PrimitiveCandidate best;
    const double dx = raw_goal.x - current.x;
    const double dy = raw_goal.y - current.y;
    for (int order = 0; order < 2; ++order) {
      PrimitiveCandidate candidate;
      candidate.cost = 0.0;
      RoutePoint planned{current.x, current.y, start_yaw};
      if (order == 0) {
        append_world_axis_move(dx, 0.0, planned, candidate);
        append_world_axis_move(0.0, dy, planned, candidate);
      } else {
        append_world_axis_move(0.0, dy, planned, candidate);
        append_world_axis_move(dx, 0.0, planned, candidate);
      }
      candidate.cost += append_rotation_steps(planned.yaw, goal_yaw, candidate.steps) *
        primitive_rotate_90_cost_m_;
      if (candidate.cost < best.cost) {
        best = candidate;
      }
    }

    if (!std::isfinite(best.cost)) {
      error = "No primitive plan found";
      return false;
    }
    output = best.steps;
    return true;
  }

  double primitive_lateral_bias_for_target(const std::string & target_name) const
  {
    const auto it = primitive_target_lateral_biases_.find(target_name);
    if (it == primitive_target_lateral_biases_.end()) {
      return 0.0;
    }
    return it->second;
  }

  RoutePoint append_final_lateral_bias_steps(
    const std::string & target_name, const RoutePoint & goal, std::vector<PrimitiveStep> & steps) const
  {
    const double lateral_bias = primitive_lateral_bias_for_target(target_name);
    if (std::abs(lateral_bias) < 1e-4) {
      return goal;
    }
    const double trim_yaw = normalize_angle(goal.yaw + (lateral_bias >= 0.0 ? M_PI_2 : -M_PI_2));
    RoutePoint compensated_goal{
      goal.x + -lateral_bias * std::sin(goal.yaw),
      goal.y + lateral_bias * std::cos(goal.yaw),
      goal.yaw};
    append_rotation_steps(goal.yaw, trim_yaw, steps);
    steps.push_back(PrimitiveStep{
      PrimitiveAction::MoveX,
      RoutePoint{
        compensated_goal.x,
        compensated_goal.y,
        trim_yaw},
      std::abs(lateral_bias)});
    append_rotation_steps(trim_yaw, goal.yaw, steps);
    return compensated_goal;
  }

  void append_world_axis_move(
    double world_dx, double world_dy, RoutePoint & planned, PrimitiveCandidate & candidate) const
  {
    if (std::abs(world_dx) >= primitive_min_segment_m_) {
      const double move_yaw = world_dx >= 0.0 ? 0.0 : M_PI;
      candidate.cost += append_rotation_steps(planned.yaw, move_yaw, candidate.steps) *
        primitive_rotate_90_cost_m_;
      planned.x += world_dx;
      planned.yaw = move_yaw;
      candidate.steps.push_back(PrimitiveStep{PrimitiveAction::MoveX, planned, std::abs(world_dx)});
      candidate.cost += std::abs(world_dx);
      return;
    }

    if (std::abs(world_dy) >= primitive_min_segment_m_) {
      const double move_yaw = world_dy >= 0.0 ? M_PI_2 : -M_PI_2;
      candidate.cost += append_rotation_steps(planned.yaw, move_yaw, candidate.steps) *
        primitive_rotate_90_cost_m_;
      planned.y += world_dy;
      planned.yaw = move_yaw;
      candidate.steps.push_back(PrimitiveStep{PrimitiveAction::MoveX, planned, std::abs(world_dy)});
      candidate.cost += std::abs(world_dy);
    }
  }

  int append_rotation_steps(double from_yaw, double to_yaw, std::vector<PrimitiveStep> & steps) const
  {
    int from_index = quadrant_index(from_yaw);
    const int to_index = quadrant_index(to_yaw);
    int delta = (to_index - from_index + 4) % 4;
    int direction = 1;
    if (delta == 3) {
      delta = 1;
      direction = -1;
    }

    for (int step = 0; step < delta; ++step) {
      from_index = (from_index + direction + 4) % 4;
      steps.push_back(PrimitiveStep{
        PrimitiveAction::Rotate,
        RoutePoint{0.0, 0.0, yaw_from_quadrant(from_index)},
        static_cast<double>(direction) * M_PI_2});
    }
    return delta;
  }

  bool snap_yaw_to_quadrant(double yaw, double & snapped) const
  {
    snapped = yaw_from_quadrant(quadrant_index(yaw));
    const double error = std::abs(normalize_angle(snapped - yaw));
    return primitive_snap_yaw_to_90deg_ && error <= primitive_yaw_snap_tolerance_rad_;
  }

  int quadrant_index(double yaw) const
  {
    int index = static_cast<int>(std::llround(normalize_angle(yaw) / M_PI_2));
    index %= 4;
    if (index < 0) {
      index += 4;
    }
    return index;
  }

  double yaw_from_quadrant(int index) const
  {
    index %= 4;
    if (index < 0) {
      index += 4;
    }
    switch (index) {
      case 0:
        return 0.0;
      case 1:
        return M_PI_2;
      case 2:
        return M_PI;
      default:
        return -M_PI_2;
    }
  }

  void update_primitive_plan(const Pose2D & current, double dt)
  {
    if (primitive_step_index_ >= primitive_steps_.size()) {
      if (route_finished(current)) {
        active_ = false;
        state_ = State::Succeeded;
        last_result_success_ = true;
        last_message_ = "GOAL_SUCCEEDED: target reached " + active_target_name_;
        publish_stop_locked();
        complete_action_succeeded_locked(last_message_);
      } else {
        const RoutePoint goal = active_route_[std::min(active_goal_index_, active_route_.size() - 1)];
        const double yaw_error = normalize_angle(goal.yaw - current.yaw);
        if (std::abs(yaw_error) > primitive_yaw_tolerance_rad_) {
          VelocityCommand cmd{0.0, 0.0, primitive_kyaw_rotate_ * yaw_error};
          cmd = limit_command(cmd);
          cmd = apply_accel_limit(cmd, dt);
          publish_cmd_locked(cmd);
          last_message_ = "GOAL_EXECUTING: final yaw align";
          return;
        }

        fail_locked("Primitive plan ended outside final tolerance");
      }
      return;
    }

    route_index_ = primitive_step_index_;
    const PrimitiveStep & step = primitive_steps_[primitive_step_index_];
    if (primitive_step_reached(current, step)) {
      ++primitive_step_index_;
      publish_stop_locked();
      last_message_ = "GOAL_EXECUTING: step reached " + std::to_string(primitive_step_index_) + "/" +
        std::to_string(primitive_steps_.size());
      return;
    }

    VelocityCommand cmd = compute_primitive_command(current, step);
    cmd = limit_command(cmd);
    cmd = apply_accel_limit(cmd, dt);
    publish_cmd_locked(cmd);
    last_message_ = "GOAL_EXECUTING: step " + std::to_string(primitive_step_index_ + 1) + "/" +
      std::to_string(primitive_steps_.size()) + " " + primitive_action_to_string(step.action);
  }

  bool primitive_step_reached(const Pose2D & current, const PrimitiveStep & step) const
  {
    const double yaw_error = std::abs(normalize_angle(step.target.yaw - current.yaw));
    if (step.action == PrimitiveAction::Rotate) {
      return yaw_error <= primitive_yaw_tolerance_rad_;
    }
    const double dx = step.target.x - current.x;
    const double dy = step.target.y - current.y;
    if (step.action == PrimitiveAction::MoveX) {
      const double along_error = std::cos(step.target.yaw) * dx + std::sin(step.target.yaw) * dy;
      return std::abs(along_error) <= primitive_xy_tolerance_m_ &&
        yaw_error <= primitive_move_yaw_gate_rad_;
    }
    const double lateral_error = -std::sin(step.target.yaw) * dx + std::cos(step.target.yaw) * dy;
    return std::abs(lateral_error) <= primitive_xy_tolerance_m_ &&
      yaw_error <= primitive_move_yaw_gate_rad_;
  }

  VelocityCommand compute_primitive_command(const Pose2D & current, const PrimitiveStep & step) const
  {
    const double yaw_error = normalize_angle(step.target.yaw - current.yaw);
    if (step.action == PrimitiveAction::Rotate ||
      std::abs(yaw_error) > primitive_move_yaw_gate_rad_)
    {
      return VelocityCommand{0.0, 0.0, primitive_kyaw_rotate_ * yaw_error};
    }

    const double dx = step.target.x - current.x;
    const double dy = step.target.y - current.y;
    if (step.action == PrimitiveAction::MoveX) {
      const double along_error = std::cos(step.target.yaw) * dx + std::sin(step.target.yaw) * dy;
      return VelocityCommand{primitive_kx_ * along_error, 0.0, primitive_kyaw_hold_ * yaw_error};
    }

    const double lateral_error = -std::sin(step.target.yaw) * dx + std::cos(step.target.yaw) * dy;
    return VelocityCommand{0.0, primitive_ky_ * lateral_error, primitive_kyaw_hold_ * yaw_error};
  }

  VelocityCommand limit_primitive_trim_command(VelocityCommand cmd) const
  {
    cmd.vx = 0.0;
    cmd.vy = clamp(cmd.vy, -primitive_trim_max_vy_mps_, primitive_trim_max_vy_mps_);
    cmd.wz = clamp(cmd.wz, -max_wz_radps_, max_wz_radps_);
    return cmd;
  }

  const char * primitive_action_to_string(PrimitiveAction action) const
  {
    switch (action) {
      case PrimitiveAction::Rotate:
        return "ROTATE_90";
      case PrimitiveAction::MoveX:
        return "MOVE_X";
      case PrimitiveAction::MoveY:
        return "MOVE_Y";
    }
    return "UNKNOWN";
  }

  VelocityCommand limit_command(VelocityCommand cmd) const
  {
    cmd.vx = clamp(cmd.vx, -max_vx_mps_, max_vx_mps_);
    cmd.vy = clamp(cmd.vy, -max_vy_mps_, max_vy_mps_);
    cmd.wz = clamp(cmd.wz, -max_wz_radps_, max_wz_radps_);
    return cmd;
  }

  VelocityCommand apply_accel_limit(VelocityCommand cmd, double dt)
  {
    cmd.vx = limit_delta(cmd.vx, last_cmd_.vx, max_ax_mps2_ * dt);
    cmd.vy = limit_delta(cmd.vy, last_cmd_.vy, max_ay_mps2_ * dt);
    cmd.wz = limit_delta(cmd.wz, last_cmd_.wz, max_awz_radps2_ * dt);
    return cmd;
  }

  void publish_cmd_locked(const VelocityCommand & cmd)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.twist.linear.x = cmd.vx;
    msg.twist.linear.y = cmd.vy;
    msg.twist.angular.z = cmd.wz;
    cmd_pub_->publish(msg);
    last_cmd_ = cmd;
  }

  void publish_stop_locked()
  {
    publish_cmd_locked(VelocityCommand{});
  }

  void publish_action_feedback_locked()
  {
    if (!active_goal_handle_) {
      return;
    }
    auto feedback = std::make_shared<NavigateToNamedTarget::Feedback>();
    feedback->status = state_to_string(state_);
    feedback->message = last_message_;
    feedback->current_step = static_cast<std::uint32_t>(primitive_step_index_);
    feedback->total_steps = static_cast<std::uint32_t>(primitive_steps_.size());
    feedback->distance_to_goal = static_cast<float>(distance_to_goal_locked());
    feedback->blocked = state_ == State::Blocked || safety_blocked_;
    active_goal_handle_->publish_feedback(feedback);
  }

  void complete_action_succeeded_locked(const std::string & message)
  {
    if (!active_goal_handle_) {
      return;
    }
    auto result = std::make_shared<NavigateToNamedTarget::Result>();
    result->success = true;
    result->status = "GOAL_SUCCEEDED";
    result->message = message;
    active_goal_handle_->succeed(result);
    active_goal_handle_.reset();
  }

  void complete_action_failed_locked(const std::string & message)
  {
    if (!active_goal_handle_) {
      return;
    }
    auto result = std::make_shared<NavigateToNamedTarget::Result>();
    result->success = false;
    result->status = "GOAL_FAILED";
    result->message = "GOAL_FAILED: " + message;
    active_goal_handle_->abort(result);
    active_goal_handle_.reset();
  }

  void complete_action_canceled_locked(const std::string & message)
  {
    if (!active_goal_handle_) {
      return;
    }
    auto result = std::make_shared<NavigateToNamedTarget::Result>();
    result->success = false;
    result->status = "GOAL_CANCELED";
    result->message = message;
    active_goal_handle_->canceled(result);
    active_goal_handle_.reset();
  }

  void publish_state_locked()
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"route\":\"" << json_escape(active_route_name_) << "\",";
    stream << "\"target\":\"" << json_escape(active_target_name_) << "\",";
    stream << "\"state\":\"" << state_to_string(state_) << "\",";
    stream << "\"mode\":\"primitive_plan\",";
    stream << "\"index\":" << route_index_ << ",";
    stream << "\"start_index\":" << active_start_index_ << ",";
    stream << "\"goal_index\":" << active_goal_index_ << ",";
    stream << "\"primitive_step\":" << primitive_step_index_ << ",";
    stream << "\"primitive_total\":" << primitive_steps_.size() << ",";
    stream << "\"distance_to_goal\":" << distance_to_goal_locked() << ",";
    stream << "\"total\":" << active_route_.size() << ",";
    stream << "\"blocked\":" << (safety_blocked_ ? "true" : "false") << ",";
    stream << "\"blocked_points\":" << safety_blocked_points_ << ",";
    stream << "\"message\":\"" << json_escape(last_message_) << "\"}";
    msg.data = stream.str();
    state_pub_->publish(msg);
  }

  void fail_locked(const std::string & message)
  {
    active_ = false;
    state_ = State::Failed;
    last_result_success_ = false;
    last_message_ = message;
    publish_stop_locked();
    publish_state_locked();
    publish_action_feedback_locked();
    complete_action_failed_locked(message);
  }

  double distance_to_goal_locked() const
  {
    if (active_route_.empty() || !have_odom_) {
      return 0.0;
    }
    const Pose2D current_origin_pose = odom_to_origin(current_odom_pose_);
    const auto & goal = active_route_[std::min(active_goal_index_, active_route_.size() - 1)];
    return std::hypot(goal.x - current_origin_pose.x, goal.y - current_origin_pose.y);
  }

  const char * state_to_string(State state) const
  {
    switch (state) {
      case State::Idle:
        return "IDLE";
      case State::FollowRoute:
        return "GOAL_EXECUTING";
      case State::Blocked:
        return "GOAL_BLOCKED";
      case State::Succeeded:
        return "GOAL_SUCCEEDED";
      case State::Failed:
        return "GOAL_FAILED";
      case State::Canceled:
        return "GOAL_CANCELED";
    }
    return "UNKNOWN";
  }

  std::mutex mutex_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string state_topic_;
  std::string base_frame_;
  std::string routes_file_;
  std::string default_route_name_{"main_route"};
  std::string footprint_text_;

  double control_rate_hz_{20.0};
  double safety_margin_m_{0.30};
  int safety_min_points_{3};
  double safety_clear_duration_sec_{1.0};
  double scan_timeout_sec_{0.5};
  double final_xy_tolerance_m_{0.03};
  double final_yaw_tolerance_rad_{3.0 * M_PI / 180.0};
  double max_route_start_distance_m_{0.50};
  bool require_final_yaw_{true};
  double max_vx_mps_{0.15};
  double max_vy_mps_{0.12};
  double max_wz_radps_{0.35};
  double max_ax_mps2_{0.30};
  double max_ay_mps2_{0.30};
  double max_awz_radps2_{0.80};
  bool primitive_snap_yaw_to_90deg_{true};
  double primitive_yaw_snap_tolerance_rad_{15.0 * M_PI / 180.0};
  double primitive_rotate_90_cost_m_{0.4};
  double primitive_min_segment_m_{0.03};
  double primitive_max_strafe_m_{0.05};
  double primitive_xy_tolerance_m_{0.03};
  double primitive_yaw_tolerance_rad_{3.0 * M_PI / 180.0};
  double primitive_move_yaw_gate_rad_{8.0 * M_PI / 180.0};
  double primitive_kx_{0.9};
  double primitive_ky_{0.55};
  double primitive_trim_max_vy_mps_{0.03};
  double primitive_kyaw_hold_{1.0};
  double primitive_kyaw_rotate_{1.0};
  std::map<std::string, double> primitive_target_lateral_biases_;

  double body_x_half_{0.24};
  double body_y_half_{0.17};
  double stop_x_half_{0.54};
  double stop_y_half_{0.47};

  std::map<std::string, Route> routes_;
  std::vector<RoutePoint> active_route_;
  std::string active_route_name_;
  std::string active_target_name_;
  std::size_t route_index_{0};
  std::size_t active_start_index_{0};
  std::size_t active_goal_index_{0};
  std::vector<PrimitiveStep> primitive_steps_;
  std::size_t primitive_step_index_{0};
  bool active_{false};
  bool last_result_success_{false};
  std::string last_message_{"Idle"};
  State state_{State::Idle};

  Pose2D current_odom_pose_;
  Pose2D origin_pose_;
  bool have_odom_{false};
  bool have_origin_{false};
  bool have_scan_{false};
  bool safety_blocked_{false};
  int safety_blocked_points_{0};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_blocked_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  VelocityCommand last_cmd_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_origin_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<hanmole_msgs::srv::FollowRoute>::SharedPtr follow_route_srv_;
  rclcpp_action::Server<NavigateToNamedTarget>::SharedPtr navigate_action_server_;
  std::shared_ptr<GoalHandleNavigateToNamedTarget> active_goal_handle_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RouteFollowerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route_follower"), "route_follower failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
