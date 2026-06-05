#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <hanmole_msgs/srv/mark_route_target.hpp>
#include <hanmole_msgs/srv/record_route.hpp>
#include <hanmole_msgs/srv/save_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
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

struct TargetMarker
{
  std::size_t index{0};
  Pose2D pose;
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

class RouteRecorderNode : public rclcpp::Node
{
public:
  RouteRecorderNode()
  : Node("route_recorder")
  {
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("routes_file", "/home/hanmole/ros2_ws/src/hanmole_navigation/config/routes.yaml");
    declare_parameter<std::string>("state_topic", "/hanmole_navigation/route_record_state");
    declare_parameter<double>("record_spacing_m", 0.05);
    declare_parameter<double>("record_yaw_spacing_deg", 3.0);

    odom_topic_ = get_parameter("odom_topic").as_string();
    routes_file_ = get_parameter("routes_file").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    record_spacing_m_ = get_parameter("record_spacing_m").as_double();
    record_yaw_spacing_rad_ = get_parameter("record_yaw_spacing_deg").as_double() * M_PI / 180.0;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&RouteRecorderNode::on_odom, this, std::placeholders::_1));
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    set_origin_srv_ = create_service<std_srvs::srv::Trigger>(
      "/hanmole_navigation/route_recorder/set_origin_here",
      std::bind(&RouteRecorderNode::handle_set_origin, this, std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "/hanmole_navigation/route_recorder/cancel_record",
      std::bind(&RouteRecorderNode::handle_cancel, this, std::placeholders::_1, std::placeholders::_2));
    start_record_srv_ = create_service<hanmole_msgs::srv::RecordRoute>(
      "/hanmole_navigation/route_recorder/start_record",
      std::bind(&RouteRecorderNode::handle_start_record, this, std::placeholders::_1, std::placeholders::_2));
    save_route_srv_ = create_service<hanmole_msgs::srv::SaveRoute>(
      "/hanmole_navigation/route_recorder/save_route",
      std::bind(&RouteRecorderNode::handle_save_route, this, std::placeholders::_1, std::placeholders::_2));
    mark_target_srv_ = create_service<hanmole_msgs::srv::MarkRouteTarget>(
      "/hanmole_navigation/route_recorder/mark_target",
      std::bind(&RouteRecorderNode::handle_mark_target, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "route_recorder ready: odom=%s routes_file=%s spacing=%.3fm yaw=%.2fdeg",
      odom_topic_.c_str(), routes_file_.c_str(), record_spacing_m_, record_yaw_spacing_rad_ * 180.0 / M_PI);
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_odom_pose_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yaw_from_quaternion(msg->pose.pose.orientation)};
    have_odom_ = true;

    if (!recording_ || !have_origin_) {
      return;
    }

    const Pose2D current = odom_to_origin(current_odom_pose_);
    if (recorded_points_.empty() || should_append_point(current, recorded_points_.back())) {
      recorded_points_.push_back(current);
      publish_state_locked("RECORDING", "point appended");
    }
  }

  bool should_append_point(const Pose2D & current, const Pose2D & last) const
  {
    const double distance = std::hypot(current.x - last.x, current.y - last.y);
    const double yaw_delta = std::abs(normalize_angle(current.yaw - last.yaw));
    return distance >= record_spacing_m_ || yaw_delta >= record_yaw_spacing_rad_;
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
    response->message = "Recorder origin set from current odometry";
    publish_state_locked("IDLE", response->message);
  }

  void handle_start_record(
    const std::shared_ptr<hanmole_msgs::srv::RecordRoute::Request> request,
    std::shared_ptr<hanmole_msgs::srv::RecordRoute::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->route_name.empty()) {
      response->success = false;
      response->message = "route_name is empty";
      return;
    }
    if (!have_origin_) {
      response->success = false;
      response->message = "Origin is not set; call /hanmole_navigation/route_recorder/set_origin_here first";
      return;
    }
    if (!have_odom_) {
      response->success = false;
      response->message = "No odometry received yet";
      return;
    }

    active_route_name_ = request->route_name;
    recorded_points_.clear();
    recorded_targets_.clear();
    recorded_points_.push_back(odom_to_origin(current_odom_pose_));
    recording_ = true;

    response->success = true;
    response->message = "Recording route: " + active_route_name_;
    publish_state_locked("RECORDING", response->message);
  }

  void handle_save_route(
    const std::shared_ptr<hanmole_msgs::srv::SaveRoute::Request> request,
    std::shared_ptr<hanmole_msgs::srv::SaveRoute::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string route_name = request->route_name.empty() ? active_route_name_ : request->route_name;
    if (route_name.empty()) {
      response->success = false;
      response->message = "route_name is empty and no active route exists";
      return;
    }
    if (recorded_points_.size() < 2) {
      response->success = false;
      response->message = "recorded route has fewer than 2 points";
      return;
    }

    try {
      write_route(route_name);
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("failed to save route: ") + error.what();
      return;
    }

    recording_ = false;
    active_route_name_ = route_name;
    response->success = true;
    response->message = "Saved route '" + route_name + "' with " + std::to_string(recorded_points_.size()) + " points";
    publish_state_locked("SAVED", response->message);
  }

  void handle_mark_target(
    const std::shared_ptr<hanmole_msgs::srv::MarkRouteTarget::Request> request,
    std::shared_ptr<hanmole_msgs::srv::MarkRouteTarget::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->target_name.empty()) {
      response->success = false;
      response->message = "target_name is empty";
      return;
    }
    if (!recording_) {
      response->success = false;
      response->message = "not recording; call start_record first";
      return;
    }
    if (!have_origin_ || !have_odom_) {
      response->success = false;
      response->message = "origin or odometry is not ready";
      return;
    }

    const std::size_t index = append_current_point_locked(true);
    recorded_targets_[request->target_name] = TargetMarker{index, recorded_points_[index]};

    response->success = true;
    response->message = "Marked target '" + request->target_name + "' at index " + std::to_string(index);
    publish_state_locked("TARGET_MARKED", response->message);
  }

  void handle_cancel(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recording_ = false;
    recorded_points_.clear();
    recorded_targets_.clear();
    active_route_name_.clear();
    response->success = true;
    response->message = "Recording canceled";
    publish_state_locked("CANCELED", response->message);
  }

  void write_route(const std::string & route_name) const
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(routes_file_);
    } catch (const std::exception &) {
      root = YAML::Node(YAML::NodeType::Map);
    }
    root["version"] = 2;
    root["frame"] = "origin";
    if (!root["routes"] || !root["routes"].IsMap()) {
      root["routes"] = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node route_node;
    route_node["description"] = "recorded route " + route_name;
    YAML::Node points_node(YAML::NodeType::Sequence);
    for (const auto & point : recorded_points_) {
      YAML::Node point_node;
      point_node["x"] = point.x;
      point_node["y"] = point.y;
      point_node["yaw"] = point.yaw;
      points_node.push_back(point_node);
    }
    route_node["points"] = points_node;

    if (!recorded_targets_.empty()) {
      YAML::Node targets_node(YAML::NodeType::Map);
      for (const auto & target_entry : recorded_targets_) {
        YAML::Node target_node;
        target_node["index"] = static_cast<int>(target_entry.second.index);
        target_node["x"] = target_entry.second.pose.x;
        target_node["y"] = target_entry.second.pose.y;
        target_node["yaw"] = target_entry.second.pose.yaw;
        targets_node[target_entry.first] = target_node;
      }
      route_node["targets"] = targets_node;
    }
    root["routes"][route_name] = route_node;

    YAML::Emitter out;
    out << root;
    if (!out.good()) {
      throw std::runtime_error(out.GetLastError());
    }

    std::ofstream file(routes_file_, std::ios::trunc);
    if (!file) {
      throw std::runtime_error("cannot open routes file: " + routes_file_);
    }
    file << out.c_str() << '\n';
  }

  std::size_t append_current_point_locked(bool force)
  {
    const Pose2D current = odom_to_origin(current_odom_pose_);
    if (recorded_points_.empty()) {
      recorded_points_.push_back(current);
      return 0;
    }
    const Pose2D & last = recorded_points_.back();
    const bool same_pose =
      std::hypot(current.x - last.x, current.y - last.y) < 1e-4 &&
      std::abs(normalize_angle(current.yaw - last.yaw)) < 1e-4;
    if ((force && !same_pose) || should_append_point(current, last)) {
      recorded_points_.push_back(current);
    }
    return recorded_points_.size() - 1;
  }

  void publish_state_locked(const std::string & state, const std::string & message)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"route\":\"" << json_escape(active_route_name_) << "\",";
    stream << "\"state\":\"" << json_escape(state) << "\",";
    stream << "\"recording\":" << (recording_ ? "true" : "false") << ",";
    stream << "\"points\":" << recorded_points_.size() << ",";
    stream << "\"targets\":" << recorded_targets_.size() << ",";
    stream << "\"message\":\"" << json_escape(message) << "\"}";
    msg.data = stream.str();
    state_pub_->publish(msg);
  }

  std::mutex mutex_;
  std::string odom_topic_;
  std::string routes_file_;
  std::string state_topic_;
  double record_spacing_m_{0.05};
  double record_yaw_spacing_rad_{3.0 * M_PI / 180.0};

  Pose2D current_odom_pose_;
  Pose2D origin_pose_;
  bool have_odom_{false};
  bool have_origin_{false};
  bool recording_{false};
  std::string active_route_name_;
  std::vector<Pose2D> recorded_points_;
  std::map<std::string, TargetMarker> recorded_targets_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_origin_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<hanmole_msgs::srv::RecordRoute>::SharedPtr start_record_srv_;
  rclcpp::Service<hanmole_msgs::srv::SaveRoute>::SharedPtr save_route_srv_;
  rclcpp::Service<hanmole_msgs::srv::MarkRouteTarget>::SharedPtr mark_target_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RouteRecorderNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route_recorder"), "route_recorder failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
