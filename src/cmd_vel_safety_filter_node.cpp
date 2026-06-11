#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace hanmole_navigation
{

struct SafetyFootprint
{
  double front{0.24};
  double rear{0.24};
  double left{0.17};
  double right{0.17};
};

struct SafetyObstaclePoint
{
  double x{0.0};
  double y{0.0};
};

class CmdVelSafetyFilterNode : public rclcpp::Node
{
public:
  CmdVelSafetyFilterNode()
  : Node("cmd_vel_safety_filter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    input_cmd_topic_ = declare_parameter<std::string>("input_cmd_topic", "/cmd_vel_safe");
    recovery_cmd_topic_ = declare_parameter<std::string>("recovery_cmd_topic", "/cmd_vel_recovery");
    output_cmd_topic_ = declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");
    safety_state_topic_ = declare_parameter<std::string>("safety_state_topic", "/cmd_vel_safety_filter/state");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    footprint_.front = declare_parameter<double>("footprint.front", 0.24);
    footprint_.rear = declare_parameter<double>("footprint.rear", 0.24);
    footprint_.left = declare_parameter<double>("footprint.left", 0.17);
    footprint_.right = declare_parameter<double>("footprint.right", 0.17);

    emergency_clearance_ = declare_parameter<double>("emergency_clearance", 0.06);
    prediction_time_ = declare_parameter<double>("prediction_time", 0.6);
    simulation_dt_ = declare_parameter<double>("simulation_dt", 0.08);
    control_frequency_ = declare_parameter<double>("control_frequency", 30.0);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.3);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    recovery_cmd_timeout_ = declare_parameter<double>("recovery_cmd_timeout", 0.3);
    stop_on_stale_scan_ = declare_parameter<bool>("stop_on_stale_scan", true);
    scale_until_safe_ = declare_parameter<bool>("scale_until_safe", true);
    min_scale_ = declare_parameter<double>("min_scale", 0.25);
    max_vel_x_ = declare_parameter<double>("max_vel_x", 0.25);
    max_vel_y_ = declare_parameter<double>("max_vel_y", 0.15);
    max_vel_theta_ = declare_parameter<double>("max_vel_theta", 0.7);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyFilterNode::onScan, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      std::bind(&CmdVelSafetyFilterNode::onCmd, this, std::placeholders::_1));
    recovery_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      recovery_cmd_topic_, 10,
      std::bind(&CmdVelSafetyFilterNode::onRecoveryCmd, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_topic_, 10);
    safety_state_pub_ = create_publisher<std_msgs::msg::String>(safety_state_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelSafetyFilterNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "cmd_vel_safety_filter ready: %s, recovery %s -> %s",
      input_cmd_topic_.c_str(), recovery_cmd_topic_.c_str(), output_cmd_topic_.c_str());
  }

private:
  void onCmd(const geometry_msgs::msg::Twist::SharedPtr cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = clampCommand(*cmd);
    last_cmd_time_ = now();
    has_cmd_ = true;
  }

  void onRecoveryCmd(const geometry_msgs::msg::Twist::SharedPtr cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_recovery_cmd_ = clampCommand(*cmd);
    last_recovery_cmd_time_ = now();
    has_recovery_cmd_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::vector<SafetyObstaclePoint> points;
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

  void onTimer()
  {
    geometry_msgs::msg::Twist cmd;
    geometry_msgs::msg::Twist recovery_cmd;
    std::vector<SafetyObstaclePoint> obstacles;
    bool has_cmd = false;
    bool has_recovery_cmd = false;
    bool has_scan = false;
    rclcpp::Time last_cmd;
    rclcpp::Time last_recovery_cmd;
    rclcpp::Time last_scan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = latest_cmd_;
      recovery_cmd = latest_recovery_cmd_;
      obstacles = obstacles_;
      has_cmd = has_cmd_;
      has_recovery_cmd = has_recovery_cmd_;
      has_scan = has_scan_;
      last_cmd = last_cmd_time_;
      last_recovery_cmd = last_recovery_cmd_time_;
      last_scan = last_scan_time_;
    }

    geometry_msgs::msg::Twist output;
    const auto current_time = now();
    if (has_recovery_cmd && (current_time - last_recovery_cmd).seconds() <= recovery_cmd_timeout_) {
      cmd = recovery_cmd;
      has_cmd = true;
      last_cmd = last_recovery_cmd;
    }
    if (!has_cmd || (now() - last_cmd).seconds() > cmd_timeout_) {
      publishSafetyState("idle");
      publishStamped(output);
      return;
    }
    if (stop_on_stale_scan_ && (!has_scan || (now() - last_scan).seconds() > scan_timeout_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "stale or missing scan, forcing stop");
      publishSafetyState("stale");
      publishStamped(output);
      return;
    }

    if (isTrajectoryCollisionFree(cmd, obstacles)) {
      publishSafetyState("passing");
      publishStamped(cmd);
      return;
    }

    if (scale_until_safe_) {
      for (double scale = 0.75; scale >= min_scale_; scale -= 0.25) {
        const auto scaled = scaleCommand(cmd, scale);
        if (isTrajectoryCollisionFree(scaled, obstacles)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "scaled unsafe cmd_vel by %.2f", scale);
          publishSafetyState("scaled");
          publishStamped(scaled);
          return;
        }
      }
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "blocked unsafe cmd_vel");
    publishSafetyState("blocked");
    publishStamped(output);
  }

  void publishSafetyState(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    safety_state_pub_->publish(message);
  }

  void publishStamped(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = base_frame_;
    stamped.twist = twist;
    cmd_pub_->publish(stamped);
  }
  geometry_msgs::msg::Twist clampCommand(const geometry_msgs::msg::Twist & cmd) const
  {
    geometry_msgs::msg::Twist out = cmd;
    out.linear.x = std::clamp(out.linear.x, -max_vel_x_, max_vel_x_);
    out.linear.y = std::clamp(out.linear.y, -max_vel_y_, max_vel_y_);
    out.angular.z = std::clamp(out.angular.z, -max_vel_theta_, max_vel_theta_);
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    return out;
  }

  geometry_msgs::msg::Twist scaleCommand(const geometry_msgs::msg::Twist & cmd, double scale) const
  {
    geometry_msgs::msg::Twist out = cmd;
    out.linear.x *= scale;
    out.linear.y *= scale;
    out.angular.z *= scale;
    return out;
  }

  bool isTrajectoryCollisionFree(const geometry_msgs::msg::Twist & cmd, const std::vector<SafetyObstaclePoint> & obstacles) const
  {
    if (std::abs(cmd.linear.x) < 1e-5 && std::abs(cmd.linear.y) < 1e-5 && std::abs(cmd.angular.z) < 1e-5) {
      return true;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(prediction_time_ / std::max(0.02, simulation_dt_))));
    for (int step = 1; step <= steps; ++step) {
      const double t = step * prediction_time_ / steps;
      if (poseCollides(cmd.linear.x * t, cmd.linear.y * t, cmd.angular.z * t, obstacles)) {
        return false;
      }
    }
    return true;
  }

  bool poseCollides(double pose_x, double pose_y, double pose_yaw, const std::vector<SafetyObstaclePoint> & obstacles) const
  {
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double front = footprint_.front + emergency_clearance_;
    const double rear = footprint_.rear + emergency_clearance_;
    const double left = footprint_.left + emergency_clearance_;
    const double right = footprint_.right + emergency_clearance_;

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
  std::string input_cmd_topic_;
  std::string recovery_cmd_topic_;
  std::string output_cmd_topic_;
  std::string safety_state_topic_;
  std::string base_frame_;
  SafetyFootprint footprint_;
  double emergency_clearance_{};
  double prediction_time_{};
  double simulation_dt_{};
  double control_frequency_{};
  double scan_timeout_{};
  double cmd_timeout_{};
  double recovery_cmd_timeout_{};
  bool stop_on_stale_scan_{};
  bool scale_until_safe_{};
  double min_scale_{};
  double max_vel_x_{};
  double max_vel_y_{};
  double max_vel_theta_{};

  std::mutex mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  geometry_msgs::msg::Twist latest_recovery_cmd_;
  std::vector<SafetyObstaclePoint> obstacles_;
  bool has_cmd_{false};
  bool has_recovery_cmd_{false};
  bool has_scan_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_recovery_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr recovery_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hanmole_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelSafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}

