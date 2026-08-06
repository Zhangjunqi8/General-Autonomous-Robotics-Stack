#include "hanmole_navigation/cmd_vel_nav_accelerator_mux.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace hanmole_navigation
{
namespace
{

constexpr std::size_t kLinearX = 0;
constexpr std::size_t kLinearY = 1;
constexpr std::size_t kAngularZ = 2;

double signOf(double value)
{
  return value < 0.0 ? -1.0 : 1.0;
}

std::vector<double> declareVectorParameter(
  rclcpp::Node & node,
  const std::string & name,
  const std::vector<double> & default_value)
{
  auto value = node.declare_parameter<std::vector<double>>(name, default_value);
  if (value.size() != 3) {
    throw std::runtime_error(name + " must contain exactly 3 values for x, y, and yaw");
  }
  return value;
}

}  // namespace

double applyDeadband(double value, double deadband)
{
  return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}

double clampVelocity(double value, double min_velocity, double max_velocity)
{
  return std::clamp(value, min_velocity, max_velocity);
}

double boostAxis(double value, const AxisTransformConfig & config)
{
  const double filtered = applyDeadband(value, config.deadband);
  if (filtered == 0.0) {
    return 0.0;
  }

  double boosted = filtered * config.gain;
  if (config.min_boost > 0.0 && std::abs(boosted) < config.min_boost) {
    boosted = signOf(boosted) * config.min_boost;
  }
  return clampVelocity(boosted, config.min_velocity, config.max_velocity);
}

double passthroughAxis(double value, const AxisTransformConfig & config)
{
  return clampVelocity(applyDeadband(value, config.deadband), config.min_velocity,
      config.max_velocity);
}

double rateLimitAxis(
  double current,
  double target,
  const AxisRateLimitConfig & config,
  double dt)
{
  if (dt <= 0.0) {
    return current;
  }

  const double delta = target - current;
  if (delta == 0.0) {
    return target;
  }

  const double max_step =
    delta > 0.0 ? std::abs(config.max_accel) * dt : std::abs(config.max_decel) * dt;
  if (std::abs(delta) <= max_step) {
    return target;
  }
  return current + signOf(delta) * max_step;
}

geometry_msgs::msg::Twist transformTwist(
  const geometry_msgs::msg::Twist & input,
  CmdVelInputMode mode,
  const std::array<AxisTransformConfig, 3> & transform_config)
{
  geometry_msgs::msg::Twist output;
  if (mode == CmdVelInputMode::kStop) {
    return output;
  }

  const auto transform_axis = [mode](double value, const AxisTransformConfig & config) {
      if (mode == CmdVelInputMode::kBoost) {
        return boostAxis(value, config);
      }
      return passthroughAxis(value, config);
    };

  output.linear.x = transform_axis(input.linear.x, transform_config[kLinearX]);
  output.linear.y = transform_axis(input.linear.y, transform_config[kLinearY]);
  output.angular.z = transform_axis(input.angular.z, transform_config[kAngularZ]);
  return output;
}

geometry_msgs::msg::Twist rateLimitTwist(
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target,
  const std::array<AxisRateLimitConfig, 3> & rate_limit_config,
  double dt)
{
  geometry_msgs::msg::Twist output;
  output.linear.x = rateLimitAxis(current.linear.x, target.linear.x, rate_limit_config[kLinearX],
      dt);
  output.linear.y = rateLimitAxis(current.linear.y, target.linear.y, rate_limit_config[kLinearY],
      dt);
  output.angular.z = rateLimitAxis(
    current.angular.z,
    target.angular.z,
    rate_limit_config[kAngularZ],
    dt);
  return output;
}

bool twistIsNearZero(const geometry_msgs::msg::Twist & twist, double epsilon)
{
  return std::abs(twist.linear.x) <= epsilon &&
         std::abs(twist.linear.y) <= epsilon &&
         std::abs(twist.angular.z) <= epsilon;
}

CmdVelInputMode parseInputMode(const std::string & mode)
{
  if (mode == "boost") {
    return CmdVelInputMode::kBoost;
  }
  if (mode == "passthrough") {
    return CmdVelInputMode::kPassthrough;
  }
  if (mode == "stop") {
    return CmdVelInputMode::kStop;
  }
  throw std::runtime_error("Unsupported cmd_vel input mode: " + mode);
}

CmdVelNavAcceleratorMuxNode::CmdVelNavAcceleratorMuxNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_nav_accelerator_mux", options)
{
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
  feedback_ = declare_parameter<std::string>("feedback", "CLOSED_LOOP");
  odom_duration_ = declare_parameter<double>("odom_duration", 0.1);
  publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
  if (publish_rate_ <= 0.0) {
    throw std::runtime_error("publish_rate must be positive");
  }

  const auto input_names = declare_parameter<std::vector<std::string>>("input_names", {"nav"});
  const auto max_velocity = declareVectorParameter(*this, "max_velocity", {1.2, 0.28, 1.4});
  const auto min_velocity = declareVectorParameter(*this, "min_velocity", {-0.15, -0.28, -1.0});
  const auto max_accel = declareVectorParameter(*this, "max_accel", {2.5, 0.3, 1.0});
  const auto max_decel = declareVectorParameter(*this, "max_decel", {-4.0, -0.6, -2.0});
  const auto deadband_velocity = declareVectorParameter(*this, "deadband_velocity",
      {0.0, 0.01, 0.001});
  const auto min_boost_velocity = declareVectorParameter(*this, "min_boost_velocity",
      {0.25, 0.0, 0.0});

  const double linear_gain = declare_parameter<double>("linear_gain", 1.5);
  const double lateral_gain = declare_parameter<double>("lateral_gain", 1.0);
  const double angular_gain = declare_parameter<double>("angular_gain", 1.0);
  const std::string output_topic = declare_parameter<std::string>("output_topic",
      "cmd_vel_smoothed");
  const std::string active_topic = declare_parameter<std::string>(
    "active_topic",
    "cmd_vel_nav_accelerator_mux/active");

  transform_config_ = {
    AxisTransformConfig{
      min_velocity[kLinearX],
      max_velocity[kLinearX],
      linear_gain,
      min_boost_velocity[kLinearX],
      deadband_velocity[kLinearX],
    },
    AxisTransformConfig{
      min_velocity[kLinearY],
      max_velocity[kLinearY],
      lateral_gain,
      min_boost_velocity[kLinearY],
      deadband_velocity[kLinearY],
    },
    AxisTransformConfig{
      min_velocity[kAngularZ],
      max_velocity[kAngularZ],
      angular_gain,
      min_boost_velocity[kAngularZ],
      deadband_velocity[kAngularZ],
    },
  };
  rate_limit_config_ = {
    AxisRateLimitConfig{max_accel[kLinearX], max_decel[kLinearX]},
    AxisRateLimitConfig{max_accel[kLinearY], max_decel[kLinearY]},
    AxisRateLimitConfig{max_accel[kAngularZ], max_decel[kAngularZ]},
  };

  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
  active_pub_ = create_publisher<std_msgs::msg::String>(
    active_topic,
    rclcpp::QoS(1).transient_local());
  publishActiveName(active_name_);

  if (feedback_ == "CLOSED_LOOP") {
    const std::string odom_topic = declare_parameter<std::string>("odom_topic",
        "/odometry/filtered");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      10,
      std::bind(&CmdVelNavAcceleratorMuxNode::odomCallback, this, std::placeholders::_1));
  } else if (feedback_ != "OPEN_LOOP") {
    throw std::runtime_error("feedback must be CLOSED_LOOP or OPEN_LOOP");
  }

  configureInputs(input_names);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  publish_timer_ = create_wall_timer(
    period,
    std::bind(&CmdVelNavAcceleratorMuxNode::publishTimerCallback, this));
}

void CmdVelNavAcceleratorMuxNode::configureInputs(const std::vector<std::string> & input_names)
{
  if (input_names.empty()) {
    throw std::runtime_error("input_names must contain at least one cmd_vel source");
  }

  inputs_.clear();
  inputs_.reserve(input_names.size());

  for (const auto & name : input_names) {
    if (name.empty()) {
      throw std::runtime_error("input_names must not contain an empty name");
    }

    InputSource source;
    source.name = name;
    source.topic = declare_parameter<std::string>("inputs." + name + ".topic", "cmd_vel_nav");
    source.timeout = declare_parameter<double>("inputs." + name + ".timeout", 0.2);
    source.priority = declare_parameter<int>("inputs." + name + ".priority", 10);
    source.mode = parseInputMode(declare_parameter<std::string>("inputs." + name + ".mode",
        "boost"));
    if (source.timeout <= 0.0) {
      throw std::runtime_error("inputs." + name + ".timeout must be positive");
    }
    inputs_.push_back(std::move(source));
  }

  for (std::size_t index = 0; index < inputs_.size(); ++index) {
    inputs_[index].subscription = create_subscription<geometry_msgs::msg::TwistStamped>(
      inputs_[index].topic,
      10,
      [this, index](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        inputCallback(index, msg);
      });
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel source '%s' on '%s' priority=%d timeout=%.3fs",
      inputs_[index].name.c_str(),
      inputs_[index].topic.c_str(),
      inputs_[index].priority,
      inputs_[index].timeout);
  }
}

void CmdVelNavAcceleratorMuxNode::inputCallback(
  std::size_t input_index,
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (input_index >= inputs_.size()) {
    return;
  }

  inputs_[input_index].last_msg = *msg;
  inputs_[input_index].last_time = get_clock()->now();
}

void CmdVelNavAcceleratorMuxNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_twist_ = msg->twist.twist;
  latest_odom_time_ = get_clock()->now();
  has_latest_odom_ = true;
}

void CmdVelNavAcceleratorMuxNode::publishTimerCallback()
{
  const auto now = get_clock()->now();
  const InputSource * active_input = selectActiveInput(now);
  const std::string next_active_name = active_input == nullptr ? "idle" : active_input->name;
  publishActiveName(next_active_name);

  geometry_msgs::msg::Twist target;
  if (active_input != nullptr && active_input->last_msg.has_value()) {
    target = transformTwist(active_input->last_msg->twist, active_input->mode, transform_config_);
  }

  double dt = 1.0 / publish_rate_;
  if (last_publish_time_.nanoseconds() > 0) {
    dt = std::clamp((now - last_publish_time_).seconds(), 0.0, 1.0);
  }
  last_publish_time_ = now;

  const auto current = currentVelocityForRateLimit(now);
  current_output_ = rateLimitTwist(current, target, rate_limit_config_, dt);
  has_output_state_ = true;

  if (active_input == nullptr && twistIsNearZero(current_output_)) {
    return;
  }

  geometry_msgs::msg::TwistStamped output_msg;
  output_msg.header.stamp = now;
  output_msg.header.frame_id = base_frame_id_;
  output_msg.twist = current_output_;
  output_pub_->publish(output_msg);
}

const CmdVelNavAcceleratorMuxNode::InputSource *
CmdVelNavAcceleratorMuxNode::selectActiveInput(const rclcpp::Time & now) const
{
  const InputSource * selected = nullptr;
  for (const auto & input : inputs_) {
    if (!inputIsFresh(input, now)) {
      continue;
    }
    if (selected == nullptr || input.priority > selected->priority) {
      selected = &input;
    }
  }
  return selected;
}

bool CmdVelNavAcceleratorMuxNode::inputIsFresh(
  const InputSource & input,
  const rclcpp::Time & now) const
{
  if (!input.last_msg.has_value()) {
    return false;
  }
  return (now - input.last_time).seconds() <= input.timeout;
}

geometry_msgs::msg::Twist CmdVelNavAcceleratorMuxNode::currentVelocityForRateLimit(
  const rclcpp::Time & now) const
{
  if (feedback_ == "CLOSED_LOOP" && has_latest_odom_ &&
    (now - latest_odom_time_).seconds() <= odom_duration_)
  {
    return latest_odom_twist_;
  }
  if (has_output_state_) {
    return current_output_;
  }
  return geometry_msgs::msg::Twist{};
}

void CmdVelNavAcceleratorMuxNode::publishActiveName(const std::string & active_name)
{
  if (active_name_ == active_name) {
    return;
  }

  active_name_ = active_name;
  std_msgs::msg::String msg;
  msg.data = active_name_;
  active_pub_->publish(msg);
}

}  // namespace hanmole_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelNavAcceleratorMuxNode)
