#include "hanmole_navigation/accelerate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#if defined(HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS)
#include "rclcpp_components/register_node_macro.hpp"
#endif

namespace hanmole_navigation
{

double boostLinearX(
  double linear_x,
  double boost_start_linear_x,
  double boost_gain,
  double max_linear_x)
{
  if (linear_x < boost_start_linear_x) {
    return linear_x;
  }
  const double boosted = boost_start_linear_x + (linear_x - boost_start_linear_x) * boost_gain;
  return std::min(boosted, max_linear_x);
}

bool forwardCorridorIsClear(
  const sensor_msgs::msg::LaserScan & scan,
  double corridor_lookahead_m,
  double corridor_half_width)
{
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);
    if (x > 0.0 && x < corridor_lookahead_m && std::abs(y) < corridor_half_width) {
      return false;
    }
  }
  return true;
}

bool twistQualifiesForBoost(
  const geometry_msgs::msg::TwistStamped & command,
  const BoostSourceConfig & config)
{
  return command.twist.linear.x >= config.boost_start_linear_x &&
         std::abs(command.twist.linear.y) <= config.max_abs_linear_y &&
         std::abs(command.twist.angular.z) <= config.max_abs_angular_z;
}

std::optional<geometry_msgs::msg::TwistStamped> makeBoostedCommandIfAllowed(
  const geometry_msgs::msg::TwistStamped & command,
  const sensor_msgs::msg::LaserScan & scan,
  const BoostSourceConfig & config)
{
  if (!twistQualifiesForBoost(command, config)) {
    return std::nullopt;
  }
  if (!forwardCorridorIsClear(scan, config.corridor_lookahead_m, config.corridor_half_width)) {
    return std::nullopt;
  }

  auto output = command;
  output.twist.linear.x = boostLinearX(
    command.twist.linear.x,
    config.boost_start_linear_x,
    config.boost_gain,
    config.max_linear_x);
  return output;
}

CmdVelBoostSourceNode::CmdVelBoostSourceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_boost_source", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "cmd_vel_nav");
  output_topic_ = declare_parameter<std::string>("output_topic", "cmd_vel_nav_fast");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  scan_timeout_ = declare_parameter<double>("scan_timeout", 0.30);

  config_.boost_start_linear_x = declare_parameter<double>("boost_start_linear_x", 0.60);
  config_.boost_gain = declare_parameter<double>("boost_gain", 4.00);
  config_.max_linear_x = declare_parameter<double>("max_linear_x", 1.20);
  config_.max_abs_linear_y = declare_parameter<double>("max_abs_linear_y", 0.10);
  config_.max_abs_angular_z = declare_parameter<double>("max_abs_angular_z", 0.15);
  config_.corridor_lookahead_m = declare_parameter<double>("corridor_lookahead_m", 3.0);
  config_.corridor_half_width = declare_parameter<double>("corridor_half_width", 0.40);
  validateConfig();

  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    input_topic_,
    10,
    std::bind(&CmdVelBoostSourceNode::cmdVelCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CmdVelBoostSourceNode::scanCallback, this, std::placeholders::_1));
}

void CmdVelBoostSourceNode::validateConfig() const
{
  if (scan_timeout_ <= 0.0) {
    throw std::runtime_error("scan_timeout must be positive");
  }
  if (config_.boost_start_linear_x < 0.0) {
    throw std::runtime_error("boost_start_linear_x must be non-negative");
  }
  if (config_.boost_gain < 1.0) {
    throw std::runtime_error("boost_gain must be at least 1.0");
  }
  if (config_.max_linear_x < config_.boost_start_linear_x) {
    throw std::runtime_error("max_linear_x must be greater than or equal to boost_start_linear_x");
  }
  if (config_.max_abs_linear_y < 0.0 || config_.max_abs_angular_z < 0.0) {
    throw std::runtime_error("straight-motion thresholds must be non-negative");
  }
  if (config_.corridor_lookahead_m <= 0.0 || config_.corridor_half_width <= 0.0) {
    throw std::runtime_error("corridor dimensions must be positive");
  }
}

void CmdVelBoostSourceNode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto now = get_clock()->now();
  if (!latestScanIsFresh(now)) {
    return;
  }

  const auto boosted = makeBoostedCommandIfAllowed(*msg, *latest_scan_, config_);
  if (boosted.has_value()) {
    output_pub_->publish(*boosted);
  }
}

void CmdVelBoostSourceNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = *msg;
  latest_scan_time_ = get_clock()->now();
}

bool CmdVelBoostSourceNode::latestScanIsFresh(const rclcpp::Time & now) const
{
  if (!latest_scan_.has_value()) {
    return false;
  }
  const double age = (now - latest_scan_time_).seconds();
  return age >= 0.0 && age <= scan_timeout_;
}

namespace
{

std::string defaultTopicForInput(const std::string & name)
{
  return name == "nav_fast" ? "cmd_vel_nav_fast" : "cmd_vel_nav";
}

double defaultTimeoutForInput(const std::string & name)
{
  return name == "nav_fast" ? 0.08 : 0.20;
}

int defaultPriorityForInput(const std::string & name)
{
  return name == "nav_fast" ? 100 : 10;
}

}  // namespace

bool stampedMuxInputIsFresh(
  const StampedMuxInputState & input,
  const rclcpp::Time & now)
{
  if (!input.last_msg.has_value()) {
    return false;
  }
  const double age = (now - input.last_time).seconds();
  return age >= 0.0 && age <= input.timeout;
}

const StampedMuxInputState * selectActiveStampedInput(
  const std::vector<StampedMuxInputState> & inputs,
  const rclcpp::Time & now)
{
  const StampedMuxInputState * selected = nullptr;
  for (const auto & input : inputs) {
    if (!stampedMuxInputIsFresh(input, now)) {
      continue;
    }
    if (selected == nullptr || input.priority > selected->priority) {
      selected = &input;
    }
  }
  return selected;
}

geometry_msgs::msg::TwistStamped makeMuxOutput(const StampedMuxInputState & input)
{
  if (!input.last_msg.has_value()) {
    throw std::runtime_error("cannot make mux output without a message");
  }
  return *input.last_msg;
}

CmdVelMuxStampedNode::CmdVelMuxStampedNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_mux_stamped", options)
{
  output_topic_ = declare_parameter<std::string>("output_topic", "cmd_vel_smoothed");
  active_topic_ = declare_parameter<std::string>("active_topic", "cmd_vel_mux_stamped/active");
  publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
  if (publish_rate_ <= 0.0) {
    throw std::runtime_error("publish_rate must be positive");
  }

  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  active_pub_ = create_publisher<std_msgs::msg::String>(
    active_topic_,
    rclcpp::QoS(1).transient_local());

  const auto input_names = declare_parameter<std::vector<std::string>>(
    "input_names",
    {"nav_fast", "nav"});
  configureInputs(input_names);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  publish_timer_ = create_wall_timer(
    period,
    std::bind(&CmdVelMuxStampedNode::publishTimerCallback, this));
  publishActiveName("idle");
}

void CmdVelMuxStampedNode::configureInputs(const std::vector<std::string> & input_names)
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

    StampedMuxInputState input;
    input.name = name;
    input.topic = declare_parameter<std::string>(
      "inputs." + name + ".topic",
      defaultTopicForInput(name));
    input.timeout = declare_parameter<double>(
      "inputs." + name + ".timeout",
      defaultTimeoutForInput(name));
    input.priority = declare_parameter<int>(
      "inputs." + name + ".priority",
      defaultPriorityForInput(name));
    if (input.timeout <= 0.0) {
      throw std::runtime_error("inputs." + name + ".timeout must be positive");
    }
    inputs_.push_back(std::move(input));
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

void CmdVelMuxStampedNode::inputCallback(
  std::size_t input_index,
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (input_index >= inputs_.size()) {
    return;
  }
  inputs_[input_index].last_msg = *msg;
  inputs_[input_index].last_time = get_clock()->now();
}

void CmdVelMuxStampedNode::publishTimerCallback()
{
  const auto now = get_clock()->now();
  const auto * selected = selectActiveStampedInput(inputs_, now);
  publishActiveName(selected == nullptr ? "idle" : selected->name);
  if (selected == nullptr) {
    return;
  }
  output_pub_->publish(makeMuxOutput(*selected));
}

void CmdVelMuxStampedNode::publishActiveName(const std::string & active_name)
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

#if defined(HANMOLE_NAVIGATION_ACCELERATE_COMPONENTS)
RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelBoostSourceNode)
RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelMuxStampedNode)
#endif

#if defined(HANMOLE_NAVIGATION_ACCELERATE_BOOST_MAIN)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelBoostSourceNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

#if defined(HANMOLE_NAVIGATION_ACCELERATE_MUX_MAIN)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelMuxStampedNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
