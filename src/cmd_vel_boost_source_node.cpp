#include "hanmole_navigation/cmd_vel_boost_source.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rclcpp_components/register_node_macro.hpp"

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

}  // namespace hanmole_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(hanmole_navigation::CmdVelBoostSourceNode)
