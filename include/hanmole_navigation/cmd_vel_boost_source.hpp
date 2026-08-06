#ifndef HANMOLE_NAVIGATION__CMD_VEL_BOOST_SOURCE_HPP_
#define HANMOLE_NAVIGATION__CMD_VEL_BOOST_SOURCE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace hanmole_navigation
{

struct BoostSourceConfig
{
  double boost_start_linear_x{0.60};
  double boost_gain{4.00};
  double max_linear_x{1.20};
  double max_abs_linear_y{0.10};
  double max_abs_angular_z{0.15};
  double corridor_lookahead_m{3.0};
  double corridor_half_width{0.40};
};

double boostLinearX(
  double linear_x,
  double boost_start_linear_x,
  double boost_gain,
  double max_linear_x);

bool forwardCorridorIsClear(
  const sensor_msgs::msg::LaserScan & scan,
  double corridor_lookahead_m,
  double corridor_half_width);

bool twistQualifiesForBoost(
  const geometry_msgs::msg::TwistStamped & command,
  const BoostSourceConfig & config);

std::optional<geometry_msgs::msg::TwistStamped> makeBoostedCommandIfAllowed(
  const geometry_msgs::msg::TwistStamped & command,
  const sensor_msgs::msg::LaserScan & scan,
  const BoostSourceConfig & config);

class CmdVelBoostSourceNode final : public rclcpp::Node
{
public:
  explicit CmdVelBoostSourceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool latestScanIsFresh(const rclcpp::Time & now) const;
  void validateConfig() const;

  BoostSourceConfig config_;
  double scan_timeout_{0.30};
  std::string input_topic_;
  std::string output_topic_;
  std::string scan_topic_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::optional<sensor_msgs::msg::LaserScan> latest_scan_;
  rclcpp::Time latest_scan_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace hanmole_navigation

#endif  // HANMOLE_NAVIGATION__CMD_VEL_BOOST_SOURCE_HPP_
