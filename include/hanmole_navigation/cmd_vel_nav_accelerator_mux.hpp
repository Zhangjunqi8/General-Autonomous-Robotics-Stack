#ifndef HANMOLE_NAVIGATION__CMD_VEL_NAV_ACCELERATOR_MUX_HPP_
#define HANMOLE_NAVIGATION__CMD_VEL_NAV_ACCELERATOR_MUX_HPP_

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hanmole_navigation
{

struct AxisTransformConfig
{
  double min_velocity;
  double max_velocity;
  double gain;
  double min_boost;
  double deadband;
};

struct AxisRateLimitConfig
{
  double max_accel;
  double max_decel;
};

enum class CmdVelInputMode
{
  kBoost,
  kPassthrough,
  kStop,
};

double applyDeadband(double value, double deadband);
double clampVelocity(double value, double min_velocity, double max_velocity);
double boostAxis(double value, const AxisTransformConfig & config);
double passthroughAxis(double value, const AxisTransformConfig & config);
double rateLimitAxis(
  double current,
  double target,
  const AxisRateLimitConfig & config,
  double dt);

geometry_msgs::msg::Twist transformTwist(
  const geometry_msgs::msg::Twist & input,
  CmdVelInputMode mode,
  const std::array<AxisTransformConfig, 3> & transform_config);

geometry_msgs::msg::Twist rateLimitTwist(
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target,
  const std::array<AxisRateLimitConfig, 3> & rate_limit_config,
  double dt);

bool twistIsNearZero(const geometry_msgs::msg::Twist & twist, double epsilon = 1e-6);
CmdVelInputMode parseInputMode(const std::string & mode);

class CmdVelNavAcceleratorMuxNode final : public rclcpp::Node
{
public:
  explicit CmdVelNavAcceleratorMuxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct InputSource
  {
    std::string name;
    std::string topic;
    double timeout;
    int priority;
    CmdVelInputMode mode;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;
    std::optional<geometry_msgs::msg::TwistStamped> last_msg;
    rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
  };

  void configureInputs(const std::vector<std::string> & input_names);
  void inputCallback(
    std::size_t input_index,
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTimerCallback();

  const InputSource * selectActiveInput(const rclcpp::Time & now) const;
  bool inputIsFresh(const InputSource & input, const rclcpp::Time & now) const;
  geometry_msgs::msg::Twist currentVelocityForRateLimit(const rclcpp::Time & now) const;
  void publishActiveName(const std::string & active_name);

  std::vector<InputSource> inputs_;
  std::array<AxisTransformConfig, 3> transform_config_;
  std::array<AxisRateLimitConfig, 3> rate_limit_config_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  geometry_msgs::msg::Twist current_output_;
  geometry_msgs::msg::Twist latest_odom_twist_;
  rclcpp::Time latest_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::string active_name_;
  std::string base_frame_id_;
  std::string feedback_;
  double odom_duration_;
  double publish_rate_;
  bool has_latest_odom_{false};
  bool has_output_state_{false};
};

}  // namespace hanmole_navigation

#endif  // HANMOLE_NAVIGATION__CMD_VEL_NAV_ACCELERATOR_MUX_HPP_
