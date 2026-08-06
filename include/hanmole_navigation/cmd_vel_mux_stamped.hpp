#ifndef HANMOLE_NAVIGATION__CMD_VEL_MUX_STAMPED_HPP_
#define HANMOLE_NAVIGATION__CMD_VEL_MUX_STAMPED_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hanmole_navigation
{

struct StampedMuxInputState
{
  std::string name;
  std::string topic;
  double timeout{0.20};
  int priority{0};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;
  std::optional<geometry_msgs::msg::TwistStamped> last_msg;
  rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
};

bool stampedMuxInputIsFresh(
  const StampedMuxInputState & input,
  const rclcpp::Time & now);

const StampedMuxInputState * selectActiveStampedInput(
  const std::vector<StampedMuxInputState> & inputs,
  const rclcpp::Time & now);

geometry_msgs::msg::TwistStamped makeMuxOutput(const StampedMuxInputState & input);

class CmdVelMuxStampedNode final : public rclcpp::Node
{
public:
  explicit CmdVelMuxStampedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void configureInputs(const std::vector<std::string> & input_names);
  void inputCallback(
    std::size_t input_index,
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publishTimerCallback();
  void publishActiveName(const std::string & active_name);

  std::vector<StampedMuxInputState> inputs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string output_topic_;
  std::string active_topic_;
  std::string active_name_;
  double publish_rate_{50.0};
};

}  // namespace hanmole_navigation

#endif  // HANMOLE_NAVIGATION__CMD_VEL_MUX_STAMPED_HPP_
