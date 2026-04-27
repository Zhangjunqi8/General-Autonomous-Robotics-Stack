#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hanmole_navigation
{

class CmdVelToTwistStamped final : public rclcpp::Node
{
public:
  CmdVelToTwistStamped()
  : Node("cmd_vel_to_twist_stamped")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/base_controller/reference");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_footprint");
    const auto qos_depth = declare_parameter<int>("qos_depth", 10);

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, rclcpp::QoS(qos_depth));
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(qos_depth),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = now();
        stamped.header.frame_id = frame_id_;
        stamped.twist = *msg;
        publisher_->publish(stamped);
      });

    RCLCPP_INFO(
      get_logger(),
      "Forwarding %s geometry_msgs/Twist to %s geometry_msgs/TwistStamped",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace hanmole_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelToTwistStamped>());
  rclcpp::shutdown();
  return 0;
}
