#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "hanmole_navigation/target_repository.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

class TargetCatalogPublisherNode : public rclcpp::Node
{
public:
  TargetCatalogPublisherNode()
  : Node("target_catalog_publisher")
  {
    target_file_ = declare_parameter<std::string>("target_file", "");
    target_group_ = declare_parameter<std::string>("target_group", "");
    const std::string pose_list_topic = declare_parameter<std::string>(
      "pose_list_topic", "/hanmole/agv/pose_list");

    if (target_file_.empty()) {
      throw std::runtime_error("target_file cannot be empty");
    }

    publisher_ = create_publisher<std_msgs::msg::String>(
      pose_list_topic,
      rclcpp::QoS(1).transient_local().reliable());

    publish_pose_list();
    publish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish_pose_list(); });
  }

private:
  void publish_pose_list()
  {
    try {
      repository_.load_from_file(target_file_);
      const std::string active_group = target_group_.empty() ?
        repository_.default_group() : target_group_;
      if (!repository_.has_group(active_group)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "target_group not found: %s", active_group.c_str());
        return;
      }

      std_msgs::msg::String message;
      message.data = repository_.pose_list_json(active_group);
      publisher_->publish(message);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to publish pose list from %s: %s",
        target_file_.c_str(),
        exception.what());
    }
  }

  hanmole_navigation::TargetRepository repository_;
  std::string target_file_;
  std::string target_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetCatalogPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
