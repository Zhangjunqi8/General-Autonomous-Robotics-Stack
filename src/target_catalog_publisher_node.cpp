#include <memory>
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
    const std::string target_file = declare_parameter<std::string>("target_file", "");
    std::string target_group = declare_parameter<std::string>("target_group", "");
    const std::string pose_list_topic = declare_parameter<std::string>(
      "pose_list_topic", "/hanmole/agv/pose_list");

    if (target_file.empty()) {
      throw std::runtime_error("target_file cannot be empty");
    }

    repository_.load_from_file(target_file);
    if (target_group.empty()) {
      target_group = repository_.default_group();
    }
    if (!repository_.has_group(target_group)) {
      throw std::runtime_error("target_group not found: " + target_group);
    }

    publisher_ = create_publisher<std_msgs::msg::String>(
      pose_list_topic,
      rclcpp::QoS(1).transient_local().reliable());

    std_msgs::msg::String message;
    message.data = repository_.catalog_json(target_group);
    publisher_->publish(message);
  }

private:
  hanmole_navigation::TargetRepository repository_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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
