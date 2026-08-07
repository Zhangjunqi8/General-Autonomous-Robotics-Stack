#include "hanmole_navigation/accelerate.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelBoostSourceNode>());
  rclcpp::shutdown();
  return 0;
}
