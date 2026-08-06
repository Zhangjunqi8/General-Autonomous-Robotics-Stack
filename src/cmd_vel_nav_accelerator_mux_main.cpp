#include "hanmole_navigation/cmd_vel_nav_accelerator_mux.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hanmole_navigation::CmdVelNavAcceleratorMuxNode>());
  rclcpp::shutdown();
  return 0;
}
