#ifndef HANMOLE_NAVIGATION__FINE_TUNE_HPP_
#define HANMOLE_NAVIGATION__FINE_TUNE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace hanmole_navigation::fine_tune
{

struct PoseError
{
  double dx{0.0};
  double dy{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  double distance{0.0};
  double current_yaw{0.0};
  double goal_yaw{0.0};
  double yaw_error{0.0};
};

struct ControlLimits
{
  double omni_linear_kp{0.7};
  double omni_yaw_kp{1.0};
  double omni_max_linear{0.08};
  double omni_max_yaw{0.30};
  double min_linear_cmd{0.015};
  double min_yaw_cmd{0.02};
};

double normalizeAngle(double angle);

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion);

PoseError buildPoseError(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & goal);

bool withinTolerance(
  const PoseError & error,
  double xy_tolerance,
  double yaw_tolerance);

geometry_msgs::msg::Twist buildOmniCommand(
  const PoseError & error,
  double xy_tolerance,
  double yaw_tolerance,
  const ControlLimits & limits = ControlLimits{});

void logPoseError(
  const PoseError & error,
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & target_name);

}  // namespace hanmole_navigation::fine_tune

#endif  // HANMOLE_NAVIGATION__FINE_TUNE_HPP_
