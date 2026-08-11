#include "hanmole_navigation/fine_tune.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hanmole_navigation::fine_tune
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double clampSymmetric(double value, double limit)
{
  return std::max(-limit, std::min(value, limit));
}

}  // namespace

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

PoseError buildPoseError(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & goal)
{
  PoseError error;
  error.dx = goal.pose.position.x - current.pose.position.x;
  error.dy = goal.pose.position.y - current.pose.position.y;
  error.current_yaw = yawFromQuaternion(current.pose.orientation);
  error.goal_yaw = yawFromQuaternion(goal.pose.orientation);

  const double c = std::cos(error.current_yaw);
  const double s = std::sin(error.current_yaw);
  error.robot_x = c * error.dx + s * error.dy;
  error.robot_y = -s * error.dx + c * error.dy;
  error.distance = std::sqrt(error.dx * error.dx + error.dy * error.dy);
  error.yaw_error = normalizeAngle(error.goal_yaw - error.current_yaw);

  return error;
}

bool withinTolerance(
  const PoseError & error,
  double xy_tolerance,
  double yaw_tolerance)
{
  return error.distance < xy_tolerance && std::abs(error.yaw_error) < yaw_tolerance;
}

geometry_msgs::msg::Twist buildOmniCommand(
  const PoseError & error,
  double xy_tolerance,
  double yaw_tolerance,
  const ControlLimits & limits)
{
  geometry_msgs::msg::Twist cmd;
  const double position_deadband = std::max(0.001, 0.5 * xy_tolerance);
  const double yaw_deadband = std::max(0.001, 0.5 * yaw_tolerance);

  if (std::abs(error.robot_x) > position_deadband) {
    double linear_x = limits.omni_linear_kp * error.robot_x;
    if (std::abs(linear_x) < limits.min_linear_cmd) {
      linear_x = error.robot_x > 0.0 ? limits.min_linear_cmd : -limits.min_linear_cmd;
    }
    cmd.linear.x = clampSymmetric(linear_x, limits.omni_max_linear);
  }

  if (std::abs(error.robot_y) > position_deadband) {
    double linear_y = limits.omni_linear_kp * error.robot_y;
    if (std::abs(linear_y) < limits.min_linear_cmd) {
      linear_y = error.robot_y > 0.0 ? limits.min_linear_cmd : -limits.min_linear_cmd;
    }
    cmd.linear.y = clampSymmetric(linear_y, limits.omni_max_linear);
  }

  if (std::abs(error.yaw_error) > yaw_deadband) {
    double angular_z = limits.omni_yaw_kp * error.yaw_error;
    if (std::abs(angular_z) < limits.min_yaw_cmd) {
      angular_z = error.yaw_error > 0.0 ? limits.min_yaw_cmd : -limits.min_yaw_cmd;
    }
    cmd.angular.z = clampSymmetric(angular_z, limits.omni_max_yaw);
  }

  return cmd;
}

void logPoseError(
  const PoseError & error,
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & target_name)
{
  static std::string last_target_name;
  static double last_distance = 0.0;
  static double last_yaw_error = 0.0;
  static std::uint64_t sequence = 0;

  if (last_target_name != target_name) {
    last_target_name = target_name;
    last_distance = error.distance;
    last_yaw_error = error.yaw_error;
    sequence = 1;

    RCLCPP_INFO(
      rclcpp::get_logger("fine_tune"),
      "seq=%llu target=%s frame=%s "
      "cur=(%.6f, %.6f, yaw=%.6f) "
      "goal=(%.6f, %.6f, yaw=%.6f) "
      "err: dx=%.6f dy=%.6f dist=%.6fm yaw_err=%.6frad "
      "body_err: x=%.6f y=%.6f",
      static_cast<unsigned long long>(sequence),
      target_name.c_str(),
      current.header.frame_id.c_str(),
      current.pose.position.x,
      current.pose.position.y,
      error.current_yaw,
      goal.pose.position.x,
      goal.pose.position.y,
      error.goal_yaw,
      error.dx,
      error.dy,
      error.distance,
      error.yaw_error,
      error.robot_x,
      error.robot_y);
    return;
  }

  const double distance_delta = error.distance - last_distance;
  const double yaw_error_delta = error.yaw_error - last_yaw_error;
  last_distance = error.distance;
  last_yaw_error = error.yaw_error;
  ++sequence;

  RCLCPP_INFO(
    rclcpp::get_logger("fine_tune"),
    "seq=%llu target=%s dist=%.6fm d_dist=%+.6fm yaw_err=%.6frad d_yaw=%+.6frad",
    static_cast<unsigned long long>(sequence),
    target_name.c_str(),
    error.distance,
    distance_delta,
    error.yaw_error,
    yaw_error_delta);
}

}  // namespace hanmole_navigation::fine_tune
