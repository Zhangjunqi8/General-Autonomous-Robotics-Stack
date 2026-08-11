#include "hanmole_navigation/fine_tune.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = yawToQuaternion(yaw);
  return pose;
}

}  // namespace

TEST(FineTuneControl, buildsPoseErrorInRobotFrame)
{
  const auto current = makePose(0.0, 0.0, M_PI_2);
  const auto goal = makePose(0.0, 1.0, M_PI);

  const auto error = hanmole_navigation::fine_tune::buildPoseError(current, goal);

  EXPECT_NEAR(error.dx, 0.0, 1e-9);
  EXPECT_NEAR(error.dy, 1.0, 1e-9);
  EXPECT_NEAR(error.robot_x, 1.0, 1e-9);
  EXPECT_NEAR(error.robot_y, 0.0, 1e-9);
  EXPECT_NEAR(error.distance, 1.0, 1e-9);
  EXPECT_NEAR(error.yaw_error, M_PI_2, 1e-9);
}

TEST(FineTuneControl, oneCentimeterToleranceIsStrict)
{
  hanmole_navigation::fine_tune::PoseError error;
  error.distance = 0.009;
  error.yaw_error = 0.009;

  EXPECT_TRUE(hanmole_navigation::fine_tune::withinTolerance(error, 0.01, 0.01));

  error.distance = 0.011;
  EXPECT_FALSE(hanmole_navigation::fine_tune::withinTolerance(error, 0.01, 0.01));
}

TEST(FineTuneControl, omniCommandUsesRobotFrameXYAndYaw)
{
  hanmole_navigation::fine_tune::PoseError error;
  error.robot_x = 0.02;
  error.robot_y = -0.02;
  error.yaw_error = 0.04;

  const auto command = hanmole_navigation::fine_tune::buildOmniCommand(error, 0.01, 0.01);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_LT(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
  EXPECT_LE(std::abs(command.linear.x), 0.08);
  EXPECT_LE(std::abs(command.linear.y), 0.08);
  EXPECT_LE(std::abs(command.angular.z), 0.30);
}
