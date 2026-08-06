#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <vector>

#include "hanmole_navigation/cmd_vel_boost_source.hpp"
#include "hanmole_navigation/cmd_vel_mux_stamped.hpp"

namespace
{

geometry_msgs::msg::TwistStamped makeCommand(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = "base_footprint";
  command.twist.linear.x = linear_x;
  command.twist.linear.y = linear_y;
  command.twist.angular.z = angular_z;
  return command;
}

sensor_msgs::msg::LaserScan makeScan(
  const std::vector<float> & ranges,
  double angle_min = -1.5707963267948966,
  double angle_increment = 0.17453292519943295)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges = ranges;
  return scan;
}

TEST(CmdVelBoostSource, frontCorridorIgnoresSideWallsOutsideNarrowChannel)
{
  const auto scan = makeScan(
    {2.0F, 2.0F},
    -1.5707963267948966,
    3.141592653589793);

  EXPECT_TRUE(hanmole_navigation::forwardCorridorIsClear(scan, 3.0, 0.40));
}

TEST(CmdVelBoostSource, frontCorridorBlocksObstacleInsideThreeMetersAndHalfWidth)
{
  const auto scan = makeScan({2.0F}, 0.0, 0.1);

  EXPECT_FALSE(hanmole_navigation::forwardCorridorIsClear(scan, 3.0, 0.40));
}

TEST(CmdVelBoostSource, frontCorridorIgnoresInvalidAndOutOfRectangleRanges)
{
  const auto scan = makeScan(
    {std::numeric_limits<float>::quiet_NaN(), 3.5F, 2.0F},
    -0.6,
    0.6);

  EXPECT_TRUE(hanmole_navigation::forwardCorridorIsClear(scan, 3.0, 0.40));
}

TEST(CmdVelBoostSource, boostLinearXIsContinuousFromConfiguredStart)
{
  EXPECT_DOUBLE_EQ(hanmole_navigation::boostLinearX(0.70, 0.70, 1.50, 1.20), 0.70);
  EXPECT_DOUBLE_EQ(hanmole_navigation::boostLinearX(0.80, 0.70, 4.00, 1.20), 1.10);
  EXPECT_DOUBLE_EQ(hanmole_navigation::boostLinearX(1.05, 0.70, 4.00, 1.20), 1.20);
}

TEST(CmdVelBoostSource, defaultThresholdsAllowSmallPathTrackingCorrections)
{
  hanmole_navigation::BoostSourceConfig config;
  const auto clear_scan = makeScan({4.0F}, 0.0, 0.1);

  const auto earlier_boost = hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.65, 0.0, 0.0),
    clear_scan,
    config);
  ASSERT_TRUE(earlier_boost.has_value());
  EXPECT_GT(earlier_boost->twist.linear.x, 0.65);

  const auto corrected_path_boost = hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.75, 0.08, 0.12),
    clear_scan,
    config);
  ASSERT_TRUE(corrected_path_boost.has_value());
  EXPECT_DOUBLE_EQ(corrected_path_boost->twist.linear.x, config.max_linear_x);
  EXPECT_DOUBLE_EQ(config.max_linear_x, 1.20);
}

TEST(CmdVelBoostSource, boostRequiresStraightFastCommandAndClearCorridor)
{
  hanmole_navigation::BoostSourceConfig config;
  config.boost_start_linear_x = 0.70;
  config.boost_gain = 4.00;
  config.max_linear_x = 1.20;
  config.max_abs_linear_y = 0.03;
  config.max_abs_angular_z = 0.10;
  config.corridor_lookahead_m = 3.0;
  config.corridor_half_width = 0.40;

  const auto clear_scan = makeScan({4.0F}, 0.0, 0.1);
  auto boosted = hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.80, 0.0, 0.0),
    clear_scan,
    config);

  ASSERT_TRUE(boosted.has_value());
  EXPECT_DOUBLE_EQ(boosted->twist.linear.x, 1.10);
  EXPECT_DOUBLE_EQ(boosted->twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(boosted->twist.angular.z, 0.0);

  EXPECT_FALSE(hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.69, 0.0, 0.0),
    clear_scan,
    config));
  EXPECT_FALSE(hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.80, 0.04, 0.0),
    clear_scan,
    config));
  EXPECT_FALSE(hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.80, 0.0, 0.11),
    clear_scan,
    config));
  EXPECT_FALSE(hanmole_navigation::makeBoostedCommandIfAllowed(
    makeCommand(0.80, 0.0, 0.0),
    makeScan({2.0F}, 0.0, 0.1),
    config));
}

TEST(CmdVelMuxStamped, selectsFreshHighestPriorityInputAndFallsBackAfterTimeout)
{
  std::vector<hanmole_navigation::StampedMuxInputState> inputs(2);
  inputs[0].name = "nav_fast";
  inputs[0].priority = 100;
  inputs[0].timeout = 0.05;
  inputs[0].last_msg = makeCommand(1.2, 0.0, 0.0);
  inputs[0].last_time = rclcpp::Time(1, 0, RCL_ROS_TIME);

  inputs[1].name = "nav";
  inputs[1].priority = 10;
  inputs[1].timeout = 0.20;
  inputs[1].last_msg = makeCommand(0.8, 0.0, 0.0);
  inputs[1].last_time = rclcpp::Time(1, 0, RCL_ROS_TIME);

  const auto fast_selection =
    hanmole_navigation::selectActiveStampedInput(inputs, rclcpp::Time(1, 40000000, RCL_ROS_TIME));
  ASSERT_NE(fast_selection, nullptr);
  EXPECT_EQ(fast_selection->name, "nav_fast");
  EXPECT_DOUBLE_EQ(fast_selection->last_msg->twist.linear.x, 1.2);

  const auto fallback_selection =
    hanmole_navigation::selectActiveStampedInput(inputs, rclcpp::Time(1, 60000000, RCL_ROS_TIME));
  ASSERT_NE(fallback_selection, nullptr);
  EXPECT_EQ(fallback_selection->name, "nav");
  EXPECT_DOUBLE_EQ(fallback_selection->last_msg->twist.linear.x, 0.8);
}

TEST(CmdVelMuxStamped, forwardsSelectedTwistStampedWithoutChangingIt)
{
  std::vector<hanmole_navigation::StampedMuxInputState> inputs(1);
  inputs[0].name = "nav";
  inputs[0].priority = 10;
  inputs[0].timeout = 0.20;
  inputs[0].last_msg = makeCommand(0.73, -0.02, 0.05);
  inputs[0].last_msg->header.stamp = rclcpp::Time(10, 123, RCL_ROS_TIME);
  inputs[0].last_msg->header.frame_id = "custom_frame";
  inputs[0].last_time = rclcpp::Time(10, 0, RCL_ROS_TIME);

  const auto * selected =
    hanmole_navigation::selectActiveStampedInput(inputs, rclcpp::Time(10, 100000000, RCL_ROS_TIME));
  ASSERT_NE(selected, nullptr);

  const auto forwarded = hanmole_navigation::makeMuxOutput(*selected);
  EXPECT_EQ(forwarded.header.stamp, inputs[0].last_msg->header.stamp);
  EXPECT_EQ(forwarded.header.frame_id, "custom_frame");
  EXPECT_DOUBLE_EQ(forwarded.twist.linear.x, 0.73);
  EXPECT_DOUBLE_EQ(forwarded.twist.linear.y, -0.02);
  EXPECT_DOUBLE_EQ(forwarded.twist.angular.z, 0.05);
}

}  // namespace
