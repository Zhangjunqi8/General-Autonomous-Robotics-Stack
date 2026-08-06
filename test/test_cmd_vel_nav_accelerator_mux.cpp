#include <gtest/gtest.h>

#include "hanmole_navigation/cmd_vel_nav_accelerator_mux.hpp"

namespace
{

using hanmole_navigation::AxisRateLimitConfig;
using hanmole_navigation::AxisTransformConfig;
using hanmole_navigation::boostAxis;
using hanmole_navigation::rateLimitAxis;

TEST(CmdVelNavAcceleratorMux, boostAxisAppliesDeadbandMinimumBoostAndClamp)
{
  const AxisTransformConfig config{
    -1.20,
    1.20,
    1.50,
    0.25,
    0.02,
  };

  EXPECT_DOUBLE_EQ(boostAxis(0.01, config), 0.0);
  EXPECT_DOUBLE_EQ(boostAxis(0.08, config), 0.25);
  EXPECT_DOUBLE_EQ(boostAxis(-0.08, config), -0.25);
  EXPECT_DOUBLE_EQ(boostAxis(1.00, config), 1.20);
}

TEST(CmdVelNavAcceleratorMux, rateLimitAxisUsesAccelAndDecelPerTick)
{
  const AxisRateLimitConfig config{
    2.50,
    -4.00,
  };

  EXPECT_NEAR(rateLimitAxis(0.0, 0.525, config, 0.02), 0.05, 1e-9);
  EXPECT_NEAR(rateLimitAxis(0.50, 0.0, config, 0.02), 0.42, 1e-9);
  EXPECT_NEAR(rateLimitAxis(0.50, 0.52, config, 0.02), 0.52, 1e-9);
}

}  // namespace
