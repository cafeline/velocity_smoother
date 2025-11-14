#include <cmath>

#include <gtest/gtest.h>

#include "velocity_smoother/command_filter.hpp"

namespace velocity_smoother
{
namespace
{

geometry_msgs::msg::Twist makeTwist(double linear_x = 0.0, double linear_y = 0.0, double angular_z = 0.0)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = linear_y;
  twist.angular.z = angular_z;
  return twist;
}

TEST(CommandFilterTest, AppliesSmoothingAccordingToTimeConstant)
{
  CommandFilter filter;
  filter.set_time_constant(0.2);
  filter.set_velocity_limits(10.0, 10.0);

  auto first = filter.filter(makeTwist(1.0, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(first.linear.x, 1.0);

  auto second = filter.filter(makeTwist(0.0, 0.0, 0.0), 0.1);
  EXPECT_NEAR(second.linear.x, 2.0 / 3.0, 1e-6);
}

TEST(CommandFilterTest, ZeroTimeConstantBypassesFilter)
{
  CommandFilter filter;
  filter.set_time_constant(0.0);
  filter.set_velocity_limits(10.0, 10.0);

  auto first = filter.filter(makeTwist(1.0), 0.0);
  EXPECT_DOUBLE_EQ(first.linear.x, 1.0);

  auto second = filter.filter(makeTwist(-1.0), 0.1);
  EXPECT_DOUBLE_EQ(second.linear.x, -1.0);
}

TEST(CommandFilterTest, ClampsPlanarSpeed)
{
  CommandFilter filter;
  filter.set_time_constant(0.0);
  filter.set_velocity_limits(1.0, 10.0);

  auto result = filter.filter(makeTwist(2.0, 0.0), 0.0);
  EXPECT_NEAR(result.linear.x, 1.0, 1e-6);
  EXPECT_NEAR(result.linear.y, 0.0, 1e-6);

  auto diag = filter.filter(makeTwist(1.0, 1.0), 0.1);
  EXPECT_NEAR(std::hypot(diag.linear.x, diag.linear.y), 1.0, 1e-6);
}

}  // namespace
}  // namespace velocity_smoother
