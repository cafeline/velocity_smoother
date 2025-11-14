#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"

namespace velocity_smoother
{

class CommandFilter
{
public:
  CommandFilter();

  void set_time_constant(double time_constant);
  void set_velocity_limits(double max_linear_speed, double max_angular_speed);

  geometry_msgs::msg::Twist filter(const geometry_msgs::msg::Twist & input, double dt_seconds);
  void reset(const geometry_msgs::msg::Twist & value);

  const geometry_msgs::msg::Twist & state() const { return state_; }

private:
  static geometry_msgs::msg::Twist clamp_linear_speed(
    const geometry_msgs::msg::Twist & input,
    double max_linear_speed);

  double compute_alpha(double dt_seconds) const;

  geometry_msgs::msg::Twist state_;
  bool initialized_;
  double time_constant_;
  double max_linear_speed_;
  double max_angular_speed_;
};

inline CommandFilter::CommandFilter()
: initialized_(false),
  time_constant_(0.2),
  max_linear_speed_(std::numeric_limits<double>::infinity()),
  max_angular_speed_(std::numeric_limits<double>::infinity())
{
  state_ = geometry_msgs::msg::Twist();
}

inline void CommandFilter::set_time_constant(double time_constant)
{
  time_constant_ = time_constant;
}

inline void CommandFilter::set_velocity_limits(double max_linear_speed, double max_angular_speed)
{
  max_linear_speed_ = max_linear_speed;
  max_angular_speed_ = max_angular_speed;
  state_.angular.x = std::clamp(state_.angular.x, -max_angular_speed_, max_angular_speed_);
  state_.angular.y = std::clamp(state_.angular.y, -max_angular_speed_, max_angular_speed_);
  state_.angular.z = std::clamp(state_.angular.z, -max_angular_speed_, max_angular_speed_);
  state_ = clamp_linear_speed(state_, max_linear_speed_);
}

inline geometry_msgs::msg::Twist CommandFilter::filter(
  const geometry_msgs::msg::Twist & input,
  double dt_seconds)
{
  geometry_msgs::msg::Twist clamped_input = clamp_linear_speed(input, max_linear_speed_);
  clamped_input.angular.x = std::clamp(clamped_input.angular.x, -max_angular_speed_, max_angular_speed_);
  clamped_input.angular.y = std::clamp(clamped_input.angular.y, -max_angular_speed_, max_angular_speed_);
  clamped_input.angular.z = std::clamp(clamped_input.angular.z, -max_angular_speed_, max_angular_speed_);

  if (!initialized_) {
    state_ = clamped_input;
    initialized_ = true;
    return state_;
  }

  const double alpha = compute_alpha(dt_seconds);

  state_.linear.x += alpha * (clamped_input.linear.x - state_.linear.x);
  state_.linear.y += alpha * (clamped_input.linear.y - state_.linear.y);
  state_.linear.z += alpha * (clamped_input.linear.z - state_.linear.z);
  state_.angular.x += alpha * (clamped_input.angular.x - state_.angular.x);
  state_.angular.y += alpha * (clamped_input.angular.y - state_.angular.y);
  state_.angular.z += alpha * (clamped_input.angular.z - state_.angular.z);

  state_ = clamp_linear_speed(state_, max_linear_speed_);
  state_.angular.x = std::clamp(state_.angular.x, -max_angular_speed_, max_angular_speed_);
  state_.angular.y = std::clamp(state_.angular.y, -max_angular_speed_, max_angular_speed_);
  state_.angular.z = std::clamp(state_.angular.z, -max_angular_speed_, max_angular_speed_);

  return state_;
}

inline void CommandFilter::reset(const geometry_msgs::msg::Twist & value)
{
  state_ = clamp_linear_speed(value, max_linear_speed_);
  state_.angular.x = std::clamp(state_.angular.x, -max_angular_speed_, max_angular_speed_);
  state_.angular.y = std::clamp(state_.angular.y, -max_angular_speed_, max_angular_speed_);
  state_.angular.z = std::clamp(state_.angular.z, -max_angular_speed_, max_angular_speed_);
  initialized_ = true;
}

inline geometry_msgs::msg::Twist CommandFilter::clamp_linear_speed(
  const geometry_msgs::msg::Twist & input,
  double max_linear_speed)
{
  geometry_msgs::msg::Twist output = input;
  if (!(max_linear_speed > 0.0)) {
    return output;
  }

  const double planar_speed = std::hypot(input.linear.x, input.linear.y);
  if (planar_speed > max_linear_speed && planar_speed > 0.0) {
    const double scale = max_linear_speed / planar_speed;
    output.linear.x = input.linear.x * scale;
    output.linear.y = input.linear.y * scale;
  }

  output.linear.z = std::clamp(input.linear.z, -max_linear_speed, max_linear_speed);
  return output;
}

inline double CommandFilter::compute_alpha(double dt_seconds) const
{
  if (time_constant_ <= 0.0) {
    return 1.0;
  }
  if (dt_seconds <= 0.0) {
    return 1.0;
  }
  return dt_seconds / (time_constant_ + dt_seconds);
}

}  // namespace velocity_smoother

