#pragma once

#include <array>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "velocity_smoother/command_filter.hpp"

namespace velocity_smoother
{

class VelocitySmootherNode : public rclcpp::Node
{
public:
  explicit VelocitySmootherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void configure_interfaces();
  void handle_raw_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_timer();
  void publish_outputs(const rclcpp::Time & stamp);
  void publish_marker(const rclcpp::Time & stamp);
  void apply_parameter_values();
  geometry_msgs::msg::Twist zero_twist() const;
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters);
  void recreate_timer();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr filtered_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  CommandFilter filter_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_input_time_;
  rclcpp::Time last_filter_time_;
  bool have_input_;
  bool timeout_active_;

  double time_constant_;
  double command_timeout_;
  double max_linear_speed_;
  double max_angular_speed_;
  double publish_frequency_;
  double marker_scale_y_;
  double marker_scale_z_;
  std::array<double, 4> marker_color_;
  std::string base_frame_;
  std::string raw_cmd_topic_;
  std::string filtered_cmd_topic_;
  std::string marker_topic_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace velocity_smoother

