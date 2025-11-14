#include "velocity_smoother/velocity_smoother_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace velocity_smoother
{

namespace
{
constexpr double kDefaultMarkerScaleY = 0.05;
constexpr double kDefaultMarkerScaleZ = 0.1;
constexpr std::array<double, 4> kDefaultMarkerColor{0.0, 0.8, 1.0, 0.9};
}  // namespace

VelocitySmootherNode::VelocitySmootherNode(const rclcpp::NodeOptions & options)
: Node("velocity_smoother", options),
  last_input_time_(this->get_clock()->now()),
  last_filter_time_(this->get_clock()->now()),
  have_input_(false),
  timeout_active_(false),
  time_constant_(0.2),
  command_timeout_(0.5),
  max_linear_speed_(0.5),
  max_angular_speed_(0.4),
  publish_frequency_(20.0),
  marker_scale_y_(kDefaultMarkerScaleY),
  marker_scale_z_(kDefaultMarkerScaleZ),
  marker_color_(kDefaultMarkerColor),
  base_frame_("base_link"),
  raw_cmd_topic_("cmd_vel_raw"),
  filtered_cmd_topic_("cmd_vel"),
  marker_topic_("cmd_vel_marker")
{
  declare_parameters();
  apply_parameter_values();
  configure_interfaces();

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&VelocitySmootherNode::on_parameter_event, this, std::placeholders::_1));

  recreate_timer();
}

void VelocitySmootherNode::declare_parameters()
{
  this->declare_parameter("time_constant", 0.2);
  this->declare_parameter("command_timeout", 0.5);
  this->declare_parameter("max_linear_speed", 0.5);
  this->declare_parameter("max_angular_speed", 0.4);
  this->declare_parameter("publish_frequency", 20.0);
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("raw_cmd_topic", raw_cmd_topic_);
  this->declare_parameter("filtered_cmd_topic", filtered_cmd_topic_);
  this->declare_parameter("marker_topic", marker_topic_);
  this->declare_parameter("marker_scale_y", marker_scale_y_);
  this->declare_parameter("marker_scale_z", marker_scale_z_);
  this->declare_parameter(
    "marker_color",
    std::vector<double>{kDefaultMarkerColor[0], kDefaultMarkerColor[1],
      kDefaultMarkerColor[2], kDefaultMarkerColor[3]});
}

void VelocitySmootherNode::configure_interfaces()
{
  auto raw_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  raw_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    raw_cmd_topic_, raw_qos,
    std::bind(&VelocitySmootherNode::handle_raw_cmd, this, std::placeholders::_1));

  auto filtered_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  filtered_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    filtered_cmd_topic_, filtered_qos);

  auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    marker_topic_, marker_qos);
}

void VelocitySmootherNode::handle_raw_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto now = this->get_clock()->now();
  const double dt = have_input_ ? (now - last_filter_time_).seconds() : 0.0;

  last_cmd_ = filter_.filter(*msg, dt);
  last_input_time_ = now;
  last_filter_time_ = now;
  have_input_ = true;
  timeout_active_ = false;

  publish_outputs(now);
}

void VelocitySmootherNode::on_timer()
{
  const auto now = this->get_clock()->now();

  if (command_timeout_ > 0.0 && have_input_) {
    const double elapsed = (now - last_input_time_).seconds();
    if (elapsed > command_timeout_ && !timeout_active_) {
      last_cmd_ = zero_twist();
      filter_.reset(last_cmd_);
      timeout_active_ = true;
      have_input_ = false;
      last_filter_time_ = now;
    }
  }

  publish_outputs(now);
}

void VelocitySmootherNode::publish_outputs(const rclcpp::Time & stamp)
{
  if ((have_input_ || timeout_active_) && filtered_cmd_pub_) {
    filtered_cmd_pub_->publish(last_cmd_);
  }
  publish_marker(stamp);
}

void VelocitySmootherNode::publish_marker(const rclcpp::Time & stamp)
{
  if (!marker_pub_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_;
  marker.header.stamp = stamp;
  marker.ns = "velocity_smoother";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = marker_scale_y_;
  marker.scale.z = marker_scale_z_;
  marker.color.r = marker_color_[0];
  marker.color.g = marker_color_[1];
  marker.color.b = marker_color_[2];
  marker.color.a = marker_color_[3];
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration(0, 0);

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  end.x = last_cmd_.linear.x;
  end.y = last_cmd_.linear.y;
  end.z = last_cmd_.linear.z;
  marker.points.clear();
  marker.points.push_back(start);
  marker.points.push_back(end);

  marker_pub_->publish(marker);
}

void VelocitySmootherNode::apply_parameter_values()
{
  time_constant_ = this->get_parameter("time_constant").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  publish_frequency_ = this->get_parameter("publish_frequency").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  raw_cmd_topic_ = this->get_parameter("raw_cmd_topic").as_string();
  filtered_cmd_topic_ = this->get_parameter("filtered_cmd_topic").as_string();
  marker_topic_ = this->get_parameter("marker_topic").as_string();
  marker_scale_y_ = this->get_parameter("marker_scale_y").as_double();
  marker_scale_z_ = this->get_parameter("marker_scale_z").as_double();

  const auto color_vector = this->get_parameter("marker_color").as_double_array();
  if (color_vector.size() == 4U) {
    std::copy(color_vector.begin(), color_vector.end(), marker_color_.begin());
  } else {
    marker_color_ = kDefaultMarkerColor;
  }

  filter_.set_time_constant(time_constant_);
  filter_.set_velocity_limits(max_linear_speed_, max_angular_speed_);
}

geometry_msgs::msg::Twist VelocitySmootherNode::zero_twist() const
{
  return geometry_msgs::msg::Twist();
}

rcl_interfaces::msg::SetParametersResult VelocitySmootherNode::on_parameter_event(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool filter_update = false;
  bool timer_update = false;

  for (const auto & param : parameters) {
    if (param.get_name() == "time_constant" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      time_constant_ = param.as_double();
      filter_update = true;
    } else if (param.get_name() == "command_timeout" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      command_timeout_ = param.as_double();
    } else if (param.get_name() == "max_linear_speed" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      max_linear_speed_ = param.as_double();
      filter_update = true;
    } else if (param.get_name() == "max_angular_speed" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      max_angular_speed_ = param.as_double();
      filter_update = true;
    } else if (param.get_name() == "publish_frequency" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      publish_frequency_ = param.as_double();
      timer_update = true;
    } else if (param.get_name() == "marker_scale_y" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      marker_scale_y_ = param.as_double();
    } else if (param.get_name() == "marker_scale_z" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      marker_scale_z_ = param.as_double();
    } else if (param.get_name() == "marker_color" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
    {
      const auto & values = param.as_double_array();
      if (values.size() == 4U) {
        std::copy(values.begin(), values.end(), marker_color_.begin());
      }
    }
  }

  if (filter_update) {
    filter_.set_time_constant(time_constant_);
    filter_.set_velocity_limits(max_linear_speed_, max_angular_speed_);
  }

  if (timer_update) {
    recreate_timer();
  }

  return result;
}

void VelocitySmootherNode::recreate_timer()
{
  if (publish_frequency_ <= 0.0) {
    timer_.reset();
    return;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_frequency_));

  timer_ = this->create_wall_timer(period, std::bind(&VelocitySmootherNode::on_timer, this));
}

}  // namespace velocity_smoother
