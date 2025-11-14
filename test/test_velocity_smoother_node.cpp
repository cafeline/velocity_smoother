#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "velocity_smoother/velocity_smoother_node.hpp"

using namespace std::chrono_literals;

namespace velocity_smoother
{
namespace
{

class VelocitySmootherNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void TearDown() override
  {
    if (test_node_) {
      executor_.remove_node(test_node_);
      test_node_.reset();
    }
    if (node_) {
      executor_.remove_node(node_);
      node_.reset();
    }
    pub_.reset();
    cmd_sub_.reset();
    marker_sub_.reset();
    received_cmd_count_ = 0;
    received_marker_count_ = 0;
  }

  void create_node(const std::function<void(rclcpp::NodeOptions &)> & configure = nullptr)
  {
    ASSERT_FALSE(node_);

    rclcpp::NodeOptions options;
    options.append_parameter_override("time_constant", 0.0);
    options.append_parameter_override("command_timeout", 0.5);
    options.append_parameter_override("publish_frequency", 20.0);
    options.append_parameter_override("max_linear_speed", 2.0);
    options.append_parameter_override("max_angular_speed", 2.0);
    if (configure) {
      configure(options);
    }

    node_ = std::make_shared<VelocitySmootherNode>(options);
    executor_.add_node(node_);

    test_node_ = std::make_shared<rclcpp::Node>("velocity_smoother_test_helper");
    pub_ = test_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);
    cmd_sub_ = test_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_cmd_ = *msg;
        ++received_cmd_count_;
      });

    marker_sub_ = test_node_->create_subscription<visualization_msgs::msg::Marker>(
      "cmd_vel_marker", 10,
      [this](visualization_msgs::msg::Marker::ConstSharedPtr msg) {
        last_marker_ = *msg;
        ++received_marker_count_;
      });

    executor_.add_node(test_node_);
  }

  bool spin_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      executor_.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return condition();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<VelocitySmootherNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;

  geometry_msgs::msg::Twist last_cmd_;
  visualization_msgs::msg::Marker last_marker_;
  int received_cmd_count_ = 0;
  int received_marker_count_ = 0;
};

TEST_F(VelocitySmootherNodeTest, PublishesFilteredCommand)
{
  create_node();

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  pub_->publish(cmd);

  ASSERT_TRUE(spin_until([this]() { return received_cmd_count_ > 0; }, 500ms));
  EXPECT_NEAR(last_cmd_.linear.x, 1.0, 1e-3);
}

TEST_F(VelocitySmootherNodeTest, CommandTimeoutOutputsZeroTwist)
{
  create_node([](rclcpp::NodeOptions & options) {
    options.append_parameter_override("command_timeout", 0.05);
    options.append_parameter_override("publish_frequency", 50.0);
  });

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  pub_->publish(cmd);

  ASSERT_TRUE(spin_until([this]() { return received_cmd_count_ > 0; }, 500ms));

  ASSERT_TRUE(spin_until(
    [this]() {
      return received_cmd_count_ > 1 && std::abs(last_cmd_.linear.x) < 1e-3;
    },
    1000ms));
}

TEST_F(VelocitySmootherNodeTest, PublishesMarkerWithoutInput)
{
  create_node([](rclcpp::NodeOptions & options) {
    options.append_parameter_override("publish_frequency", 20.0);
  });

  ASSERT_TRUE(spin_until([this]() { return received_marker_count_ > 0; }, 500ms));
  EXPECT_EQ(last_marker_.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(last_marker_.header.frame_id, "base_link");
}

}  // namespace
}  // namespace velocity_smoother
