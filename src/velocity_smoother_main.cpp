#include "rclcpp/rclcpp.hpp"

#include "velocity_smoother/velocity_smoother_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<velocity_smoother::VelocitySmootherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

