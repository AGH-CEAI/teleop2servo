#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/teleop2servo_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<teleop2servo::Teleop2ServoNode>());
  rclcpp::shutdown();
  return 0;
}
