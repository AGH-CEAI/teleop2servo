#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/keyboard_teleop_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<teleop2servo::KeyboardTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
