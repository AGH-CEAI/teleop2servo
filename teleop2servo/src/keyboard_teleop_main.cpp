#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/keyboard_teleop_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop2servo::KeyboardTeleopNode>();
  // Robimy obiekt typu AbstractTeleop np. GamepadTelop
  // auto teleop = GamepadTelop(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
