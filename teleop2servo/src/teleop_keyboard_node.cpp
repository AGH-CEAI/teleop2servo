#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_keyboard_node.hpp"
#include "teleop2servo/teleop_utils.hpp"

using namespace std::chrono_literals;
using teleop2servo::ControlMode;
using teleop2servo::SpeedMode;
using teleop2servo::ActiveCmd;
using teleop2servo::ActiveCmdType;
using teleop2servo::next;
using teleop2servo::to_string;

namespace teleop2servo
{

TeleopKeyboardNode::TeleopKeyboardNode()
: Node(
    "keyboard_teleop",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  keyboard_.start();

  std::cout << "START KEYBOARD hahahaha :)))" << std::endl;
}

TeleopKeyboardNode::~TeleopKeyboardNode()
{
  keyboard_.stop();
}

} // namespace teleop2servo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop2servo::TeleopKeyboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
