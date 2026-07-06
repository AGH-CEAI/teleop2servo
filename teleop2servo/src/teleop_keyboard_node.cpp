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
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    teleop_publisher_(*this, TeleopDevice::KEYBOARD)
{
  load_keyboard_parameters();
  setup_timers();
  keyboard_.start();
  last_input_time_ = this->now();
}

TeleopKeyboardNode::~TeleopKeyboardNode()
{
  keyboard_.stop();
}

template <typename T>
void TeleopKeyboardNode::load_param(const std::string& name, T& value) {
  this->declare_parameter<T>(name, value);
  this->get_parameter(name, value);
}

void TeleopKeyboardNode::load_keyboard_parameters() {
  load_param("reading_keyboard_hz", keyboard_config_.reading_keyboard_hz);
}

void TeleopKeyboardNode::setup_timers()
{
  key_timer_ = this->create_wall_timer(
    10ms, std::bind(&TeleopKeyboardNode::handle_key_input, this)
  );
}

void TeleopKeyboardNode::handle_key_input() {
  char c;
  while (keyboard_.read_key(c)) {
    last_input_time_ = this->now();

    if (teleop_publisher_.get_stop_button_pressed() && c != KeyboardMapping::block_device){
      break;
    }

    switch (c) {
      case KeyboardMapping::block_device: teleop_publisher_.unblock_teleop_device(); continue;
      case KeyboardMapping::switch_control_mode: teleop_publisher_.switch_control_mode(); continue;
      case KeyboardMapping::switch_speed_mode: teleop_publisher_.switch_speed_mode(); continue;
      default: break;
    }

    // TODO: CONTINUE
  }
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
