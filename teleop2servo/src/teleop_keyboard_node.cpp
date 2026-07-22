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
  const auto period =
      std::chrono::milliseconds(
          static_cast<int>(1000.0 / keyboard_config_.reading_keyboard_hz));

  key_timer_ = this->create_wall_timer(
      period,
      std::bind(&TeleopKeyboardNode::handle_key_input, this));
}

std::optional<char> TeleopKeyboardNode::key_press(char c) {
  const auto now = this->now();
  if (!last_active_char_ ||
    *last_active_char_ != c ||
    (now - last_input_time_).seconds() > new_key_press_timeout_)
  {
    last_active_char_ = c;
    last_input_time_ = now;
    return c;
  }
  else {
    return std::nullopt;
  }
}

bool TeleopKeyboardNode::check_safety_procedure(char c) {
  if (!teleop_publisher_.get_stop_button_pressed()) {
    return true;
  }

  if (auto key = key_press(c)) {
    if (*key == KeyboardMapping::block_device){
      teleop_publisher_.unblock_teleop_device();
    }
  }
  return false;
}

bool TeleopKeyboardNode::check_state_buttons(char c) {
  switch (c) {
    case KeyboardMapping::block_device:
    case KeyboardMapping::switch_control_mode:
    case KeyboardMapping::switch_speed_mode:
      break;
    default:
      return false;
  }

  auto key = key_press(c);
  if (!key) {
    return false;
  }

  switch (*key) {
    case KeyboardMapping::block_device:
      teleop_publisher_.block_teleop_device();
      return true;
    case KeyboardMapping::switch_control_mode:
      teleop_publisher_.switch_control_mode();
      return true;
    case KeyboardMapping::switch_speed_mode:
      teleop_publisher_.switch_speed_mode();
      return true;
    default:
      return false;
  }
}

bool TeleopKeyboardNode::can_publish_continuous(char c)
{
  const auto now = this->now();

  if (!last_active_char_ || *last_active_char_ != c) {
    last_active_char_ = c;
    last_input_time_ = now;
    continuous_repeat_seen_ = false;
    return true;
  }

  const double elapsed = (now - last_input_time_).seconds();

  if (!continuous_repeat_seen_) {
    if (elapsed >= initial_key_timeout_s_) {
      last_input_time_ = now;
      continuous_repeat_seen_ = true;
      return true;
    }
  }
  else {
    if (elapsed >= repeat_key_timeout_s_) {
      last_input_time_ = now;
      return true;
    }
  }

  return false;
}

void TeleopKeyboardNode::create_cmd_joint(
  char c,
  const SpeedMode speed_mode,
  ActiveCmd& cmd)
{
  cmd.type = ActiveCmdType::JOINT;
  cmd.joint_velocities.assign(6, 0.0);
  const auto& config = teleop_publisher_.get_config();
  const double value =
      (speed_mode == SpeedMode::STEP) ? config.joint_vel_step : config.joint_vel_cont_max * get_speed_val(speed_mode);

  switch (c) {
    case KeyboardMapping::joint_1_positive:
      cmd.joint_velocities[0] = value;
      break;
    case KeyboardMapping::joint_1_negative:
      cmd.joint_velocities[0] = -value;
      break;

    case KeyboardMapping::joint_2_positive:
      cmd.joint_velocities[1] = value;
      break;
    case KeyboardMapping::joint_2_negative:
      cmd.joint_velocities[1] = -value;
      break;

    case KeyboardMapping::joint_3_positive:
      cmd.joint_velocities[2] = value;
      break;
    case KeyboardMapping::joint_3_negative:
      cmd.joint_velocities[2] = -value;
      break;

    case KeyboardMapping::joint_4_positive:
      cmd.joint_velocities[3] = value;
      break;
    case KeyboardMapping::joint_4_negative:
      cmd.joint_velocities[3] = -value;
      break;

    case KeyboardMapping::joint_5_positive:
      cmd.joint_velocities[4] = value;
      break;
    case KeyboardMapping::joint_5_negative:
      cmd.joint_velocities[4] = -value;
      break;

    case KeyboardMapping::joint_6_positive:
      cmd.joint_velocities[5] = value;
      break;
    case KeyboardMapping::joint_6_negative:
      cmd.joint_velocities[5] = -value;
      break;

    default:
      cmd.type = ActiveCmdType::NONE;
      cmd.joint_velocities.clear();
      break;
  }
}

// void TeleopKeyboardNode::create_cmd_twist(
//   char c,
//   const ControlMode control_mode,
//   const SpeedMode speed_mode,
//   ActiveCmd& cmd)
// {

// }

void TeleopKeyboardNode::handle_key_input() {
  char c;
  ActiveCmd cmd = ActiveCmd();

  if (!keyboard_.read_key(c)) {
    std::cout << ">>> CMD is ZIOBRO!" << std::endl;
    teleop_publisher_.set_active_cmd(cmd);  // stop
    last_active_char_.reset();
    return;
  }
  std::cout << ">>> CMD will be NON zero!" << std::endl;

  if (!check_safety_procedure(c))
    return;

  if (check_state_buttons(c))
    return;

  const ControlMode control_mode = teleop_publisher_.get_control_mode();
  const SpeedMode speed_mode = teleop_publisher_.get_speed_mode();

  bool has_motion = false;
  if (speed_mode == SpeedMode::STEP) {
    has_motion = key_press(c).has_value();
  } else {
    has_motion = can_publish_continuous(c);
  }

  if (has_motion) {
    if (control_mode == ControlMode::JOINT){
      create_cmd_joint(c, speed_mode, cmd);
    } //else {
    //   create_cmd_twist(c, control_mode, speed_mode, cmd);
    // }
  }

  teleop_publisher_.set_active_cmd(cmd);
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
