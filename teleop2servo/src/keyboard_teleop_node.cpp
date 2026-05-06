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

#include "teleop2servo/keyboard_config.hpp"
#include "teleop2servo/keyboard_teleop_node.hpp"
#include "teleop2servo/utils.hpp"


using namespace std::chrono_literals;
using teleop2servo::ControlMode;
using teleop2servo::SpeedMode;
using teleop2servo::ActiveCmd;
using teleop2servo::ActiveCmdType;
using teleop2servo::JointMove;
using teleop2servo::next;
using teleop2servo::to_string;

namespace teleop2servo
{

KeyboardTeleopNode::KeyboardTeleopNode()
: Node("keyboard_teleop",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  load_parameters();
  build_keymap();
  setup_publishers();
  setup_timers();

  keyboard_.start();

  last_input_time_ = this->now();
  print_instruction_and_status();
}

KeyboardTeleopNode::~KeyboardTeleopNode()
{
  keyboard_.stop();
}

void KeyboardTeleopNode::load_parameters()
{
  this->get_parameter_or("publish_hz", publish_hz_, 250);
  this->get_parameter_or("step_publish_ticks", step_publish_ticks_, 2);

  this->get_parameter_or("twist_topic", twist_topic_, std::string("/servo_node/delta_twist_cmds"));
  this->get_parameter_or("joint_topic", joint_topic_, std::string("/servo_node/delta_joint_cmds"));
  this->get_parameter_or("queue_size", queue_size_, 10);

  this->get_parameter_or("base_frame_id", base_frame_id_, std::string("base_link"));
  this->get_parameter_or("ee_frame_id", ee_frame_id_, std::string("tool0"));

  this->get_parameter_or("joint_names", joint_names_, std::vector<std::string>{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  });

  this->get_parameter_or("joint_vel_step", joint_vel_step_, 1.0);
  this->get_parameter_or("joint_vel_cont_max", joint_vel_cont_max_, 1.0);

  this->get_parameter_or("twist_lin_step", twist_lin_step_, 0.05);
  this->get_parameter_or("twist_lin_cont_max", twist_lin_cont_max_, 1.0);

  this->get_parameter_or("twist_rot_step", twist_rot_step_, 0.20);
  this->get_parameter_or("twist_rot_cont_max", twist_rot_cont_max_, 0.35);
}

void KeyboardTeleopNode::build_keymap()
{
  joint_keymap_[static_cast<char>(KEYCODE_1)] = {1, +1};
  joint_keymap_[static_cast<char>(KEYCODE_Q)] = {1, -1};
  joint_keymap_[static_cast<char>(KEYCODE_2)] = {2, +1};
  joint_keymap_[static_cast<char>(KEYCODE_W)] = {2, -1};
  joint_keymap_[static_cast<char>(KEYCODE_3)] = {3, +1};
  joint_keymap_[static_cast<char>(KEYCODE_E)] = {3, -1};
  joint_keymap_[static_cast<char>(KEYCODE_4)] = {4, +1};
  joint_keymap_[static_cast<char>(KEYCODE_R)] = {4, -1};
  joint_keymap_[static_cast<char>(KEYCODE_5)] = {5, +1};
  joint_keymap_[static_cast<char>(KEYCODE_T)] = {5, -1};
  joint_keymap_[static_cast<char>(KEYCODE_6)] = {6, +1};
  joint_keymap_[static_cast<char>(KEYCODE_Y)] = {6, -1};
}

void KeyboardTeleopNode::setup_publishers()
{
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, queue_size_);
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_topic_, queue_size_);
}

void KeyboardTeleopNode::setup_timers()
{
  key_timer_ = this->create_wall_timer(
    5ms, std::bind(&KeyboardTeleopNode::poll_keyboard, this)
  );

  const int hz = std::max(1, publish_hz_);
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / hz),
    std::bind(&KeyboardTeleopNode::publish_loop, this)
  );
}

void KeyboardTeleopNode::print_instruction_and_status()
{
  if (control_mode_ == ControlMode::JOINT)
  {
    RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP KEYBOARD =================" COLOR_RESET);
    RCLCPP_INFO(get_logger(),
      "Mode: " COLOR_CYAN "%s" COLOR_RESET
      " | Speed: " COLOR_YELLOW "%s" COLOR_RESET,
      to_string(control_mode_).c_str(),
      to_string(speed_mode_).c_str()
    );
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "TAB          : switch control modes [JOINT / BASE / TOOL]");
    RCLCPP_INFO(get_logger(), "SHIFT + 's'  : switch speed [STEP / CONT 5%% / CONT 10%% / CONT 25%%]");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "KEY LAYOUT:");
    RCLCPP_INFO(get_logger(), "'1' / 'q' -> J1+/J1-");
    RCLCPP_INFO(get_logger(), "'2' / 'w' -> J2+/J2-");
    RCLCPP_INFO(get_logger(), "'3' / 'e' -> J3+/J3-");
    RCLCPP_INFO(get_logger(), "'4' / 'r' -> J4+/J4-");
    RCLCPP_INFO(get_logger(), "'5' / 't' -> J5+/J5-");
    RCLCPP_INFO(get_logger(), "'6' / 'y' -> J6+/J6-");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);
  }
  else
  {
    RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP KEYBOARD =================" COLOR_RESET);
    RCLCPP_INFO(get_logger(),
      "Mode: " COLOR_CYAN "%s" COLOR_RESET
      " | Speed: " COLOR_YELLOW "%s" COLOR_RESET,
      to_string(control_mode_).c_str(),
      to_string(speed_mode_).c_str()
    );
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "TAB          : switch control modes [JOINT / BASE / TOOL]");
    RCLCPP_INFO(get_logger(), "SHIFT + 's'  : switch speed [STEP / CONT 5%% / CONT 10%% / CONT 25%%]");
    RCLCPP_INFO(get_logger(), "'r'          : toggle rotation mode");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "KEY LAYOUT:");
    RCLCPP_INFO(get_logger(), "Rotation: %s", rotation_ ? COLOR_GREEN "ON" COLOR_RESET : COLOR_RED "OFF" COLOR_RESET);
    RCLCPP_INFO(get_logger(), "  'd' / 'a'  : X axis  + / -");
    RCLCPP_INFO(get_logger(), "  'w' / 's'  : Y axis  + / -");
    RCLCPP_INFO(get_logger(), "  'e' / 'q'  : Z axis  + / -");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);
  }
}

void KeyboardTeleopNode::poll_keyboard()
{
  char c;
  while (keyboard_.read_key(c)) {

    last_input_time_ = this->now();

    // In STEP mode repeated same-key events within cooldown are ignored.
    if (speed_mode_ == SpeedMode::STEP &&
      active_char_ == c &&
      (last_input_time_ - step_lock_time_).seconds() < step_key_cooldown_s_)
    {
      step_lock_time_ = last_input_time_;
      continue;
    }

    switch (c) {
      case KEYCODE_SPACE: stop_motion("space"); continue;
      case KEYCODE_TAB: switch_control_mode(); continue;
      case KEYCODE_CAPITAL_S: switch_speed_mode(); continue;
      default: handle_char_key(static_cast<char>(c)); continue;
    }
  }
}

void KeyboardTeleopNode::stop_motion(const std::string &reason)
{
  (void)reason;
  step_ticks_remaining_ = 0;
  continuous_repeat_seen_ = false;
  active_char_ = 0;
  active_cmd_ = ActiveCmd{};
  have_active_cmd_ = true; // once with ActiveCmdType::NONE to stop motion.
}

void KeyboardTeleopNode::switch_control_mode()
{
  stop_motion("mode switched");
  control_mode_ = next(control_mode_);
  print_instruction_and_status();
}

void KeyboardTeleopNode::switch_speed_mode()
{
  stop_motion("speed switched");
  speed_mode_ = next(speed_mode_);
  print_instruction_and_status();
}

void KeyboardTeleopNode::handle_char_key(char c)
{
  if (control_mode_ == ControlMode::JOINT){
    auto it = joint_keymap_.find(c);
    if (it == joint_keymap_.end()){
      return;
    }
    const int joint = it->second.joint;
    const int sign = it->second.sign;

    active_cmd_ = ActiveCmd{};
    active_cmd_.type = ActiveCmdType::JOINT;
    active_cmd_.joint_index = joint - 1;
    active_cmd_.joint_sign = sign;
    have_active_cmd_ = true;
  }
  else  // all cartesian movements
  {
    // TODO(issue#3) implement arrow controlling
    return;
  }

  if (speed_mode_ == SpeedMode::STEP) {
    active_char_ = c;
    step_lock_time_ = this->now();
    step_ticks_remaining_ = std::max(1, step_publish_ticks_);
  } else {
    step_ticks_remaining_ = 0;

    if (active_char_ == c){
      continuous_repeat_seen_ = true;
    } else {
      active_char_ = c;
      continuous_repeat_seen_ = false;
    }
  }
}

double KeyboardTeleopNode::joint_vel_for_speed_mode() const
{
  return (speed_mode_ == SpeedMode::STEP) ? joint_vel_step_ : joint_vel_cont_max_ * get_speed_val(speed_mode_);
}

double KeyboardTeleopNode::twist_lin_for_speed_mode() const
{
  return (speed_mode_ == SpeedMode::STEP) ? twist_lin_step_ : twist_lin_cont_max_ * get_speed_val(speed_mode_);
}

double KeyboardTeleopNode::twist_rot_for_speed_mode() const
{
  return (speed_mode_ == SpeedMode::STEP) ? twist_rot_step_ : twist_rot_cont_max_ * get_speed_val(speed_mode_);
}

void KeyboardTeleopNode::publish_stop_once(const rclcpp::Time & now)
{
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = base_frame_id_;
  for (const auto & name : joint_names_){
    joint_msg.joint_names.push_back(name);
    joint_msg.velocities.push_back(0.0);
  }

  joint_pub_->publish(joint_msg);

  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = now;
  twist_msg.header.frame_id = base_frame_id_;

  twist_pub_->publish(twist_msg);
}

void KeyboardTeleopNode::publish_joint(const rclcpp::Time & now)
{
  if (joint_names_.size() < 6) {
    throw std::runtime_error("joint_names must contain at least 6 joints");
  }

  const int idx = active_cmd_.joint_index;
  const double vel = joint_vel_for_speed_mode() * static_cast<double>(active_cmd_.joint_sign);

  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = base_frame_id_;  // often BASE frame is used for joint jog
  joint_msg.joint_names.push_back(joint_names_.at(idx));
  joint_msg.velocities.push_back(vel);

  joint_pub_->publish(joint_msg);
}

void KeyboardTeleopNode::publish_twist(const rclcpp::Time & now)
{
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = now;
  twist_msg.header.frame_id = base_frame_id_;

  twist_msg.twist.linear.x = active_cmd_.lin_x * twist_lin_for_speed_mode();
  twist_msg.twist.linear.y = active_cmd_.lin_y * twist_lin_for_speed_mode();
  twist_msg.twist.linear.z = active_cmd_.lin_z * twist_lin_for_speed_mode();

  twist_msg.twist.angular.x = active_cmd_.ang_x * twist_rot_for_speed_mode();
  twist_msg.twist.angular.y = active_cmd_.ang_y * twist_rot_for_speed_mode();
  twist_msg.twist.angular.z = active_cmd_.ang_z * twist_rot_for_speed_mode();

  twist_pub_->publish(twist_msg);
}

void KeyboardTeleopNode::publish_loop()
{
  if (!have_active_cmd_) return;

  const auto now = this->now();
  const double dt = (now - last_input_time_).seconds();

  const double timeout_s = continuous_repeat_seen_ ? repeat_key_timeout_s_ : initial_key_timeout_s_;

  if (speed_mode_ != SpeedMode::STEP && dt > timeout_s) stop_motion("timeout");

  switch (active_cmd_.type)
  {
    case ActiveCmdType::NONE:
      publish_stop_once(now);
      have_active_cmd_ = false;
      return;

    case ActiveCmdType::JOINT:
      publish_joint(now);
      break;

    case ActiveCmdType::TWIST:
      publish_twist(now);
      break;
  }

  if (speed_mode_ == SpeedMode::STEP && step_ticks_remaining_ > 0)
  {
    --step_ticks_remaining_;
    if (step_ticks_remaining_ <= 0) stop_motion("one step");
  }
}

} //namespace teleop2servo
