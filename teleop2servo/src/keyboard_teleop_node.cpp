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


using namespace std::chrono_literals;
using teleop2servo::ControlMode;
using teleop2servo::SpeedMode;
using teleop2servo::ActiveCmd;
using teleop2servo::ActiveCmdType;
using teleop2servo::JointMove;
using teleop2servo::to_string;

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"

namespace teleop2servo
{

KeyboardTeleopNode::KeyboardTeleopNode()
: Node("keyboard_teleop",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  // load parameters
  this->get_parameter_or("publish_hz", publish_hz_, 100);
  this->get_parameter_or("stop_moving_timeout_s", stop_moving_timeout_s_, 2.0);

  this->get_parameter_or("twist_topic", twist_topic_, std::string("/servo_node/delta_twist_cmds"));
  this->get_parameter_or("joint_topic", joint_topic_, std::string("/servo_node/delta_joint_cmds"));
  this->get_parameter_or("queue_size", queue_size_, 10);

  this->get_parameter_or("base_frame_id", base_frame_id_, std::string("base"));
  this->get_parameter_or("eef_frame_id", eef_frame_id_, std::string("tool0"));

  this->get_parameter_or("joint_names", joint_names_, std::vector<std::string>{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  });

  this->get_parameter_or("joint_vel_step", joint_vel_step_, 0.5);
  this->get_parameter_or("joint_vel_cont_slow", joint_vel_cont_slow_, 1.0);

  this->get_parameter_or("twist_lin_step", twist_lin_step_, 0.05);
  this->get_parameter_or("twist_lin_cont_slow", twist_lin_cont_slow_, 0.10);

  this->get_parameter_or("twist_rot_step", twist_rot_step_, 0.20);
  this->get_parameter_or("twist_rot_cont_slow", twist_rot_cont_slow_, 0.35);

  build_keymap();

  // TESTING
  pub_ = this->create_publisher<std_msgs::msg::String>("/keyboard_teleop/event", 10);

  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, queue_size_);
  joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_topic_, queue_size_);

  keyboard_.start();

  key_timer_ = this->create_wall_timer(
    5ms, std::bind(&KeyboardTeleopNode::poll_keyboard, this)
  );

  const int hz = std::max(1, publish_hz_);
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / hz),
    std::bind(&KeyboardTeleopNode::publish_loop, this)
  );

  last_input_time_ = this->now();
  print_instruction_and_status();
}

KeyboardTeleopNode::~KeyboardTeleopNode()
{
  keyboard_.stop();
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

void KeyboardTeleopNode::print_instruction_and_status()
{
  RCLCPP_INFO(get_logger(), "\n\n================ TELEOP KEYBOARD =================");
  RCLCPP_INFO(get_logger(),
    "Mode: " COLOR_CYAN "%s" COLOR_RESET
    " | Speed: " COLOR_YELLOW "%s" COLOR_RESET
    " | Rotation: %s",
    to_string(control_mode_).c_str(),
    to_string(speed_mode_).c_str(),
    rotation_ ? COLOR_GREEN "ON" COLOR_RESET : COLOR_RED "OFF" COLOR_RESET
  );
  RCLCPP_INFO(get_logger(), "---------------------------");
  RCLCPP_INFO(get_logger(), "TAB: switch control modes (JOINTS/BASE)");
  RCLCPP_INFO(get_logger(), "--- Joint control keymap:");
  RCLCPP_INFO(get_logger(), "1/q -> J1, 2/w -> J2, 3/e -> J3, 4/r -> J4, 5/t -> J5, 6/y -> J6");
  RCLCPP_INFO(get_logger(), "--- Base control keymap:");
  RCLCPP_INFO(get_logger(), "arrows -> axis X/Y, .; -> axis Z");
  RCLCPP_INFO(get_logger(), "With ? switch on/off rotation around axis");
  RCLCPP_INFO(get_logger(), "---------------------------");
  RCLCPP_INFO(get_logger(), "'s' to switch speed");
  RCLCPP_INFO(get_logger(), "modes: STEP / CONT_SLOW");
  RCLCPP_INFO(get_logger(), "---------------------------");
  RCLCPP_INFO(get_logger(), "Ctrl+C to exit.");
}

void KeyboardTeleopNode::poll_keyboard()
{
  char c;
  while (keyboard_.read_key(c)) {

    last_input_time_ = this->now();

    if (step_lock_char_ == c && (last_input_time_ - step_lock_time_).seconds() < 0.4) {
      step_lock_time_ = last_input_time_;
      continue;
    }

    // change the control
    switch (c) {
      case KEYCODE_SPACE: stop_motion("space"); continue;
      case KEYCODE_TAB: switch_control_mode(); continue;
      case KEYCODE_CAPITAL_S: switch_speed_mode(); continue;
      default: handle_char_key(static_cast<char>(c)); continue;
      }
  }
}

void KeyboardTeleopNode::switch_control_mode()
{
  if (control_mode_ == ControlMode::JOINTS) control_mode_ = ControlMode::BASE;
  else control_mode_ = ControlMode::JOINTS;

  stop_motion("mode switch");
  print_instruction_and_status();
}

void KeyboardTeleopNode::switch_speed_mode()
{
  if (speed_mode_ == SpeedMode::STEP) speed_mode_ = SpeedMode::CONT_SLOW;
  else speed_mode_ = SpeedMode::STEP;

  stop_motion("speed switch");
  print_instruction_and_status();
}

void KeyboardTeleopNode::toggle_rotation()
{
  rotation_ = !rotation_;
  stop_motion("rotation toggle");
  print_instruction_and_status();
}

void KeyboardTeleopNode::handle_char_key(char c)
{
  if (control_mode_ == ControlMode::JOINTS){
    auto it = joint_keymap_.find(c);
    if (it == joint_keymap_.end()){
      return;
    }
    const int joint = it->second.joint;
    const int sign = it->second.sign;

    {
      // TESTING
      std::ostringstream ss;
      ss << "[JOINTS] J" << joint << " sign=" << (sign > 0 ? "+" : "-")
        << " | speed=" << to_string(speed_mode_);

      active_cmd_testing = ss.str();
      have_active_cmd_ = true;
    }

    // Validate joint_names_ size (expect 6)
    if (joint < 1 || joint > static_cast<int>(joint_names_.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "joint_names has size %zu; key requested J%d",
                            joint_names_.size(), joint);
      return;
    }

    active_cmd_ = ActiveCmd{};
    active_cmd_.type = ActiveCmdType::JOINT;
    active_cmd_.joint_index = joint - 1;
    active_cmd_.joint_sign = sign;

    have_active_cmd_ = true;

    if (speed_mode_ == SpeedMode::STEP) {
      step_lock_time_ = this->now();
      step_lock_char_ = c;
      step_pending_one_shot_ = true;   // public ones
    } else {
      step_pending_one_shot_ = false;  // public continuously
    }
    return;
  }

  if (control_mode_ == ControlMode::BASE) {
    // TODO(issue#3) implement arrow controlling
  }
}

void KeyboardTeleopNode::stop_motion(const std::string &reason)
{
  (void)reason;
  have_active_cmd_ = false;
  step_pending_one_shot_ = false;
  active_cmd_testing.clear();
  active_cmd_ = ActiveCmd{};
}

double KeyboardTeleopNode::joint_vel_for_speed_mode() const
{
  return (speed_mode_ == SpeedMode::STEP) ? joint_vel_step_ : joint_vel_cont_slow_;
}

void KeyboardTeleopNode::publish_loop()
{
  if (!have_active_cmd_) return;

  const auto now = this->now();
  const double dt = (now - last_input_time_).seconds();

  if (speed_mode_ != SpeedMode::STEP && dt > stop_moving_timeout_s_) {
    stop_motion("timeout");
    return;
  }

  if (active_cmd_.type == ActiveCmdType::JOINT) {
    control_msgs::msg::JointJog msg;
    msg.header.stamp = now;
    msg.header.frame_id = base_frame_id_;  // often BASE frame is used for joint jog

    const int idx = active_cmd_.joint_index;
    const double vel = joint_vel_for_speed_mode() * static_cast<double>(active_cmd_.joint_sign);

    msg.joint_names.push_back(joint_names_.at(idx));
    msg.velocities.push_back(vel);

    joint_pub_->publish(msg);
  }

  // std_msgs::msg::String msg;
  // msg.data = active_cmd_;
  // pub_->publish(msg);

  if (speed_mode_ == SpeedMode::STEP && step_pending_one_shot_) {
    stop_motion("one step");
    return;
  }
}

} //namespace teleop2servo
