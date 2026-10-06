#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/teleop_keyboard_node.hpp"

namespace teleop2servo {

TeleopKeyboardNode::TeleopKeyboardNode(const rclcpp::NodeOptions& options)
    : Node("keyboard_teleop_node", options), teleop_publisher_(*this, TeleopDevice::KEYBOARD) {
  load_keyboard_parameters();
  keyboard_.start();
  setup_timers();
}

TeleopKeyboardNode::~TeleopKeyboardNode() {
  keyboard_.stop();
}

template <typename T>
void TeleopKeyboardNode::load_param(const std::string& name, T& value) {
  this->declare_parameter<T>(name, value);
  this->get_parameter(name, value);
}

void TeleopKeyboardNode::load_keyboard_parameters() {
  load_param("reading_keyboard_hz", keyboard_config_.reading_keyboard_hz);
  load_param("key_initial_timeout_s", keyboard_config_.key_initial_timeout_s);
  load_param("key_repeat_timeout_s", keyboard_config_.key_repeat_timeout_s);
}

void TeleopKeyboardNode::setup_timers() {
  const double hz = std::max(1.0, keyboard_config_.reading_keyboard_hz);
  key_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / hz),
                                       std::bind(&TeleopKeyboardNode::handle_key_input, this));
}

// Keys are lowercased, so Caps Lock and Shift don't change their meaning.
std::optional<char> TeleopKeyboardNode::read_last_key() {
  std::optional<char> last;
  char c;
  while (keyboard_.read_key(c)) {
    last = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return last;
}

// The terminal reports no key releases, only characters: one on press, then, after the
// system auto-repeat delay (~0.5 s), a stream of repeats (~30 ms apart).
// Press, hold and release are reconstructed from that timing.
void TeleopKeyboardNode::handle_key_input() {
  const auto now = SteadyClock::now();
  const double since_last_key = std::chrono::duration<double>(now - last_key_time_).count();

  // A key arrived: classify it as a new press or an auto-repeat of the held key.
  if (const auto key = read_last_key()) {
    const bool new_press = !held_key_ || *held_key_ != *key;
    const bool autorepeat = !new_press && since_last_key < keyboard_config_.key_repeat_timeout_s;

    // Another key interrupted an unresolved char of the previous one, so that one was a tap.
    if (pending_tap_ && new_press)
      process_key(*held_key_, false, false);

    key_repeat_seen_ = !new_press && (key_repeat_seen_ || autorepeat);
    held_key_ = *key;
    last_key_time_ = now;

    // Same key after a pause: a new tap or the first auto-repeat. Decided on the next ticks.
    pending_tap_ = !new_press && !autorepeat;
    if (pending_tap_)
      return;

    process_key(*key, new_press, autorepeat);
    return;
  }

  // No key in this tick.
  if (!held_key_)
    return;

  // No repeat followed the unresolved char, so the key is not held: it was a tap.
  if (pending_tap_ && since_last_key >= keyboard_config_.key_repeat_timeout_s) {
    pending_tap_ = false;
    process_key(*held_key_, false, false);
  }

  // No char within the timeout means the key was released. Before the first repeat
  // the gap is the long auto-repeat delay, afterwards only the short repeat interval.
  const double release_timeout =
      key_repeat_seen_ ? keyboard_config_.key_repeat_timeout_s : keyboard_config_.key_initial_timeout_s;

  if (since_last_key > release_timeout) {
    held_key_.reset();
    key_repeat_seen_ = false;
    pending_tap_ = false;
    on_key_release();
  }
}

void TeleopKeyboardNode::process_key(char c, bool new_press, bool autorepeat) {
  // While blocked only the unblock key is handled; block / mode / speed keys never move the robot.
  if (!check_safety_procedure(c, new_press))
    return;

  if (check_state_buttons(c, new_press))
    return;

  if (check_gripper_button(c, autorepeat))
    return;

  const ControlMode control_mode = teleop_publisher_.get_control_mode();
  const SpeedMode speed_mode = teleop_publisher_.get_speed_mode();

  // STEP: one step per tap, auto-repeats ignored. CONT: start on press, motion lasts until release.
  const bool trigger = (speed_mode == SpeedMode::STEP) ? !autorepeat : new_press;
  if (!trigger)
    return;

  ActiveCmd cmd = ActiveCmd();

  if (control_mode == ControlMode::JOINT) {
    create_cmd_joint(c, speed_mode, cmd);
  } else {
    create_cmd_twist(c, control_mode, speed_mode, cmd);
  }

  teleop_publisher_.set_active_cmd(cmd);
}

// Stops CONT motion; a running STEP still finishes (see TeleopPublisher::set_active_cmd).
void TeleopKeyboardNode::on_key_release() {
  teleop_publisher_.set_active_cmd(ActiveCmd());
}

// Returns true when the device is unblocked and the key may be processed further.
bool TeleopKeyboardNode::check_safety_procedure(char c, bool new_press) {
  if (!teleop_publisher_.is_device_blocked())
    return true;

  if (new_press && c == KeyboardMapping::block_device)
    teleop_publisher_.unblock_teleop_device();

  return false;
}

// Returns true for state keys (block / mode / speed / home); they act only on a new press, not on repeats.
bool TeleopKeyboardNode::check_state_buttons(char c, bool new_press) {
  switch (c) {
    case KeyboardMapping::block_device:
      if (new_press)
        teleop_publisher_.block_teleop_device();
      return true;

    case KeyboardMapping::switch_control_mode:
      if (new_press)
        teleop_publisher_.switch_control_mode();
      return true;

    case KeyboardMapping::switch_speed_mode:
      if (new_press)
        teleop_publisher_.switch_speed_mode();
      return true;

    case KeyboardMapping::go_home:
      if (new_press)
        teleop_publisher_.go_home();
      return true;

    default:
      return false;
  }
}

// Toggles on every tap, also on a quick re-tap of the same key; holding the key toggles only once.
bool TeleopKeyboardNode::check_gripper_button(char c, bool autorepeat) {
  if (c != KeyboardMapping::toggle_gripper)
    return false;

  if (!autorepeat)
    teleop_publisher_.toggle_gripper();
  return true;
}

void TeleopKeyboardNode::create_cmd_joint(char c, const SpeedMode speed_mode, ActiveCmd& cmd) const {
  const auto& config = teleop_publisher_.get_config();

  const double scale =
      (speed_mode == SpeedMode::STEP) ? config.joint_vel_step : config.joint_vel_cont_max * get_speed_val(speed_mode);

  static constexpr std::array<std::pair<char, char>, 6> joint_keys = {{
      {KeyboardMapping::joint_1_positive, KeyboardMapping::joint_1_negative},
      {KeyboardMapping::joint_2_positive, KeyboardMapping::joint_2_negative},
      {KeyboardMapping::joint_3_positive, KeyboardMapping::joint_3_negative},
      {KeyboardMapping::joint_4_positive, KeyboardMapping::joint_4_negative},
      {KeyboardMapping::joint_5_positive, KeyboardMapping::joint_5_negative},
      {KeyboardMapping::joint_6_positive, KeyboardMapping::joint_6_negative},
  }};

  // Command only the joint whose +/- key was pressed; an unmapped key leaves the command NONE.
  for (std::size_t i = 0; i < joint_keys.size() && i < config.joint_names.size(); ++i) {
    const auto& [positive, negative] = joint_keys[i];
    if (c != positive && c != negative)
      continue;

    cmd.joint_velocities.assign(config.joint_names.size(), 0.0);
    cmd.joint_velocities[i] = (c == positive) ? scale : -scale;
    cmd.type = ActiveCmdType::JOINT;
    return;
  }
}

void TeleopKeyboardNode::create_cmd_twist(char c,
                                          const ControlMode control_mode,
                                          const SpeedMode speed_mode,
                                          ActiveCmd& cmd) const {
  const auto& config = teleop_publisher_.get_config();

  cmd.twist_msg.header.frame_id = (control_mode == ControlMode::BASE) ? config.base_frame_id : config.ee_frame_id;

  const double scale_lin =
      (speed_mode == SpeedMode::STEP) ? config.twist_lin_step : config.twist_lin_cont_max * get_speed_val(speed_mode);

  const double scale_ang =
      (speed_mode == SpeedMode::STEP) ? config.twist_ang_step : config.twist_ang_cont_max * get_speed_val(speed_mode);

  auto direction = [c](char positive, char negative) {
    if (c == positive)
      return 1.0;
    if (c == negative)
      return -1.0;
    return 0.0;
  };

  // Each axis gets +1, -1 or 0 from the key; only one key is handled at a time, so at most one axis moves.
  auto& twist = cmd.twist_msg.twist;
  twist.linear.x = direction(KeyboardMapping::x_positive, KeyboardMapping::x_negative) * scale_lin;
  twist.linear.y = direction(KeyboardMapping::y_positive, KeyboardMapping::y_negative) * scale_lin;
  twist.linear.z = direction(KeyboardMapping::z_positive, KeyboardMapping::z_negative) * scale_lin;

  twist.angular.x = direction(KeyboardMapping::roll_positive, KeyboardMapping::roll_negative) * scale_ang;
  twist.angular.y = direction(KeyboardMapping::pitch_positive, KeyboardMapping::pitch_negative) * scale_ang;
  twist.angular.z = direction(KeyboardMapping::yaw_positive, KeyboardMapping::yaw_negative) * scale_ang;

  const bool any_motion = twist.linear.x != 0.0 || twist.linear.y != 0.0 || twist.linear.z != 0.0 ||
                          twist.angular.x != 0.0 || twist.angular.y != 0.0 || twist.angular.z != 0.0;

  if (any_motion) {
    cmd.type = ActiveCmdType::TWIST;
  }
}

}  // namespace teleop2servo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop2servo::TeleopKeyboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
