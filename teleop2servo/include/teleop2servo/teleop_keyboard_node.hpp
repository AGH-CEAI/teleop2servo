#ifndef TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_
#define TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/keyboard_reader.hpp"
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_device_mapping.hpp"
#include "teleop2servo/teleop_publisher.hpp"
#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo
{

class TeleopKeyboardNode : public rclcpp::Node
{
public:
  explicit TeleopKeyboardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TeleopKeyboardNode() override;

private:
  using SteadyClock = std::chrono::steady_clock;

  // ==== init ====
  template <typename T>
  void load_param(const std::string& name, T& value);
  void load_keyboard_parameters();
  void setup_timers();

  // ==== callbacks / main loops ====
  void handle_key_input();

  // ==== input processing ====
  std::optional<char> read_last_key();
  void process_key(char c, bool new_press, bool autorepeat);
  void on_key_release();

  bool check_safety_procedure(char c, bool new_press);
  bool check_state_buttons(char c, bool new_press);

  // ==== active command creation ====
  void create_cmd_joint(char c, const SpeedMode speed_mode, ActiveCmd& cmd) const;
  void create_cmd_twist(char c, const ControlMode control_mode, const SpeedMode speed_mode, ActiveCmd& cmd) const;

private:
  KeyboardConfig keyboard_config_;
  KeyboardReader keyboard_;
  TeleopPublisher teleop_publisher_;
  rclcpp::TimerBase::SharedPtr key_timer_;

  std::optional<char> held_key_;
  bool key_repeat_seen_{false};
  bool pending_tap_{false};
  SteadyClock::time_point last_key_time_;
};

} //namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_
