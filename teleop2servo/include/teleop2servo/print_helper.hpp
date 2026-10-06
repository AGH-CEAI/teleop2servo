#ifndef TELEOP2SERVO__PRINT_HELPER_HPP_
#define TELEOP2SERVO__PRINT_HELPER_HPP_

#include <string>

#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo {

class PrintHelper {
 public:
  static std::string build_teleop_msg_layout_and_instructions(TeleopDevice teleop_device,
                                                              ControlMode control_mode,
                                                              SpeedMode speed_mode,
                                                              bool device_blocked,
                                                              GripperState gripper_state,
                                                              bool homing);

 private:
  static std::string build_banner(TeleopDevice teleop_device);
  static std::string build_status(ControlMode control_mode,
                                  SpeedMode speed_mode,
                                  bool device_blocked,
                                  GripperState gripper_state,
                                  bool homing);
  static std::string build_gamepad_instructions(ControlMode control_mode,
                                                bool device_blocked,
                                                bool gripper_enabled,
                                                bool homing);
  static std::string build_gamepad_header(bool gripper_enabled);
  static std::string build_gamepad_safety_procedure();
  static std::string build_gamepad_joint_instructions();
  static std::string build_gamepad_twist_instructions();
  static std::string build_gamepad_homing_info();
  static std::string build_keyboard_instructions(ControlMode control_mode,
                                                 bool device_blocked,
                                                 bool gripper_enabled,
                                                 bool homing);
  static std::string build_keyboard_header(bool gripper_enabled);
  static std::string build_keyboard_layout(ControlMode control_mode, bool gripper_enabled);
  static std::string build_keyboard_homing_info();
  static std::string build_keyboard_safety_procedure();
  static std::string build_keyboard_joint_instructions(bool gripper_enabled);
  static std::string build_keyboard_twist_instructions(bool gripper_enabled);
  static std::string build_footer();
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__PRINT_HELPER_HPP_
