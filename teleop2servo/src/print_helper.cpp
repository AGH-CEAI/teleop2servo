#include <string>
#include <sstream>

#include "teleop2servo/teleop_device_mapping.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo {

std::string PrintHelper::build_teleop_msg_layout_and_instructions(TeleopDevice teleop_device,
                                                                  ControlMode control_mode,
                                                                  SpeedMode speed_mode,
                                                                  bool device_blocked,
                                                                  bool homing) {
  std::ostringstream oss;
  const bool gripper_enabled = gripper_state != GripperState::DISABLED;
  oss << build_banner(teleop_device) << build_status(control_mode, speed_mode, device_blocked, gripper_enabled, homing);

  switch (teleop_device) {
    case TeleopDevice::GAMEPAD:
      oss << build_gamepad_instructions(control_mode, device_blocked, gripper_enabled, homing);
      break;
    case TeleopDevice::KEYBOARD:
      oss << build_keyboard_instructions(control_mode, device_blocked, gripper_enabled, homing);
      break;
    default:
      oss << Color::BOLD << "\n\nERROR: TeleopDevice with this name not found...\n" << Color::RESET;
      break;
  }

  return oss.str();
}

std::string PrintHelper::build_status(ControlMode control_mode,
                                      SpeedMode speed_mode,
                                      bool device_blocked,
                                      GripperState gripper_state) {
  static constexpr std::size_t BOX_INNER_WIDTH = 72;

  std::string plain;
  std::string colored;
  auto add = [&plain, &colored](const std::string& text, const char* color = nullptr) {
    plain += text;
    colored += color ? std::string(color) + text + Color::RESET : text;
  };

  add("  STATUS: ");
  if (homing) {
    add("HOMING", Color::BLUE);
  } else {
    add(device_blocked ? "BLOCKED" : "READY", device_blocked ? Color::RED : Color::GREEN);
  }
  add("      MODE: ");
  add(std::string(to_string(control_mode)), Color::CYAN);
  add("      SPEED: ");
  add(std::string(to_string(speed_mode)), Color::YELLOW);
  if (gripper_state != GripperState::DISABLED) {
    add("    GRIPPER: ");
    add(std::string(to_string(gripper_state)), Color::GREEN);
  }

  const std::size_t padding = plain.size() < BOX_INNER_WIDTH ? BOX_INNER_WIDTH - plain.size() : 0;

  std::string horizontal;
  for (std::size_t i = 0; i < BOX_INNER_WIDTH; ++i) {
    horizontal += "═";
  }

  std::ostringstream oss;

  oss << Color::BOLD << "╔" << horizontal << "╗" << Color::RESET << "\n"
      << Color::BOLD << "║" << Color::RESET << colored << std::string(padding, ' ') << Color::BOLD << "║"
      << Color::RESET << "\n"
      << Color::BOLD << "╚" << horizontal << "╝" << Color::RESET << "\n";

  return oss.str();
}

std::string PrintHelper::build_banner(TeleopDevice teleop_device) {
  static constexpr const char* BANNER = R"BANNER(
 _          _                      ____
| |_   ___ | |  ___   ___   _ __  |___ \  ___   ___  _ __ __   __  ___
| __| / _ \| | / _ \ / _ \ | '_ \   __) |/ __| / _ \| '__|\ \ / / / _ \
| |_ |  __/| ||  __/| (_) || |_) | / __/ \__ \|  __/| |    \ V / | (_) |
 \__| \___||_| \___| \___/ | .__/ |_____||___/ \___||_|     \_/   \___/
                           |_|)BANNER";
  static constexpr std::size_t BANNER_WIDTH = 72;

  const std::string device = (teleop_device == TeleopDevice::KEYBOARD) ? " KEYBOARD " : " GAMEPAD ";
  const std::size_t left = (BANNER_WIDTH - device.size()) / 2;
  const std::size_t right = BANNER_WIDTH - device.size() - left;

  std::ostringstream oss;

  oss << "\n\n\n"
      << Color::BOLD << Color::CYAN << BANNER << Color::RESET << "\n"
      << Color::BOLD << std::string(left, '=') << device << std::string(right, '=') << Color::RESET << "\n";

  return oss.str();
}

std::string PrintHelper::build_footer() {
  std::ostringstream oss;

  oss << "\n---------------------------\n" << Color::RED << "Ctrl+C to exit." << Color::RESET;

  return oss.str();
}

std::string PrintHelper::build_gamepad_instructions(ControlMode control_mode,
                                                    bool device_blocked,
                                                    bool gripper_enabled,
                                                    bool homing) {
  std::ostringstream oss;

  oss << build_gamepad_header(gripper_enabled);

  if (homing) {
    oss << build_gamepad_homing_info();
  } else if (device_blocked) {
    oss << build_gamepad_safety_procedure();
  } else if (control_mode == ControlMode::JOINT) {
    oss << build_gamepad_joint_instructions();
  } else {
    oss << build_gamepad_twist_instructions();
  }

  oss << build_footer();

  return oss.str();
}

std::string PrintHelper::build_gamepad_header(bool gripper_enabled) {
  std::ostringstream oss;

  oss << R"(
           [BACK LEFT]      [BACK RIGHT]
    [ LT ]                               [ RT ]
    [ LB ]                               [ RB ]
        .---------------------------------.
      .'                                   '.
     /  LEFT D-PAD              RIGHT MOUSE  \
    /    [↑] [↓]     [<] ON [>]   (X / Y)     \
    |    [←] [→]                              |
    |                             )"
      << Color::YELLOW << "Y" << Color::RESET << R"(           |
    |        LEFT STICK        )"
      << Color::CYAN << "X" << Color::RESET << R"(     )" << Color::RED << "B" << Color::RESET << R"(        |
    |         (X / Y)             )"
      << Color::GREEN << "A" << Color::RESET << R"(           |
    |                                         |
    \         .---------------------.         /
     \       /                       \       /
      '-----'                         '-----')"
      << "\n---------------------------\n"
      << "CONTROLS:\n"
      << "  " << Color::RED << "B" << Color::RESET << ": Block gamepad\n"
      << "  " << Color::CYAN << "X" << Color::RESET << ": Switch Mode (JOINT/BASE/TOOL)\n"
      << "  " << Color::YELLOW << "Y" << Color::RESET << ": Switch Speed (STEP/CONT 5%-100%)\n" if (gripper_enabled) oss
      << "\n  " << Color::GREEN << "A" << Color::RESET << ": Open / Close gripper";
  << "  " << Color::BLUE << "[>]" << Color::RESET << ": Go home (right arrow next to ON)"
  << "\n---------------------------\n";

  return oss.str();
}

std::string PrintHelper::build_gamepad_homing_info() {
  std::ostringstream oss;

  oss << "GOING HOME... (Servo stopped, MoveIt is moving the robot)\n"
      << "Abort: (press) " << Color::RED << "B" << Color::RESET << " - blocks the gamepad";

  return oss.str();
}

std::string PrintHelper::build_gamepad_safety_procedure() {
  std::ostringstream oss;

  oss << "Enable gamepad: [BACK LEFT] or [BACK RIGHT] + (press) " << Color::RED << "B" << Color::RESET;

  return oss.str();
}

std::string PrintHelper::build_gamepad_joint_instructions() {
  std::ostringstream oss;

  oss << "JOINT MOVEMENT:\n"
      << "  J1: [LT] positive / [RT] negative\n"
      << "  J2: [LB] positive / [RB] negative\n"
      << "  J3: D-PAD [UP] positive / [DOWN] negative\n"
      << "  J4: D-PAD [LEFT] positive / [RIGHT] negative\n"
      << "  Hold [RIGHT MOUSE] to control J5/J6 instead:\n"
      << "    J5: D-PAD [UP] positive / [DOWN] negative\n"
      << "    J6: D-PAD [LEFT] positive / [RIGHT] negative";

  return oss.str();
}

std::string PrintHelper::build_gamepad_twist_instructions() {
  std::ostringstream oss;

  oss << "TWIST MOVEMENT:\n"
      << "  Linear X/Y: LEFT STICK\n"
      << "  Linear Z: [LT] positive / [RT] negative\n"
      << "  Angular X/Y: RIGHT MOUSE\n"
      << "  Angular Z: [LB] positive / [RB] negative\n"
      << "In STEP mode, press the stick/mouse button to apply one step.";

  return oss.str();
}

std::string PrintHelper::build_keyboard_instructions(ControlMode control_mode,
                                                     bool device_blocked,
                                                     bool gripper_enabled,
                                                     bool homing) {
  std::ostringstream oss;

  oss << build_keyboard_header(gripper_enabled);

  if (homing) {
    oss << build_keyboard_homing_info();
  } else if (device_blocked) {
    oss << build_keyboard_safety_procedure();
  } else if (control_mode == ControlMode::JOINT) {
    oss << build_keyboard_joint_instructions(gripper_enabled);
  } else {
    oss << build_keyboard_twist_instructions(gripper_enabled);
  }

  oss << build_footer();

  return oss.str();
}

std::string PrintHelper::build_keyboard_header(bool gripper_enabled) {
  using M = KeyboardMapping;
  std::ostringstream oss;

  oss << "CONTROLS:\n"
      << "  " << Color::RED << M::block_device << Color::RESET << ": Block keyboard\n"
      << "  " << Color::CYAN << M::switch_control_mode << Color::RESET << ": Switch Mode (JOINT/BASE/TOOL)\n"
      << "  " << Color::YELLOW << M::switch_speed_mode << Color::RESET
      << ": Switch Speed (STEP/CONT 5%-100%)\n" if (gripper_enabled) oss << "\n  " << Color::GREEN << M::toggle_gripper
      << Color::RESET << ": Open / Close gripper";
  << "  " << Color::BLUE << M::go_home << Color::RESET << ": Go home"
  << "\n---------------------------\n";

  return oss.str();
}

std::string PrintHelper::build_keyboard_layout(ControlMode control_mode, bool gripper_enabled) {
  using M = KeyboardMapping;
  std::ostringstream oss;

  const std::string joint_keys{M::joint_1_positive, M::joint_1_negative, M::joint_2_positive, M::joint_2_negative,
                               M::joint_3_positive, M::joint_3_negative, M::joint_4_positive, M::joint_4_negative,
                               M::joint_5_positive, M::joint_5_negative, M::joint_6_positive, M::joint_6_negative};

  const std::string twist_keys{M::x_positive,     M::x_negative,     M::y_positive,    M::y_negative,
                               M::z_positive,     M::z_negative,     M::roll_positive, M::roll_negative,
                               M::pitch_positive, M::pitch_negative, M::yaw_positive,  M::yaw_negative};

  std::string active_keys = (control_mode == ControlMode::JOINT) ? joint_keys : twist_keys;
  if (gripper_enabled)
    active_keys += M::toggle_gripper;

  auto row = [&oss, &active_keys](const std::string& prefix, const std::string& keys, const std::string& suffix = "") {
    oss << prefix;
    for (const char c : keys) {
      const bool active = active_keys.find(c) != std::string::npos;
      oss << (active ? Color::BOLD : Color::DIM) << "[" << c << "]" << Color::RESET;
    }
    oss << suffix << "\n";
  };

  const std::string speed_key = std::string(Color::YELLOW) + "[" + M::switch_speed_mode + "]" + Color::RESET;
  const std::string mode_key = std::string(Color::CYAN) + "[" + M::switch_control_mode + "]" + Color::RESET;
  const std::string block_key = std::string(Color::RED) + "[" + M::block_device + "]" + Color::RESET;
  const std::string home_key = std::string(Color::BLUE) + "[" + M::go_home + "]" + Color::RESET;

  row("\n" + mode_key + speed_key + " ", "1234567890", " " + block_key);
  row("       ", "qwertyuiop");
  row("         ", "asdfghjkl");
  row("            ", "zxcvbnm", "      " + home_key);

  return oss.str();
}

std::string PrintHelper::build_keyboard_homing_info() {
  std::ostringstream oss;

  oss << "GOING HOME... (Servo stopped, MoveIt is moving the robot)\n"
      << "Abort: (press) " << Color::RED << KeyboardMapping::block_device << Color::RESET << " - blocks the keyboard";

  return oss.str();
}

std::string PrintHelper::build_keyboard_safety_procedure() {
  std::ostringstream oss;

  oss << "Enable keyboard: (press) " << Color::RED << KeyboardMapping::block_device << Color::RESET;

  return oss.str();
}

std::string PrintHelper::build_keyboard_joint_instructions(bool gripper_enabled) {
  using M = KeyboardMapping;
  std::ostringstream oss;

  oss << Color::RESET << build_keyboard_layout(ControlMode::JOINT, gripper_enabled) << "\nJOINT MOVEMENT:\n"
      << "  J1: [" << M::joint_1_positive << "] positive / [" << M::joint_1_negative << "] negative\n"
      << "  J2: [" << M::joint_2_positive << "] positive / [" << M::joint_2_negative << "] negative\n"
      << "  J3: [" << M::joint_3_positive << "] positive / [" << M::joint_3_negative << "] negative\n"
      << "  J4: [" << M::joint_4_positive << "] positive / [" << M::joint_4_negative << "] negative\n"
      << "  J5: [" << M::joint_5_positive << "] positive / [" << M::joint_5_negative << "] negative\n"
      << "  J6: [" << M::joint_6_positive << "] positive / [" << M::joint_6_negative << "] negative\n"
      << "One key at a time. STEP: tap for one step. CONT: hold to move.";

  return oss.str();
}

std::string PrintHelper::build_keyboard_twist_instructions(bool gripper_enabled) {
  using M = KeyboardMapping;
  std::ostringstream oss;

  oss << Color::RESET << build_keyboard_layout(ControlMode::BASE, gripper_enabled) << "\nTWIST MOVEMENT:\n"
      << "  Linear X:  [" << M::x_positive << "] positive / [" << M::x_negative << "] negative\n"
      << "  Linear Y:  [" << M::y_positive << "] positive / [" << M::y_negative << "] negative\n"
      << "  Linear Z:  [" << M::z_positive << "] positive / [" << M::z_negative << "] negative\n"
      << "  Angular X: [" << M::roll_positive << "] positive / [" << M::roll_negative << "] negative\n"
      << "  Angular Y: [" << M::pitch_positive << "] positive / [" << M::pitch_negative << "] negative\n"
      << "  Angular Z: [" << M::yaw_positive << "] positive / [" << M::yaw_negative << "] negative\n"
      << "One key at a time. STEP: tap for one step. CONT: hold to move.";

  return oss.str();
}

}  // namespace teleop2servo
