#include <string>
#include <sstream>

#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo {

std::string PrintHelper::build_teleop_msg_layout_and_instructions(TeleopDevice teleop_device,
                                                                  ControlMode control_mode,
                                                                  SpeedMode speed_mode,
                                                                  bool device_blocked) {
  std::ostringstream oss;
  switch (teleop_device) {
    case TeleopDevice::GAMEPAD:
      oss << build_gamepad_instructions(control_mode, speed_mode, device_blocked);
      break;
    default:
      oss << Color::BOLD << "\n\nERROR: TeleopDevice with this name not found...\n" << Color::RESET;
      break;
  }

  return oss.str();
}

std::string PrintHelper::build_gamepad_instructions(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad) {
  std::ostringstream oss;

  oss << build_gamepad_header(control_mode, speed_mode, stop_gamepad);

  if (stop_gamepad == true) {
    oss << build_gamepad_safety_procedure();
  } else if (control_mode == ControlMode::JOINT) {
    oss << build_gamepad_joint_instructions();
  } else {
    oss << build_gamepad_twist_instructions();
  }

  oss << build_footer();

  return oss.str();
}

std::string PrintHelper::build_gamepad_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad) {
  std::ostringstream oss;

  oss << Color::BOLD << "\n\n================ TELEOP GAMEPAD =================\n"
      << Color::RESET << R"(
           [BACK LEFT]      [BACK RIGHT]
    [ LT ]                               [ RT ]
    [ LB ]                               [ RB ]
        .---------------------------------.
      .'                                   '.
     /  LEFT D-PAD              RIGHT MOUSE  \
    /    [↑] [↓]      < ON >      (X / Y)     \
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
      << "Status: " << (stop_gamepad ? Color::RED : Color::RESET) << (stop_gamepad ? "BLOCKED" : "Gamepad ready")
      << Color::RESET << "\n---------------------------\n"
      << "Mode: " << Color::CYAN << to_string(control_mode) << Color::RESET << " | Speed: " << Color::YELLOW
      << to_string(speed_mode) << Color::RESET << "\n---------------------------\n"
      << Color::RED << "B" << Color::RESET << ": Block gamepad\n"
      << Color::CYAN << "X" << Color::RESET << ": Switch Mode (JOINT/BASE/TOOL)\n"
      << Color::YELLOW << "Y" << Color::RESET << ": Switch Speed (STEP/CONT 5%-100%)"
      << "\n---------------------------\n";

  return oss.str();
}

std::string PrintHelper::build_gamepad_safety_procedure() {
  std::ostringstream oss;

  oss << "Enable gamepad: [BACK LEFT] + [BACK RIGHT] + (press) " << Color::RED << "B" << Color::RESET;

  return oss.str();
}

std::string PrintHelper::build_footer() {
  std::ostringstream oss;

  oss << "\n---------------------------\n" << Color::RED << "Ctrl+C to exit." << Color::RESET;

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

}  // namespace teleop2servo
