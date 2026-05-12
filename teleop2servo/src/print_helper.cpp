#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo
{

void PrintHelper::print_gamepad_layout_and_instructions(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad)
{
    std::ostringstream oss;

    oss << build_gamepad_header(control_mode, speed_mode, stop_gamepad);

    if (speed_mode == true) {
        oss << build_gamepad_safety_procedure();
    }

    oss << build_footer();

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

std::string PrintHelper::build_gamepad_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad) const
{
    std::ostringstream oss;

    oss <<
    Color::BOLD
    <<"\n\n================ TELEOP GAMEPAD =================\n"
    << Color::RESET
    << R"(
            [BACK LEFT]      [BACK RIGHT]
    [ LB ]                               [ RB ]
    [ LT ]                               [ RT ]
        .---------------------------------.
      .'                                   '.
     /  LEFT D-PAD              RIGHT MOUSE  \
    /    [↑] [↓]      < ON >      (X / Y)     \
    |    [←] [→]                              |
    |                             )"
    << Color::YELLOW << "Y" << Color::RESET
    << R"(           |
    |        LEFT STICK        )"
    << Color::CYAN << "X" << Color::RESET
    << R"(     )"
    << Color::RED << "B" << Color::RESET
    << R"(        |
    |         (X / Y)             )"
    << Color::GREEN << "A" << Color::RESET
    << R"(           |
    |                                         |
    \         .---------------------.         /
     \       /                       \       /
      '-----'                         '-----')"
    << "\n---------------------------\n"
    << "Status: "
    << (stop_gamepad ? Color::RED : Color::RESET)
    << (stop_gamepad
        ? "BLOCKED"
        : "Gamepad ready")
    << Color::RESET
    << "\n---------------------------\n"
    << "Mode: "
    << Color::CYAN
    << to_string(control_mode)
    << Color::RESET
    << " | Speed: "
    << Color::YELLOW
    << to_string(speed_mode)
    << Color::RESET
    << "\n---------------------------\n";

    return oss.str();
}

std::string PrintHelper::build_gamepad_safety_procedure() const
{
    std::ostringstream oss;

    oss <<
    "Enable gamepad: [BACK LEFT] + [BACK RIGHT] + (press) "
    << Color::RED << "B" << Color::RESET;

    return oss.str();
}

std::string PrintHelper::build_footer() const
{
    std::ostringstream oss;

    oss <<
    "\n---------------------------\n"
    << Color::RED << "Ctrl+C to exit." << Color::RESET;

    return oss.str();
}

std::string PrintHelper::build_gamepad_joint_instructions() const
{
    std::ostringstream oss;

    oss <<
    "\nJOINT MOVEMENT:\n"
    << "  J1: [LT] positive / [RT] negative\n"
    << "  J2: [LB] positive / [RB] negative\n"
    << "  J3: D-PAD [UP] positive / [DOWN] negative\n"
    << "  J4: D-PAD [LEFT] positive / [RIGHT] negative\n"
    << "  Hold [RIGHT MOUSE] to control J5/J6 instead:\n"
    << "    J5: D-PAD [UP] positive / [DOWN] negative\n"
    << "    J6: D-PAD [LEFT] positive / [RIGHT] negative\n";

    return oss.str();
}

std::string PrintHelper::build_gamepad_twist_instructions() const
{
    std::ostringstream oss;

    oss <<
    "\nTWIST MOVEMENT:\n"
    << "  Linear X/Y: LEFT STICK\n"
    << "  Linear Z: [LT] positive / [RT] negative\n"
    << "  Angular X/Y: RIGHT MOUSE\n"
    << "  Angular Z: [LB] positive / [RB] negative\n"
    << "  In STEP mode, press the stick/mouse button to apply one step.";

    return oss.str();
}


} // namespace teleop2servo
