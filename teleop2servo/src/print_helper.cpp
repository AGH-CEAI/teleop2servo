#include <string>

#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo
{


void PrintHelper::print_gamepad_layout_and_instructions(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad)
{
    std::ostringstream oss;

    oss << build_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad);

    if (state_.stop_button_pressed == true) {
        oss << build_safety_procedure();
    }

    oss << build_footer();

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

std::string PrintHelper::build_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad) const
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
    << (state_.stop_button_pressed ? Color::RED : Color::RESET)
    << (state_.stop_button_pressed
        ? "BLOCKED"
        : "Gamepad ready")
    << Color::RESET
    << "\n---------------------------\n"
    << "Mode: "
    << Color::CYAN
    << to_string(state_.control_mode)
    << Color::RESET
    << " | Speed: "
    << Color::YELLOW
    << to_string(state_.speed_mode)
    << Color::RESET
    << "\n---------------------------\n";

    return oss.str();
}

std::string PrintHelper::build_safety_procedure() const
{

}

std::string PrintHelper::build_footer() const
{

}

};

} // namespace teleop2servo
