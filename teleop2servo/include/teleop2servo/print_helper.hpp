#ifndef TELEOP2SERVO__PRINT_HELPER_HPP_
#define TELEOP2SERVO__PRINT_HELPER_HPP_

#include <string>

#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo
{

class PrintHelper
{
public:
    static std::string build_teleop_msg_layout_and_instructions(
        TeleopDevice teleop_device,
        ControlMode control_mode,
        SpeedMode speed_mode,
        bool device_blocked);
private:
    static std::string build_gamepad_instructions(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad);
    static std::string build_gamepad_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad);
    static std::string build_gamepad_safety_procedure();
    static std::string build_gamepad_joint_instructions();
    static std::string build_gamepad_twist_instructions();
    static std::string build_footer();
};

} // namespace teleop2servo

#endif  // TELEOP2SERVO__PRINT_HELPER_HPP_
