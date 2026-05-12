#ifndef TELEOP2SERVO__PRINT_HELPER_HPP_
#define TELEOP2SERVO__PRINT_HELPER_HPP_

#include <string>

#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo
{

class PrintHelper
{
public:
    void print_gamepad_layout_and_instructions(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad);
    std::string build_gamepad_header(ControlMode control_mode, SpeedMode speed_mode, bool stop_gamepad) const;
    std::string build_gamepad_safety_procedure() const;
    std::string build_gamepad_joint_instructions() const;
    std::string build_gamepad_twist_instructions() const;
    std::string build_footer() const;
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__PRINT_HELPER_HPP_
