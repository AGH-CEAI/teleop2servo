#ifndef TELEOP2SERVO__GAMEPAD_CONFIG_HPP_
#define TELEOP2SERVO__GAMEPAD_CONFIG_HPP_

namespace teleop2servo
{

enum class Axis : int
{
    left_stick_x = 0,
    left_stick_y = 1,
    right_mouse_x = 2,
    right_mouse_y = 3,
    left_mouse_x = 4,
    left_mouse_y = 5,
    left_trigger = 6,
    right_trigger = 7
};

enum class Button : int
{
    left_mouse_touch = 0,
    right_mouse_touch = 1,
    a = 2,
    b = 3,
    x = 4,
    y = 5,
    left_bumper = 6,
    right_bumper = 7,
    left_trigger_button = 8,
    right_trigger_button = 9,
    left_arrow = 10,
    right_arrow = 11,
    power_on_off = 12,
    left_stick_button = 13,
    right_mouse_button = 14,
    left_back_button = 15,
    right_back_button = 16,
    left_mouse_top_button = 17,
    left_mouse_down_button = 18,
    left_mouse_left_button = 19,
    left_mouse_right_button = 20,
};

} // namespace teleop2servo

#endif  // TELEOP2SERVO__GAMEPAD_CONFIG_HPP_
