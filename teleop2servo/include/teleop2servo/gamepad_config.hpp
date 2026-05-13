#ifndef TELEOP2SERVO__GAMEPAD_CONFIG_HPP_
#define TELEOP2SERVO__GAMEPAD_CONFIG_HPP_

namespace teleop2servo
{// TODO (issue#8): Configure key mapping from external yaml file

enum class Axis : int
{
    left_stick_x = 0,
    left_stick_y = 1,
    right_pad_x = 2,
    right_pad_y = 3,
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
    left_trigger_click = 8,
    right_trigger_click = 9,
    left_arrow = 10,
    right_arrow = 11,
    power_on_off = 12,
    left_stick_click = 13,
    right_pad_click = 14,
    left_back_click = 15,
    right_back_click = 16,
    left_pad_top_click = 17,
    left_pad_down_click = 18,
    left_pad_left_click = 19,
    left_pad_rigth_click = 20,
};

} // namespace teleop2servo

#endif  // TELEOP2SERVO__GAMEPAD_CONFIG_HPP_
