#ifndef TELEOP2SERVO__TELEOP_DEVICE_MAPPING_HPP_
#define TELEOP2SERVO__TELEOP_DEVICE_MAPPING_HPP_

namespace teleop2servo
{// TODO (issue#8): Configure key mapping from external yaml file

enum class Axis : int
{
    left_stick_x = 0,
    left_stick_y = 1,
    right_pad_x = 2,
    right_pad_y = 3,
    left_pad_x = 4,
    left_pad_y = 5,
    left_trigger = 6,
    right_trigger = 7
};

enum class Button : int
{
    left_pad_touch = 0,
    right_pad_touch = 1,
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
    left_pad_right_click = 20,
};

struct GamepadMapping
{
    static constexpr Button block_device = Button::b;
    static constexpr Button switch_control_mode = Button::x;
    static constexpr Button switch_speed_mode = Button::y;

    static constexpr Axis x_axis = Axis::left_stick_x;
    static constexpr Axis y_axis = Axis::left_stick_y;

    static constexpr Button z_positive = Button::right_trigger_click;
    static constexpr Button z_negative = Button::left_trigger_click;

    static constexpr Axis roll_axis = Axis::right_pad_x;
    static constexpr Axis pitch_axis = Axis::right_pad_y;

    static constexpr Button yaw_positive = Button::right_bumper;
    static constexpr Button yaw_negative = Button::left_bumper;

    static constexpr Button linear_step_button = Button::left_stick_click;
    static constexpr Button angular_step_button = Button::right_pad_click;

    static constexpr Button joint_modifier = Button::right_pad_click;

    static constexpr Button joint_1_positive = Button::right_trigger_click;
    static constexpr Button joint_1_negative = Button::left_trigger_click;

    static constexpr Button joint_2_positive = Button::right_bumper;
    static constexpr Button joint_2_negative = Button::left_bumper;

    static constexpr Button joint_3_positive = Button::left_pad_down_click;
    static constexpr Button joint_3_negative = Button::left_pad_top_click;

    static constexpr Button joint_4_positive = Button::left_pad_left_click;
    static constexpr Button joint_4_negative = Button::left_pad_right_click;

    static constexpr Button joint_5_positive = Button::left_pad_down_click;
    static constexpr Button joint_5_negative = Button::left_pad_top_click;

    static constexpr Button joint_6_positive = Button::left_pad_left_click;
    static constexpr Button joint_6_negative = Button::left_pad_right_click;

    static constexpr Button safety_left = Button::left_pad_left_click;
    static constexpr Button safety_right = Button::left_pad_right_click;
};

} // namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_DEVICE_MAPPING_HPP_
