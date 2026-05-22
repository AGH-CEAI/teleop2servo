#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <string>

#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/teleop_gamepad_node.hpp"

namespace teleop2servo{

TeleopGamepadNode::TeleopGamepadNode(const rclcpp::NodeOptions& options)
    : Node("gamepad_teleop_node", options),
    teleop_publisher_(*this, TeleopDevice::GAMEPAD)
{
    load_gamepad_parameters();
    setup_subscribers();
}

TeleopGamepadNode::~TeleopGamepadNode() = default;

template<typename T>
void TeleopGamepadNode::load_param(const std::string& name, T& value)
{
    this->declare_parameter<T>(name, value);
    this->get_parameter(name, value);
}

void TeleopGamepadNode::load_gamepad_parameters()
{
    load_param("joy_topic", gamepad_config_.joy_topic);
}

void TeleopGamepadNode::setup_subscribers()
{
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    gamepad_config_.joy_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&TeleopGamepadNode::joy_callback, this, std::placeholders::_1)
    );
}


bool TeleopGamepadNode::button_pressed(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Button button) const
{
    const int index = static_cast<int>(button);
    return msg &&
        static_cast<size_t>(index) < msg->buttons.size() &&
        msg->buttons[index] != 0;
}

bool TeleopGamepadNode::rising_edge(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Button button) const
{
    const bool now_pressed = button_pressed(msg, button);
    const bool was_pressed = button_pressed(previous_joy_msg_, button);
    return now_pressed && !was_pressed;
}

std::optional<Button> TeleopGamepadNode::rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg) const
{
    for (const auto button : magic_enum::enum_values<Button>()) {
        if (rising_edge(msg, button)) {
            return button;
        }
    }
    return std::nullopt;
}


double TeleopGamepadNode::axis_value(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Axis axis) const
{
    const int index = static_cast<int>(axis);
    if (!msg || index < 0 || static_cast<size_t>(index) >= msg->axes.size()) return 0.0;

    return static_cast<double>(msg->axes[index]);
}

bool TeleopGamepadNode::joy_in_use(
    const sensor_msgs::msg::Joy::SharedPtr & msg) const
{
    if (!msg) return false;
    for (const auto& button : msg->buttons) {
        if (button != 0) {
            return true;
        }
    }
    for (const auto& axis : msg->axes) {
        if (std::fabs(axis) > gamepad_config_.EPS) {
            return true;
        }
    }
    return false;
}

bool TeleopGamepadNode::check_safety_procedure(const sensor_msgs::msg::Joy::SharedPtr & msg)
{
    const bool back_left = button_pressed(msg, GamepadMapping::safety_left);
    const bool back_right = button_pressed(msg, GamepadMapping::safety_right);
    const bool b_pressed = rising_edge(msg, GamepadMapping::block_device);
    const bool enable_sequence = back_left && back_right && b_pressed;

    if (!teleop_publisher_.get_stop_button_pressed()) return true;

    if (enable_sequence) teleop_publisher_.unblock_teleop_device();

    return false;
}

bool TeleopGamepadNode::check_state_buttons(
    const sensor_msgs::msg::Joy::SharedPtr & msg
)
{
    if(const auto button = rising_edge(msg)){
        switch (*button) {
            case GamepadMapping::block_device:
                teleop_publisher_.block_teleop_device();
                return true;

            case GamepadMapping::switch_control_mode:
                teleop_publisher_.switch_control_mode();
                return true;

            case GamepadMapping::switch_speed_mode:
                teleop_publisher_.switch_speed_mode();
                return true;

            default:
                return false;
        }
    }
    return false;
}

double TeleopGamepadNode::button_pair_direction(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const Button positive,
    const Button negative,
    const SpeedMode speed_mode) const
{
    bool pos = false;
    bool neg = false;

    if (speed_mode == SpeedMode::STEP) {
        pos = rising_edge(msg, positive);
        neg = rising_edge(msg, negative);
    } else {
        pos = button_pressed(msg, positive);
        neg = button_pressed(msg, negative);
    }

    if (pos == neg) {
        return 0.0;  // both pressed or both released
    }

    return pos ? 1.0 : -1.0;
}

std::pair<double, double> TeleopGamepadNode::axis_direction(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Axis x_axis,
    Axis y_axis,
    Button step_button,
    SpeedMode speed_mode) const
{
    auto apply_deadzone = [](double value) {
        constexpr double DEADZONE = 0.05;
        return std::abs(value) < DEADZONE ? 0.0 : value;
    };

    auto apply_axis_lock = [](double& x, double& y) {
        constexpr double DOMINANT = 0.8;
        constexpr double SECONDARY = 0.2;

        if (std::abs(x) >= DOMINANT && std::abs(y) <= SECONDARY) {
            x = (x >= 0.0) ? 1.0 : -1.0;
            y = 0.0;
        } else if (std::abs(y) >= DOMINANT && std::abs(x) <= SECONDARY) {
            x = 0.0;
            y = (y >= 0.0) ? 1.0 : -1.0;
        }
    };

    double x = apply_deadzone(axis_value(msg, x_axis));
    double y = apply_deadzone(axis_value(msg, y_axis));

    apply_axis_lock(x, y);

    if (speed_mode == SpeedMode::STEP && !rising_edge(msg, step_button)) {
        return {0.0, 0.0};
    }

    return {x, y};
}

void TeleopGamepadNode::create_cmd_joint(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    const auto & config = teleop_publisher_.get_config();

    cmd.joint_velocities.assign(config.joint_names.size(), 0.0);

    const double scale =
        (speed_mode == SpeedMode::STEP)
        ? config.joint_vel_step
        : config.joint_vel_cont_max * get_speed_val(speed_mode);

    const bool modifier = button_pressed(msg, GamepadMapping::joint_modifier);

    auto set_joint = [this, &cmd, &msg, speed_mode, scale](std::size_t i, Button positive, Button negative) {
        if (i >= cmd.joint_velocities.size()) return;

        const double direction = button_pair_direction(msg, positive, negative, speed_mode);
        cmd.joint_velocities[i] = direction * scale;
    };

    set_joint(0, GamepadMapping::joint_1_positive, GamepadMapping::joint_1_negative);
    set_joint(1, GamepadMapping::joint_2_positive, GamepadMapping::joint_2_negative);

    if (!modifier) {
        set_joint(2, GamepadMapping::joint_3_positive, GamepadMapping::joint_3_negative);
        set_joint(3, GamepadMapping::joint_4_positive, GamepadMapping::joint_4_negative);
    } else {
        set_joint(4,  GamepadMapping::joint_5_positive, GamepadMapping::joint_5_negative);
        set_joint(5,  GamepadMapping::joint_6_positive, GamepadMapping::joint_6_negative);
    }

    for (double v : cmd.joint_velocities) {
        if (std::abs(v) > gamepad_config_.EPS) {
            cmd.type = ActiveCmdType::JOINT;
            return;
        }
    }
}

void TeleopGamepadNode::create_cmd_twist(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const ControlMode control_mode,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    const auto & config = teleop_publisher_.get_config();

    cmd.twist_msg.header.frame_id =
        (control_mode == ControlMode::BASE) ? config.base_frame_id : config.ee_frame_id;

    const double scale_lin =
        (speed_mode == SpeedMode::STEP)
        ? config.twist_lin_step
        : config.twist_lin_cont_max * get_speed_val(speed_mode);

    const double scale_ang =
        (speed_mode == SpeedMode::STEP)
        ? config.twist_ang_step
        : config.twist_ang_cont_max * get_speed_val(speed_mode);

    const auto [linear_y_dir, linear_x_dir] =
        axis_direction(
            msg,
            GamepadMapping::x_axis,
            GamepadMapping::y_axis,
            GamepadMapping::linear_step_button,
            speed_mode);

    const double linear_z_dir =
        button_pair_direction(
            msg,
            GamepadMapping::z_positive,
            GamepadMapping::z_negative,
            speed_mode);

    const auto [angular_y_dir, angular_x_dir] =
        axis_direction(
            msg,
            GamepadMapping::roll_axis,
            GamepadMapping::pitch_axis,
            GamepadMapping::angular_step_button,
            speed_mode);

    const double angular_z_dir =
        button_pair_direction(
            msg,
            GamepadMapping::yaw_positive,
            GamepadMapping::yaw_negative,
            speed_mode);

    // prepare msg ('*(-1.0)' to change directoin for more intuitive)
    cmd.twist_msg.twist.linear.x = linear_x_dir * scale_lin * (-1.0);
    cmd.twist_msg.twist.linear.y = linear_y_dir * scale_lin * (-1.0);
    cmd.twist_msg.twist.linear.z = linear_z_dir * scale_lin;

    cmd.twist_msg.twist.angular.x = angular_x_dir * scale_ang;
    cmd.twist_msg.twist.angular.y = angular_y_dir * scale_ang;
    cmd.twist_msg.twist.angular.z = angular_z_dir * scale_ang;

    const bool any_motion =
        std::abs(cmd.twist_msg.twist.linear.x) > gamepad_config_.EPS ||
        std::abs(cmd.twist_msg.twist.linear.y) > gamepad_config_.EPS ||
        std::abs(cmd.twist_msg.twist.linear.z) > gamepad_config_.EPS ||
        std::abs(cmd.twist_msg.twist.angular.x) > gamepad_config_.EPS ||
        std::abs(cmd.twist_msg.twist.angular.y) > gamepad_config_.EPS ||
        std::abs(cmd.twist_msg.twist.angular.z) > gamepad_config_.EPS;

    if (any_motion) {
        cmd.type = ActiveCmdType::TWIST;
    }
}

void TeleopGamepadNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!previous_joy_msg_) {
        previous_joy_msg_ = msg;
        return;
    }

    if (!check_safety_procedure(msg)) {
        previous_joy_msg_ = msg;
        return;
    }

    ActiveCmd cmd = ActiveCmd();

    if (joy_in_use(msg)) {

        if (check_state_buttons(msg)) {
            previous_joy_msg_ = msg;
            return;
        }

        const ControlMode control_mode =
            teleop_publisher_.get_control_mode();

        const SpeedMode speed_mode =
            teleop_publisher_.get_speed_mode();

        if (control_mode == ControlMode::JOINT) {
            create_cmd_joint(msg, speed_mode, cmd);
        } else {
            create_cmd_twist(msg, control_mode, speed_mode, cmd);
        }
    }

    teleop_publisher_.set_active_cmd(cmd);
    previous_joy_msg_ = msg;
}

} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::TeleopGamepadNode)
