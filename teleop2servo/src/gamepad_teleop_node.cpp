#include <string>
#include <functional>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>

#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/gamepad_teleop_node.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/gamepad_config.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo{

GamepadTeleopNode::GamepadTeleopNode(const rclcpp::NodeOptions& options)
    : Node("gamepad_teleop_node", options)
{
    // TODO (issue#6) Change the controller for servo in constructor, after stopping change it back.
    load_parameters();
    setup_subscribers();
    setup_publishers();
    setup_timers();

    print_gamepad_layout_and_instructions();
}

GamepadTeleopNode::~GamepadTeleopNode() = default;

template<typename T>
void GamepadTeleopNode::load_param(const std::string& name, T& value)
{
    this->declare_parameter<T>(name, value);
    this->get_parameter(name, value);
}

void GamepadTeleopNode::load_parameters()
{
    load_param("servo_publish_hz", config_.servo_publish_hz);
    load_param("servo_ticks_per_policy_step", config_.servo_ticks_per_policy_step);

    load_param("joy_topic", config_.joy_topic);
    load_param("twist_topic", config_.twist_topic);
    load_param("joint_topic", config_.joint_topic);
    load_param("queue_size", config_.queue_size);

    load_param("base_frame_id", config_.base_frame_id);
    load_param("ee_frame_id", config_.ee_frame_id);

    load_param("joint_names", config_.joint_names);

    load_param("joint_vel_step", config_.joint_vel_step);
    load_param("joint_vel_cont_max", config_.joint_vel_cont_max);

    load_param("twist_lin_step", config_.twist_lin_step);
    load_param("twist_lin_cont_max", config_.twist_lin_cont_max);

    load_param("twist_ang_step", config_.twist_ang_step);
    load_param("twist_ang_cont_max", config_.twist_ang_cont_max);
}

void GamepadTeleopNode::setup_subscribers()
{
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    config_.joy_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&GamepadTeleopNode::joy_callback, this, std::placeholders::_1)
    );
}

void GamepadTeleopNode::setup_publishers()
{
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(config_.twist_topic, config_.queue_size);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(config_.joint_topic, config_.queue_size);
}

void GamepadTeleopNode::setup_timers()
{
  const int hz = std::max(1.0, config_.servo_publish_hz);
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / hz),
    std::bind(&GamepadTeleopNode::publish_loop, this)
  );
}

void GamepadTeleopNode::print_gamepad_layout_and_instructions()
{
    TeleopState state;
    {
        std::scoped_lock lock(state_mutex_);
        state = state_;
    }
    std::string str = PrintHelper::print_gamepad_layout_and_instructions(state.control_mode, state.speed_mode, state.stop_button_pressed);
    RCLCPP_INFO(get_logger(), "%s", str.c_str());
}

bool GamepadTeleopNode::button_pressed(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Button button) const
{
    const int index = static_cast<int>(button);
    return msg &&
        static_cast<size_t>(index) < msg->buttons.size() &&
        msg->buttons[index] != 0;
}

bool GamepadTeleopNode::rising_edge(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Button button) const
{
    const bool now_pressed = button_pressed(msg, button);
    const bool was_pressed = button_pressed(previous_joy_msg_, button);
    return now_pressed && !was_pressed;
}

std::optional<Button> GamepadTeleopNode::rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg) const
{
    for (const auto button : magic_enum::enum_values<Button>()) {
        if (rising_edge(msg, button)) {
            return button;
        }
    }
    return std::nullopt;
}


double GamepadTeleopNode::axis_value(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    Axis axis) const
{
    const int index = static_cast<int>(axis);
    if (!msg || index < 0 || static_cast<size_t>(index) >= msg->axes.size()) return 0.0;

    return static_cast<double>(msg->axes[index]);
}

void GamepadTeleopNode::block_gamepad()
{
    {
        std::scoped_lock lock(state_mutex_);
        stop_motion();
        state_.stop_button_pressed = true;
    }
    print_gamepad_layout_and_instructions();
}

void GamepadTeleopNode::stop_motion()
{
    state_.active_cmd = ActiveCmd();
    state_.have_active_cmd = true; // once send zeros
}

void GamepadTeleopNode::switch_control_mode()
{
    {
        std::scoped_lock lock(state_mutex_);
        stop_motion();
        state_.control_mode = next(state_.control_mode);
    }
    print_gamepad_layout_and_instructions();
}

void GamepadTeleopNode::switch_speed_mode()
{
    {
        std::scoped_lock lock(state_mutex_);
        stop_motion();
        state_.speed_mode = next(state_.speed_mode);
    }
    print_gamepad_layout_and_instructions();
}

bool GamepadTeleopNode::joy_in_use(
    const sensor_msgs::msg::Joy::SharedPtr & msg) const
{
    if (!msg) return false;
    constexpr double eps = 1e-6;
    for (const auto& button : msg->buttons) {
        if (button != 0) {
            return true;
        }
    }
    for (const auto& axis : msg->axes) {
        if (std::fabs(axis) > eps) {
            return true;
        }
    }
    return false;
}

bool GamepadTeleopNode::check_safety_procedure(const sensor_msgs::msg::Joy::SharedPtr & msg)
{
    const bool back_left = button_pressed(msg, Button::left_back_click);
    const bool back_right = button_pressed(msg, Button::right_back_click);
    const bool b_pressed = rising_edge(msg, Button::b);
    const bool enable_sequence = back_left && back_right && b_pressed;
    {
        std::scoped_lock lock(state_mutex_);
        if (!state_.stop_button_pressed) return true;
        if (enable_sequence) state_.stop_button_pressed = false;
    }

    if (enable_sequence) print_gamepad_layout_and_instructions();

    return false;
}

bool GamepadTeleopNode::check_state_buttons(
    const sensor_msgs::msg::Joy::SharedPtr & msg
)
{
    if(const auto button = rising_edge(msg)){
        switch (*button) {
            case Button::b : block_gamepad(); return true;
            case Button::x : switch_control_mode(); return true;
            case Button::y : switch_speed_mode(); return true;
            default: return false;
        }
    }
    return false;
}

double GamepadTeleopNode::button_pair_direction(
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

std::pair<double, double> GamepadTeleopNode::axis_direction(
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
        constexpr double SEDONDARY = 0.2;

        if (std::abs(x) >= DOMINANT && std::abs(y) <= SEDONDARY) {
            x = (x >= 0.0) ? 1.0 : -1.0;
            y = 0.0;
        } else if (std::abs(y) >= DOMINANT && std::abs(x) <= SEDONDARY) {
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

void GamepadTeleopNode::create_cmd_joint(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    cmd.joint_velocities.assign(config_.joint_names.size(), 0.0);

    const double scale =
        (speed_mode == SpeedMode::STEP)
        ? config_.joint_vel_step
        : config_.joint_vel_cont_max * get_speed_val(speed_mode);

    const bool modifier = button_pressed(msg, Button::right_pad_click);

    auto set_joint = [this, &cmd, &msg, speed_mode, scale](std::size_t i, Button positive, Button negative) {
        if (i >= cmd.joint_velocities.size()) return;

        const double direction = button_pair_direction(msg, positive, negative, speed_mode);
        cmd.joint_velocities[i] = direction * scale;
    };

    set_joint(0, Button::left_trigger_click, Button::right_trigger_click);
    set_joint(1, Button::left_bumper, Button::right_bumper);

    if (!modifier) {
        set_joint(2, Button::left_pad_top_click, Button::left_pad_down_click); // j3
        set_joint(3, Button::left_pad_left_click, Button::left_pad_rigth_click);    // j4
    } else {
        set_joint(4, Button::left_pad_top_click, Button::left_pad_down_click); // j5
        set_joint(5, Button::left_pad_left_click, Button::left_pad_rigth_click);    // j6
    }

    bool any_motion = false;
    for (double v : cmd.joint_velocities) {
        if (std::abs(v) > 1e-9) {
            any_motion = true;
            break;
        }
    }
    if (any_motion) cmd.type = ActiveCmdType::JOINT;
}

void GamepadTeleopNode::create_cmd_twist(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const ControlMode control_mode,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    cmd.twist_msg.header.frame_id =
        (control_mode == ControlMode::BASE) ? config_.base_frame_id : config_.ee_frame_id;

    const double scale_lin =
        (speed_mode == SpeedMode::STEP)
        ? config_.twist_lin_step
        : config_.twist_lin_cont_max * get_speed_val(speed_mode);

    const double scale_ang =
        (speed_mode == SpeedMode::STEP)
        ? config_.twist_ang_step
        : config_.twist_ang_cont_max * get_speed_val(speed_mode);

    const auto [linear_y_dir, linear_x_dir] =
        axis_direction(
            msg,
            Axis::left_stick_x,
            Axis::left_stick_y,
            Button::left_stick_click,
            speed_mode
        );

    const double linear_z_dir =
        button_pair_direction(msg, Button::left_trigger_click, Button::right_trigger_click, speed_mode);

    const auto [angular_y_dir, angular_x_dir] =
        axis_direction(
            msg,
            Axis::right_pad_x,
            Axis::right_pad_y,
            Button::right_pad_click,
            speed_mode
        );

    const double angular_z_dir =
        button_pair_direction(msg, Button::left_bumper, Button::right_bumper, speed_mode);

    // prepare msg ('*(-1.0)' to change directoin for more intuitive)
    cmd.twist_msg.twist.linear.x = linear_x_dir * scale_lin * (-1.0);
    cmd.twist_msg.twist.linear.y = linear_y_dir * scale_lin * (-1.0);
    cmd.twist_msg.twist.linear.z = linear_z_dir * scale_lin;

    cmd.twist_msg.twist.angular.x = angular_x_dir * scale_ang;
    cmd.twist_msg.twist.angular.y = angular_y_dir * scale_ang;
    cmd.twist_msg.twist.angular.z = angular_z_dir * scale_ang;

    constexpr double EPS = 1e-7;
    const bool any_motion =
        std::abs(cmd.twist_msg.twist.linear.x) > EPS ||
        std::abs(cmd.twist_msg.twist.linear.y) > EPS ||
        std::abs(cmd.twist_msg.twist.linear.z) > EPS ||
        std::abs(cmd.twist_msg.twist.angular.x) > EPS ||
        std::abs(cmd.twist_msg.twist.angular.y) > EPS ||
        std::abs(cmd.twist_msg.twist.angular.z) > EPS;

    if (any_motion) {
        cmd.type = ActiveCmdType::TWIST;
    }
}

void GamepadTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
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

        ControlMode control_mode;
        SpeedMode speed_mode;
        {
            std::scoped_lock lock(state_mutex_);
            control_mode = state_.control_mode;
            speed_mode = state_.speed_mode;
        }


        if (control_mode == ControlMode::JOINT) {
            create_cmd_joint(msg, speed_mode, cmd);
        } else {
            create_cmd_twist(msg, control_mode, speed_mode, cmd);
        }

        if(cmd.type != ActiveCmdType::NONE)
        {
            std::scoped_lock lock(state_mutex_);
            state_.have_active_cmd = true;
            if (speed_mode == SpeedMode::STEP) {
                state_.remaining_step_ticks = config_.servo_ticks_per_policy_step;
            } else {
                state_.remaining_step_ticks = 0;
            }
        }
    }

    {
        std::scoped_lock lock(state_mutex_);
        state_.active_cmd = cmd;
    }

    previous_joy_msg_ = msg;
}


void GamepadTeleopNode::publish_stop_once(const rclcpp::Time & now)
    {
    auto joint_msg = control_msgs::msg::JointJog();
    joint_msg.header.stamp = now;
    joint_msg.header.frame_id = config_.base_frame_id;
    for (const auto & name : config_.joint_names){
        joint_msg.joint_names.push_back(name);
        joint_msg.velocities.push_back(0.0);
    }

    joint_pub_->publish(joint_msg);

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = config_.base_frame_id;

    twist_pub_->publish(twist_msg);
}

void GamepadTeleopNode::publish_joint(
    const rclcpp::Time & now,
    const ActiveCmd & cmd)
{
    auto joint_msg = control_msgs::msg::JointJog();
    joint_msg.header.stamp = now;
    joint_msg.header.frame_id = config_.base_frame_id;

    joint_msg.joint_names = config_.joint_names;
    joint_msg.velocities = cmd.joint_velocities;

    joint_pub_->publish(joint_msg);
}

void GamepadTeleopNode::publish_twist(
    const rclcpp::Time & now,
    const ActiveCmd & cmd)
{
    auto twist_msg = cmd.twist_msg;
    twist_msg.header.stamp = now;

    twist_pub_->publish(twist_msg);
}

void GamepadTeleopNode::publish_loop()
{
    ActiveCmd cmd;

    {
        std::scoped_lock lock(state_mutex_);

        if (!state_.have_active_cmd) return;

        cmd = state_.active_cmd;

        if (state_.remaining_step_ticks > 0) {
            --state_.remaining_step_ticks;
            if (state_.remaining_step_ticks <= 0) {
                state_.active_cmd = ActiveCmd();
                state_.have_active_cmd = true;
            }
        }
        else if (cmd.type == ActiveCmdType::NONE) {
            state_.have_active_cmd = false;         // once send zeros to servo
        }
    }

    const auto now = this->now();

    switch (cmd.type)
    {
        case ActiveCmdType::NONE:
        publish_stop_once(now);
        return;

        case ActiveCmdType::JOINT:
        publish_joint(now, cmd);
        break;

        case ActiveCmdType::TWIST:
        publish_twist(now, cmd);
        break;
    }
}


} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::GamepadTeleopNode)
