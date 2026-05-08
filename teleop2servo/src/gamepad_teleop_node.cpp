#include <string>
#include <functional>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/gamepad_teleop_node.hpp"
#include "teleop2servo/utils.hpp"
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/gamepad_config.hpp"

namespace teleop2servo{

GamepadTeleopNode::GamepadTeleopNode(const rclcpp::NodeOptions& options)
    : Node("gamepad_teleop", options)
{
    load_parameters();
    setup_subscribers();
    setup_publishers();
    setup_timers();

    print_gamepad_layout_and_instructions();
}

GamepadTeleopNode::~GamepadTeleopNode() = default;

template<typename T>
void GamepadTeleopNode::loadParam(const std::string& name, T& value)
{
    this->declare_parameter<T>(name, value);
    this->get_parameter(name, value);
}

void GamepadTeleopNode::load_parameters()
{
    loadParam("publish_hz", config_.publish_hz);
    loadParam("step_publish_ticks", config_.step_publish_ticks);

    loadParam("joy_topic", config_.joy_topic);
    loadParam("twist_topic", config_.twist_topic);
    loadParam("joint_topic", config_.joint_topic);
    loadParam("queue_size", config_.queue_size);

    loadParam("base_frame_id", config_.base_frame_id);
    loadParam("ee_frame_id", config_.ee_frame_id);

    loadParam("joint_names", config_.joint_names);

    loadParam("joint_vel_step", config_.joint_vel_step);
    loadParam("joint_vel_cont_max", config_.joint_vel_cont_max);

    loadParam("twist_lin_step", config_.twist_lin_step);
    loadParam("twist_lin_cont_max", config_.twist_lin_cont_max);

    loadParam("twist_rot_step", config_.twist_rot_step);
    loadParam("twist_rot_cont_max", config_.twist_rot_cont_max);
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
  const int hz = std::max(1.0, config_.publish_hz);
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / hz),
    std::bind(&GamepadTeleopNode::publish_loop, this)
  );
}

void GamepadTeleopNode::print_gamepad_layout_and_instructions()
{
    RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP GAMEPAD =================" COLOR_RESET);
    RCLCPP_INFO(get_logger(), R"(

                               [ BACK ]     [ START ]
                    [ LB ]                               [ RB ]
                    [ LT ]                               [ RT ]
                        .---------------------------------.
                      .'                                   '.
                     /    LEFT STICK           RIGHT STICK   \
                    /      (LX / LY)            (RX / RY)     \
                    |                                          |
                    |          D-PAD              )"
                    COLOR_YELLOW "Y" COLOR_RESET R"(            |
                    |         [↑] [↓]         )"
                    COLOR_CYAN "X" COLOR_RESET R"(       )"
                           COLOR_RED "B" COLOR_RESET R"(        |
                    \         [←] [→]             )"
                     COLOR_GREEN "A" COLOR_RESET R"(           /
                     '.                                     .'
                       '-----------------------------------'
    )");

    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(),
        "Mode: " COLOR_CYAN "%s" COLOR_RESET
        " | Speed: " COLOR_YELLOW "%s" COLOR_RESET,
        to_string(state_.control_mode).c_str(),
        to_string(state_.speed_mode).c_str()
    );
    RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);
}

bool GamepadTeleopNode::button_pressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    Button button) const
{
    const int index = static_cast<int>(button);
    return msg &&
        index >= 0 &&
        static_cast<size_t>(index) < msg->buttons.size() &&
        msg->buttons[index] != 0;
}

bool GamepadTeleopNode::rising_edge(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    Button button) const
{
    const bool now_pressed = button_pressed(msg, button);
    const bool was_pressed = button_pressed(previous_joy_msg_, button);
    return now_pressed && !was_pressed;
}

double GamepadTeleopNode::axis_value(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    Axis axis) const
{
    const int index = static_cast<int>(axis);
    if (!msg || index < 0 || static_cast<size_t>(index) >= msg->axes.size()) return 0.0;
    const double value = static_cast<double>(msg->axes[index]);
    return value;
}

void GamepadTeleopNode::stop_motion(const std::string &reason)
{
    (void)reason;
    {
        std::scoped_lock lock(state_mutex_);
        state_.active_cmd = ActiveCmd();
        state_.have_active_cmd = true; // once send zeros
    }
}

void GamepadTeleopNode::switch_control_mode()
{
    stop_motion("mode switched");
    {
        std::scoped_lock lock(state_mutex_);
        state_.control_mode = next(state_.control_mode);
    }
    print_gamepad_layout_and_instructions();
}

void GamepadTeleopNode::switch_speed_mode()
{
    stop_motion("speed switched");
    {
        std::scoped_lock lock(state_mutex_);
        state_.speed_mode = next(state_.speed_mode);
    }
    print_gamepad_layout_and_instructions();
}

double GamepadTeleopNode::joint_vel_for_speed_mode(SpeedMode speed_mode) const
{
    return (speed_mode == SpeedMode::STEP) ? config_.joint_vel_step : config_.joint_vel_cont_max * get_speed_val(speed_mode);
}

double GamepadTeleopNode::twist_lin_for_speed_mode(SpeedMode speed_mode) const
{
    return (speed_mode == SpeedMode::STEP) ? config_.twist_lin_step : config_.twist_lin_cont_max * get_speed_val(speed_mode);
}

double GamepadTeleopNode::twist_rot_for_speed_mode(SpeedMode speed_mode) const
{
    return (speed_mode == SpeedMode::STEP) ? config_.twist_rot_step : config_.twist_rot_cont_max * get_speed_val(speed_mode);
}

bool GamepadTeleopNode::joy_is_idle(
    const sensor_msgs::msg::Joy::SharedPtr msg) const
{
    if (!msg) return true;
    constexpr double eps = 1e-6;
    for (const auto& button : msg->buttons) {
        if (button != 0) {
            return false;
        }
    }
    for (const auto& axis : msg->axes) {
        if (std::fabs(axis) > eps) {
            return false;
        }
    }
    return true;
}

bool GamepadTeleopNode::check_state_buttons(
    const sensor_msgs::msg::Joy::SharedPtr msg
)
{
    if (rising_edge(msg, Button::b)){
        stop_motion("b pressed");
        previous_joy_msg_ = msg;
        return true;
    }
    if (rising_edge(msg, Button::x)){
        switch_control_mode();
        previous_joy_msg_ = msg;
        return true;
    }
    if (rising_edge(msg, Button::y)){
        switch_speed_mode();
        previous_joy_msg_ = msg;
        return true;
    }
    return false;
}

double GamepadTeleopNode::button_pair_direction(
    const sensor_msgs::msg::Joy::SharedPtr msg,
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

void GamepadTeleopNode::create_cmd_joint(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    cmd.type = ActiveCmdType::JOINT;
    cmd.joint_velocities.assign(config_.joint_names.size(), 0.0);

    const double scale =
        (speed_mode == SpeedMode::STEP)
            ? config_.joint_vel_step
            : config_.joint_vel_cont_max * get_speed_val(speed_mode);

    const bool modifier = button_pressed(msg, Button::right_mouse_button);

    auto set_joint = [&](std::size_t i, Button positive, Button negative) {
        if (i >= cmd.joint_velocities.size()) return;

        const double direction =
            button_pair_direction(msg, positive, negative, speed_mode);

        cmd.joint_velocities[i] = direction * scale;
    };

    set_joint(0, Button::left_trigger_button, Button::right_trigger_button);
    set_joint(1, Button::left_bumper, Button::right_bumper);

    if (modifier) {
        set_joint(4, Button::left_mouse_top_button, Button::left_mouse_down_button); // j5
        set_joint(5, Button::left_mouse_left_button, Button::left_mouse_right_button);    // j6
    } else {
        set_joint(2, Button::left_mouse_top_button, Button::left_mouse_down_button); // j3
        set_joint(3, Button::left_mouse_left_button, Button::left_mouse_right_button);    // j4
    }
}

void GamepadTeleopNode::create_cmd_twist(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const ControlMode control_mode,
    const SpeedMode speed_mode,
    ActiveCmd& cmd
)
{
    (void)msg;
    (void)control_mode;
    (void)speed_mode;
    (void)cmd;
    return;
}

void GamepadTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!previous_joy_msg_) {
        previous_joy_msg_ = msg;
        return;
    }

    ActiveCmd cmd = ActiveCmd();

    if (!joy_is_idle(msg)) {

        if (check_state_buttons(msg)) return;

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

        {
            std::scoped_lock lock(state_mutex_);
            state_.have_active_cmd = true;
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


void GamepadTeleopNode::publish_loop()
{
    ActiveCmd cmd;

    {
        std::scoped_lock lock(state_mutex_);

        if (!state_.have_active_cmd) return;

        cmd = state_.active_cmd;

        if (cmd.type == ActiveCmdType::NONE) {
            state_.have_active_cmd = false;   // once send zeros to servo
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
        // publish_twist(now, cmd);
        break;
    }
}


} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::GamepadTeleopNode)
