#include <string>
#include <functional>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/teleop_publisher.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo{

TeleopPublisher::TeleopPublisher(rclcpp::Node & node, TeleopDevice teleop_device)
    : node_(node),
    teleop_device_(teleop_device)
{
    // TODO (issue#6) Change the controller for servo in constructor, after stopping change it back.
    load_parameters();
    setup_publishers();
    setup_timers();

    print_instructions();
}

TeleopPublisher::~TeleopPublisher() = default;

template<typename T>
void TeleopPublisher::load_param(const std::string& name, T& value)
{
    node_.declare_parameter<T>(name, value);
    node_.get_parameter(name, value);
}

void TeleopPublisher::load_parameters()
{
    load_param("servo_publish_hz", config_.servo_publish_hz);
    load_param("servo_ticks_per_policy_step", config_.servo_ticks_per_policy_step);

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

void TeleopPublisher::setup_publishers()
{
    twist_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(config_.twist_topic, config_.queue_size);
    joint_pub_ = node_.create_publisher<control_msgs::msg::JointJog>(config_.joint_topic, config_.queue_size);
}

void TeleopPublisher::setup_timers()
{
    const int hz = std::max(1.0, config_.servo_publish_hz);
    pub_timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(1000 / hz),
        std::bind(&TeleopPublisher::publish_loop, this)
    );
}

ControlMode TeleopPublisher::get_control_mode() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_.control_mode;
}

SpeedMode TeleopPublisher::get_speed_mode() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_.speed_mode;
}

bool TeleopPublisher::get_stop_button_pressed() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_.stop_button_pressed;
}

const TeleopConfig & TeleopPublisher::get_config() const
{
    return config_;
}

void TeleopPublisher::set_active_cmd(const ActiveCmd & cmd)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (cmd.type != ActiveCmdType::NONE)
    {
        state_.have_active_cmd = true;
        if (state_.speed_mode == SpeedMode::STEP) {
            state_.remaining_step_ticks = config_.servo_ticks_per_policy_step;
        } else {
            state_.remaining_step_ticks = 0;
        }
    }

    state_.active_cmd = cmd;
}

void TeleopPublisher::stop_motion()
{
    state_.active_cmd = ActiveCmd();
    state_.have_active_cmd = true; // once send zeros
    state_.remaining_step_ticks = 0;
}

void TeleopPublisher::switch_control_mode()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_motion();
        state_.control_mode = next(state_.control_mode);
    }
    print_instructions();
}

void TeleopPublisher::switch_speed_mode()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_motion();
        state_.speed_mode = next(state_.speed_mode);
    }
    print_instructions();
}

void TeleopPublisher::block_teleop_device()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_motion();
        state_.stop_button_pressed = true;
    }
    print_instructions();
}

void TeleopPublisher::unblock_teleop_device()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_motion();
        state_.stop_button_pressed = false;
    }
    print_instructions();
}


void TeleopPublisher::print_instructions()
{
    std::string str = PrintHelper::build_teleop_msg_layout_and_instructions(
        teleop_device_,
        get_control_mode(),
        get_speed_mode(),
        get_stop_button_pressed());
    RCLCPP_INFO(node_.get_logger(), "%s", str.c_str());
}

void TeleopPublisher::publish_stop_once(const rclcpp::Time & now)
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

void TeleopPublisher::publish_joint(
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

void TeleopPublisher::publish_twist(
    const rclcpp::Time & now,
    const ActiveCmd & cmd)
{
    auto twist_msg = cmd.twist_msg;
    twist_msg.header.stamp = now;

    twist_pub_->publish(twist_msg);
}

void TeleopPublisher::publish_loop()
{
    ActiveCmd cmd;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

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

    const auto now = node_.now();

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
