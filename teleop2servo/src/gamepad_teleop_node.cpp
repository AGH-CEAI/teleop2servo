#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/gamepad_teleop_node.hpp"
#include "teleop2servo/utils.hpp"
#include "teleop2servo/teleop_config.hpp"

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
    "/joy",
    rclcpp::SensorDataQoS(),
    std::bind(&GamepadTeleopNode::joyCallback, this, _1)
    );
}

void GamepadTeleopNode::setup_publishers()
{
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, queue_size_);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_topic_, queue_size_);
}

void GamepadTeleopNode::setup_timers()
{
  const int hz = std::max(1, publish_hz_);
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / hz),
    std::bind(&KeyboardTeleopNode::publish_loop, this)
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
                     /      LEFT STICK       RIGHT STICK     \
                    /        (LX / LY)        (RX / RY)       \
                    |                                          |
                    |      D-PAD                   )"
                    COLOR_YELLOW "Y" COLOR_RESET R"(           |
                    |    [↑] [↓]               )"
                    COLOR_CYAN "X" COLOR_RESET R"(       )"
                           COLOR_RED "B" COLOR_RESET R"(       |
                    |    [←] [→]                   )"
                     COLOR_GREEN "A" COLOR_RESET R"(           |
                    \                                         /
                     '.                                     .'
                       '-----------------------------------'
    )");

    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(),
        "Mode: " COLOR_CYAN "MODE" COLOR_RESET
        " | Speed: " COLOR_YELLOW "SPEED" COLOR_RESET
    );
    RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);
}

void GamepadTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // TODO
}

void KeyboardTeleopNode::stop_motion(const std::string &reason)
{
    (void)reason;
    // TODO
}

void KeyboardTeleopNode::switch_control_mode()
{
    stop_motion("mode switched");
    control_mode_ = next(control_mode_);
    print_gamepad_layout_and_instructions();
}

void KeyboardTeleopNode::switch_speed_mode()
{
    stop_motion("speed switched");
    speed_mode_ = next(speed_mode_);
    print_gamepad_layout_and_instructions();
}

double KeyboardTeleopNode::joint_vel_for_speed_mode() const
{
    return (speed_mode_ == SpeedMode::STEP) ? joint_vel_step_ : joint_vel_cont_max_ * get_speed_val(speed_mode_);
}

double KeyboardTeleopNode::twist_lin_for_speed_mode() const
{
    return (speed_mode_ == SpeedMode::STEP) ? twist_lin_step_ : twist_lin_cont_max_ * get_speed_val(speed_mode_);
}

double KeyboardTeleopNode::twist_rot_for_speed_mode() const
{
    return (speed_mode_ == SpeedMode::STEP) ? twist_rot_step_ : twist_rot_cont_max_ * get_speed_val(speed_mode_);
}

void KeyboardTeleopNode::publish_stop_once(const rclcpp::Time & now)
{
    // TODO -> check
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = base_frame_id_;
  for (const auto & name : joint_names_){
    joint_msg.joint_names.push_back(name);
    joint_msg.velocities.push_back(0.0);
  }

  joint_pub_->publish(joint_msg);

  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = now;
  twist_msg.header.frame_id =
    active_cmd_.frame_id.empty() ? base_frame_id_ : active_cmd_.frame_id;

  twist_pub_->publish(twist_msg);
}

void KeyboardTeleopNode::publish_joint(const rclcpp::Time & now)
{
  if (joint_names_.size() < 6) {
    throw std::runtime_error("joint_names must contain at least 6 joints");
  }

  const int idx = active_cmd_.joint_index;
  const double vel = joint_vel_for_speed_mode() * static_cast<double>(active_cmd_.joint_sign);

  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = base_frame_id_;  // often BASE frame is used for joint jog
  joint_msg.joint_names.push_back(joint_names_.at(idx));
  joint_msg.velocities.push_back(vel);

  joint_pub_->publish(joint_msg);
}

void KeyboardTeleopNode::publish_twist(const rclcpp::Time & now)
{
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = now;
  twist_msg.header.frame_id =
    active_cmd_.frame_id.empty() ? base_frame_id_ : active_cmd_.frame_id;

  twist_msg.twist.linear.x = active_cmd_.lin_x * twist_lin_for_speed_mode();
  twist_msg.twist.linear.y = active_cmd_.lin_y * twist_lin_for_speed_mode();
  twist_msg.twist.linear.z = active_cmd_.lin_z * twist_lin_for_speed_mode();

  twist_msg.twist.angular.x = active_cmd_.ang_x * twist_rot_for_speed_mode();
  twist_msg.twist.angular.y = active_cmd_.ang_y * twist_rot_for_speed_mode();
  twist_msg.twist.angular.z = active_cmd_.ang_z * twist_rot_for_speed_mode();

  twist_pub_->publish(twist_msg);
}

void KeyboardTeleopNode::publish_loop()
{
  if (!have_active_cmd_) return;

  const auto now = this->now();
  const double dt = (now - last_input_time_).seconds();

  const double timeout_s = continuous_repeat_seen_ ? repeat_key_timeout_s_ : initial_key_timeout_s_;

  if (speed_mode_ != SpeedMode::STEP && dt > timeout_s) stop_motion("timeout");

  switch (active_cmd_.type)
  {
    case ActiveCmdType::NONE:
      publish_stop_once(now);
      have_active_cmd_ = false;
      return;

    case ActiveCmdType::JOINT:
      publish_joint(now);
      break;

    case ActiveCmdType::TWIST:
      publish_twist(now);
      break;
  }

  if (speed_mode_ == SpeedMode::STEP && step_ticks_remaining_ > 0)
  {
    --step_ticks_remaining_;
    if (step_ticks_remaining_ <= 0) stop_motion("one step");
  }
}


} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::GamepadTeleopNode)



// CONSTRUCT GEOMETRY MESSAGE

// control_msgs::msg::JointJog msg;

// msg.joint_names = config_.joint_names;
// msg.velocities = active_cmd_.joint_velocities;
