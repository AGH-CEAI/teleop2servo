#include <string>
#include <functional>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/teleop_publisher.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo {

TeleopPublisher::TeleopPublisher(rclcpp::Node& node, TeleopDevice teleop_device)
    : node_(node), teleop_device_(teleop_device) {
  load_parameters();
  setup_publishers();
  setup_timers();
  setup_servo_activation();
  setup_go_home();
  setup_gripper();

  print_instructions();
}

TeleopPublisher::~TeleopPublisher() {
  const auto context = node_.get_node_base_interface()->get_context();
  context->remove_pre_shutdown_callback(pre_shutdown_handle_);

  if (rclcpp::ok(context))
    on_shutdown();
}

template <typename T>
void TeleopPublisher::load_param(const std::string& name, T& value) {
  node_.declare_parameter<T>(name, value);
  node_.get_parameter(name, value);
}

void TeleopPublisher::load_parameters() {
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

  auto& servo = config_.servo_activation;
  load_param("servo_activation.enabled", servo.enabled);
  load_param("servo_activation.service_timeout_s", servo.service_timeout_s);
  load_param("servo_activation.switch_controller_service", servo.switch_controller_service);
  load_param("servo_activation.activate_controllers", servo.activate_controllers);
  load_param("servo_activation.deactivate_controllers", servo.deactivate_controllers);
  load_param("servo_activation.start_servo_service", servo.start_servo_service);
  load_param("servo_activation.stop_servo_service", servo.stop_servo_service);

  auto& home = config_.go_home;
  load_param("go_home.enabled", home.enabled);
  load_param("go_home.server_timeout_s", home.server_timeout_s);
  load_param("go_home.move_group_action", home.move_group_action);
  load_param("go_home.planning_group", home.planning_group);
  load_param("go_home.joint_positions", home.joint_positions);
  load_param("go_home.joint_tolerance", home.joint_tolerance);
  load_param("go_home.max_velocity_scaling", home.max_velocity_scaling);
  load_param("go_home.max_acceleration_scaling", home.max_acceleration_scaling);
  load_param("go_home.planning_time_s", home.planning_time_s);

  auto& gripper = config_.gripper;
  load_param("gripper.enabled", gripper.enabled);
  load_param("gripper.action_name", gripper.action_name);
  load_param("gripper.open_position", gripper.open_position);
  load_param("gripper.close_position", gripper.close_position);
  load_param("gripper.max_effort", gripper.max_effort);
}

void TeleopPublisher::setup_publishers() {
  twist_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(config_.twist_topic, config_.queue_size);
  joint_pub_ = node_.create_publisher<control_msgs::msg::JointJog>(config_.joint_topic, config_.queue_size);
}

void TeleopPublisher::setup_timers() {
  const int hz = std::max(1.0, config_.servo_publish_hz);
  pub_timer_ =
      node_.create_wall_timer(std::chrono::milliseconds(1000 / hz), std::bind(&TeleopPublisher::publish_loop, this));
}

void TeleopPublisher::setup_servo_activation() {
  if (config_.servo_activation.enabled)
    servo_manager_ = std::make_unique<ServoManager>(node_, config_.servo_activation);

  pre_shutdown_handle_ =
      node_.get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() { on_shutdown(); });
}

void TeleopPublisher::setup_go_home() {
  const auto& home = config_.go_home;
  if (!home.enabled)
    return;

  if (home.joint_positions.size() != config_.joint_names.size()) {
    RCLCPP_ERROR(node_.get_logger(), "go_home.joint_positions has %zu values, joint_names has %zu. Go home disabled.",
                 home.joint_positions.size(), config_.joint_names.size());
    return;
  }

  home_manager_ = std::make_unique<HomeManager>(node_, home, config_.joint_names);
}

void TeleopPublisher::setup_gripper() {
  if (config_.gripper.enabled)
    gripper_manager_ = std::make_unique<GripperManager>(node_, config_.gripper);
}

void TeleopPublisher::on_shutdown() {
  stop_motion();

  if (home_manager_)
    home_manager_->cancel();

  if (servo_manager_)
    servo_manager_->deactivate();
}

ControlMode TeleopPublisher::get_control_mode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_.control_mode;
}

SpeedMode TeleopPublisher::get_speed_mode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_.speed_mode;
}

bool TeleopPublisher::is_device_blocked() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_.device_blocked;
}

bool TeleopPublisher::is_homing() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_.homing;
}

GripperState TeleopPublisher::get_gripper_state() const {
  return gripper_manager_ ? gripper_manager_->get_state() : GripperState::DISABLED;
}

const TeleopConfig& TeleopPublisher::get_config() const {
  return config_;
}

void TeleopPublisher::set_active_cmd(const ActiveCmd& cmd) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Servo is stopped while MoveIt drives the robot home.
  if (state_.homing)
    return;

  if (cmd.type == ActiveCmdType::NONE) {
    if (state_.remaining_step_ticks > 0)
      return;

    state_.active_cmd = cmd;
    return;
  }

  state_.have_active_cmd = true;
  if (state_.speed_mode == SpeedMode::STEP) {
    state_.remaining_step_ticks = config_.servo_ticks_per_policy_step;
  } else {
    state_.remaining_step_ticks = 0;
  }

  state_.active_cmd = cmd;
}

void TeleopPublisher::stop_motion_locked() {
  state_.active_cmd = ActiveCmd();
  state_.have_active_cmd = true;  // once send zeros
  state_.remaining_step_ticks = 0;
}

void TeleopPublisher::stop_motion() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  stop_motion_locked();
}

void TeleopPublisher::switch_control_mode() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_motion_locked();
    state_.control_mode = next(state_.control_mode);
  }
  print_instructions();
}

void TeleopPublisher::switch_speed_mode() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_motion_locked();
    state_.speed_mode = next(state_.speed_mode);
  }
  print_instructions();
}

void TeleopPublisher::block_teleop_device() {
  bool was_homing = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_motion_locked();
    state_.device_blocked = true;
    was_homing = std::exchange(state_.homing, false);
  }

  if (was_homing)
    home_manager_->cancel();

  if (servo_manager_)
    servo_manager_->deactivate();

  print_instructions();
}

void TeleopPublisher::unblock_teleop_device() {
  if (servo_manager_ && !servo_manager_->activate()) {
    RCLCPP_ERROR(node_.get_logger(), "MoveIt Servo activation failed, device stays blocked.");
    print_instructions();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_motion_locked();
    state_.device_blocked = false;
  }
  print_instructions();
}

// Servo is stopped and the initial controllers restored, then MoveIt moves the robot home.
// Afterwards Servo is activated again (see on_home_done).
void TeleopPublisher::go_home() {
  if (!home_manager_) {
    RCLCPP_WARN(node_.get_logger(), "Go home is disabled (go_home.enabled).");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_.device_blocked || state_.homing)
      return;

    stop_motion_locked();
    state_.homing = true;
  }

  if (servo_manager_)
    servo_manager_->deactivate();

  if (!home_manager_->start([this](bool success) { on_home_done(success); })) {
    on_home_done(false);
    return;
  }

  print_instructions();
}

void TeleopPublisher::on_home_done(bool success) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_.homing)
      return;  // cancelled by blocking the device

    state_.homing = false;
  }

  if (success)
    RCLCPP_INFO(node_.get_logger(), "Home position reached.");

  if (servo_manager_ && !servo_manager_->activate()) {
    RCLCPP_ERROR(node_.get_logger(), "MoveIt Servo activation failed, device blocked.");
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_motion_locked();
    state_.device_blocked = true;
  }

  print_instructions();
}

void TeleopPublisher::toggle_gripper() {
  if (!gripper_manager_) {
    RCLCPP_WARN(node_.get_logger(), "Gripper control is disabled (gripper.enabled: false).");
    return;
  }

  if (gripper_manager_->toggle())
    print_instructions();
}

void TeleopPublisher::print_instructions() {
  std::string str = PrintHelper::build_teleop_msg_layout_and_instructions(
      teleop_device_, get_control_mode(), get_speed_mode(), is_device_blocked(), get_gripper_state(), is_homing());
  RCLCPP_INFO(node_.get_logger(), "%s", str.c_str());
}

void TeleopPublisher::publish_loop() {
  ActiveCmd cmd;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!state_.have_active_cmd)
      return;

    cmd = state_.active_cmd;

    if (state_.remaining_step_ticks > 0) {
      --state_.remaining_step_ticks;
      if (state_.remaining_step_ticks <= 0) {
        state_.active_cmd = ActiveCmd();
        state_.have_active_cmd = true;
      }
    } else if (cmd.type == ActiveCmdType::NONE) {
      state_.have_active_cmd = false;  // once send zeros to servo
    }
  }

  const auto now = node_.now();

  switch (cmd.type) {
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

void TeleopPublisher::publish_stop_once(const rclcpp::Time& now) {
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = config_.base_frame_id;
  for (const auto& name : config_.joint_names) {
    joint_msg.joint_names.push_back(name);
    joint_msg.velocities.push_back(0.0);
  }

  joint_pub_->publish(joint_msg);

  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = now;
  twist_msg.header.frame_id = config_.base_frame_id;

  twist_pub_->publish(twist_msg);
}

void TeleopPublisher::publish_joint(const rclcpp::Time& now, const ActiveCmd& cmd) {
  auto joint_msg = control_msgs::msg::JointJog();
  joint_msg.header.stamp = now;
  joint_msg.header.frame_id = config_.base_frame_id;

  joint_msg.joint_names = config_.joint_names;
  joint_msg.velocities = cmd.joint_velocities;

  joint_pub_->publish(joint_msg);
}

void TeleopPublisher::publish_twist(const rclcpp::Time& now, const ActiveCmd& cmd) {
  auto twist_msg = cmd.twist_msg;
  twist_msg.header.stamp = now;

  twist_pub_->publish(twist_msg);
}

}  // namespace teleop2servo
