#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/home_manager.hpp"

namespace teleop2servo {

HomeManager::HomeManager(rclcpp::Node& node, const HomeConfig& config, const std::vector<std::string>& joint_names)
    : node_(node), config_(config), joint_names_(joint_names) {
  client_ = rclcpp_action::create_client<MoveGroup>(&node_, config_.move_group_action);
}

bool HomeManager::start(DoneCallback on_done) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto timeout = std::chrono::duration<double>(config_.server_timeout_s);
  if (!client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_.get_logger(), "Action %s not available.", config_.move_group_action.c_str());
    return false;
  }

  const std::uint64_t goal_id = ++goal_id_;
  goal_handle_.reset();
  on_done_ = std::move(on_done);

  rclcpp_action::Client<MoveGroup>::SendGoalOptions options;

  options.goal_response_callback = [this, goal_id](const GoalHandle::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node_.get_logger(), "Go home goal rejected by %s.", config_.move_group_action.c_str());
      finish(goal_id, false);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_id != goal_id_) {
      // cancel() came before the goal was accepted
      client_->async_cancel_goal(goal_handle);
      return;
    }
    goal_handle_ = goal_handle;
  };

  options.result_callback = [this, goal_id](const GoalHandle::WrappedResult& result) {
    const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
                         result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (!success) {
      const int error_code = result.result ? result.result->error_code.val : 0;
      RCLCPP_ERROR(node_.get_logger(), "Go home failed (result code: %d, MoveIt error code: %d).",
                   static_cast<int>(result.code), error_code);
    }
    finish(goal_id, success);
  };

  RCLCPP_INFO(node_.get_logger(), "Going home (velocity scaling: %.2f, acceleration scaling: %.2f)...",
              config_.max_velocity_scaling, config_.max_acceleration_scaling);

  client_->async_send_goal(build_goal(), options);
  return true;
}

void HomeManager::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);

  ++goal_id_;
  on_done_ = nullptr;

  if (goal_handle_) {
    RCLCPP_WARN(node_.get_logger(), "Go home cancelled.");
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

HomeManager::MoveGroup::Goal HomeManager::build_goal() const {
  MoveGroup::Goal goal;

  auto& request = goal.request;
  request.group_name = config_.planning_group;
  request.num_planning_attempts = 1;
  request.allowed_planning_time = config_.planning_time_s;
  request.max_velocity_scaling_factor = config_.max_velocity_scaling;
  request.max_acceleration_scaling_factor = config_.max_acceleration_scaling;
  request.start_state.is_diff = true;  // plan from the current state

  moveit_msgs::msg::Constraints constraints;
  for (std::size_t i = 0; i < joint_names_.size() && i < config_.joint_positions.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = joint_names_[i];
    joint_constraint.position = config_.joint_positions[i];
    joint_constraint.tolerance_above = config_.joint_tolerance;
    joint_constraint.tolerance_below = config_.joint_tolerance;
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);
  }
  request.goal_constraints.push_back(constraints);

  goal.planning_options.plan_only = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  return goal;
}

void HomeManager::finish(std::uint64_t goal_id, bool success) {
  DoneCallback on_done;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_id != goal_id_)
      return;

    goal_handle_.reset();
    on_done = std::move(on_done_);
    on_done_ = nullptr;
  }

  // outside the lock: on_done may call back into this class
  if (on_done)
    on_done(success);
}

}  // namespace teleop2servo
