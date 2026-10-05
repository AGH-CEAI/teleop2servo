#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "teleop2servo/gripper_manager.hpp"

namespace teleop2servo {

GripperManager::GripperManager(rclcpp::Node& node, const GripperConfig& config) : node_(node), config_(config) {
  client_ = rclcpp_action::create_client<GripperCommand>(&node_, config_.action_name);
}

GripperState GripperManager::get_state() const {
  return closed_.load() ? GripperState::CLOSED : GripperState::OPEN;
}

bool GripperManager::toggle() {
  const bool close = !closed_.load();

  if (!send_goal(close ? config_.close_position : config_.open_position))
    return false;

  closed_.store(close);
  return true;
}

// Non-blocking: the goal is sent and its result only logged, so input handling never waits for the gripper.
// A new goal preempts the previous one in GripperActionController.
bool GripperManager::send_goal(double position) {
  if (!client_->action_server_is_ready()) {
    RCLCPP_ERROR(node_.get_logger(), "Gripper action server %s not available.", config_.action_name.c_str());
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = config_.max_effort;

  auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

  options.goal_response_callback = [this](const GoalHandle::SharedPtr& handle) {
    if (!handle)
      RCLCPP_ERROR(node_.get_logger(), "Gripper goal rejected.");
  };

  options.result_callback = [this, position](const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED || result.code == rclcpp_action::ResultCode::CANCELED)
      return;

    // Aborted on stall is expected when the fingers stop on a grasped object.
    const bool stalled = result.result && result.result->stalled;
    RCLCPP_WARN(node_.get_logger(), "Gripper goal %.4f not reached%s.", position,
                stalled ? " (stalled, object grasped?)" : "");
  };

  client_->async_send_goal(goal, options);
  return true;
}

}  // namespace teleop2servo
