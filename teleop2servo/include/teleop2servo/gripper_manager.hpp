#ifndef TELEOP2SERVO__GRIPPER_MANAGER_HPP_
#define TELEOP2SERVO__GRIPPER_MANAGER_HPP_

#include <atomic>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo {

class GripperManager {
 public:
  GripperManager(rclcpp::Node& node, const GripperConfig& config);

  bool toggle();
  GripperState get_state() const;

 private:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  bool send_goal(double position);

 private:
  rclcpp::Node& node_;
  GripperConfig config_;

  rclcpp_action::Client<GripperCommand>::SharedPtr client_;

  // Last commanded state; the gripper is assumed open on start.
  std::atomic<bool> closed_{false};
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__GRIPPER_MANAGER_HPP_
