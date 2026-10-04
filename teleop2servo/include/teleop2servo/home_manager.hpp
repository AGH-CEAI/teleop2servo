#ifndef TELEOP2SERVO__HOME_MANAGER_HPP_
#define TELEOP2SERVO__HOME_MANAGER_HPP_

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "teleop2servo/teleop_config.hpp"

namespace teleop2servo {

// Plans and executes a joint move to the home position with the MoveGroup action
// (the same as aegis_director RobotDirector::joint_move / pymoveit2 move_to_configuration).
class HomeManager {
 public:
  using DoneCallback = std::function<void(bool success)>;

  HomeManager(rclcpp::Node& node, const HomeConfig& config, const std::vector<std::string>& joint_names);

  // Sends the goal; on_done is called once with the result, unless cancel() is called first.
  bool start(DoneCallback on_done);
  void cancel();

 private:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveGroup>;

  MoveGroup::Goal build_goal() const;
  void finish(std::uint64_t goal_id, bool success);

 private:
  rclcpp::Node& node_;
  HomeConfig config_;
  std::vector<std::string> joint_names_;

  rclcpp_action::Client<MoveGroup>::SharedPtr client_;

  std::mutex mutex_;
  std::uint64_t goal_id_{0};  // bumped on start / cancel, so callbacks of an old goal are ignored
  GoalHandle::SharedPtr goal_handle_;
  DoneCallback on_done_;
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__HOME_MANAGER_HPP_
