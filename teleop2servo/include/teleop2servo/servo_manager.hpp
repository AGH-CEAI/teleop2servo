#ifndef TELEOP2SERVO__SERVO_MANAGER_HPP_
#define TELEOP2SERVO__SERVO_MANAGER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "teleop2servo/teleop_config.hpp"

namespace teleop2servo {

class ServoManager {
 public:
  ServoManager(rclcpp::Node& node, const ServoActivationConfig& config);

  bool activate();
  void deactivate();

 private:
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using Trigger = std_srvs::srv::Trigger;

  bool switch_controllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate);
  bool trigger(const rclcpp::Client<Trigger>::SharedPtr& client);

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                                              const typename ServiceT::Request::SharedPtr& request);

 private:
  rclcpp::Node& node_;
  ServoActivationConfig config_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_servo_client_;

  std::mutex mutex_;
  bool active_{false};
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__SERVO_MANAGER_HPP_
