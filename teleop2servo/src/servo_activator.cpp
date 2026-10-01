#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/servo_activator.hpp"

namespace teleop2servo {

ServoActivator::ServoActivator(rclcpp::Node& node, const ServoActivationConfig& config)
    : node_(node), config_(config) {
  callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_.get_node_base_interface());

  if (!config_.switch_controller_service.empty()) {
    switch_client_ = node_.create_client<SwitchController>(config_.switch_controller_service,
                                                           rmw_qos_profile_services_default, callback_group_);
  }
  if (!config_.start_servo_service.empty()) {
    start_servo_client_ =
        node_.create_client<Trigger>(config_.start_servo_service, rmw_qos_profile_services_default, callback_group_);
  }
  if (!config_.stop_servo_service.empty()) {
    stop_servo_client_ =
        node_.create_client<Trigger>(config_.stop_servo_service, rmw_qos_profile_services_default, callback_group_);
  }
}

bool ServoActivator::activate() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (active_)
    return true;

  RCLCPP_INFO(node_.get_logger(), "Activating MoveIt Servo...");

  if (!switch_controllers(config_.activate_controllers, config_.deactivate_controllers))
    return false;

  if (start_servo_client_ && !trigger(start_servo_client_)) {
    switch_controllers(config_.deactivate_controllers, config_.activate_controllers);
    return false;
  }

  active_ = true;
  RCLCPP_INFO(node_.get_logger(), "MoveIt Servo active.");
  return true;
}

void ServoActivator::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!active_)
    return;

  RCLCPP_INFO(node_.get_logger(), "Deactivating MoveIt Servo, restoring initial controllers...");

  if (stop_servo_client_)
    trigger(stop_servo_client_);

  switch_controllers(config_.deactivate_controllers, config_.activate_controllers);

  active_ = false;
}

bool ServoActivator::switch_controllers(const std::vector<std::string>& activate,
                                        const std::vector<std::string>& deactivate) {
  if (!switch_client_ || (activate.empty() && deactivate.empty()))
    return true;

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(config_.service_timeout_s);

  const auto response = call<SwitchController>(switch_client_, request);
  if (!response)
    return false;

  if (!response->ok) {
    RCLCPP_ERROR(node_.get_logger(), "Switching controllers failed (%s).", switch_client_->get_service_name());
    return false;
  }
  return true;
}

bool ServoActivator::trigger(const rclcpp::Client<Trigger>::SharedPtr& client) {
  const auto response = call<Trigger>(client, std::make_shared<Trigger::Request>());
  if (!response)
    return false;

  if (!response->success) {
    RCLCPP_ERROR(node_.get_logger(), "%s failed: %s", client->get_service_name(), response->message.c_str());
    return false;
  }
  return true;
}

template <typename ServiceT>
typename ServiceT::Response::SharedPtr ServoActivator::call(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                                                            const typename ServiceT::Request::SharedPtr& request) {
  const auto timeout = std::chrono::duration<double>(config_.service_timeout_s);

  if (!client->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_.get_logger(), "Service %s not available.", client->get_service_name());
    return nullptr;
  }

  auto result = client->async_send_request(request);
  if (executor_.spin_until_future_complete(result.future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(result);
    RCLCPP_ERROR(node_.get_logger(), "Service %s timed out.", client->get_service_name());
    return nullptr;
  }
  return result.future.get();
}

}  // namespace teleop2servo
