#ifndef TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_
#define TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/publisher.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "teleop2servo/keyboard_reader.hpp"
#include "teleop2servo/teleop_device_mapping.hpp"
#include "teleop2servo/teleop_publisher.hpp"
#include "teleop2servo/teleop_utils.hpp"


namespace teleop2servo
{

class TeleopKeyboardNode : public rclcpp::Node
{
public:
  TeleopKeyboardNode();
  ~TeleopKeyboardNode() override;

private:
  template <typename T>
  void load_param(const std::string& name, T& value);
  void load_keyboard_parameters();

  void setup_timers();

  void handle_key_input();


private:
  teleop2servo::KeyboardReader keyboard_;
  teleop2servo::KeyboardConfig keyboard_config_;
  rclcpp::TimerBase::SharedPtr key_timer_;

  TeleopPublisher teleop_publisher_;
  rclcpp::Time last_input_time_;

};

} //namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_
