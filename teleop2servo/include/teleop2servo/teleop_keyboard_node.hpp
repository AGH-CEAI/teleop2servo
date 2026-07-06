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
#include "teleop2servo/teleop_utils.hpp"


namespace teleop2servo
{

class TeleopKeyboardNode : public rclcpp::Node
{
public:
  TeleopKeyboardNode();
  ~TeleopKeyboardNode() override;

private:
  teleop2servo::KeyboardReader keyboard_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} //namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_KEYBOARD_NODE_HPP_
