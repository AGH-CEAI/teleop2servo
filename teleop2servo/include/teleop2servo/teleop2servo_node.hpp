#ifndef TELEOP2SERVO__TELEOP2SERVO_NODE_HPP_
#define TELEOP2SERVO__TELEOP2SERVO_NODE_HPP_

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
#include "teleop2servo/utils.hpp"


namespace teleop2servo
{

class Teleop2ServoNode : public rclcpp::Node
{
public:
  Teleop2ServoNode();
  ~Teleop2ServoNode() override;

private:
  ControlMode control_mode_{ControlMode::JOINTS};
  SpeedMode speed_mode_{SpeedMode::STEP};
  bool rotation_{false};

  rclcpp::Time last_input_time_;
  double stop_moving_timeout_s_{2.0};

  bool have_active_cmd_{false};
  bool step_pending_one_shot_{false};
  rclcpp::Time step_lock_time_;
  char step_lock_char_{0};

  // TESTING
  std::string active_cmd_testing;

  ActiveCmd active_cmd_;

  teleop2servo::KeyboardReader keyboard_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //testing msgs
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  int publish_hz_;
  int queue_size_;

  std::string twist_topic_;
  std::string joint_topic_;
  std::string base_frame_id_;
  std::string eef_frame_id_;

  std::vector<std::string> joint_names_;

  double joint_vel_step_;
  double joint_vel_cont_slow_;

  double twist_lin_step_;
  double twist_lin_cont_slow_;

  double twist_rot_step_;
  double twist_rot_cont_slow_;

  std::unordered_map<char, JointMove> joint_keymap_;

  void build_keymap();
  void print_instruction_and_status();
  void poll_keyboard();
  void switch_control_mode();
  void switch_speed_mode();
  void toggle_rotation();
  void handle_char_key(char c);
  void stop_motion(const std::string &reason);
  double joint_vel_for_speed_mode() const;
  void publish_loop();
};

} //namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP2SERVO_NODE_HPP_
