#ifndef TELEOP2SERVO__KEYBOARD_TELEOP_NODE_HPP_
#define TELEOP2SERVO__KEYBOARD_TELEOP_NODE_HPP_

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

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode();
  ~KeyboardTeleopNode() override;

private:
  // ==== Enum / state ====
  ControlMode control_mode_{ControlMode::JOINT};
  SpeedMode speed_mode_{SpeedMode::STEP};
  bool rotation_{false};

  ActiveCmd active_cmd_;
  bool have_active_cmd_{false};

  rclcpp::Time last_input_time_;
  
  char active_char_{0};
  bool continnuous_repeat_seen_{false};
  double initial_key_timeout_s_{0.4};

  bool step_pending_one_shot_{false};
  rclcpp::Time step_lock_time_;

  // ==== ROS interfaces ====
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  // ==== keyboard inpyt ====
  teleop2servo::KeyboardReader keyboard_;
  std::unordered_map<char, JointMove> joint_keymap_;
  
  // ==== parameters ====
  int publish_hz_;
  int queue_size_;
  
  double repeat_key_timeout_s_;
  
  std::string twist_topic_;
  std::string joint_topic_;
  std::string base_frame_id_;
  std::string ee_frame_id_;

  std::vector<std::string> joint_names_;

  double joint_vel_step_;
  double joint_vel_cont_max_;

  double twist_lin_step_;
  double twist_lin_cont_max_; 

  double twist_rot_step_;
  double twist_rot_cont_max_;

  void load_parameters();
  void build_keymap();
  void setup_publishers();
  void setup_timers();

  void print_instruction_and_status();
  void poll_keyboard();
  void switch_control_mode();
  void switch_speed_mode();
  void handle_char_key(char c);

  void stop_motion(const std::string &reason);

  double joint_vel_for_speed_mode() const;
  double twist_lin_for_speed_mode() const;
  double twist_rot_for_speed_mode() const;

  void publish_loop();
  void publish_stop_once(const rclcpp::Time & now);
  void publish_joint(const rclcpp::Time & now);
  void publish_twist(const rclcpp::Time & now);

} //namespace teleop2servo

#endif  // TELEOP2SERVO__KEYBOARD_TELEOP_NODE_HPP_
