#ifndef TELEOP2SERVO__TELEOP_PUBLISHER_HPP_
#define TELEOP2SERVO__TELEOP_PUBLISHER_HPP_

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo {

class TeleopPublisher {
 public:
  explicit TeleopPublisher(rclcpp::Node& node, TeleopDevice teleop_device);
  ~TeleopPublisher();

  ControlMode get_control_mode() const;
  SpeedMode get_speed_mode() const;
  bool get_stop_button_pressed() const;
  const TeleopConfig& get_config() const;

  void stop_motion();
  void set_active_cmd(const ActiveCmd& cmd);
  void switch_control_mode();
  void switch_speed_mode();
  void block_teleop_device();
  void unblock_teleop_device();

 private:
  template <typename T>
  void load_param(const std::string& name, T& value);
  void load_parameters();
  void setup_publishers();
  void setup_timers();

  void publish_loop();

  void stop_motion_locked();  // call only with state_mutex_
  void publish_stop_once(const rclcpp::Time& now);
  void publish_joint(const rclcpp::Time& now, const ActiveCmd& cmd);
  void publish_twist(const rclcpp::Time& now, const ActiveCmd& cmd);

  void print_instructions();

 private:
  rclcpp::Node& node_;
  TeleopDevice teleop_device_;
  TeleopConfig config_;
  TeleopState state_;

  mutable std::mutex state_mutex_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_PUBLISHER_HPP_
