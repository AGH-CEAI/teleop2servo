#ifndef TELEOP2SERVO__TELEOP_CONFIG_HPP_
#define TELEOP2SERVO__TELEOP_CONFIG_HPP_

#include <string>
#include <vector>

namespace teleop2servo {

struct ServoActivationConfig {
  bool enabled = false;
  double service_timeout_s = 2.0;

  std::string switch_controller_service = "/controller_manager/switch_controller";
  std::vector<std::string> activate_controllers = {"forward_position_controller"};
  std::vector<std::string> deactivate_controllers = {"scaled_joint_trajectory_controller"};

  std::string start_servo_service = "/servo_node/start_servo";
  std::string stop_servo_service = "/servo_node/stop_servo";
};

struct HomeConfig {
  bool enabled = false;
  double server_timeout_s = 2.0;

  std::string move_group_action = "/move_action";
  std::string planning_group = "aegis_arm";

  // In the order of TeleopConfig::joint_names.
  std::vector<double> joint_positions = {0.0, -2.094395, 2.094395, -1.570796, -1.570796, 0.0};
  double joint_tolerance = 0.001;

  double max_velocity_scaling = 0.1;
  double max_acceleration_scaling = 0.1;
  double planning_time_s = 5.0;
};

struct TeleopConfig {
  double servo_publish_hz = 250.0;
  int servo_ticks_per_policy_step = 10;

  std::string twist_topic = "/servo_node/delta_twist_cmds";
  std::string joint_topic = "/servo_node/delta_joint_cmds";

  int queue_size = 10;

  std::string base_frame_id = "base_link";
  std::string ee_frame_id = "tool0";

  std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

  double joint_vel_step = 0.1;
  double joint_vel_cont_max = 0.8;

  double twist_lin_step = 0.1;
  double twist_lin_cont_max = 0.5;

  double twist_ang_step = 0.1;
  double twist_ang_cont_max = 0.8;

  ServoActivationConfig servo_activation;
  HomeConfig go_home;
};

struct GamepadConfig {
  std::string joy_topic = "/joy";
  static constexpr double EPS = 1e-7;
};

struct KeyboardConfig {
  double reading_keyboard_hz = 250.0;
  double key_initial_timeout_s = 0.55;
  double key_repeat_timeout_s = 0.1;
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_CONFIG_HPP_
