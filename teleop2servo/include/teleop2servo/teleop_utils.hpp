#ifndef TELEOP2SERVO__TELEOP_UTILS_HPP_
#define TELEOP2SERVO__TELEOP_UTILS_HPP_

#include <string>
#include <string_view>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>

struct Color
{
  static constexpr const char* RESET  = "\033[0m";
  static constexpr const char* RED    = "\033[31m";
  static constexpr const char* GREEN  = "\033[32m";
  static constexpr const char* YELLOW = "\033[33m";
  static constexpr const char* BLUE   = "\033[34m";
  static constexpr const char* CYAN   = "\033[36m";
  static constexpr const char* BOLD   = "\033[1m";
};


namespace teleop2servo
{

enum class ControlMode {JOINT, BASE, TOOL};
enum class SpeedMode {STEP, CONT_P5, CONT_P10, CONT_P25, CONT_P50};
enum class ActiveCmdType { NONE, JOINT, TWIST };

struct ActiveCmd
{
  ActiveCmdType type{ActiveCmdType::NONE};
  std::vector<double> joint_velocities;
  geometry_msgs::msg::TwistStamped twist_msg;
  std::string frame_id{"base_link"};
};

struct TeleopState
{
  ControlMode control_mode{ControlMode::JOINT};
  SpeedMode speed_mode{SpeedMode::STEP};
  ActiveCmd active_cmd;
  int remaining_step_ticks = 0;
  bool have_active_cmd{false};
  rclcpp::Time last_input_time;
};


constexpr double get_speed_val(SpeedMode m)
{
  switch (m) {
    case SpeedMode::CONT_P5:   return 0.05;
    case SpeedMode::CONT_P10:  return 0.10;
    case SpeedMode::CONT_P25:  return 0.25;
    case SpeedMode::CONT_P50:  return 0.50;
    default:                   return 0.0;
  }
}

constexpr ControlMode next(ControlMode mode)
{
    switch (mode)
    {
        case ControlMode::JOINT:  return ControlMode::BASE;
        case ControlMode::BASE:   return ControlMode::TOOL;
        case ControlMode::TOOL:   return ControlMode::JOINT;
        default:                  return ControlMode::JOINT;
    }
}

constexpr SpeedMode next(SpeedMode mode)
{
  switch(mode)
  {
    case SpeedMode::STEP:     return SpeedMode::CONT_P5;
    case SpeedMode::CONT_P5:  return SpeedMode::CONT_P10;
    case SpeedMode::CONT_P10: return SpeedMode::CONT_P25;
    case SpeedMode::CONT_P25: return SpeedMode::CONT_P50;
    case SpeedMode::CONT_P50: return SpeedMode::STEP;
    default:                  return SpeedMode::STEP;
  }
}

constexpr std::string_view to_string(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINT:  return "JOINT";
    case ControlMode::BASE:   return "BASE";
    case ControlMode::TOOL:   return "TOOL";
    default:                  return "UNKNOWN";
  }
}

constexpr std::string_view to_string(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP:      return "STEP";
    case SpeedMode::CONT_P5:   return "CONT 5%";
    case SpeedMode::CONT_P10:  return "CONT 10%";
    case SpeedMode::CONT_P25:  return "CONT 25%";
    case SpeedMode::CONT_P50:  return "CONT 50%";
    default:                   return "UNKNOWN";
  }
}

} //namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_UTILS_HPP_
