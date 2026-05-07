#ifndef TELEOP2SERVO__UTILS_HPP_
#define TELEOP2SERVO__UTILS_HPP_

#include <string>
#include <vector>

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"

namespace teleop2servo
{
enum class ControlMode {JOINT, BASE, TOOL};
enum class SpeedMode {STEP, CONT_P5, CONT_P10, CONT_P25};

enum class ActiveCmdType { NONE, JOINT, TWIST };

struct ActiveCmdGamepad
{
  ActiveCmdType type{ActiveCmdType::NONE};

  std::vector<double> joint_velocities;

  geometry_msgs::msg::Twist twist;

  std::string frame_id{"base_link"};
};

inline double get_speed_val(SpeedMode m)
{
  switch (m) {
    case SpeedMode::CONT_P5:   return 0.05;
    case SpeedMode::CONT_P10:  return 0.10;
    case SpeedMode::CONT_P25:  return 0.25;
    default:                   return 0.0;
  }
}

inline ControlMode next(ControlMode mode)
{
    switch (mode)
    {
        case ControlMode::JOINT: return ControlMode::BASE;
        case ControlMode::BASE:   return ControlMode::TOOL;
        case ControlMode::TOOL:   return ControlMode::JOINT;
    }
    return ControlMode::JOINT; // fallback
}

inline SpeedMode next(SpeedMode mode)
{
  switch(mode)
  {
    case SpeedMode::STEP: return SpeedMode::CONT_P5;
    case SpeedMode::CONT_P5: return SpeedMode::CONT_P10;
    case SpeedMode::CONT_P10: return SpeedMode::CONT_P25;
    case SpeedMode::CONT_P25: return SpeedMode::STEP;
  }
  return SpeedMode::STEP; // fallback
}

inline std::string to_string(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINT: return "JOINT";
    case ControlMode::BASE: return "BASE";
    case ControlMode::TOOL: return "TOOL";
    default: return "UNKNOWN";
  }
}

inline std::string to_string(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP:      return "STEP";
    case SpeedMode::CONT_P5:   return "CONT 5%";
    case SpeedMode::CONT_P10:  return "CONT 10%";
    case SpeedMode::CONT_P25:  return "CONT 25%";
    default:                   return "UNKNOWN";
  }
}

} //namespace teleop2servo

#endif  // TELEOP2SERVO__UTILS_HPP_
