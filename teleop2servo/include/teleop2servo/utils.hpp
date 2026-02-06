#ifndef TELEOP2SERVO__UTILS_HPP_
#define TELEOP2SERVO__UTILS_HPP_

#include <string>

namespace teleop2servo
{

struct JointMove
{
  int joint;   // 1..6
  int sign;    // +1 or -1
};

enum class ControlMode { JOINTS, BASE };
enum class SpeedMode {STEP, CONT_SLOW };

enum class ActiveCmdType { NONE, JOINT, TWIST };

struct ActiveCmd
{
    ActiveCmdType type{ActiveCmdType::NONE};

    // joint command
    int joint_index{0};  // 0..5
    int joint_sign{+1};

    // twist command (base cartesian)
    double lin_x{0}, lin_y{0}, lin_z{0};
    double ang_x{0}, ang_y{0}, ang_z{0};
};

inline std::string to_string(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINTS: return "JOINTS";
    case ControlMode::BASE: return "BASE";
    default: return "UNKNOWN";
  }
}

inline std::string to_string(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP: return "STEP";
    case SpeedMode::CONT_SLOW: return "CONT_SLOW";
    default: return "UNKNOWN";
  }
}


} //namespace teleop2servo

#endif  // TELEOP2SERVO__UTILS_HPP_
