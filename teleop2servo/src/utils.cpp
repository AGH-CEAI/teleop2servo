#include <string>

#include "teleop2servo/utils.hpp"

namespace teleop2servo
{

std::string toString(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINTS: return "JOINTS";
    case ControlMode::BASE: return "BASE";
    default: return "UNKNOWN";
  }
}

std::string toString(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP: return "STEP";
    case SpeedMode::CONT_SLOW: return "CONT_SLOW";
    default: return "UNKNOWN";
  }
}

} //namespace teleop2servo
