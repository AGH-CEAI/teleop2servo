#ifndef TELEOP2SERVO__TELEOP_CONFIG_HPP_
#define TELEOP2SERVO__TELEOP_CONFIG_HPP_

#include <string>
#include <vector>

namespace teleop2servo
{

struct TeleopConfig
{
    double servo_publish_hz = 250.0;
    int servo_ticks_per_policy_step = 10;

    std::string twist_topic = "/servo_node/delta_twist_cmds";
    std::string joint_topic = "/servo_node/delta_joint_cmds";

    int queue_size = 10;

    std::string base_frame_id = "base_link";
    std::string ee_frame_id = "tool0";

    std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        double joint_vel_step = 0.1;
        double joint_vel_cont_max = 0.8;

        double twist_lin_step = 0.1;
        double twist_lin_cont_max = 0.5;

        double twist_ang_step = 0.1;
        double twist_ang_cont_max = 0.8;
};

struct GamepadConfig
{
        std::string joy_topic = "/joy";
        double EPS = 1e-7;
};

struct KeyboardConfig
{
        double reading_keyboard_hz = 250.0;
};

} // namespace teleop2servo

#endif  // TELEOP2SERVO__TELEOP_CONFIG_HPP_
