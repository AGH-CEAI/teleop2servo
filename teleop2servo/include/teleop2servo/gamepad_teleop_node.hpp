#ifndef TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
#define TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include "teleop2servo/teleop_config.hpp"

namespace teleop2servo
{

class GamepadTeleopNode : public rclcpp::Node
{
public:
    GamepadTeleopNode(const rclcpp::NodeOptions& options);
    ~GamepadTeleopNode() override;

private:
    // ==== init ====
    TeleopConfig config_;

    template<typename T>
    void loadParam(const std::string& name, T& value);
    void load_parameters();
    void setup_subscribers();
    void setup_publishers();
    void setup_timers();

    void print_gamepad_layout_and_instructions();
    void print_joint_instructions();
    void print_cartesian_instructions();

    // ==== state ====
    ControlMode control_mode_{ControlMode::JOINT};
    SpeedMode speed_mode_{SpeedMode::STEP};

    ActiveCmdGamepad active_cmd_{};

    // ==== ROS ====
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::mutex state_mutex_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void switch_control_mode();
    void switch_speed_mode();

    void stop_motion(const std::string &reason);

    double joint_vel_for_speed_mode() const;
    double twist_lin_for_speed_mode() const;
    double twist_rot_for_speed_mode() const;

    void publish_loop();
    void publish_stop_once(const rclcpp::Time & now);
    void publish_joint(const rclcpp::Time & now);
    void publish_twist(const rclcpp::Time & now);
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
