#ifndef TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
#define TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/utils.hpp"
#include "teleop2servo/gamepad_config.hpp"

namespace teleop2servo
{

class GamepadTeleopNode : public rclcpp::Node
{
public:
    GamepadTeleopNode(const rclcpp::NodeOptions& options);
    ~GamepadTeleopNode() override;

private:
    TeleopConfig config_;

    TeleopState state_;

    sensor_msgs::msg::Joy::SharedPtr previous_joy_msg_; // only used inside joy_callback()

    // ==== ROS ====
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::mutex state_mutex_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    // ==== init ====
    template<typename T>
    void loadParam(const std::string& name, T& value);
    void load_parameters();
    void setup_subscribers();
    void setup_publishers();
    void setup_timers();

    void print_gamepad_layout_and_instructions();
    void print_joint_instructions();
    void print_cartesian_instructions();

    bool button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, Button button) const;
    bool rising_edge(const sensor_msgs::msg::Joy::SharedPtr msg, Button button) const;
    double axis_value(const sensor_msgs::msg::Joy::SharedPtr msg, Axis axis) const;
    bool check_state_buttons(const sensor_msgs::msg::Joy::SharedPtr msg);


    double button_pair_direction(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        Button positive,
        Button negative,
        SpeedMode speed_mode
    ) const;

    void create_cmd_joint(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        const SpeedMode speed_mode,
        ActiveCmd& cmd
    );

    void create_cmd_twist(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        const ControlMode control_mode,
        const SpeedMode speed_mode,
        ActiveCmd& cmd
    );

    bool joy_is_idle(const sensor_msgs::msg::Joy::SharedPtr msg) const;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void switch_control_mode();
    void switch_speed_mode();

    void stop_motion(const std::string &reason);

    double joint_vel_for_speed_mode(SpeedMode speed_mode) const;
    double twist_lin_for_speed_mode(SpeedMode speed_mode) const;
    double twist_rot_for_speed_mode(SpeedMode speed_mode) const;

    void publish_loop();
    void publish_stop_once(const rclcpp::Time & now);
    void publish_joint(const rclcpp::Time & now, const ActiveCmd & cmd);
    void publish_twist(const rclcpp::Time & now, const ActiveCmd & cmd);
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
