#ifndef TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
#define TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_utils.hpp"
#include "teleop2servo/gamepad_config.hpp"
#include "teleop2servo/print_helper.hpp"

namespace teleop2servo
{

class GamepadTeleopNode : public rclcpp::Node
{
public:
    GamepadTeleopNode(const rclcpp::NodeOptions& options);
    ~GamepadTeleopNode() override;

    private:
    // ==== init ====
    void load_parameters();
    void setup_subscribers();
    void setup_publishers();
    void setup_timers();

    template<typename T>
    void load_param(const std::string& name, T& value);

    // ==== callbacks / main loopps ====
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish_loop();

    // ==== input processing ====
    bool button_pressed(const sensor_msgs::msg::Joy::SharedPtr & msg, Button button) const;
    bool rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg, Button button) const;
    double axis_value(const sensor_msgs::msg::Joy::SharedPtr & msg, Axis axis) const;

    bool check_state_buttons(const sensor_msgs::msg::Joy::SharedPtr & msg);
    bool check_safety_procedure(const sensor_msgs::msg::Joy::SharedPtr & msg);
    bool joy_in_use(const sensor_msgs::msg::Joy::SharedPtr & msg) const;

    double button_pair_direction(
        const sensor_msgs::msg::Joy::SharedPtr & msg,
        Button positive,
        Button negative,
        SpeedMode speed_mode
    ) const;

    std::pair<double, double> axis_direction(
        const sensor_msgs::msg::Joy::SharedPtr & msg,
        Axis x_axis,
        Axis y_axis,
        Button step_button,
        SpeedMode speed_mode
    ) const;

    // ==== active command creation ====
    void create_cmd_joint(
        const sensor_msgs::msg::Joy::SharedPtr & msg,
        const SpeedMode speed_mode,
        ActiveCmd& cmd
    );

    void create_cmd_twist(
        const sensor_msgs::msg::Joy::SharedPtr & msg,
        const ControlMode control_mode,
        const SpeedMode speed_mode,
        ActiveCmd& cmd
    );

    // ==== mode/state changes ====
    void block_gamepad();
    void switch_control_mode();
    void switch_speed_mode();
    void stop_motion();

    // ==== publishing ====
    void publish_stop_once(const rclcpp::Time & now);
    void publish_joint(const rclcpp::Time & now, const ActiveCmd & cmd);
    void publish_twist(const rclcpp::Time & now, const ActiveCmd & cmd);

    // ==== logging ====
    void print_gamepad_layout_and_instructions();
    std::string build_header() const;
    std::string build_safety_procedure() const;
    std::string build_footer() const;

private:
    TeleopConfig config_;
    TeleopState state_;
    PrintHelper print_helper_;

    // previous_joy_msg_ is accessed only from joy_callback().
    sensor_msgs::msg::Joy::SharedPtr previous_joy_msg_;

    // protect state_
    std::mutex state_mutex_;

    // ==== ROS entities ====
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
