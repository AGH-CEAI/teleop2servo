#ifndef TELEOP2SERVO__TELEOP_GAMEPAD_NODE_HPP_
#define TELEOP2SERVO__TELEOP_GAMEPAD_NODE_HPP_

#include <string>
#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop2servo/teleop_config.hpp"
#include "teleop2servo/teleop_device_mapping.hpp"
#include "teleop2servo/teleop_publisher.hpp"
#include "teleop2servo/teleop_utils.hpp"

namespace teleop2servo
{

class TeleopGamepadNode : public rclcpp::Node
{
public:
    TeleopGamepadNode(const rclcpp::NodeOptions& options);
    ~TeleopGamepadNode() override;

private:
    // ==== init ====
    void load_gamepad_parameters();
    void setup_subscribers();

    template<typename T>
    void load_param(const std::string& name, T& value);

    // ==== callbacks / main loops ====
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // ==== input processing ====
    bool button_pressed(const sensor_msgs::msg::Joy::SharedPtr & msg, Button button) const;
    bool rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg, Button button) const;
    std::optional<Button> rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg) const;
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

private:
    GamepadConfig gamepad_config_;
    TeleopPublisher teleop_publisher_;

    // previous_joy_msg_ is accessed only from joy_callback().
    sensor_msgs::msg::Joy::SharedPtr previous_joy_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__TELEOP_GAMEPAD_NODE_HPP_
