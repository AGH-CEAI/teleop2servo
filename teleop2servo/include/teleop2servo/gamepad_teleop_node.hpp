#ifndef TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
#define TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

const std::string JOY_TOPIC = "/joy";


namespace teleop2servo
{
    class GamepadTeleopNode : public rclcpp::Node
    {
    public:
        GamepadTeleopNode(const rclcpp::NodeOptions& options);
        ~GamepadTeleopNode() override;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        void print_gamepad_layout();

    };

} // namespace teleop2servo


#endif  // TELEOP2SERVO__GAMEPAD_TELEOP_NODE_HPP_
