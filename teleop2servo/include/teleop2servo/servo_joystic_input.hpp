#ifndef TELEOP2SERVO__SERVO_JOYSTIC_INPUT_HPP_
#define TELEOP2SERVO__SERVO_JOYSTIC_INPUT_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

const std::string JOY_TOPIC = "/joy";


namespace teleop2servo
{
    class JoyToServoPubAegis : public rclcpp::Node
    {
    public:
        JoyToServoPubAegis(const rclcpp::NodeOptions& options);
        ~JoyToServoPubAegis() override;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;


    };

} // namespace teleop2servo


#endif  // TELEOP2SERVO__SERVO_JOYSTIC_INPUT_HPP_
