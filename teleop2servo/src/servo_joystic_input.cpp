#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/servo_joystic_input.hpp"

namespace teleop2servo{

JoyToServoPubAegis::JoyToServoPubAegis(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options)
{
    RCLCPP_INFO(this->get_logger(), "##### Joy to servo node run successfully ######");
}

JoyToServoPubAegis::~JoyToServoPubAegis() = default;

} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::JoyToServoPubAegis)
