#ifndef TELEOP2SERVO__TELEOP_PUBLISHER_HPP_
#define TELEOP2SERVO__TELEOP_PUBLISHER_HPP_

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

class TeleopPublisher
{
public:
    TeleopPublisher(rclcpp::Node & node);
    ~TeleopPublisher() override;

    TeleopState get_teleop_state();
    void set_teleop_state(TeleopState state);

private:
    // ==== init ====
    void load_teleop_parameters();
    void setup_publishers();
    void setup_timers();

    template<typename T>
    void load_param(const std::string& name, T& value);


    // ==== callbacks / main loops ====
    void publish_loop();

    // ==== mode/state changes ====
    void switch_control_mode();
    void switch_speed_mode();
    void stop_motion();

    // ==== publishing ====
    void publish_stop_once(const rclcpp::Time & now);
    void publish_joint(const rclcpp::Time & now, const ActiveCmd & cmd);
    void publish_twist(const rclcpp::Time & now, const ActiveCmd & cmd);

    // ==== logging ====
    void print_instructions();


private:
    TeleopConfig config_;
    TeleopState state_;

    // protect state_
    std::mutex state_mutex_;

    // ==== ROS entities ====
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

} // namespace teleop2servo


#endif  // TELEOP2SERVO__TELEOP_PUBLISHER_HPP_
