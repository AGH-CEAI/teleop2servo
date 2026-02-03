#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TeleopEventListener : public rclcpp::Node
{
public:
  TeleopEventListener()
  : Node("teleop_event_listener")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_keyboard/event",
      10,
      std::bind(&TeleopEventListener::onEvent, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Listening on: /teleop_keyboard/event");
  }

private:
  void onEvent(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "RX: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopEventListener>());
  rclcpp::shutdown();
  return 0;
}