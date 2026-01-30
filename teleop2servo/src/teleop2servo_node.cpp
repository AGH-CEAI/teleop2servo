#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <string>
#include <unordered_map>
#include <sstream>

using namespace std::chrono_literals;

// ---------- KeyboardReader ----------
class KeyboardReader
{
public:
  void start()
  {
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);   // raw mode, no echo
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // non-blocking
    running_.store(true);
  }

  void stop()
  {
    running_.store(false);

    // Restore terminal mode
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
  }

  bool readKey(char &c)
  {
    if (!running_.load()) return false;
    const int n = ::read(STDIN_FILENO, &c, 1);
    return n == 1;
  }

private:
  termios orig_{};
  std::atomic<bool> running_{false};
};

static KeyboardReader* g_keyboard = nullptr;

void sigintHandler(int)
{
  if (g_keyboard) {
    g_keyboard->stop();
  }
  rclcpp::shutdown();
}

// ---------- Mapping ----------
struct JointMove
{
  int joint;   // 1..N
  int sign;    // +1 or -1
};

// ---------- Node ----------
class Teleop2ServoNode : public rclcpp::Node
{
public:
  Teleop2ServoNode()
  : Node("teleop_keyboard",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    this->get_parameter_or("publish_hz", publish_hz_, 100);

    buildKeymap();

    // Publisher + Subscriber for test
    pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_keyboard/event", 10);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/teleop_keyboard/event", 10,
      std::bind(&Teleop2ServoNode::onEvent, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Teleop keyboard started. publish_hz=%d", publish_hz_);
    RCLCPP_INFO(get_logger(), "Publishing events to: /teleop_keyboard/event");
    RCLCPP_INFO(get_logger(), "Keys: 1/q -> J1, 2/w -> J2, 3/e -> J3, 4/r -> J4, 5/t -> J5, 6/y -> J6");
    RCLCPP_INFO(get_logger(), "Press keys. Ctrl+C to exit.");

    // Start keyboard
    keyboard_.start();
    g_keyboard = &keyboard_;
    signal(SIGINT, sigintHandler);

    // Timer to poll keyboard
    const int hz = std::max(1, publish_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / hz),
      std::bind(&Teleop2ServoNode::pollKeyboard, this));
  }

  ~Teleop2ServoNode() override
  {
    keyboard_.stop();
  }

private:
  void buildKeymap()
  {
    keymap_['1'] = {1, +1};  keymap_['q'] = {1, -1};
    keymap_['2'] = {2, +1};  keymap_['w'] = {2, -1};
    keymap_['3'] = {3, +1};  keymap_['e'] = {3, -1};
    keymap_['4'] = {4, +1};  keymap_['r'] = {4, -1};
    keymap_['5'] = {5, +1};  keymap_['t'] = {5, -1};
    keymap_['6'] = {6, +1};  keymap_['y'] = {6, -1};
  }

  void pollKeyboard()
  {
    char c;
    while (keyboard_.readKey(c)) {
      auto it = keymap_.find(c);
      if (it == keymap_.end()) {
        // Show unknown key codes to help debug terminal sequences
        const int ascii = static_cast<int>(static_cast<unsigned char>(c));
        if (c >= 32 && c <= 126) {
          RCLCPP_INFO(get_logger(), "Unknown key: '%c' (ascii=%d)", c, ascii);
        } else {
          RCLCPP_INFO(get_logger(), "Unknown key code: ascii=%d", ascii);
        }
        continue;
      }

      const int joint = it->second.joint;
      const char sign_char = (it->second.sign > 0) ? '+' : '-';

      std_msgs::msg::String msg;
      std::ostringstream ss;
      ss << "Joint " << joint << " is moving " << sign_char;
      msg.data = ss.str();

      pub_->publish(msg);
    }
  }

  void onEvent(const std_msgs::msg::String::SharedPtr msg)
  {
    // Clean output in launch logs
    RCLCPP_INFO(get_logger(), "%s", msg->data.c_str());
  }

  KeyboardReader keyboard_;
  rclcpp::TimerBase::SharedPtr timer_;

  int publish_hz_{100};
  std::unordered_map<char, JointMove> keymap_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop2ServoNode>());
  rclcpp::shutdown();
  return 0;
}