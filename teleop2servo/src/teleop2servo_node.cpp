#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <string>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_TAB 0x09
#define KEYCODE_S 0x73
#define KEYCODE_SPACE 0x20
#define KEYCODE_DOT 0x2E
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79


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
  int joint;   // 1..6
  int sign;    // +1 or -1
};

enum class ControlMode { JOINTS, BASE };
enum class SpeedMode {STEP, CONT_SLOW };

enum class SpecialKey {
  NONE, 
  TAB,
  ARROW_UP,
  ARROW_DOWN,
  ARROW_RIGHT,
  ARROW_LEFT,
};

static std::string toString(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINTS: return "JOINTS";
    case ControlMode::BASE: return "BASE";
    default: return "UNKNOWN";
  }
}

static std::string toString(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP: return "STEP";
    case SpeedMode::CONT_SLOW: return "CONT_SLOW";
    default: return "UNKNOWN";
  }
}

enum class ActiveCmdType { NONE, JOINT, TWIST };

struct ActiveCmd
{
  ActiveCmdType type{ActiveCmdType::NONE};

  // joint command
  int joint_index{0};  // 0..5
  int joint_sign{+1};

  // twist command (base cartesian)
  double lin_x{0}, lin_y{0}, lin_z{0};
  double ang_x{0}, ang_y{0}, ang_z{0};
};


// ---------- Node ----------
class Teleop2ServoNode : public rclcpp::Node
{
public:
  Teleop2ServoNode()
  : Node("teleop_keyboard",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // load parameters
    this->get_parameter_or("publish_hz", publish_hz_, 100);
    this->get_parameter_or("stop_moving_timeout_s", stop_moving_timeout_s_, 2.0);

    this->get_parameter_or("twist_topic", twist_topic_, std::string("/servo_node/delta_twist_cmds"));
    this->get_parameter_or("joint_topic", joint_topic_, std::string("/servo_node/delta_joint_cmds"));
    this->get_parameter_or("queue_size", queue_size_, 10);

    this->get_parameter_or("base_frame_id", base_frame_id_, std::string("base"));
    this->get_parameter_or("eef_frame_id", eef_frame_id_, std::string("tool0"));

    this->get_parameter_or("joint_names", joint_names_, std::vector<std::string>{
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    });

    this->get_parameter_or("joint_vel_step", joint_vel_step_, 0.5);
    this->get_parameter_or("joint_vel_cont_slow", joint_vel_cont_slow_, 1.0);

    this->get_parameter_or("twist_lin_step", twist_lin_step_, 0.05);
    this->get_parameter_or("twist_lin_cont_slow", twist_lin_cont_slow_, 0.10);

    this->get_parameter_or("twist_rot_step", twist_rot_step_, 0.20);
    this->get_parameter_or("twist_rot_cont_slow", twist_rot_cont_slow_, 0.35);

    buildKeymap();
    
    // TESTING
    pub_ = this->create_publisher<std_msgs::msg::String>("/teleop_keyboard/event", 10);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, queue_size_);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_topic_, queue_size_);

    keyboard_.start();
    g_keyboard = &keyboard_;
    signal(SIGINT, sigintHandler);

    key_timer_ = this->create_wall_timer(
      5ms, std::bind(&Teleop2ServoNode::pollKeyboard, this)
    );

    const int hz = std::max(1, publish_hz_);
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / hz),
      std::bind(&Teleop2ServoNode::publishLoop, this)
    );
    
    last_input_time_ = this->now();
    printInstructionAndStatus();
  }

  ~Teleop2ServoNode() override
  {
    keyboard_.stop();
  }

private:
  ControlMode control_mode_{ControlMode::JOINTS};
  SpeedMode speed_mode_{SpeedMode::STEP};
  bool rotation_{false};

  rclcpp::Time last_input_time_;
  double stop_moving_timeout_s_{2.0};

  bool have_active_cmd_{false};
  bool step_pending_one_shot_{false};
  rclcpp::Time step_lock_time_;
  char step_lock_char_;
  
  // TESTING
  std::string active_cmd_testing;

  ActiveCmd active_cmd_;

  KeyboardReader keyboard_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  int publish_hz_;
  int queue_size_;

  std::string twist_topic_;
  std::string joint_topic_;
  std::string base_frame_id_;
  std::string eef_frame_id_;

  std::vector<std::string> joint_names_;

  double joint_vel_step_;
  double joint_vel_cont_slow_;

  double twist_lin_step_;
  double twist_lin_cont_slow_;

  double twist_rot_step_;
  double twist_rot_cont_slow_;

  std::unordered_map<char, JointMove> joint_keymap_;

  void buildKeymap()
  {
    joint_keymap_[static_cast<char>(KEYCODE_1)] = {1, +1};
    joint_keymap_[static_cast<char>(KEYCODE_Q)] = {1, -1};
    joint_keymap_[static_cast<char>(KEYCODE_2)] = {2, +1};
    joint_keymap_[static_cast<char>(KEYCODE_W)] = {2, -1};
    joint_keymap_[static_cast<char>(KEYCODE_3)] = {3, +1};
    joint_keymap_[static_cast<char>(KEYCODE_E)] = {3, -1};
    joint_keymap_[static_cast<char>(KEYCODE_4)] = {4, +1};
    joint_keymap_[static_cast<char>(KEYCODE_R)] = {4, -1};
    joint_keymap_[static_cast<char>(KEYCODE_5)] = {5, +1};
    joint_keymap_[static_cast<char>(KEYCODE_T)] = {5, -1};
    joint_keymap_[static_cast<char>(KEYCODE_6)] = {6, +1};
    joint_keymap_[static_cast<char>(KEYCODE_Y)] = {6, -1};
  }

  void printInstructionAndStatus() 
  {
    RCLCPP_INFO(get_logger(), "\n\n================ TELEOP KEYBOARD =================");
    RCLCPP_INFO(get_logger(), "Control mode : %s", toString(control_mode_).c_str());
    RCLCPP_INFO(get_logger(), "Speed mode   : %s", toString(speed_mode_).c_str());
    RCLCPP_INFO(get_logger(), "Rotation     : %s", rotation_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "TAB: switch control modes (JOINTS/BASE)");
    RCLCPP_INFO(get_logger(), "--- Joint control keymap:");
    RCLCPP_INFO(get_logger(), "1/q -> J1, 2/w -> J2, 3/e -> J3, 4/r -> J4, 5/t -> J5, 6/y -> J6");
    RCLCPP_INFO(get_logger(), "--- Base control keymap:");
    RCLCPP_INFO(get_logger(), "arrows -> axis X/Y, .; -> axis Z");
    RCLCPP_INFO(get_logger(), "With . switch on/off rotation around axis");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "'s' to switch speed");
    RCLCPP_INFO(get_logger(), "modes: STEP / CONT_SLOW");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "Ctrl+C to exit.");
  }

  void pollKeyboard()
  {
    char c;
    while (keyboard_.readKey(c)) {

      last_input_time_ = this->now();

      if (step_lock_char_ == c && (last_input_time_ - step_lock_time_).seconds() < 0.5) {
        step_lock_time_ = last_input_time_;
        continue;
      }
      
      // change the controll
      switch (c) {
        case KEYCODE_TAB: switchControlMode(); continue;
        case KEYCODE_S: switchSpeedMode(); continue;
        case KEYCODE_SPACE: stopMotion("space"); continue;
        default: handleCharKey(static_cast<char>(c)); continue;
       }
    }
  }

  void switchControlMode()
  {
    if (control_mode_ == ControlMode::JOINTS) control_mode_ = ControlMode::BASE;
    else control_mode_ = ControlMode::JOINTS;

    stopMotion("mode switch");
    printInstructionAndStatus();
  }

  void switchSpeedMode()
  {
    if (speed_mode_ == SpeedMode::STEP) speed_mode_ = SpeedMode::CONT_SLOW;
    else speed_mode_ = SpeedMode::STEP;
    
    stopMotion("speed switch");
    printInstructionAndStatus();
  }

  void toogleRotation()
  {
    rotation_ = !rotation_;
    stopMotion("rotation toggle");
    printInstructionAndStatus();
  }

  void handleCharKey(char c)
  {
    if (control_mode_ == ControlMode::JOINTS){
      auto it = joint_keymap_.find(c);
      if (it == joint_keymap_.end()){
        return;
      }
      const int joint = it->second.joint;
      const int sign = it->second.sign;

      {
        // TESTING
        std::ostringstream ss;
        ss << "[JOINTS] J" << joint << " sign=" << (sign > 0 ? "+" : "-")
          << " | speed=" << toString(speed_mode_);

        active_cmd_testing = ss.str();
        have_active_cmd_ = true;
      }

      // Validate joint_names_ size (expect 6)
      if (joint < 1 || joint > static_cast<int>(joint_names_.size())) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "joint_names has size %zu; key requested J%d",
                             joint_names_.size(), joint);
        return;
      }

      active_cmd_ = ActiveCmd{};
      active_cmd_.type = ActiveCmdType::JOINT;
      active_cmd_.joint_index = joint - 1;
      active_cmd_.joint_sign = sign;

      have_active_cmd_ = true;

      if (speed_mode_ == SpeedMode::STEP) {
        step_lock_time_ = this->now();
        step_lock_char_ = c;
        step_pending_one_shot_ = true;   // publick ones
      } else {
        step_pending_one_shot_ = false;  // publick continusly
      }
      return;
    }

    if (control_mode_ == ControlMode::BASE) {
      // TODO implement arrow controling
    } 
  }

  void stopMotion(const std::string &reason)
  {
    (void)reason;
    have_active_cmd_ = false;
    step_pending_one_shot_ = false;
    active_cmd_testing.clear();
    active_cmd_ = ActiveCmd{};
  }

  double jointVelForSpeedMode() const
  {
    return (speed_mode_ == SpeedMode::STEP) ? joint_vel_step_ : joint_vel_cont_slow_;
  }

  // ----- Continous publish loop -----
  void publishLoop()
  {
    if (!have_active_cmd_) return;

    const auto now = this->now();
    const double dt = (now - last_input_time_).seconds();

    if (speed_mode_ != SpeedMode::STEP && dt > stop_moving_timeout_s_) {
      stopMotion("timeout");
      return;
    }

    if (active_cmd_.type == ActiveCmdType::JOINT) {
      control_msgs::msg::JointJog msg;
      msg.header.stamp = now;
      msg.header.frame_id = base_frame_id_;  // often BASE frame is used for joint jog

      const int idx = active_cmd_.joint_index;
      const double vel = jointVelForSpeedMode() * static_cast<double>(active_cmd_.joint_sign);

      msg.joint_names.push_back(joint_names_.at(idx));
      msg.velocities.push_back(vel);

      joint_pub_->publish(msg);
    }

    // std_msgs::msg::String msg;
    // msg.data = active_cmd_;
    // pub_->publish(msg);

    if (speed_mode_ == SpeedMode::STEP && step_pending_one_shot_) {
      stopMotion("one step");
      return;
    }
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop2ServoNode>());
  rclcpp::shutdown();
  return 0;
}