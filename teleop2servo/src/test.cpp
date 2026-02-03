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
#include <algorithm>

using namespace std::chrono_literals;

// ==============================
// KeyboardReader (jak u Ciebie)
// ==============================
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

// ==============================
// Mapping / Events
// ==============================
struct JointMove { int joint; int sign; };   // 1..6, +/-1

enum class ControlMode { JOINTS, BASE /*, TOOL (future) */ };
enum class SpeedMode   { STEP, CONT_SLOW /*, CONT_FAST (future) */ };

enum class SpecialKey {
  NONE,
  TAB,
  SHIFT_TAB,
  ARROW_UP,
  ARROW_DOWN,
  ARROW_LEFT,
  ARROW_RIGHT
};

static std::string toString(ControlMode m)
{
  switch (m) {
    case ControlMode::JOINTS: return "JOINTS";
    case ControlMode::BASE:   return "BASE";
  }
  return "UNKNOWN";
}

static std::string toString(SpeedMode m)
{
  switch (m) {
    case SpeedMode::STEP:      return "STEP";
    case SpeedMode::CONT_SLOW: return "CONT_SLOW";
  }
  return "UNKNOWN";
}

// ==============================
// Node
// ==============================
class Teleop2ServoNode : public rclcpp::Node
{
public:
  Teleop2ServoNode()
  : Node("teleop_keyboard",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    this->get_parameter_or("publish_hz", publish_hz_, 100);
    this->get_parameter_or("topic_out", topic_out_, std::string("/teleop_keyboard/event"));

    buildKeymap();

    pub_ = this->create_publisher<std_msgs::msg::String>(topic_out_, 10);

    keyboard_.start();
    g_keyboard = &keyboard_;
    signal(SIGINT, sigintHandler);

    // Timer do odczytu klawiatury (szybko, ale bez spiny)
    key_timer_ = this->create_wall_timer(
      5ms, std::bind(&Teleop2ServoNode::pollKeyboard, this));

    // Timer do CIĄGŁEGO publikowania
    const int hz = std::max(1, publish_hz_);
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / hz),
      std::bind(&Teleop2ServoNode::publishLoop, this));

    printInstructionAndStatus();
  }

  ~Teleop2ServoNode() override
  {
    keyboard_.stop();
  }

private:
  // ---------- FSM state ----------
  ControlMode control_mode_{ControlMode::JOINTS};
  SpeedMode speed_mode_{SpeedMode::STEP};
  bool rotation_{false};                 // dotyczy BASE (toggle 'R')
  bool dirty_ui_{false};                 // jeśli true -> wypisz instrukcję ponownie

  // Aktywna komenda "do serwa" (na razie tekst)
  bool have_active_cmd_{false};
  bool step_pending_one_shot_{false};    // w STEP publikujemy raz
  std::string active_cmd_;

  // ---------- Config ----------
  int publish_hz_{100};
  std::string topic_out_;

  // ---------- IO ----------
  KeyboardReader keyboard_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  std::unordered_map<char, JointMove> joint_keymap_;

private:
  void buildKeymap()
  {
    // JOINTS: 1/q -> J1, 2/w -> J2, ...
    joint_keymap_['1'] = {1, +1};  joint_keymap_['q'] = {1, -1};
    joint_keymap_['2'] = {2, +1};  joint_keymap_['w'] = {2, -1};
    joint_keymap_['3'] = {3, +1};  joint_keymap_['e'] = {3, -1};
    joint_keymap_['4'] = {4, +1};  joint_keymap_['r'] = {4, -1};
    joint_keymap_['5'] = {5, +1};  joint_keymap_['t'] = {5, -1};
    joint_keymap_['6'] = {6, +1};  joint_keymap_['y'] = {6, -1};
  }

  void printInstructionAndStatus()
  {
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "Controling robot via keyboard");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "Control mode: %s | Speed mode: %s | Rotation: %s",
                toString(control_mode_).c_str(),
                toString(speed_mode_).c_str(),
                rotation_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "TAB: switch control mode (JOINTS/BASE)");
    RCLCPP_INFO(get_logger(), "Shift+TAB: switch speed mode (STEP/CONT_SLOW)");
    RCLCPP_INFO(get_logger(), "S: (fallback) switch speed mode");
    RCLCPP_INFO(get_logger(), "SPACE: stop (clear active command)");
    RCLCPP_INFO(get_logger(), "R: toggle rotation flag (BASE only)");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "JOINTS:");
    RCLCPP_INFO(get_logger(), "  1/q -> J1 +/- | 2/w -> J2 +/- | 3/e -> J3 +/- | 4/r -> J4 +/- | 5/t -> J5 +/- | 6/y -> J6 +/-");
    RCLCPP_INFO(get_logger(), "BASE:");
    RCLCPP_INFO(get_logger(), "  Arrows -> X/Y | '.' ';' -> Z | (with Rotation=ON you interpret as rot axes)");
    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(), "Publishing to: %s | publish_hz=%d", topic_out_.c_str(), publish_hz_);
    RCLCPP_INFO(get_logger(), "Ctrl+C to exit.");
    RCLCPP_INFO(get_logger(), "");
  }

  // ---------- Input decoding ----------
  // Czyta ESC-sekwencje (arrows / shift-tab) w trybie non-blocking
  SpecialKey tryReadSpecialFromEsc()
  {
    // Oczekujemy: ESC [ A/B/C/D (arrows) albo ESC [ Z (shift-tab)
    char c1, c2;
    if (!keyboard_.readKey(c1)) return SpecialKey::NONE;
    if (c1 != '[') return SpecialKey::NONE;
    if (!keyboard_.readKey(c2)) return SpecialKey::NONE;

    switch (c2) {
      case 'A': return SpecialKey::ARROW_UP;
      case 'B': return SpecialKey::ARROW_DOWN;
      case 'C': return SpecialKey::ARROW_RIGHT;
      case 'D': return SpecialKey::ARROW_LEFT;
      case 'Z': return SpecialKey::SHIFT_TAB; // typowo shift+tab
      default:  return SpecialKey::NONE;
    }
  }

  void pollKeyboard()
  {
    char c;
    while (keyboard_.readKey(c)) {
      // ESC sekwencje
      if (static_cast<unsigned char>(c) == 0x1B) { // ESC
        const SpecialKey sk = tryReadSpecialFromEsc();
        if (sk != SpecialKey::NONE) {
          handleSpecialKey(sk);
          continue;
        }
        continue;
      }

      // TAB
      if (c == '\t') {
        handleSpecialKey(SpecialKey::TAB);
        continue;
      }

      // SPACE -> STOP
      if (c == ' ') {
        stopMotion("SPACE stop");
        continue;
      }

      // Rotation toggle (duże R)
      if (c == 'R') {
        rotation_ = !rotation_;
        dirty_ui_ = true;
        continue;
      }

      // Fallback speed toggle (bo SHIFT solo zwykle niewykrywalny)
      if (c == 'S') {
        toggleSpeedMode();
        continue;
      }

      // Zwykłe klawisze mapowane na akcję
      handleCharKey(c);
    }

    if (dirty_ui_) {
      dirty_ui_ = false;
      printInstructionAndStatus();
    }
  }

  void handleSpecialKey(SpecialKey sk)
  {
    switch (sk) {
      case SpecialKey::TAB:
        toggleControlMode();
        return;
      case SpecialKey::SHIFT_TAB:
        toggleSpeedMode();
        return;
      case SpecialKey::ARROW_UP:
      case SpecialKey::ARROW_DOWN:
      case SpecialKey::ARROW_LEFT:
      case SpecialKey::ARROW_RIGHT:
        handleArrow(sk);
        return;
      default:
        return;
    }
  }

  void handleCharKey(char c)
  {
    if (control_mode_ == ControlMode::JOINTS) {
      auto it = joint_keymap_.find(c);
      if (it == joint_keymap_.end()) {
        debugUnknownKey(c);
        return;
      }

      const int joint = it->second.joint;
      const int sign = it->second.sign;

      // TODO: tutaj docelowo tworzysz JointJog do MoveIt Servo
      std::ostringstream ss;
      ss << "[JOINTS] J" << joint << " sign=" << (sign > 0 ? "+" : "-")
         << " | speed=" << toString(speed_mode_);

      setActiveCommand(ss.str());
      return;
    }

    if (control_mode_ == ControlMode::BASE) {
      // BASE: '.' i ';' na Z
      if (c == '.') {
        setActiveCommand(makeBaseCmd("Z", +1));
        return;
      }
      if (c == ';') {
        setActiveCommand(makeBaseCmd("Z", -1));
        return;
      }

      debugUnknownKey(c);
      return;
    }
  }

  void handleArrow(SpecialKey sk)
  {
    if (control_mode_ != ControlMode::BASE) {
      // arrows ignorujemy w JOINTS
      return;
    }

    switch (sk) {
      case SpecialKey::ARROW_UP:
        setActiveCommand(makeBaseCmd("Y", +1));
        break;
      case SpecialKey::ARROW_DOWN:
        setActiveCommand(makeBaseCmd("Y", -1));
        break;
      case SpecialKey::ARROW_LEFT:
        setActiveCommand(makeBaseCmd("X", -1));
        break;
      case SpecialKey::ARROW_RIGHT:
        setActiveCommand(makeBaseCmd("X", +1));
        break;
      default:
        break;
    }
  }

  std::string makeBaseCmd(const std::string &axis, int sign)
  {
    // TODO: tu docelowo TwistStamped do Servo (liniowy/katowy zależnie od rotation_)
    std::ostringstream ss;
    ss << "[BASE] axis=" << axis << " sign=" << (sign > 0 ? "+" : "-")
       << " | rotation=" << (rotation_ ? "ON" : "OFF")
       << " | speed=" << toString(speed_mode_);
    return ss.str();
  }

  void debugUnknownKey(char c)
  {
    const int ascii = static_cast<int>(static_cast<unsigned char>(c));
    if (c >= 32 && c <= 126) {
      RCLCPP_INFO(get_logger(), "Unknown key: '%c' (ascii=%d)", c, ascii);
    } else {
      RCLCPP_INFO(get_logger(), "Unknown key code: ascii=%d", ascii);
    }
  }

  // ---------- State transitions ----------
  void toggleControlMode()
  {
    if (control_mode_ == ControlMode::JOINTS) control_mode_ = ControlMode::BASE;
    else control_mode_ = ControlMode::JOINTS;

    // opcjonalnie: przy zmianie trybu czyścimy komendę
    stopMotion("mode switch");
    dirty_ui_ = true;
  }

  void toggleSpeedMode()
  {
    if (speed_mode_ == SpeedMode::STEP) speed_mode_ = SpeedMode::CONT_SLOW;
    else speed_mode_ = SpeedMode::STEP;

    // w STEP nie ma sensu trzymać aktywnej komendy “w nieskończoność”
    if (speed_mode_ == SpeedMode::STEP) {
      // zostawimy active_cmd_, ale oznaczymy że ma pójść one-shot przy kolejnym ustawieniu
      // (lub możesz stopMotion tutaj — zależnie od preferencji)
    }

    dirty_ui_ = true;
  }

  void setActiveCommand(const std::string &cmd)
  {
    active_cmd_ = cmd;
    have_active_cmd_ = true;

    if (speed_mode_ == SpeedMode::STEP) {
      step_pending_one_shot_ = true;   // opublikuj raz w publishLoop()
    } else {
      step_pending_one_shot_ = false;  // publikuj ciągle
    }
  }

  void stopMotion(const std::string &reason)
  {
    (void)reason;
    have_active_cmd_ = false;
    step_pending_one_shot_ = false;
    active_cmd_.clear();
  }

  // ---------- Continuous publish loop ----------
  void publishLoop()
  {
    if (!have_active_cmd_) return;

    std_msgs::msg::String msg;
    msg.data = active_cmd_;
    pub_->publish(msg);

    // STEP: po jednej publikacji czyścimy
    if (speed_mode_ == SpeedMode::STEP && step_pending_one_shot_) {
      step_pending_one_shot_ = false;
      have_active_cmd_ = false;
      active_cmd_.clear();
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
