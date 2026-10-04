#ifndef TELEOP2SERVO__KEYBOARD_READER_HPP_
#define TELEOP2SERVO__KEYBOARD_READER_HPP_

#include <atomic>
#include <optional>

#include <termios.h>

namespace teleop2servo {

class KeyboardReader {
 public:
  KeyboardReader() = default;
  ~KeyboardReader();

  void start();
  void stop();

  bool read_key(char& c);

  // Caps Lock LED state from /sys/class/leds; std::nullopt when no keyboard LED can be read.
  std::optional<bool> caps_lock_on() const;

 private:
  termios orig_{};
  std::atomic<bool> running_{false};
  int fd_{-1};
};

}  // namespace teleop2servo

#endif  // TELEOP2SERVO__KEYBOARD_READER_HPP_
