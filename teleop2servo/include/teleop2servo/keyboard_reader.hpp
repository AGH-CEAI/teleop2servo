#ifndef TELEOP2SERVO__KEYBOARD_READER_HPP_
#define TELEOP2SERVO__KEYBOARD_READER_HPP_

#include <atomic>

#include <termios.h>

namespace teleop2servo
{

class KeyboardReader
{
public:
  KeyboardReader() = default;
  ~KeyboardReader();

  void start();
  void stop();

  bool read_key(char &c);

private:
  termios orig_{};
  std::atomic<bool> running_{false};
};

} //namespace teleop2servo

#endif  // TELEOP2SERVO__KEYBOARD_READER_HPP_
