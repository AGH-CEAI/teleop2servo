#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <termios.h>
#include <unistd.h>

#include "teleop2servo/keyboard_reader.hpp"

namespace teleop2servo {

KeyboardReader::~KeyboardReader() {
  stop();
}

void KeyboardReader::start() {
  fd_ = ::open("/dev/tty", O_RDONLY | O_NONBLOCK);

  if (fd_ < 0) {
    throw std::runtime_error("Failed to open /dev/tty");
  }

  tcgetattr(fd_, &orig_);

  termios raw = orig_;
  raw.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fd_, TCSANOW, &raw);

  running_.store(true);
}

void KeyboardReader::stop() {
  running_.store(false);

  if (fd_ >= 0) {
    tcsetattr(fd_, TCSANOW, &orig_);
    ::close(fd_);
    fd_ = -1;
  }
}

bool KeyboardReader::read_key(char& c) {
  if (!running_.load() || fd_ < 0) {
    return false;
  }

  const int n = ::read(fd_, &c, 1);
  return n == 1;
}

std::optional<bool> KeyboardReader::caps_lock_on() const {
  bool led_found = false;
  std::error_code ec;

  for (const auto& entry : std::filesystem::directory_iterator("/sys/class/leds", ec)) {
    if (entry.path().filename().string().find("::capslock") == std::string::npos)
      continue;

    std::ifstream file(entry.path() / "brightness");
    int brightness = 0;
    if (!(file >> brightness))
      continue;

    led_found = true;
    if (brightness > 0)
      return true;
  }

  if (!led_found)
    return std::nullopt;
  return false;
}

}  // namespace teleop2servo
