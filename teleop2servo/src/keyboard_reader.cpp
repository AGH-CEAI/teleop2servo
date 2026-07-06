#include <fcntl.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#include "teleop2servo/keyboard_reader.hpp"

namespace teleop2servo
{

KeyboardReader::~KeyboardReader()
{
    stop();
}

void KeyboardReader::start()
{
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

void KeyboardReader::stop()
{
    running_.store(false);

    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &orig_);
        ::close(fd_);
        fd_ = -1;
    }
}

bool KeyboardReader::read_key(char & c)
{
  if (!running_.load() || fd_ < 0) {
    return false;
  }

  const int n = ::read(fd_, &c, 1);
  return n == 1;
}

} //namespace teleop2servo
