#include <fcntl.h>
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
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);   // raw mode, no echo
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // non-blocking
    running_.store(true);
}

void KeyboardReader::stop()
{
    running_.store(false);

    // Restore terminal mode
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

bool KeyboardReader::read_key(char &c)
{
    if (!running_.load()) return false;
    const int n = ::read(STDIN_FILENO, &c, 1);
    return n == 1;
}

} //namespace teleop2servo
