#include "cmr_rovernet_cpp/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace cmr_rovernet_cpp {
namespace {

speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported baud rate " + std::to_string(baud));
  }
}

[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::runtime_error(what + ": " + std::strerror(errno));
}

}  // namespace

SerialPort::~SerialPort() {close();}

void SerialPort::open(const std::string & path, int baud)
{
  close();
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw_errno("could not open port " + path);
  }
  struct termios tio {};
  if (::tcgetattr(fd, &tio) != 0) {
    ::close(fd);
    throw_errno("tcgetattr " + path);
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
  tio.c_cflag |= CS8;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  const speed_t sp = to_speed(baud);
  ::cfsetispeed(&tio, sp);
  ::cfsetospeed(&tio, sp);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    ::close(fd);
    throw_errno("tcsetattr " + path);
  }
  fd_ = fd;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::write_all(const uint8_t * data, std::size_t size)
{
  if (fd_ < 0) {
    throw std::runtime_error("port not open");
  }
  std::size_t n = 0;
  while (n < size) {
    const ssize_t r = ::write(fd_, data + n, size - n);
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN) {
        struct pollfd p {fd_, POLLOUT, 0};
        ::poll(&p, 1, 100);
        continue;
      }
      throw_errno("write failed");
    }
    n += static_cast<std::size_t>(r);
  }
}

std::size_t SerialPort::read(uint8_t * out, std::size_t size, std::chrono::milliseconds timeout)
{
  if (fd_ < 0) {
    throw std::runtime_error("port not open");
  }
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + timeout;
  std::size_t got = 0;
  while (got < size) {
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count();
    if (remaining <= 0) {
      break;
    }
    struct pollfd p {fd_, POLLIN, 0};
    const int pr = ::poll(&p, 1, static_cast<int>(remaining));
    if (pr < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw_errno("poll failed");
    }
    if (pr == 0) {
      break;
    }
    if (p.revents & (POLLERR | POLLNVAL)) {
      throw std::runtime_error("device reports readiness to read but returned no data "
                               "(device disconnected or multiple access on port?)");
    }
    const ssize_t r = ::read(fd_, out + got, size - got);
    if (r < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      throw_errno("read failed");
    }
    if (r == 0) {
      // pyserial raises SerialException in this case (hang-up).
      throw std::runtime_error("device reports readiness to read but returned no data "
                               "(device disconnected or multiple access on port?)");
    }
    got += static_cast<std::size_t>(r);
  }
  return got;
}

}  // namespace cmr_rovernet_cpp
