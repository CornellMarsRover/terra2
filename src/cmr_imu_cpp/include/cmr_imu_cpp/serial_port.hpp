// (Copy of the helper in cmr_servo_control_cpp; kept per-package.)
// Minimal POSIX termios serial port (raw 8N1, no flow control), the subset of
// pyserial behaviour the original Python nodes relied on, plus exclusive
// ownership: every open takes a non-blocking flock(LOCK_EX) on the port fd
// before any byte is sent (same convention as cmr_rovernet_cpp's DeviceLock),
// and optionally TIOCEXCL, so two nodes can never share one adapter.
#pragma once

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace cmr_port
{

// Thrown when the device is exclusively held by another process/node.
class PortBusyError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

inline speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default:
      throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud));
  }
}

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {close();}
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens the port in raw 8N1 mode and takes an exclusive flock on it (plus
  // TIOCEXCL if requested) before anything is written. Throws PortBusyError if
  // another process holds the port, std::runtime_error on any other failure.
  void open(const std::string & path, int baud, bool tty_exclusive = true)
  {
    close();  // releases our previous lock (closing the fd) before re-locking
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
      const int err = errno;
      if (err == EBUSY) {  // someone else set TIOCEXCL
        throw PortBusyError("port busy: " + path + " is held exclusively by another process");
      }
      throw std::runtime_error(
              "could not open port " + path + ": [Errno " + std::to_string(err) + "] " +
              std::strerror(err));
    }
    while (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      if (errno == EINTR) {continue;}
      const int err = errno;
      ::close(fd);
      if (err == EWOULDBLOCK) {
        throw PortBusyError(
                "port busy: " + path + " is locked by another process (another node owns it)");
      }
      throw std::runtime_error("flock " + path + ": " + std::strerror(err));
    }
    if (tty_exclusive) {
      ::ioctl(fd, TIOCEXCL);  // best effort; root bypasses it, flock still applies
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
      int err = errno;
      ::close(fd);
      throw std::runtime_error("tcgetattr(" + path + "): " + std::strerror(err));
    }
    ::cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
    tio.c_cflag |= CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    const speed_t speed = baud_to_speed(baud);
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
      int err = errno;
      ::close(fd);
      throw std::runtime_error("tcsetattr(" + path + "): " + std::strerror(err));
    }
    // pyserial resets the input buffer when opening a port.
    ::tcflush(fd, TCIFLUSH);
    fd_ = fd;
    path_ = path;
    rx_.clear();
  }

  bool is_open() const {return fd_ >= 0;}
  const std::string & path() const {return path_;}
  int fd() const {return fd_;}

  // true if @p path exists and is not held by another process right now.
  static bool try_probe(const std::string & path)
  {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {return false;}
    const bool ok = ::flock(fd, LOCK_EX | LOCK_NB) == 0;
    if (ok) {::flock(fd, LOCK_UN);}
    ::close(fd);
    return ok;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::ioctl(fd_, TIOCNXCL);
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
    rx_.clear();
  }

  // Waits up to timeout_ms (<0 = forever) for data and reads up to n bytes.
  // Returns number of bytes read (0 on timeout). Throws on I/O error.
  size_t read_some(uint8_t * buf, size_t n, int timeout_ms)
  {
    if (!rx_.empty()) {
      const size_t k = std::min(n, rx_.size());
      std::memcpy(buf, rx_.data(), k);
      rx_.erase(0, k);
      return k;
    }
    return raw_read(buf, n, timeout_ms);
  }

  // Reads exactly n bytes unless the overall timeout expires (<0 = forever).
  // Returns the number of bytes actually read (pyserial read(n) semantics).
  size_t read_exact(uint8_t * buf, size_t n, int timeout_ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    size_t got = 0;
    while (got < n) {
      int remaining = -1;
      if (timeout_ms >= 0) {
        remaining = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count());
        if (remaining <= 0) {break;}
      }
      const size_t k = read_some(buf + got, n - got, remaining);
      if (k == 0 && timeout_ms >= 0 &&
        std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      got += k;
    }
    return got;
  }

  // Reads one line terminated by '\n' (terminator kept). Returns false on timeout
  // (any partial data stays buffered). timeout_ms < 0 waits forever.
  bool read_line(std::string & line, int timeout_ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
      const auto pos = rx_.find('\n');
      if (pos != std::string::npos) {
        line = rx_.substr(0, pos + 1);
        rx_.erase(0, pos + 1);
        return true;
      }
      int remaining = -1;
      if (timeout_ms >= 0) {
        remaining = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count());
        if (remaining <= 0) {return false;}
      }
      uint8_t buf[512];
      const size_t k = raw_read(buf, sizeof(buf), remaining);
      rx_.append(reinterpret_cast<const char *>(buf), k);
    }
  }

  // Writes all bytes, waiting up to timeout_ms (<0 = forever) for the port to
  // accept them. Returns false on timeout. Throws on I/O error.
  bool write_all(const uint8_t * data, size_t n, int timeout_ms = -1)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    size_t sent = 0;
    while (sent < n) {
      if (fd_ < 0) {throw std::runtime_error("Attempting to use a port that is not open");}
      const ssize_t w = ::write(fd_, data + sent, n - sent);
      if (w > 0) {
        sent += static_cast<size_t>(w);
        continue;
      }
      if (w < 0 && errno != EAGAIN && errno != EINTR) {
        throw std::runtime_error(std::string("write failed: ") + std::strerror(errno));
      }
      int remaining = -1;
      if (timeout_ms >= 0) {
        remaining = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count());
        if (remaining <= 0) {return false;}
      }
      pollfd p{fd_, POLLOUT, 0};
      ::poll(&p, 1, remaining);
    }
    return true;
  }

  bool write_all(const std::string & s, int timeout_ms = -1)
  {
    return write_all(reinterpret_cast<const uint8_t *>(s.data()), s.size(), timeout_ms);
  }

private:
  size_t raw_read(uint8_t * buf, size_t n, int timeout_ms)
  {
    if (fd_ < 0) {throw std::runtime_error("Attempting to use a port that is not open");}
    pollfd p{fd_, POLLIN, 0};
    int r = ::poll(&p, 1, timeout_ms);
    if (r < 0) {
      if (errno == EINTR) {return 0;}
      throw std::runtime_error(std::string("poll failed: ") + std::strerror(errno));
    }
    if (r == 0) {return 0;}
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error(
              "device reports readiness to read but returned no data "
              "(device disconnected or multiple access on port?)");
    }
    const ssize_t k = ::read(fd_, buf, n);
    if (k < 0) {
      if (errno == EAGAIN || errno == EINTR) {return 0;}
      throw std::runtime_error(std::string("read failed: ") + std::strerror(errno));
    }
    if (k == 0) {
      throw std::runtime_error(
              "device reports readiness to read but returned no data "
              "(device disconnected or multiple access on port?)");
    }
    return static_cast<size_t>(k);
  }

  int fd_{-1};
  std::string path_;
  std::string rx_;
};

}  // namespace cmr_port
