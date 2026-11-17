// Small POSIX termios serial port, equivalent to
// serial.Serial(port, baud, timeout=...) with pyserial defaults
// (8N1, raw, no flow control).
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace cmr_rovernet_cpp {

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Throws std::runtime_error on failure.
  void open(const std::string & path, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// Writes all bytes (pyserial write()).  Throws on error.
  void write_all(const uint8_t * data, std::size_t size);

  /// pyserial read(size) with a total timeout: returns once @p size bytes
  /// were read or the timeout expired (may return fewer bytes).
  /// Throws on I/O error / device hang-up.
  std::size_t read(uint8_t * out, std::size_t size, std::chrono::milliseconds timeout);

private:
  int fd_ = -1;
};

}  // namespace cmr_rovernet_cpp
