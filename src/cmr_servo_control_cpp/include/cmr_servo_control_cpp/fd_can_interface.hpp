// C++ port of cmr_servo_control/CMR_CANFD/FdCanInterface.py and ServoController.py.
//
// Text-protocol interface to an mjbots fdcanusb ("can std <id> <hex> <flags>").
// The Python version ran on an asyncio loop thread; here writes are serialized
// with a mutex and the optional receive loop runs on a dedicated thread.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "cmr_servo_control_cpp/serial_port.hpp"

namespace cmr_servo_control_cpp
{

class FdCanInterface
{
public:
  using Clock = std::chrono::steady_clock;
  using RxCallback =
    std::function<void(int can_id, const std::string & data_hex,
      const std::vector<std::string> & flags)>;

  explicit FdCanInterface(
    std::string port = "/dev/tty.usbmodemD8957AFF1", int baud = 115200,
    double timeout = 1.0);
  ~FdCanInterface();

  // Opens the port and takes the exclusive flock on it (no bytes are sent).
  // Throws cmr_port::PortBusyError if another node owns the adapter.
  void open_port();
  bool is_port_open() const {return ser_.is_open();}

  // open(): open the port if needed, sleep 1 s, then discard any buffered lines.
  // Throws std::runtime_error if the deadline passes or the port cannot be opened.
  void open(Clock::time_point deadline);
  void close();

  // Sends the fdcanusb bus configuration commands, printing "> cmd → resp".
  void configure_bus(Clock::time_point deadline);

  // Fire-and-forget frame transmit (no response is awaited).
  void write_frame(int std_id, const std::string & data_hex, const std::string & flags = "FB");

  // Blocking receive loop; returns when `running` becomes false or the port fails.
  void read_loop(const std::atomic<bool> & running);

  // Reads and discards all input until `running` becomes false or the port fails.
  void drain_loop(const std::atomic<bool> & running);

  RxCallback rx_callback;

  const std::string & port() const {return port_;}
  int baud() const {return baud_;}

private:
  void clear_buffer();
  std::string send_command(const std::string & cmd, Clock::time_point deadline);

  std::string port_;
  int baud_;
  double timeout_;
  cmr_port::SerialPort ser_;
  std::mutex write_mutex_;
};

class ServoController
{
public:
  ServoController(FdCanInterface * can, int servo_id = 0, int home = 0, int can_id = 1);

  // control_data is taken as a (Python, arbitrary precision) integer; only the
  // low 12 bits reach the frame, so callers pass any int64.
  std::vector<uint8_t> make_control_frame(
    int control_mode, int64_t control_data, int home = 0, int reset_home = 0,
    int clear_faults = 0, int query_data = 0) const;

  void go_to_position(int64_t degrees);
  void stop();
  void set_home(int64_t degrees);
  void go_home();

  int servo_id() const {return servo_id_;}
  int can_id() const {return can_id_;}

private:
  FdCanInterface * can_;
  int servo_id_;
  int home_;
  int can_id_;
};

// bytes.hex() (lowercase)
std::string to_hex_lower(const std::vector<uint8_t> & data);

}  // namespace cmr_servo_control_cpp
