// C++ port of cmr_servo_control/CMR_CANFD/FdCanInterface.py and ServoController.py.
#include "cmr_servo_control_cpp/fd_can_interface.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace cmr_servo_control_cpp
{

namespace
{

std::string strip(const std::string & s)
{
  const char * ws = " \t\r\n\v\f";
  const auto b = s.find_first_not_of(ws);
  if (b == std::string::npos) {return "";}
  const auto e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

// line.decode('ascii', errors='ignore')
std::string ascii_ignore(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (static_cast<unsigned char>(c) < 0x80) {out.push_back(c);}
  }
  return out;
}

std::vector<std::string> split_ws(const std::string & s)
{
  std::istringstream iss(s);
  std::vector<std::string> parts;
  std::string p;
  while (iss >> p) {parts.push_back(p);}
  return parts;
}

std::string hex_upper(int v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%X", v);
  return buf;
}

std::string py_list_repr(const std::vector<std::string> & items)
{
  std::string out = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i) {out += ", ";}
    out += "'" + items[i] + "'";
  }
  return out + "]";
}

int remaining_ms(FdCanInterface::Clock::time_point deadline)
{
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    deadline - FdCanInterface::Clock::now()).count();
  if (ms <= 0) {
    throw std::runtime_error("Timed out waiting for fdcanusb startup");
  }
  return static_cast<int>(ms);
}

}  // namespace

std::string to_hex_lower(const std::vector<uint8_t> & data)
{
  static const char * digits = "0123456789abcdef";
  std::string out;
  out.reserve(data.size() * 2);
  for (uint8_t b : data) {
    out.push_back(digits[b >> 4]);
    out.push_back(digits[b & 0x0F]);
  }
  return out;
}

FdCanInterface::FdCanInterface(std::string port, int baud, double timeout)
: port_(std::move(port)), baud_(baud), timeout_(timeout)
{
}

FdCanInterface::~FdCanInterface()
{
  close();
}

void FdCanInterface::open_port()
{
  std::lock_guard<std::mutex> lk(write_mutex_);
  // flock only (no TIOCEXCL): the fdcanusb adapters are shared hardware with
  // cmr_rovernet_cpp, whose DeviceLock probes the lock from a separate fd.
  ser_.open(port_, baud_, /*tty_exclusive=*/false);
}

void FdCanInterface::open(Clock::time_point deadline)
{
  if (!ser_.is_open()) {
    open_port();
  }
  // await asyncio.sleep(1.0)
  const auto wake = Clock::now() + std::chrono::seconds(1);
  if (wake > deadline) {
    std::this_thread::sleep_until(deadline);
    throw std::runtime_error("Timed out waiting for fdcanusb startup");
  }
  std::this_thread::sleep_until(wake);
  clear_buffer();
}

void FdCanInterface::close()
{
  std::lock_guard<std::mutex> lk(write_mutex_);
  ser_.close();
}

void FdCanInterface::clear_buffer()
{
  std::string line;
  while (true) {
    // readline() with a 0.1 s timeout; stop at the first timeout.
    if (!ser_.read_line(line, 100)) {
      break;
    }
    if (line.empty()) {
      break;
    }
  }
}

std::string FdCanInterface::send_command(const std::string & cmd, Clock::time_point deadline)
{
  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    ser_.write_all(cmd + "\n", remaining_ms(deadline));
  }
  std::string line;
  while (true) {
    if (!ser_.read_line(line, remaining_ms(deadline))) {
      remaining_ms(deadline);  // throws once the deadline has passed
      continue;
    }
    const std::string text = strip(ascii_ignore(line));
    if (text == "OK" || text.rfind("ERR", 0) == 0) {
      return text;
    }
  }
}

void FdCanInterface::configure_bus(Clock::time_point deadline)
{
  static const char * cmds[] = {
    "can off",
    "conf set can.bitrate 1000000",
    "conf set can.fdcan_frame on",
    "conf set can.bitrate_switch on",
    "conf set can.termination on",
    "can on",
  };
  for (const char * cmd : cmds) {
    const std::string resp = send_command(cmd, deadline);
    std::cout << "> " << cmd << " → " << resp << std::endl;
  }
}

void FdCanInterface::write_frame(int std_id, const std::string & data_hex, const std::string & flags)
{
  const std::string cmd = "can std " + hex_upper(std_id) + " " + data_hex + " " + flags;
  std::cout << "TX ID=0x" << hex_upper(std_id) << ", data=" << data_hex << ", flags=" << flags <<
    std::endl;
  std::lock_guard<std::mutex> lk(write_mutex_);
  if (!ser_.write_all(cmd + "\n", 1000)) {
    std::cout << "Timeout while draining writer in write_frame()" << std::endl;
  }
}

void FdCanInterface::read_loop(const std::atomic<bool> & running)
{
  std::string raw;
  while (running.load()) {
    // Poll with a timeout so shutdown can stop the thread.
    if (!ser_.read_line(raw, 200)) {
      continue;
    }
    const std::string line = strip(ascii_ignore(raw));
    if (line.rfind("rcv ", 0) != 0) {
      continue;
    }
    const auto parts = split_ws(line);
    // NOTE(port): Python's int(parts[1], 16) / parts[2] raise on a malformed
    // "rcv" line, which silently kills the asyncio read_loop task forever.
    // Here the malformed line is skipped instead.
    if (parts.size() < 3) {
      continue;
    }
    char * end = nullptr;
    const long can_id = std::strtol(parts[1].c_str(), &end, 16);
    if (end == parts[1].c_str() || *end != '\0') {
      continue;
    }
    const std::string & data_hex = parts[2];
    const std::vector<std::string> flags(parts.begin() + 3, parts.end());
    std::cout << "RX ID=0x" << hex_upper(static_cast<int>(can_id)) << ", data=" << data_hex <<
      ", flags=" << py_list_repr(flags) << std::endl;
    if (rx_callback) {
      try {
        rx_callback(static_cast<int>(can_id), data_hex, flags);
      } catch (const std::exception & exc) {
        std::cout << "RX callback error: " << exc.what() << std::endl;
      }
    }
  }
}

void FdCanInterface::drain_loop(const std::atomic<bool> & running)
{
  uint8_t buf[512];
  while (running.load()) {
    ser_.read_some(buf, sizeof(buf), 200);
  }
}

ServoController::ServoController(FdCanInterface * can, int servo_id, int home, int can_id)
: can_(can), servo_id_(servo_id), home_(home), can_id_(can_id)
{
}

std::vector<uint8_t> ServoController::make_control_frame(
  int control_mode, int64_t control_data, int home, int reset_home,
  int clear_faults, int query_data) const
{
  std::vector<uint8_t> frame(32, 0);
  frame[0] = static_cast<uint8_t>(
    (1 << 6) | ((servo_id_ & 0x0F) << 2) | ((home & 0x01) << 1) | (reset_home & 0x01));
  // Arithmetic shift on negative values matches Python's >> on ints.
  frame[1] = static_cast<uint8_t>(((control_mode & 0x03) << 6) | ((control_data >> 6) & 0x3F));
  frame[2] = static_cast<uint8_t>((control_data & 0x3F) << 2);
  frame[3] = static_cast<uint8_t>(((clear_faults & 0x01) << 7) | ((query_data & 0x01) << 6));
  return frame;
}

void ServoController::go_to_position(int64_t degrees)
{
  const auto frame = make_control_frame(0, degrees);
  can_->write_frame(can_id_, to_hex_lower(frame), "FB");
}

void ServoController::stop()
{
  const auto frame = make_control_frame(2, 0);
  can_->write_frame(can_id_, to_hex_lower(frame), "FB");
}

void ServoController::set_home(int64_t degrees)
{
  const auto frame = make_control_frame(3, degrees, 0, 1);
  can_->write_frame(can_id_, to_hex_lower(frame), "FB");
}

void ServoController::go_home()
{
  const auto frame = make_control_frame(0, home_);
  can_->write_frame(can_id_, to_hex_lower(frame), "FB");
}

}  // namespace cmr_servo_control_cpp
