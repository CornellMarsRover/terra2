// Persistent fdcanusb transport + moteus controllers.
//
// The Python nodes (moteus_drive_gui.make_transport_and_controllers) opened a
// brand new moteus.Fdcanusb for every single control cycle.  Here one
// transport is opened lazily, kept for the life of the node, and only rebuilt
// after an I/O error or when the resolved device node disappears.
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <mjbots/moteus/moteus.h>
#include <rclcpp/logger.hpp>

namespace cmr_rovernet_cpp {

namespace mm = mjbots::moteus;

/// Fdcanusb whose I/O errors are reported back to the caller of
/// BlockingCycle (as std::runtime_error) instead of escaping from the
/// transport's event-loop thread (which would std::terminate the process).
class SafeFdcanusb : public mm::Fdcanusb
{
public:
  SafeFdcanusb(const std::string & device, const Options & options);

  void Cycle(
    const mm::CanFdFrame * frames, size_t size, std::vector<mm::CanFdFrame> * replies,
    mm::CompletionCallback completed_callback) override;

  void BlockingCycle(
    const mm::CanFdFrame * frames, size_t size,
    std::vector<mm::CanFdFrame> * replies) override;

private:
  std::mutex error_mutex_;
  std::string error_;
  /// How long the adapter may take to acknowledge ("OK") a cycle's frames.
  const uint32_t ack_timeout_ns_;
};

/// Thrown when another process (e.g. a second rovernet node) holds the
/// exclusive lock on a CAN adapter.  Callers log it and retry later.
class PortBusyError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Exclusive, non-blocking flock(2) on a device node, held for as long as the
/// object lives.  Prevents two processes from opening the same fdcanusb and
/// stealing each other's replies.  flock is per open-file-description, so a
/// second lock attempt from this same process on a new fd also fails: always
/// release (reset) before re-acquiring.
class DeviceLock
{
public:
  /// Throws PortBusyError if the lock is held elsewhere, std::runtime_error
  /// if the path cannot be opened.
  explicit DeviceLock(const std::string & path);
  ~DeviceLock();
  DeviceLock(const DeviceLock &) = delete;
  DeviceLock & operator=(const DeviceLock &) = delete;

  /// true if @p path could be locked right now (lock is released again),
  /// false if it is busy (PortBusyError).  Any other error (EACCES, ENOENT,
  /// ...) propagates with its real message.
  static bool try_probe(const std::string & path);

private:
  int fd_ = -1;
};

/// moteus_drive_gui.make_query_resolution(): INT8 mode/fault, F32 for
/// position, abs_position, velocity, torque, voltage, temperature,
/// motor_temperature, q_current, d_current, power.
mm::Query::Format drive_gui_query_format();

/// moteus_drive_gui.find_fdcanusb_ports(): sorted /dev/serial/by-id/*fdcanusb*
std::vector<std::string> find_fdcanusb_ports();

/// Port resolution.  An explicit path that exists is returned as-is.  A
/// missing explicit path, "auto", "" and "None" all go to the probing
/// autodetect: an adapter is only returned if at least one of @p ids answered
/// a query on it (always probed, even when it is the only adapter).  Throws
/// (PortBusyError / std::runtime_error) when no adapter qualifies.
/// NOTE(port): differs from this branch's Python resolve_port, see the .cpp.
std::string resolve_port(
  const rclcpp::Logger & logger, const std::string & port_arg, const std::vector<int> & ids);

/// Drop @p path from the autodetect cache so the next resolve re-probes.
void forget_detected_port(const std::string & path);

struct BusOptions
{
  std::string port_arg;
  /// true: moteus_drive_gui.resolve_port semantics ("auto", fallback).
  /// false: path is passed straight to Fdcanusb (armnet behaviour).
  bool resolve = true;
  std::vector<int> ids;
  /// How long to wait for a query reply (Python asyncio.wait_for timeout).
  double reply_timeout_s = 0.3;
  mm::Query::Format query_format;
};

class MoteusBus
{
public:
  MoteusBus(rclcpp::Logger logger, BusOptions options);
  ~MoteusBus();

  /// Opens (or re-opens) the transport if needed; returns the device path in
  /// use ("" = moteus autodetect).  Throws on failure.
  std::string ensure_open();

  /// Drop the transport; the next call re-resolves and re-opens it.
  void reset();

  const std::string & device() const {return device_;}
  /// true while a transport (and its exclusive device lock) is held.
  bool is_open() const {return static_cast<bool>(transport_);}

  /// Controller for @p id (must be one of options.ids).  ensure_open() first.
  mm::Controller & controller(int id);

  /// Send @p frames in a single fdcanusb cycle.  Throws on failure (the
  /// transport is reset first).
  void send(const std::vector<mm::CanFdFrame> & frames);

  /// controller.set_stop() for each id, in one cycle.  Returns a status per
  /// motor ("stop sent" - meaning sent and acknowledged by the adapter - or
  /// the error).  Throws only if the port cannot be
  /// resolved/opened (as the Python make_transport_and_controllers did).
  std::map<int, std::string> stop(const std::vector<int> & ids);

  /// stop(), but throws std::runtime_error unless every stop frame was sent
  /// and acknowledged by the adapter.
  void stop_or_throw(const std::vector<int> & ids);

  /// controller.query() with the bus query format.  Returns std::nullopt on
  /// timeout; throws on transport failure.
  std::optional<mm::Controller::Result> query(int id);

private:
  rclcpp::Logger logger_;
  BusOptions options_;
  std::string device_;
  // Order matters: the transport is destroyed before the lock is released.
  std::unique_ptr<DeviceLock> lock_;
  std::shared_ptr<SafeFdcanusb> transport_;
  std::map<int, std::unique_ptr<mm::Controller>> controllers_;
};

// --- Frame builders matching the Python set_position(...) keyword sets. ---
// Only the keyword arguments the Python passed are encoded (F32); every other
// field is omitted from the frame, exactly like moteus-python does for None.

/// set_position(position=nan, velocity=v, maximum_torque, accel_limit,
///              watchdog_timeout)
mm::CanFdFrame make_drive_velocity_frame(
  mm::Controller & ctrl, double velocity, double maximum_torque, double accel_limit,
  double watchdog_timeout);

/// set_position(position=p, velocity_limit, maximum_torque, watchdog_timeout)
mm::CanFdFrame make_steer_position_frame(
  mm::Controller & ctrl, double position, double velocity_limit, double maximum_torque,
  double watchdog_timeout);

/// rovernet_utils._send_moteus_command_async with drives_velocity=None,
/// ff_torque=None and no watchdog: set_position(position=p, velocity=nan,
/// maximum_torque, velocity_limit, accel_limit, feedforward_torque=0.0)
mm::CanFdFrame make_arm_position_frame(
  mm::Controller & ctrl, double position, double maximum_torque, double velocity_limit,
  double accel_limit);

}  // namespace cmr_rovernet_cpp
