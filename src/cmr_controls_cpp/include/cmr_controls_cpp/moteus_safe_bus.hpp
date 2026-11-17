// fdcanusb helpers for cmr_controls_cpp, adapted from cmr_rovernet_cpp
// (moteus_bus.hpp/.cpp: SafeFdcanusb, count_responders, autodetect_port).
#pragma once

#include <cerrno>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mjbots/moteus/moteus.h>

#include "rclcpp/rclcpp.hpp"

#include "cmr_controls_cpp/device_lock.hpp"

namespace cmr_controls_cpp
{

namespace mm = mjbots::moteus;

/// Thrown out of constructors when Ctrl-C arrives during a busy/absent-adapter
/// retry loop; main() catches it and exits cleanly.
class ShutdownRequested : public std::runtime_error
{
public:
  ShutdownRequested()
  : std::runtime_error("shutdown requested") {}
};

/// Fdcanusb whose I/O errors are reported to the BlockingCycle caller as
/// std::runtime_error instead of escaping from the transport's event-loop
/// thread (which would std::terminate the process without a stop).
class SafeFdcanusb : public mm::Fdcanusb
{
public:
  SafeFdcanusb(const std::string & device, const Options & options = {})
  : mm::Fdcanusb(device, options) {}

  void Cycle(
    const mm::CanFdFrame * frames, size_t size, std::vector<mm::CanFdFrame> * replies,
    mm::CompletionCallback completed_callback) override
  {
    auto loop = std::atomic_load(&UNPROTECTED_event_loop_);
    if (!loop) {
      throw std::runtime_error("fdcanusb transport is shut down");
    }
    loop->Post(
      [this, frames, size, replies, completed_callback]() {
        try {
          this->CHILD_Cycle(frames, size, replies, completed_callback);
        } catch (const std::exception & exc) {
          {
            std::lock_guard<std::mutex> lock(error_mutex_);
            error_ = exc.what();
          }
          completed_callback(EIO);
        }
      });
  }

  void BlockingCycle(
    const mm::CanFdFrame * frames, size_t size,
    std::vector<mm::CanFdFrame> * replies) override
  {
    mm::BlockingCallback cbk;
    Cycle(frames, size, replies, cbk.callback());
    if (cbk.Wait() != 0) {
      std::lock_guard<std::mutex> lock(error_mutex_);
      throw std::runtime_error("fdcanusb I/O error: " + error_);
    }
  }

private:
  std::mutex error_mutex_;
  std::string error_;
};

/// Result of choosing an adapter: the path and its exclusive lock, which is
/// held from the probe onward so no other node can grab it in between.
struct SelectedAdapter
{
  std::string path;
  std::unique_ptr<DeviceLock> lock;
};

/// Thrown when no free adapter has any of the node's motors (yet). Callers
/// log it (throttled) and retry, like PortBusyError.
class NoRespondersError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

inline std::string ids_str(const std::vector<int> & ids)
{
  std::ostringstream o;
  o << "[";
  for (size_t i = 0; i < ids.size(); ++i) {o << (i ? ", " : "") << ids[i];}
  o << "]";
  return o.str();
}

/// Counts how many of @p ids answer a query on the adapter at @p path (the
/// caller already holds its lock). 0.1 s reply timeout per ID, as
/// moteus_drive_gui.PROBE_TIMEOUT_S. Only queries are sent.
inline int count_responders(const std::string & path, const std::vector<int> & ids)
{
  try {
    mm::Fdcanusb::Options o;
    o.min_rcv_wait_ns = 100000000;  // 0.1 s
    auto transport = std::make_shared<SafeFdcanusb>(path, o);
    int responders = 0;
    for (int id : ids) {
      mm::Controller::Options co;
      co.id = id;
      co.transport = transport;
      mm::Controller ctrl(co);
      try {
        if (ctrl.SetQuery()) {
          ++responders;
        }
      } catch (const std::exception &) {
      }
    }
    return responders;
  } catch (const std::exception &) {
    return 0;
  }
}

/// Resolves the adapter for a node owning motor @p ids.
///  - explicit path: lock it (PortBusyError if busy); no probe, user's choice.
///  - "auto": lock + probe every fdcanusb candidate, keep the one with the most
///    responders (> 0) and keep its lock. If none of the free adapters has any
///    of our motors and some adapter is busy, the right one is presumably the
///    busy one -> PortBusyError (never fall through to another adapter).
///    No candidate at all -> std::runtime_error. All free, none answering ->
///    NoRespondersError.
inline SelectedAdapter select_adapter(
  const std::string & can_port, const std::vector<int> & ids, std::string * scores_out = nullptr)
{
  if (can_port != "auto" && !can_port.empty()) {
    SelectedAdapter sel;
    sel.lock = std::make_unique<DeviceLock>(can_port);
    sel.path = can_port;
    return sel;
  }

  const auto candidates = fdcanusb_candidates();
  if (candidates.empty()) {
    throw std::runtime_error("Could not detect fdcanusb");
  }

  SelectedAdapter best;
  int best_score = 0;
  bool any_busy = false;
  std::ostringstream scores;
  scores << "{";
  bool first = true;
  for (const auto & path : candidates) {
    scores << (first ? "" : ", ") << "'" << path << "': ";
    first = false;
    std::unique_ptr<DeviceLock> lock;
    try {
      lock = std::make_unique<DeviceLock>(path);  // before any probe frame
    } catch (const PortBusyError &) {
      any_busy = true;
      scores << "busy";
      continue;
    } catch (const std::exception &) {
      scores << "unopenable";
      continue;
    }
    const int score = count_responders(path, ids);
    scores << score;
    if (score > best_score) {  // first maximum wins
      best_score = score;
      best.path = path;
      best.lock = std::move(lock);  // previous best's lock released here
    }
  }
  scores << "}";
  if (scores_out) {*scores_out = scores.str();}

  if (best_score > 0) {
    return best;
  }
  if (any_busy) {
    throw PortBusyError(
            "no free fdcanusb answers for motor IDs " + ids_str(ids) +
            " and an adapter is locked by another node; waiting for it: " + scores.str());
  }
  throw NoRespondersError(
          "No fdcanusb adapter answered for motor IDs " + ids_str(ids) +
          "; responders per adapter: " + scores.str());
}


/// One node's fdcanusb: exclusive lock + SafeFdcanusb + one controller per ID
/// (with and without the default query). Every frame goes through cycle(),
/// which turns transport errors into log + best-effort stop + reopen.
class MotorBus
{
public:
  MotorBus(rclcpp::Logger logger, std::string can_port, std::vector<int> ids)
  : logger_(std::move(logger)), can_port_(std::move(can_port)), ids_(std::move(ids)) {}

  ~MotorBus() {close();}

  /// Blocks until the adapter is open: busy / no-responder adapters are logged
  /// (throttled) and retried. Ctrl-C -> ShutdownRequested. "No fdcanusb at all"
  /// and other hard errors propagate (moteus.Fdcanusb() raised there too).
  void open_blocking()
  {
    while (true) {
      if (!rclcpp::ok()) {throw ShutdownRequested();}
      if (try_open(/*hard_errors_throw=*/true)) {return;}
      for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  bool is_open() const {return static_cast<bool>(transport_);}
  const std::string & device() const {return device_;}

  mm::Controller & ctrl(int id, bool query) {return *(query ? query_ : noquery_).at(id);}

  /// Send @p build()'s frames in one cycle. If the bus is closed, one reopen is
  /// attempted first (the command is dropped if that fails). On an I/O error:
  /// log, try a stop on the failing transport, drop it, reopen and send a stop.
  /// Returns false if the frames were not (fully) sent.
  bool cycle(
    const std::function<std::vector<mm::CanFdFrame>()> & build,
    std::vector<mm::CanFdFrame> * replies = nullptr)
  {
    if (!transport_ && !try_open(false)) {
      return false;
    }
    std::vector<mm::CanFdFrame> local;
    if (!replies) {replies = &local;}
    const auto frames = build();
    try {
      transport_->BlockingCycle(frames.data(), frames.size(), replies);
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "fdcanusb %s failed: %s; stopping and reopening",
        device_.c_str(), e.what());
    }
    send_stop_quietly();
    close();
    if (try_open(false)) {
      send_stop_quietly();
    }
    return false;
  }

  /// Unchecked cycle for the startup sequence: errors propagate (Python crashed).
  void cycle_or_throw(const std::vector<mm::CanFdFrame> & frames)
  {
    std::vector<mm::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);
  }

  /// Transport closed first, then the adapter lock is released.
  void close()
  {
    query_.clear();
    noquery_.clear();
    transport_.reset();
    lock_.reset();
  }

private:
  bool try_open(bool hard_errors_throw)
  {
    close();
    std::string scores;
    try {
      auto sel = select_adapter(can_port_, ids_, &scores);
      auto transport = std::make_shared<SafeFdcanusb>(sel.path);
      for (int id : ids_) {
        mm::Controller::Options o;
        o.id = id;
        o.transport = transport;
        o.default_query = true;
        query_[id] = std::make_unique<mm::Controller>(o);
        o.default_query = false;
        noquery_[id] = std::make_unique<mm::Controller>(o);
      }
      lock_ = std::move(sel.lock);
      transport_ = transport;
      device_ = sel.path;
      RCLCPP_INFO(
        logger_, "Opened fdcanusb %s (exclusive) for motor IDs %s%s%s", device_.c_str(),
        ids_str(ids_).c_str(), scores.empty() ? "" : "; responders ", scores.c_str());
      return true;
    } catch (const PortBusyError & e) {
      RCLCPP_WARN_THROTTLE(logger_, steady_, 5000, "%s; retrying", e.what());
    } catch (const NoRespondersError & e) {
      RCLCPP_WARN_THROTTLE(logger_, steady_, 5000, "%s; retrying", e.what());
    } catch (const std::exception & e) {
      close();
      if (hard_errors_throw) {throw;}
      RCLCPP_ERROR_THROTTLE(logger_, steady_, 5000, "Cannot open fdcanusb: %s", e.what());
    }
    close();
    return false;
  }

  void send_stop_quietly()
  {
    if (!transport_) {return;}
    try {
      std::vector<mm::CanFdFrame> frames, replies;
      for (int id : ids_) {frames.push_back(noquery_.at(id)->MakeStop());}
      transport_->BlockingCycle(frames.data(), frames.size(), &replies);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Stop after fdcanusb error failed: %s", e.what());
    }
  }

  rclcpp::Logger logger_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  std::string can_port_;
  std::vector<int> ids_;
  std::string device_;
  // Order: lock first so it is released after the transport is destroyed.
  std::unique_ptr<DeviceLock> lock_;
  std::shared_ptr<SafeFdcanusb> transport_;
  std::map<int, std::unique_ptr<mm::Controller>> query_;
  std::map<int, std::unique_ptr<mm::Controller>> noquery_;
};

}  // namespace cmr_controls_cpp
