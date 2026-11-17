#include "cmr_rovernet_cpp/moteus_bus.hpp"

#include <fcntl.h>
#include <glob.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

#include <rclcpp/logging.hpp>

namespace cmr_rovernet_cpp {
namespace {

constexpr double kProbeTimeoutS = 0.1;  // moteus_drive_gui.PROBE_TIMEOUT_S

bool path_exists(const std::string & path)
{
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0;
}

mm::Fdcanusb::Options transport_options(double reply_timeout_s)
{
  mm::Fdcanusb::Options o;
  const double clamped = std::max(0.0, std::min(reply_timeout_s, 4.0));
  o.min_rcv_wait_ns = static_cast<uint32_t>(clamped * 1e9);
  // How long to wait for the adapter's "OK" acknowledgements (Python:
  // asyncio.wait_for(..., timeout_s) around every command).  The transport
  // returns as soon as all OKs arrived, so this costs nothing when healthy;
  // SafeFdcanusb turns an expired wait into an I/O error.
  o.min_ok_wait_ns = static_cast<uint32_t>(std::max(0.02, clamped) * 1e9);
  return o;
}

std::string ids_str(const std::set<int> & ids)
{
  std::ostringstream o;
  o << "[";
  bool first = true;
  for (int id : ids) {
    o << (first ? "" : ", ") << id;
    first = false;
  }
  o << "]";
  return o.str();
}

// Minimal rate limiter for the free functions below (no node clock here).
bool log_due(const std::string & key, double period_s)
{
  static std::mutex mutex;
  static std::map<std::string, std::chrono::steady_clock::time_point> last;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex);
  const auto it = last.find(key);
  if (it != last.end() && std::chrono::duration<double>(now - it->second).count() < period_s) {
    return false;
  }
  last[key] = now;
  return true;
}

// moteus_drive_gui._count_responders.  Returns -1 if the adapter is locked by
// another process (owned by another node); the lock is taken before any probe
// frame is sent and released when probing finishes.  Only read-only query
// frames are sent while probing.
int count_responders(
  const rclcpp::Logger & logger, const std::string & path, const std::vector<int> & ids)
{
  std::unique_ptr<DeviceLock> lock;
  try {
    lock = std::make_unique<DeviceLock>(path);
  } catch (const PortBusyError &) {
    return -1;
  } catch (const std::exception & exc) {
    // An unopenable adapter scores 0; surface the reason.
    if (log_due("probe:" + path, 5.0)) {
      RCLCPP_WARN(logger, "cannot probe CAN adapter %s: %s", path.c_str(), exc.what());
    }
    return 0;
  }
  try {
    auto transport = std::make_shared<SafeFdcanusb>(path, transport_options(kProbeTimeoutS));
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

std::mutex g_detect_mutex;
std::map<std::set<int>, std::string> g_detected_ports;

// Failed detections are remembered briefly so a node retrying at its control
// rate does not re-probe (and briefly lock) every adapter 10 times a second.
constexpr double kDetectRetryS = 2.0;
struct DetectFailure
{
  std::chrono::steady_clock::time_point when;
  std::string what;
  bool busy = false;
};
std::map<std::set<int>, DetectFailure> g_detect_failures;

// Find the fdcanusb that this node's motors are on.
//
// NOTE(port): this branch's Python (moteus_drive_gui.resolve_port) takes the
// first /dev/serial/by-id, ttyACM or ttyUSB candidate for "auto" without
// looking at what is on the bus.  The rover has two fdcanusb adapters (drives
// and arm), so an adapter is only ever accepted here after at least one of
// this node's motor IDs answered a query on it - also when it is the only
// adapter present (it may be the *other* subsystem's: e.g. drive adapter
// unplugged, arm adapter present).  With no responder nothing is opened; the
// caller logs the error and retries.  Adapters locked by another node are
// never probed.
std::string autodetect_port(const rclcpp::Logger & logger, const std::vector<int> & ids_in)
{
  const std::set<int> key(ids_in.begin(), ids_in.end());
  if (key.empty()) {
    throw std::runtime_error("cannot autodetect an fdcanusb adapter without motor IDs to probe");
  }
  std::lock_guard<std::mutex> lock(g_detect_mutex);
  // Only probed adapters are ever cached (see below).
  const auto cached = g_detected_ports.find(key);
  if (cached != g_detected_ports.end() && path_exists(cached->second)) {
    return cached->second;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto failed = g_detect_failures.find(key);
  if (failed != g_detect_failures.end() &&
    std::chrono::duration<double>(now - failed->second.when).count() < kDetectRetryS)
  {
    if (failed->second.busy) {
      throw PortBusyError(failed->second.what);
    }
    throw std::runtime_error(failed->second.what);
  }
  const auto fail = [&](const std::string & what, bool busy) {
      g_detect_failures[key] = DetectFailure{std::chrono::steady_clock::now(), what, busy};
      if (busy) {
        throw PortBusyError(what);
      }
      throw std::runtime_error(what);
    };

  const auto candidates = find_fdcanusb_ports();
  if (candidates.empty()) {
    fail("Could not find an fdcanusb adapter in /dev/serial/by-id/.", false);
  }

  std::string chosen;
  const std::vector<int> ids(key.begin(), key.end());
  int best = -1;
  bool any_free = false;
  std::ostringstream scores;
  scores << "{";
  bool first = true;
  for (const auto & path : candidates) {
    const int score = count_responders(logger, path, ids);
    scores << (first ? "" : ", ") << "'" << path << "': ";
    first = false;
    if (score < 0) {
      scores << "busy";
      continue;
    }
    scores << score;
    any_free = true;
    if (score > best) {  // first maximum
      best = score;
      chosen = path;
    }
  }
  scores << "}";
  if (!any_free) {
    fail("all fdcanusb adapters are busy (locked by another process): " + scores.str(), true);
  }
  if (best <= 0) {
    fail(
      "No fdcanusb adapter answered for motor IDs " + ids_str(key) +
      "; responders per adapter: " + scores.str() + " (not opening any adapter)", false);
  }

  g_detect_failures.erase(key);
  g_detected_ports[key] = chosen;
  RCLCPP_INFO(
    logger, "auto-selected CAN adapter %s for motor IDs %s (responders per adapter: %s)",
    chosen.c_str(), ids_str(key).c_str(), scores.str().c_str());
  return chosen;
}

mm::PositionMode::Format ignore_all_format()
{
  mm::PositionMode::Format f;
  f.position = mm::kIgnore;
  f.velocity = mm::kIgnore;
  f.feedforward_torque = mm::kIgnore;
  f.kp_scale = mm::kIgnore;
  f.kd_scale = mm::kIgnore;
  f.maximum_torque = mm::kIgnore;
  f.stop_position = mm::kIgnore;
  f.watchdog_timeout = mm::kIgnore;
  f.velocity_limit = mm::kIgnore;
  f.accel_limit = mm::kIgnore;
  f.fixed_voltage_override = mm::kIgnore;
  f.ilimit_scale = mm::kIgnore;
  f.fixed_current_override = mm::kIgnore;
  f.ignore_position_bounds = mm::kIgnore;
  return f;
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

// ---------------------------------------------------------------------------
DeviceLock::DeviceLock(const std::string & path)
{
  int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0 && (errno == EACCES || errno == EISDIR)) {
    fd = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  }
  if (fd < 0) {
    const int err = errno;
    if (err == EBUSY) {
      // Another process opened the tty with TIOCEXCL (e.g. cmr_imu_cpp / cmr_rtkgps_cpp).
      throw PortBusyError("port busy: " + path + " is held exclusively by another process");
    }
    throw std::runtime_error("could not open " + path + ": " + std::strerror(err));
  }
  while (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    if (errno == EINTR) {
      continue;
    }
    const int err = errno;
    ::close(fd);
    if (err == EWOULDBLOCK) {
      throw PortBusyError(
              "port busy: " + path + " is locked by another process (another node owns it)");
    }
    throw std::runtime_error("flock " + path + ": " + std::strerror(err));
  }
  fd_ = fd;
}

DeviceLock::~DeviceLock()
{
  if (fd_ >= 0) {
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
  }
}

bool DeviceLock::try_probe(const std::string & path)
{
  try {
    DeviceLock lock(path);
    return true;
  } catch (const PortBusyError &) {
    return false;
  }
}

// ---------------------------------------------------------------------------
SafeFdcanusb::SafeFdcanusb(const std::string & device, const Options & options)
: mm::Fdcanusb(device, options), ack_timeout_ns_(options.min_ok_wait_ns) {}

void SafeFdcanusb::Cycle(
  const mm::CanFdFrame * frames, size_t size, std::vector<mm::CanFdFrame> * replies,
  mm::CompletionCallback completed_callback)
{
  auto loop = std::atomic_load(&UNPROTECTED_event_loop_);
  if (!loop) {
    throw std::runtime_error("fdcanusb transport is shut down");
  }
  loop->Post(
    [this, frames, size, replies, completed_callback]() {
      try {
        // The vendored transport silently gives up waiting for the adapter's
        // "OK" lines after min_ok_wait and reports success, so a mute or
        // wrong tty looks healthy.  For a cycle that expects no CAN replies
        // it returns the moment every frame was acknowledged; if it instead
        // ran for the whole ack timeout, at least one OK never arrived.
        // (Cycles with reply-required frames are judged by their replies.)
        bool only_acks = size > 0;
        for (size_t i = 0; i < size; ++i) {
          only_acks = only_acks && !frames[i].reply_required;
        }
        auto ack_failed = std::make_shared<bool>(false);
        const auto start = std::chrono::steady_clock::now();
        // The completion is posted to this same loop and runs after this
        // lambda returns, i.e. after ack_failed has been decided.
        this->CHILD_Cycle(
          frames, size, replies,
          [ack_failed, completed_callback](int rc) {
            completed_callback(*ack_failed ? EIO : rc);
          });
        const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - start).count();
        if (only_acks && elapsed_ns >= static_cast<int64_t>(ack_timeout_ns_)) {
          std::lock_guard<std::mutex> lock(error_mutex_);
          error_ = "adapter did not acknowledge " + std::to_string(size) + " frame(s) within " +
            std::to_string(static_cast<double>(ack_timeout_ns_) * 1e-9) + " s";
          *ack_failed = true;
        }
      } catch (const std::exception & exc) {
        {
          std::lock_guard<std::mutex> lock(error_mutex_);
          error_ = exc.what();
        }
        completed_callback(EIO);
      }
    });
}

void SafeFdcanusb::BlockingCycle(
  const mm::CanFdFrame * frames, size_t size, std::vector<mm::CanFdFrame> * replies)
{
  mm::BlockingCallback cbk;
  Cycle(frames, size, replies, cbk.callback());
  if (cbk.Wait() != 0) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    throw std::runtime_error("fdcanusb I/O error: " + error_);
  }
}

// ---------------------------------------------------------------------------
mm::Query::Format drive_gui_query_format()
{
  mm::Query::Format q;
  q.mode = mm::kInt8;
  q.fault = mm::kInt8;
  q.position = mm::kFloat;
  q.abs_position = mm::kFloat;
  q.velocity = mm::kFloat;
  q.torque = mm::kFloat;
  q.voltage = mm::kFloat;
  q.temperature = mm::kFloat;
  q.motor_temperature = mm::kFloat;
  q.q_current = mm::kFloat;
  q.d_current = mm::kFloat;
  q.power = mm::kFloat;
  return q;
}

std::vector<std::string> find_fdcanusb_ports()
{
  std::vector<std::string> out;
  glob_t g {};
  if (::glob("/dev/serial/by-id/*fdcanusb*", 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) {
      out.emplace_back(g.gl_pathv[i]);
    }
  }
  ::globfree(&g);
  std::sort(out.begin(), out.end());
  return out;
}

void forget_detected_port(const std::string & path)
{
  std::lock_guard<std::mutex> lock(g_detect_mutex);
  for (auto it = g_detected_ports.begin(); it != g_detected_ports.end(); ) {
    if (it->second == path) {
      it = g_detected_ports.erase(it);
    } else {
      ++it;
    }
  }
}

std::string resolve_port(
  const rclcpp::Logger & logger, const std::string & port_arg, const std::vector<int> & ids)
{
  // An explicit path that exists is used as configured (and locked by the
  // caller); no probe needed.
  if (!port_arg.empty() && port_arg != "None" && port_arg != "auto" && path_exists(port_arg)) {
    return port_arg;
  }
  // NOTE(port): this branch's Python returns an explicit path verbatim (and
  // then fails to open it if it is missing), hands "" to moteus' own
  // first-adapter autodetect and maps "auto" to the first serial candidate.
  // Here all three cases fall back to the responder-probing autodetect (see
  // autodetect_port), so a swapped / re-enumerated adapter is tolerated but an
  // adapter is never opened unless this node's motors answered on it.
  if (!port_arg.empty() && port_arg != "None" && port_arg != "auto") {
    if (log_due("missing:" + port_arg, 5.0)) {
      RCLCPP_WARN(
        logger, "configured CAN adapter %s not found; falling back to probing autodetect",
        port_arg.c_str());
    }
  }
  return autodetect_port(logger, ids);
}

// ---------------------------------------------------------------------------
MoteusBus::MoteusBus(rclcpp::Logger logger, BusOptions options)
: logger_(std::move(logger)), options_(std::move(options)) {}

MoteusBus::~MoteusBus() {reset();}

void MoteusBus::reset()
{
  controllers_.clear();
  transport_.reset();  // closes the tty first ...
  lock_.reset();       // ... then releases our flock, so a re-open can re-take it
}

std::string MoteusBus::ensure_open()
{
  if (transport_) {
    // Python re-resolved the port every cycle; we only re-check that the
    // device we opened is still present (e.g. adapter unplugged).
    if (path_exists(device_)) {
      return device_;
    }
    RCLCPP_WARN(logger_, "CAN adapter %s disappeared; re-opening transport", device_.c_str());
    reset();
  }

  std::string device = options_.resolve ?
    resolve_port(logger_, options_.port_arg, options_.ids) : options_.port_arg;
  if (device.empty()) {
    // resolve=false (armnet) with an empty path: moteus default transport
    // autodetect, as moteus.Fdcanusb(None) did in the Python.
    device = mm::Fdcanusb::DetectFdcanusb();
    if (device.empty()) {
      throw std::runtime_error("Could not detect fdcanusb");
    }
  }

  // Take the exclusive lock *before* opening the transport (and so before any
  // frame is sent).  Our own previous lock was released by reset() above /
  // never existed, so this cannot self-deadlock.  Busy -> PortBusyError,
  // which callers log (throttled) and retry on the next cycle.
  lock_.reset();
  std::unique_ptr<DeviceLock> lock;
  try {
    lock = std::make_unique<DeviceLock>(device);
  } catch (const PortBusyError &) {
    // An autodetected adapter that is now owned by another node must not
    // stay cached: re-probe on the next attempt.
    forget_detected_port(device);
    throw;
  }
  auto transport =
    std::make_shared<SafeFdcanusb>(device, transport_options(options_.reply_timeout_s));
  std::map<int, std::unique_ptr<mm::Controller>> controllers;
  for (int id : options_.ids) {
    mm::Controller::Options co;
    co.id = id;
    co.transport = transport;
    co.query_format = options_.query_format;
    co.default_query = false;  // moteus-python set_*(query=False) default
    controllers[id] = std::make_unique<mm::Controller>(co);
  }
  lock_ = std::move(lock);
  transport_ = transport;
  controllers_ = std::move(controllers);
  device_ = device;
  RCLCPP_INFO(logger_, "Opened moteus fdcanusb transport on %s", device_.c_str());
  return device_;
}

mm::Controller & MoteusBus::controller(int id)
{
  const auto it = controllers_.find(id);
  if (it == controllers_.end()) {
    throw std::out_of_range("no moteus controller for id " + std::to_string(id));
  }
  return *it->second;
}

void MoteusBus::send(const std::vector<mm::CanFdFrame> & frames)
{
  if (frames.empty()) {
    return;
  }
  ensure_open();
  try {
    std::vector<mm::CanFdFrame> replies;
    transport_->BlockingCycle(frames.data(), frames.size(), &replies);
  } catch (...) {
    reset();
    throw;
  }
}

std::map<int, std::string> MoteusBus::stop(const std::vector<int> & ids)
{
  ensure_open();
  std::vector<mm::CanFdFrame> frames;
  frames.reserve(ids.size());
  for (int id : ids) {
    frames.push_back(controller(id).MakeStop());
  }
  std::map<int, std::string> statuses;
  try {
    send(frames);
    for (int id : ids) {
      statuses[id] = "stop sent";
    }
  } catch (const std::exception & exc) {
    for (int id : ids) {
      statuses[id] = exc.what();
    }
  }
  return statuses;
}

void MoteusBus::stop_or_throw(const std::vector<int> & ids)
{
  for (const auto & kv : stop(ids)) {
    if (kv.second != "stop sent") {
      throw std::runtime_error("stop not acknowledged for motor " + std::to_string(kv.first) +
              ": " + kv.second);
    }
  }
}

std::optional<mm::Controller::Result> MoteusBus::query(int id)
{
  ensure_open();
  try {
    auto r = controller(id).SetQuery();
    if (!r) {
      return std::nullopt;
    }
    return *r;
  } catch (...) {
    reset();
    throw;
  }
}

// ---------------------------------------------------------------------------
mm::CanFdFrame make_drive_velocity_frame(
  mm::Controller & ctrl, double velocity, double maximum_torque, double accel_limit,
  double watchdog_timeout)
{
  mm::PositionMode::Command cmd;
  cmd.position = kNaN;
  cmd.velocity = velocity;
  cmd.maximum_torque = maximum_torque;
  cmd.accel_limit = accel_limit;
  cmd.watchdog_timeout = watchdog_timeout;
  auto fmt = ignore_all_format();
  fmt.position = mm::kFloat;
  fmt.velocity = mm::kFloat;
  fmt.maximum_torque = mm::kFloat;
  fmt.accel_limit = mm::kFloat;
  fmt.watchdog_timeout = mm::kFloat;
  return ctrl.MakePosition(cmd, &fmt);
}

mm::CanFdFrame make_steer_position_frame(
  mm::Controller & ctrl, double position, double velocity_limit, double maximum_torque,
  double watchdog_timeout)
{
  mm::PositionMode::Command cmd;
  cmd.position = position;
  cmd.velocity_limit = velocity_limit;
  cmd.maximum_torque = maximum_torque;
  cmd.watchdog_timeout = watchdog_timeout;
  auto fmt = ignore_all_format();
  fmt.position = mm::kFloat;
  fmt.velocity_limit = mm::kFloat;
  fmt.maximum_torque = mm::kFloat;
  fmt.watchdog_timeout = mm::kFloat;
  return ctrl.MakePosition(cmd, &fmt);
}

mm::CanFdFrame make_arm_position_frame(
  mm::Controller & ctrl, double position, double maximum_torque, double velocity_limit,
  double accel_limit)
{
  mm::PositionMode::Command cmd;
  cmd.position = position;
  cmd.velocity = kNaN;
  cmd.feedforward_torque = 0.0;
  cmd.maximum_torque = maximum_torque;
  cmd.velocity_limit = velocity_limit;
  cmd.accel_limit = accel_limit;
  auto fmt = ignore_all_format();
  fmt.position = mm::kFloat;
  fmt.velocity = mm::kFloat;
  fmt.feedforward_torque = mm::kFloat;
  fmt.maximum_torque = mm::kFloat;
  fmt.velocity_limit = mm::kFloat;
  fmt.accel_limit = mm::kFloat;
  return ctrl.MakePosition(cmd, &fmt);
}

}  // namespace cmr_rovernet_cpp
