#pragma once

#include <chrono>
#include <limits>

namespace cmr_rovernet_cpp {

/// Monotonic seconds (Python time.monotonic(), as used by command_mux.py and
/// usama_control_testing.py).  NOTE(port): the drive_only / steer_only /
/// single_drive_wheel Python nodes still use time.time() (wall clock) for
/// command staleness; a monotonic clock keeps the same timeouts but is immune
/// to NTP / RTC jumps on the Jetson.
inline double mono_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// "Never updated" timestamp (Python used 0.0 epoch seconds).
constexpr double kNever = -std::numeric_limits<double>::infinity();

}  // namespace cmr_rovernet_cpp
