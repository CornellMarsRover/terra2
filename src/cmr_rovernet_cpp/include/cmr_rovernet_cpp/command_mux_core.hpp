// C++ port of cmr_rovernet/command_mux_core.py: transport-independent drive
// command selection; callers supply monotonic receipt times.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cmr_rovernet_cpp {

class CommandMux
{
public:
  using Values = std::vector<double>;

  /// Throws std::invalid_argument (Python ValueError) unless @p timeout is
  /// positive and finite.
  explicit CommandMux(double timeout = 0.5);

  /// Select "teleop" or "autonomy" (anything else throws
  /// std::invalid_argument) and discard every stored command, so the newly
  /// selected source must send a fresh command before anything is output.
  void select(const std::string & source);

  /// Latch the emergency stop and discard every stored command.
  void emergency_stop();

  /// Release the emergency stop; stored commands are discarded again, so a
  /// fresh command is required before anything is output.
  void reset();

  /// Store @p values for @p source at time @p now.  Unknown source throws
  /// std::invalid_argument.  A non-finite value or time erases that source's
  /// stored command and returns false.  While estopped the command is
  /// accepted (true) but not stored.
  bool receive(const std::string & source, const Values & values, double now);

  /// The selected source's command if it is fresh
  /// (0 <= now - received < timeout) and the mux is not estopped.
  std::optional<Values> output(double now) const;

  double timeout() const {return timeout_;}
  const std::string & source() const {return source_;}
  bool estop() const {return estop_;}

private:
  double timeout_;
  std::string source_ = "teleop";
  bool estop_ = false;
  std::map<std::string, std::pair<Values, double>> commands_;
};

}  // namespace cmr_rovernet_cpp
