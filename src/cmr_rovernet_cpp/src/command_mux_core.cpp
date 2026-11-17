#include "cmr_rovernet_cpp/command_mux_core.hpp"

#include <cmath>
#include <stdexcept>

namespace cmr_rovernet_cpp {

namespace {

bool known_source(const std::string & source)
{
  return source == "teleop" || source == "autonomy";
}

}  // namespace

CommandMux::CommandMux(double timeout)
: timeout_(timeout)
{
  // `timeout <= 0` is written as a negated `>` so the intent (reject
  // everything that is not strictly positive) is explicit; NaN is already
  // rejected by isfinite, exactly as in Python.
  if (!std::isfinite(timeout) || !(timeout > 0)) {
    throw std::invalid_argument("timeout must be positive and finite");
  }
}

void CommandMux::select(const std::string & source)
{
  if (!known_source(source)) {
    throw std::invalid_argument("source must be teleop or autonomy");
  }
  source_ = source;
  commands_.clear();
}

void CommandMux::emergency_stop()
{
  estop_ = true;
  commands_.clear();
}

void CommandMux::reset()
{
  estop_ = false;
  commands_.clear();
}

bool CommandMux::receive(const std::string & source, const Values & values, double now)
{
  if (!known_source(source)) {
    throw std::invalid_argument("unknown command source");
  }
  bool finite = std::isfinite(now);
  for (double v : values) {
    finite = finite && std::isfinite(v);
  }
  if (!finite) {
    commands_.erase(source);
    return false;
  }
  if (!estop_) {
    commands_[source] = {values, now};
  }
  return true;
}

std::optional<CommandMux::Values> CommandMux::output(double now) const
{
  if (estop_ || !std::isfinite(now)) {
    return std::nullopt;
  }
  const auto it = commands_.find(source_);
  if (it == commands_.end()) {
    return std::nullopt;
  }
  // Python: `not 0 <= now - command[1] < self.timeout`
  const double age = now - it->second.second;
  if (!(0 <= age && age < timeout_)) {
    return std::nullopt;
  }
  return it->second.first;
}

}  // namespace cmr_rovernet_cpp
