#include "cmr_rovernet_cpp/drive_modes.hpp"

#include <cmath>
#include <stdexcept>

namespace cmr_rovernet_cpp {

const char * drive_mode_value(DriveMode mode)
{
  switch (mode) {
    case DriveMode::TRANSLATION_ROTATION: return "translation_rotation";
    case DriveMode::ACKERMANN: return "ackermann";
    case DriveMode::STEADY_HEADING: return "steady_heading";
    case DriveMode::POINT_TURN: return "point_turn";
  }
  return "translation_rotation";
}

namespace {

// Python str.strip() whitespace (ASCII subset) and str.lower() (ASCII).
bool is_py_space(char c)
{
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f' ||
         (c >= 0x1c && c <= 0x1f);
}

}  // namespace

DriveMode parse_drive_mode(const std::string & value)
{
  std::size_t begin = 0;
  std::size_t end = value.size();
  while (begin < end && is_py_space(value[begin])) {
    ++begin;
  }
  while (end > begin && is_py_space(value[end - 1])) {
    --end;
  }
  std::string s = value.substr(begin, end - begin);
  for (char & c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  for (DriveMode mode : {DriveMode::TRANSLATION_ROTATION, DriveMode::ACKERMANN,
      DriveMode::STEADY_HEADING, DriveMode::POINT_TURN})
  {
    if (s == drive_mode_value(mode)) {
      return mode;
    }
  }
  throw std::invalid_argument("'" + s + "' is not a valid DriveMode");
}

std::optional<DriveMode> drive_mode_for_dpad(long long dpad)
{
  // D-pad values are defined by cmr_controller_remote/connect.py.
  switch (dpad) {
    case 0: return DriveMode::TRANSLATION_ROTATION;  // up
    case 2: return DriveMode::ACKERMANN;             // right
    case 4: return DriveMode::POINT_TURN;            // down
    case 6: return DriveMode::STEADY_HEADING;        // left
    default: return std::nullopt;
  }
}

std::array<double, 3> shape_motion_for_mode(DriveMode mode, double vx, double vy, double omega)
{
  switch (mode) {
    case DriveMode::TRANSLATION_ROTATION:
      return {vx, vy, omega};
    case DriveMode::STEADY_HEADING:
      return {vx, vy, 0.0};
    case DriveMode::POINT_TURN:
      return {0.0, 0.0, omega};
    case DriveMode::ACKERMANN:
      break;
  }
  // Ackermann-style driving has no sideways translation and no stationary
  // spin. Scaling omega by forward demand preserves a consistent curvature
  // as the operator changes speed.
  return {vx, 0.0, omega * std::abs(vx)};
}

std::array<double, 3> shape_motion_for_mode(
  const std::string & mode, double vx, double vy, double omega)
{
  return shape_motion_for_mode(parse_drive_mode(mode), vx, vy, omega);
}

}  // namespace cmr_rovernet_cpp
