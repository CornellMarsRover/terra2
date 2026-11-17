// Port of autonomous_navigation/drive_command.py: convert autonomy decisions
// to normalized shared-drive commands. Python ValueError -> std::invalid_argument.
#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{
namespace drive_command
{

struct ChassisCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  bool operator==(const ChassisCommand & o) const
  {
    return linear_x == o.linear_x && linear_y == o.linear_y && angular_z == o.angular_z;
  }
};

inline double clamp(double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("command values must be finite");
  }
  return py_max(-1.0, py_min(1.0, value));
}

// Require recent pose and waypoint samples before allowing rover motion.
inline bool inputs_fresh(
  double now_s, std::optional<double> pose_time_s, std::optional<double> waypoint_time_s,
  double timeout_s)
{
  if (timeout_s <= 0.0 || !std::isfinite(timeout_s)) {
    throw std::invalid_argument("timeout_s must be finite and positive");
  }
  if (!pose_time_s || !waypoint_time_s) {
    return false;
  }
  const double ages[2] = {now_s - *pose_time_s, now_s - *waypoint_time_s};
  for (double age : ages) {
    if (!(std::isfinite(age) && 0.0 <= age && age <= timeout_s)) {
      return false;
    }
  }
  return true;
}

// Return whether a planar waypoint is within the arrival tolerance.
inline bool waypoint_reached(
  double px, double py, double wx, double wy, double tolerance = 0.3)
{
  if (tolerance < 0.0) {
    throw std::invalid_argument("tolerance must not be negative");
  }
  return py_hypot(px - wx, py - wy) <= tolerance;
}

// Map forward demand and heading error to normalized drive axes.
inline ChassisCommand forward_heading_command(
  double linear_x, double heading_error_deg, double max_heading_error_deg = 35.0)
{
  if (max_heading_error_deg <= 0.0) {
    throw std::invalid_argument("max_heading_error_deg must be positive");
  }
  const double heading =
    py_max(-max_heading_error_deg, py_min(max_heading_error_deg, heading_error_deg));
  ChassisCommand c;
  c.linear_x = clamp(linear_x);
  c.angular_z = clamp(py_radians(heading));
  return c;
}

// Map a signed turn demand to the shared drive command.
inline ChassisCommand point_turn_command(double angular_z)
{
  ChassisCommand c;
  c.angular_z = clamp(angular_z);
  return c;
}

}  // namespace drive_command
}  // namespace autonomous_navigation_cpp
