// C++ port of cmr_rovernet/drive_modes.py: pure (ROS-independent) drive-mode
// selection and command shaping helpers.
#pragma once

#include <array>
#include <optional>
#include <string>

namespace cmr_rovernet_cpp {

/// Operator-selectable ways to use the rover's swerve modules.
enum class DriveMode
{
  TRANSLATION_ROTATION,  // "translation_rotation"
  ACKERMANN,             // "ackermann"
  STEADY_HEADING,        // "steady_heading"
  POINT_TURN,            // "point_turn"
};

/// DriveMode.value
const char * drive_mode_value(DriveMode mode);

/// parse_drive_mode(): `DriveMode(str(value).strip().lower())`.
/// Throws std::invalid_argument (Python ValueError) for an unknown mode.
DriveMode parse_drive_mode(const std::string & value);

/// drive_mode_for_dpad(): cardinal D-pad press -> mode (0 up, 2 right, 4 down,
/// 6 left); diagonals, neutral and unknown values -> nullopt (Python None).
std::optional<DriveMode> drive_mode_for_dpad(long long dpad);

/// shape_motion_for_mode(): returns {vx, vy, omega} after applying the mode's
/// motion constraints (before swerve kinematics).
std::array<double, 3> shape_motion_for_mode(DriveMode mode, double vx, double vy, double omega);
/// Overload for the Python `mode: str` form (throws like parse_drive_mode).
std::array<double, 3> shape_motion_for_mode(
  const std::string & mode, double vx, double vy, double omega);

}  // namespace cmr_rovernet_cpp
