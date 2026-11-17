// Shared drive/steer logic ported from cmr_rovernet/usama_control_testing.py
// (constants, controller button decoding, swerve kinematics), used by the
// usama_control_testing / drive_only / steer_only / single_drive_wheel nodes.
#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cmr_rovernet_cpp {
namespace drive {

const std::vector<int> & drive_ids();  // [1, 2, 3, 4]
const std::vector<int> & steer_ids();  // [5, 7, 6, 8]
const std::vector<int> & all_ids();    // DRIVE_IDS + STEER_IDS

constexpr double STEER_DEGREES_TO_POSITION = 50.0 / 360.0;

double forward_sign(int motor_id);  // FORWARD_SIGN / DRIVE_FORWARD_SIGN
double steer_sign(int motor_id);    // STEER_SIGN
std::map<int, double> steer_center_offsets();  // STEER_CENTER_OFFSETS

constexpr int BTN_L1 = 0;
constexpr int BTN_R1 = 1;
constexpr int BTN_L2 = 2;
constexpr int BTN_R2 = 3;
constexpr int BTN_TRIANGLE = 7;

constexpr int64_t LEGACY_L1 = 1;
constexpr int64_t LEGACY_L2_MIN = 65536;
constexpr int64_t LEGACY_L2 = 16711680;
constexpr int64_t LEGACY_TRIANGLE = 16777216;

constexpr double STICK_RANGE = 2.5;
constexpr int64_t TRIGGER_DEADBAND = 5;
constexpr int64_t TRIGGER_MAX = 255;
constexpr int64_t TRIGGER_PRESSED_THRESHOLD = 32;
constexpr double HALF_SPEED_MULTIPLIER = 0.5;
constexpr double DOUBLE_SPEED_MULTIPLIER = 2.0;
constexpr double TRIPLE_SPEED_MULTIPLIER = 3.0;

constexpr double ROVER_LENGTH = 39.0;
constexpr double ROVER_WIDTH = 39.0;
constexpr double COMMAND_EPSILON = 1e-3;

struct DecodedButtons
{
  bool legacy_packed = false;
  int64_t l1 = 0;
  int64_t r1 = 0;
  int64_t triangle = 0;
  int64_t l2 = 0;
  int64_t r2 = 0;
};

/// UsamaControlRosNode._decode_controller_buttons (None -> std::nullopt).
std::optional<DecodedButtons> decode_controller_buttons(const std::vector<int32_t> & buttons);
int64_t legacy_r2_value(int64_t trigger_val);
int64_t legacy_l2_value(int64_t trigger_val);

/// Exact Python round(x, ndigits) (correctly rounded decimal, half-even).
double py_round(double x, int ndigits);

/// Python `max(-limit, min(limit, value))`, including its handling of the
/// corner cases (first argument wins on ties; NaN compares false -> +limit).
inline double clamp(double value, double limit = 1.0)
{
  const double m = value < limit ? value : limit;  // min(limit, value)
  return m > -limit ? m : -limit;                  // max(-limit, m)
}

/// true if every linear/angular component is finite.
template<typename TwistT>
bool twist_finite(const TwistT & t)
{
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
         std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
}

inline double apply_deadzone(double value, double deadzone)
{
  return (value < 0 ? -value : value) < deadzone ? 0.0 : value;
}

/// Ordered (motor_id, value) list, preserving the Python dict insertion order.
using MotorTargets = std::vector<std::pair<int, double>>;

struct SwerveTargets
{
  MotorTargets drive;  // ids 1, 2, 3, 4
  MotorTargets steer;  // ids 5, 7, 6, 8
};

SwerveTargets calculate_swerve_targets(
  double vx, double vy, double omega, double drive_rps,
  const std::map<int, double> & center_offsets);

/// steer_only_control.calculate_steer_targets
MotorTargets calculate_steer_targets(
  double vx, double vy, double omega, const std::map<int, double> & center_offsets);

/// "{1: 0.5, 2: 0.5}" style rendering for log messages.
std::string format_targets(const MotorTargets & targets);
std::string format_double(double v);

}  // namespace drive
}  // namespace cmr_rovernet_cpp
