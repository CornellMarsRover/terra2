#include "cmr_rovernet_cpp/drive_common.hpp"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace cmr_rovernet_cpp {
namespace drive {

const std::vector<int> & drive_ids()
{
  static const std::vector<int> ids{1, 2, 3, 4};
  return ids;
}

const std::vector<int> & steer_ids()
{
  static const std::vector<int> ids{5, 7, 6, 8};
  return ids;
}

const std::vector<int> & all_ids()
{
  static const std::vector<int> ids{1, 2, 3, 4, 5, 7, 6, 8};
  return ids;
}

double forward_sign(int motor_id)
{
  switch (motor_id) {
    case 1: return 1.0;   // front left
    case 2: return 1.0;   // back left
    case 3: return -1.0;  // front right
    case 4: return -1.0;  // back right
    default: return 1.0;
  }
}

double steer_sign(int /*motor_id*/) {return 1.0;}

std::map<int, double> steer_center_offsets()
{
  return {{5, 0.0}, {7, 0.0}, {6, 0.0}, {8, 0.0}};
}

std::optional<DecodedButtons> decode_controller_buttons(const std::vector<int32_t> & buttons)
{
  DecodedButtons d;
  if (buttons.size() >= 8) {
    d.legacy_packed = false;
    d.l1 = buttons[BTN_L1];
    d.r1 = buttons[BTN_R1];
    d.triangle = buttons[BTN_TRIANGLE];
    d.l2 = buttons[BTN_L2];
    d.r2 = buttons[BTN_R2];
    return d;
  }
  if (buttons.size() >= 2) {
    const int64_t trigger_val = buttons[0];
    const int64_t button_val = buttons[1];
    d.legacy_packed = true;
    d.l1 = trigger_val == LEGACY_L1 ? 1 : 0;
    d.r1 = 0;
    d.triangle = button_val == LEGACY_TRIANGLE ? 1 : 0;
    d.l2 = legacy_l2_value(trigger_val);
    d.r2 = legacy_r2_value(trigger_val);
    return d;
  }
  return std::nullopt;
}

int64_t legacy_r2_value(int64_t trigger_val)
{
  // NOTE(port): faithful to the Python `hex(v & 0xFFFFFFFF)[2:4]`, which takes
  // the two most significant *printed* hex digits (no zero padding), so the
  // decoded byte depends on the magnitude of the packed value rather than
  // always being the top byte.
  const auto masked = static_cast<uint32_t>(static_cast<uint64_t>(trigger_val) & 0xFFFFFFFFu);
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%x", masked);
  const std::string first_two = std::string(buf).substr(0, 2);
  const int64_t result_int = std::stoll(first_two, nullptr, 16);
  if (result_int >= 0 && result_int <= TRIGGER_MAX) {
    return result_int;
  }
  return 0;
}

int64_t legacy_l2_value(int64_t trigger_val)
{
  if (LEGACY_L2_MIN <= trigger_val && trigger_val <= LEGACY_L2) {
    const double frac = static_cast<double>(trigger_val - LEGACY_L2_MIN) /
      static_cast<double>(LEGACY_L2 - LEGACY_L2_MIN);
    return static_cast<int64_t>(std::nearbyint(frac * static_cast<double>(TRIGGER_MAX)));
  }
  return 0;
}

// Python's round(x, ndigits): correctly rounded to the nearest decimal with
// ties-to-even on the exact binary value.  glibc printf performs exactly this
// rounding, so format and parse back (same helper as cmr_controls_cpp).
double py_round(double x, int ndigits)
{
  if (!std::isfinite(x)) {
    return x;
  }
  char buf[512];
  std::snprintf(buf, sizeof(buf), "%.*f", ndigits, x);
  return std::strtod(buf, nullptr);
}

namespace {

// Python `x**2` is libm pow(x, 2.0), which is not always the correctly
// rounded x*x.  Call the real pow (the volatile exponent stops the compiler
// from folding it into a multiplication) so wheel speeds match the Python
// originals bit for bit.
double py_sq(double x)
{
  static const volatile double two = 2.0;
  return std::pow(x, two);
}

struct Wheel
{
  double a, b, c, d;
};

Wheel swerve_terms(double vx, double vy, double omega)
{
  const double radius = std::sqrt(py_sq(ROVER_LENGTH) + py_sq(ROVER_WIDTH));
  vx = py_round(vx, 3);
  vy = py_round(vy, 3);
  omega = py_round(omega, 3);
  Wheel w;
  w.a = vy - omega * (ROVER_LENGTH / radius);
  w.b = vy + omega * (ROVER_LENGTH / radius);
  w.c = vx - omega * (ROVER_WIDTH / radius);
  w.d = vx + omega * (ROVER_WIDTH / radius);
  return w;
}

double wheel_angle(double y, double x)
{
  return py_round(std::atan2(y, x) * 180.0 / M_PI, 3) * STEER_DEGREES_TO_POSITION;
}

}  // namespace

SwerveTargets calculate_swerve_targets(
  double vx, double vy, double omega, double drive_rps,
  const std::map<int, double> & center_offsets)
{
  const Wheel w = swerve_terms(vx, vy, omega);

  double ws1 = std::sqrt(py_sq(w.b) + py_sq(w.c));
  double ws2 = std::sqrt(py_sq(w.b) + py_sq(w.d));
  double ws3 = std::sqrt(py_sq(w.a) + py_sq(w.d));
  double ws4 = std::sqrt(py_sq(w.a) + py_sq(w.c));

  const double max_speed = std::max({ws1, ws2, ws3, ws4, 1.0});
  ws1 /= max_speed;
  ws2 /= max_speed;
  ws3 /= max_speed;
  ws4 /= max_speed;

  const double wa1 = wheel_angle(w.b, w.c);
  const double wa2 = wheel_angle(w.b, w.d);
  const double wa3 = wheel_angle(w.a, w.d);
  const double wa4 = wheel_angle(w.a, w.c);

  SwerveTargets t;
  t.drive = {
    {1, forward_sign(1) * drive_rps * ws2},
    {2, forward_sign(2) * drive_rps * ws3},
    {3, forward_sign(3) * drive_rps * ws4},
    {4, forward_sign(4) * drive_rps * ws1},
  };
  t.steer = {
    {5, center_offsets.at(5) + steer_sign(5) * wa2},
    {7, center_offsets.at(7) + steer_sign(7) * wa1},
    {6, center_offsets.at(6) + steer_sign(6) * wa3},
    {8, center_offsets.at(8) + steer_sign(8) * wa4},
  };
  return t;
}

MotorTargets calculate_steer_targets(
  double vx, double vy, double omega, const std::map<int, double> & center_offsets)
{
  const Wheel w = swerve_terms(vx, vy, omega);
  const double wa1 = wheel_angle(w.b, w.c);
  const double wa2 = wheel_angle(w.b, w.d);
  const double wa3 = wheel_angle(w.a, w.d);
  const double wa4 = wheel_angle(w.a, w.c);
  return {
    {5, center_offsets.at(5) + steer_sign(5) * wa2},
    {7, center_offsets.at(7) + steer_sign(7) * wa1},
    {6, center_offsets.at(6) + steer_sign(6) * wa3},
    {8, center_offsets.at(8) + steer_sign(8) * wa4},
  };
}

// Python repr(float): shortest round-trip representation.
std::string format_double(double v)
{
  if (std::isnan(v)) {
    return "nan";
  }
  if (std::isinf(v)) {
    return v > 0 ? "inf" : "-inf";
  }
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), v);
  std::string s(buf, res.ptr);
  if (s.find_first_of(".en") == std::string::npos) {
    s += ".0";
  }
  return s;
}

std::string format_targets(const MotorTargets & targets)
{
  std::ostringstream o;
  o << "{";
  bool first = true;
  for (const auto & kv : targets) {
    o << (first ? "" : ", ") << kv.first << ": " << format_double(kv.second);
    first = false;
  }
  o << "}";
  return o.str();
}

}  // namespace drive
}  // namespace cmr_rovernet_cpp
