// ROS-free Kalman filter cores.
//  - Kalman4: port of autonomous_navigation/kalman_localization.py (4-state CV)
//  - Kalman2: port of autonomous_navigation/new_kalman.py (2-state, gated)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include <Eigen/Dense>

namespace autonomous_navigation_cpp
{

// lat/lon (deg) -> local (north, west) metres w.r.t. reference (both nodes).
inline std::array<double, 2> latlon_to_nw(double lat0, double lon0, double lat, double lon)
{
  constexpr double R_e = 6378137.0;
  constexpr double kDegToRad = M_PI / 180.0;  // math.radians: x * (pi / 180)
  const double lat0_rad = lat0 * kDegToRad;
  const double lon0_rad = lon0 * kDegToRad;
  const double lat_rad = lat * kDegToRad;
  const double lon_rad = lon * kDegToRad;
  const double d_lat = lat_rad - lat0_rad;
  const double d_lon = lon_rad - lon0_rad;
  const double mean_lat = (lat_rad + lat0_rad) / 2.0;
  return {d_lat * R_e, -d_lon * R_e * std::cos(mean_lat)};
}

class Kalman4
{
public:
  struct LogRow
  {
    std::array<double, 2> meas;
    std::array<double, 4> cov;  // row-major 2x2
  };

  Eigen::Vector4d x = Eigen::Vector4d::Zero();      // [n, w, vn, vw]
  Eigen::Matrix4d P = Eigen::Matrix4d::Identity() * 0.005;

  std::optional<int64_t> last_camera_time;          // ns
  double last_camera_n{0.0};
  double last_camera_w{0.0};
  Eigen::Vector4d last_camera_cov = Eigen::Vector4d::Zero();
  double current_yaw{0.0};

  std::optional<double> initial_lat, initial_lon;

  // depth_callback. `cov4` = msg.twist.covariance[0:4].
  // NOTE(port): covariance[0:4] of the row-major 6x6 is row 0 (xx, xy, xz, x-roll),
  // not the top-left 2x2 block, despite the comment in kalman_localization.py:156.
  // Preserved.
  std::optional<LogRow> predict(int64_t now_ns, double n, double w, double yaw,
    const std::array<double, 4> & cov4)
  {
    const Eigen::Vector4d cov_new(cov4[0], cov4[1], cov4[2], cov4[3]);
    if (!last_camera_time) {
      last_camera_time = now_ns;
      last_camera_n = n;
      last_camera_w = w;
      last_camera_cov = cov_new;
      current_yaw = yaw;
      return std::nullopt;
    }
    const double dt = static_cast<double>(now_ns - *last_camera_time) * 1e-9;
    if (dt <= 1e-6) {
      return std::nullopt;
    }
    last_camera_time = now_ns;
    const double dn = n - last_camera_n;
    const double dw = w - last_camera_w;
    last_camera_n = n;
    last_camera_w = w;
    current_yaw = yaw;

    const Eigen::Vector4d Sflat = (cov_new + last_camera_cov) / (dt * dt);
    last_camera_cov = cov_new;
    Eigen::Matrix2d S;
    S << Sflat(0), Sflat(1), Sflat(2), Sflat(3);  // reshape(2, 2), row-major

    Eigen::Matrix<double, 4, 2> J;
    J << dt, 0.0, 0.0, dt, 1.0, 0.0, 0.0, 1.0;
    const Eigen::Matrix4d Q = (J * S) * J.transpose();

    const double v_n = dn / dt;
    const double v_w = dw / dt;
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0, 2) = dt;
    A(1, 3) = dt;

    x(2) = v_n;
    x(3) = v_w;
    x = (A * x).eval();
    P = ((A * P) * A.transpose() + Q).eval();

    return LogRow{{dn, dw}, {S(0, 0), S(0, 1), S(1, 0), S(1, 1)}};
  }

  // gps_callback. Returns nullopt on the first (reference) fix.
  // NOTE(port): NavSatFix.altitude is used as the GPS variance (Python quirk).
  std::optional<LogRow> correct(double lat, double lon, double altitude)
  {
    if (!initial_lat) {
      initial_lat = lat;
      initial_lon = lon;
      return std::nullopt;
    }
    const auto nw = latlon_to_nw(*initial_lat, *initial_lon, lat, lon);
    const Eigen::Vector2d z(nw[0], nw[1]);
    Eigen::Matrix2d R;
    R << altitude, 0.0, 0.0, altitude;
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    const Eigen::Vector2d y = z - H * x;
    const Eigen::Matrix2d S = (H * P) * H.transpose() + R;
    const Eigen::Matrix<double, 4, 2> K = (P * H.transpose()) * S.inverse();
    x = (x + K * y).eval();
    P = ((Eigen::Matrix4d::Identity() - K * H) * P).eval();
    return LogRow{{z(0), z(1)}, {R(0, 0), R(0, 1), R(1, 0), R(1, 1)}};
  }
};

class Kalman2
{
public:
  enum class Predict { NonFinite, Init, TooSoon, Applied, TooFast };
  enum class Correct { Invalid, ReferenceSet, Updated, Gated };

  Eigen::Vector2d x = Eigen::Vector2d::Zero();
  Eigen::Matrix2d P = (Eigen::Vector2d(1e3, 1e3)).asDiagonal();
  double min_var{1e-2};
  double gate_thresh{9.21};
  double vel_limit{1.5};

  std::optional<double> last_z_time;   // ns as float, like Python
  Eigen::Vector2d last_z_meas = Eigen::Vector2d::Zero();
  std::optional<double> last_g_time;
  std::optional<double> initial_lat, initial_lon;
  bool initialized{false};             // set by the first valid GPS fix
  double current_yaw{0.0};

  // depth_callback. `speed` (if given) receives |v| for Applied / TooFast.
  Predict predict(double t_ns, double mx, double my, double yaw, double * speed = nullptr)
  {
    if (!(std::isfinite(mx) && std::isfinite(my)) || !std::isfinite(yaw)) {
      return Predict::NonFinite;
    }
    const Eigen::Vector2d meas(mx, my);
    if (!last_z_time) {
      last_z_time = t_ns;
      last_z_meas = meas;
      current_yaw = yaw;
      return Predict::Init;
    }
    const double dt = (t_ns - *last_z_time) * 1e-9;
    if (dt <= 1e-6) {
      return Predict::TooSoon;
    }
    const Eigen::Vector2d delta = meas - last_z_meas;
    const Eigen::Vector2d v = delta / dt;
    const double v_norm = v.norm();
    if (speed) {
      *speed = v_norm;
    }
    last_z_time = t_ns;
    last_z_meas = meas;
    current_yaw = yaw;
    if (v_norm < vel_limit) {
      Eigen::Matrix2d R;
      R << 0.0001, 0.0, 0.0, 0.0001;
      x += delta;
      P += R;
      return Predict::Applied;
    }
    return Predict::TooFast;
  }

  // gps_callback. `status` = NavSatFix.status.status, `cov9` =
  // NavSatFix.position_covariance.
  Correct correct(
    double t_ns, int status, double lat, double lon, const std::array<double, 9> & cov9)
  {
    if (status < 0 || !(std::isfinite(lat) && std::isfinite(lon) && std::isfinite(cov9[0]) &&
      std::isfinite(cov9[1]) && std::isfinite(cov9[3]) && std::isfinite(cov9[4])))
    {
      return Correct::Invalid;
    }
    if (!initial_lat) {
      initial_lat = lat;
      initial_lon = lon;
      initialized = true;
      return Correct::ReferenceSet;
    }
    const auto nw = latlon_to_nw(*initial_lat, *initial_lon, lat, lon);
    const Eigen::Vector2d z(nw[0], nw[1]);
    Eigen::Matrix2d R;
    R << cov9[0], cov9[1], cov9[3], cov9[4];
    const Eigen::Vector2d y = z - x;
    const Eigen::Matrix2d S = P + R;
    // NOTE(port): numpy raises LinAlgError on a singular S (node dies); Eigen
    // yields non-finite values, the gate test fails and the fix is skipped.
    const Eigen::Matrix2d Sinv = S.inverse();
    const double maha2 = y.transpose() * Sinv * y;
    Correct result = Correct::Gated;
    if (maha2 < gate_thresh) {
      const Eigen::Matrix2d K = P * Sinv;
      x = (x + K * y).eval();
      P = ((Eigen::Matrix2d::Identity() - K) * P).eval();
      P(0, 0) = std::max(P(0, 0), min_var);
      P(1, 1) = std::max(P(1, 1), min_var);
      result = Correct::Updated;
    }
    last_g_time = t_ns;
    return result;
  }
};

}  // namespace autonomous_navigation_cpp
