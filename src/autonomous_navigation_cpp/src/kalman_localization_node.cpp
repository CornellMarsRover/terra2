// Port of autonomous_navigation/kalman_localization.py (executable
// `kalman_localization`, node `kalman_localization_node`).
#include <array>
#include <charconv>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "autonomous_navigation_cpp/kalman_core.hpp"

namespace autonomous_navigation_cpp
{

namespace
{
// str(float) style: shortest round-trip, always with a '.' or exponent.
std::string py_float_str(double v)
{
  char buf[64];
  auto res = std::to_chars(buf, buf + sizeof(buf), v);
  std::string s(buf, res.ptr);
  if (s.find_first_of(".eni") == std::string::npos) {
    s += ".0";
  }
  return s;
}
}  // namespace

class KalmanLocalizationNode : public rclcpp::Node
{
public:
  explicit KalmanLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("kalman_localization_node", options)
  {
    record_data_ = declare_parameter<bool>("record_data", false);
    // NOTE(port): Python overrides the parameter with True (kalman_localization.py:60).
    record_data_ = true;
    if (record_data_) {
      const char * home = std::getenv("HOME");
      const std::filesystem::path log_dir = std::filesystem::path(home ? home : ".") /
        "kalman_logs";
      std::error_code ec;
      std::filesystem::create_directories(log_dir, ec);
      char ts[32];
      const std::time_t t = std::time(nullptr);
      std::tm tm{};
      localtime_r(&t, &tm);
      std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
      const std::string filename = (log_dir / ("kalman_log_" + std::string(ts) + ".csv")).string();
      csv_.open(filename, std::ios::out | std::ios::trunc);
      if (csv_) {
        csv_ << "ros_time_s,source,meas_n,meas_w,cov_00,cov_01,cov_10,cov_11,x_n,x_w,x_vn,x_vw";
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            csv_ << ",P_" << i << j;
          }
        }
        csv_ << "\r\n";  // csv.writer default lineterminator
        csv_.flush();
        RCLCPP_INFO(get_logger(), "Recording enabled → %s", filename.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Could not open %s for writing", filename.c_str());
      }
    }

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/rtk/navsatfix_data", 10,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        const auto row = kf_.correct(m->latitude, m->longitude, m->altitude);
        if (!row) {
          RCLCPP_INFO(get_logger(), "Initial GPS ref: lat=%.6f, lon=%.6f",
          *kf_.initial_lat, *kf_.initial_lon);
          return;
        }
        log_measurement("gps", *row);
      });
    zed_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/zed/pose", 30,
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
        const int64_t now_ns = static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(m->header.stamp.nanosec);
        const std::array<double, 4> cov4{
          m->twist.covariance[0], m->twist.covariance[1],
          m->twist.covariance[2], m->twist.covariance[3]};
        const auto row = kf_.predict(now_ns, m->twist.twist.linear.x, m->twist.twist.linear.y,
        m->twist.twist.angular.z, cov4);
        if (row) {
          log_measurement("zed", *row);
        }
      });
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/autonomy/pose/robot/global", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
        [this]() {
          geometry_msgs::msg::TwistStamped msg;
          msg.header.stamp = now();
          msg.twist.linear.x = kf_.x(0);
          msg.twist.linear.y = kf_.x(1);
          msg.twist.angular.z = kf_.current_yaw;
          pub_->publish(msg);
        });
    RCLCPP_INFO(get_logger(), "Kalman Localization node started.");
  }

private:
  void log_measurement(const char * source, const Kalman4::LogRow & row)
  {
    if (!record_data_ || !csv_) {
      return;
    }
    const double t = static_cast<double>(now().nanoseconds()) * 1e-9;
    std::string line = py_float_str(t) + "," + source;
    for (double v : row.meas) {line += "," + py_float_str(v);}
    for (double v : row.cov) {line += "," + py_float_str(v);}
    for (int i = 0; i < 4; ++i) {line += "," + py_float_str(kf_.x(i));}
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        line += "," + py_float_str(kf_.P(i, j));
      }
    }
    line += "\r\n";
    csv_ << line;
    csv_.flush();
  }

  bool record_data_{false};
  std::ofstream csv_;
  Kalman4 kf_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr zed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::KalmanLocalizationNode)
