// Port of autonomous_navigation/new_kalman.py (executable `new_kalman`,
// node `kalman_localization_node`).
#include <array>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "autonomous_navigation_cpp/kalman_core.hpp"

namespace autonomous_navigation_cpp
{

class NewKalmanNode : public rclcpp::Node
{
public:
  explicit NewKalmanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("kalman_localization_node", options)
  {
    zed_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/zed/pose", 30,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        const double t_ns = static_cast<double>(m->header.stamp.sec) * 1e9 +
        static_cast<double>(m->header.stamp.nanosec);
        double speed = 0.0;
        switch (kf_.predict(
          t_ns, m->twist.linear.x, m->twist.linear.y, m->twist.angular.z, &speed))
        {
          case Kalman2::Predict::NonFinite:
            RCLCPP_ERROR(get_logger(), "Ignoring non-finite ZED pose");
            break;
          case Kalman2::Predict::TooFast:
            RCLCPP_WARN(
              get_logger(), "ZED update skipped (|v|=%.2f m/s > %g)", speed, kf_.vel_limit);
            break;
          default:
            break;
        }
      });
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/rtk/navsatfix_data", 10,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        const double t_ns = static_cast<double>(m->header.stamp.sec) * 1e9 +
        static_cast<double>(m->header.stamp.nanosec);
        std::array<double, 9> cov;
        std::copy(m->position_covariance.begin(), m->position_covariance.end(), cov.begin());
        switch (kf_.correct(t_ns, m->status.status, m->latitude, m->longitude, cov)) {
          case Kalman2::Correct::Invalid:
            RCLCPP_ERROR(get_logger(), "Ignoring invalid GPS fix");
            break;
          case Kalman2::Correct::ReferenceSet:
            RCLCPP_INFO(
              get_logger(), "Initial GPS ref set to lat=%.6f, lon=%.6f",
              *kf_.initial_lat, *kf_.initial_lon);
            break;
          default:
            break;
        }
      });
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/autonomy/pose/robot/global", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1),
      [this]() {
        // Nothing is published until the first valid GPS fix set the reference.
        if (!kf_.initialized) {
          return;
        }
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = now();
        msg.twist.linear.x = kf_.x(0);
        msg.twist.linear.y = kf_.x(1);
        // NOTE(port): docstring says yaw is wrapped to [-pi, pi]; it is not.
        msg.twist.angular.z = kf_.current_yaw;
        pub_->publish(msg);
      });
    RCLCPP_INFO(get_logger(), "2-state Kalman Localization node started.");
  }

private:
  Kalman2 kf_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr zed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::NewKalmanNode)
