// ROS-free decision logic of autonomous_navigation/controller.py
// (ControllerNode.follow_waypoint and its callbacks). The node wrapper feeds
// it clock readings and publishes what step() returns.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autonomous_navigation_cpp/drive_command.hpp"
#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{

class ControllerCore
{
public:
  // builtin_interfaces/Time as produced by clock.now().to_msg().
  struct Stamp
  {
    int32_t sec{0};
    uint32_t nanosec{0};
  };
  static Stamp stamp_from_ns(int64_t ns)
  {
    // rclpy Time.to_msg(): divmod(nanoseconds, 10**9) (floor division).
    int64_t sec = ns / 1000000000LL;
    int64_t rem = ns % 1000000000LL;
    if (rem < 0) {
      rem += 1000000000LL;
      sec -= 1;
    }
    return Stamp{static_cast<int32_t>(sec), static_cast<uint32_t>(rem)};
  }

  struct Output
  {
    drive_command::ChassisCommand drive;   // always published
    std::optional<std::string> movement;   // /autonomy/move/move_type, if any
  };

  // NOTE(port): Python only discovers an invalid input_timeout_s /
  // waypoint_tolerance inside follow_waypoint (ValueError from drive_command,
  // which kills the node once the first waypoint arrives). Here they are
  // rejected at construction.
  ControllerCore(
    bool real, double waypoint_tolerance, double input_timeout_s, int64_t now_ns)
  : waypoint_tolerance_(waypoint_tolerance), input_timeout_s_(input_timeout_s),
    last_command_time_(stamp_from_ns(now_ns)), min_wait_(real ? 1.0 : 0.5)
  {
    if (input_timeout_s <= 0.0 || !std::isfinite(input_timeout_s)) {
      throw std::invalid_argument("timeout_s must be finite and positive");
    }
    if (waypoint_tolerance < 0.0) {
      throw std::invalid_argument("tolerance must not be negative");
    }
  }

  // update_pose(). Returns false (state untouched) for a non-finite pose.
  bool update_pose(double x, double y, double yaw, int64_t now_ns)
  {
    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw))) {
      return false;
    }
    robot_position_ = {x, y};
    yaw_ = yaw;
    last_pose_time_s_ = static_cast<double>(now_ns) * 1e-9;
    return true;
  }

  // update_waypoint(). Returns false for a malformed payload (fewer than two
  // values, or any non-finite value anywhere in the array).
  bool update_waypoint(const std::vector<float> & data, int64_t now_ns)
  {
    if (data.size() < 2) {
      return false;
    }
    for (float v : data) {
      if (!std::isfinite(v)) {
        return false;
      }
    }
    waypoint_ = std::make_pair(static_cast<double>(data[0]), static_cast<double>(data[1]));
    last_waypoint_time_s_ = static_cast<double>(now_ns) * 1e-9;
    return true;
  }

  // set_stop(): any message latches stopped forever.
  void set_stop() {stopped_ = true;}

  static drive_command::ChassisCommand stop_command()
  {
    return drive_command::forward_heading_command(0.0, 0.0);
  }

  // follow_waypoint().
  Output step(int64_t now_ns)
  {
    const double now_s = static_cast<double>(now_ns) * 1e-9;
    if (!waypoint_ || !drive_command::inputs_fresh(
        now_s, last_pose_time_s_, last_waypoint_time_s_, input_timeout_s_))
    {
      return {stop_command(), std::string("stopped")};
    }
    if (stopped_) {
      return {stop_command(), std::nullopt};
    }
    const double x_error = waypoint_->first - robot_position_.first;
    const double y_error = waypoint_->second - robot_position_.second;
    if (drive_command::waypoint_reached(
        robot_position_.first, robot_position_.second, waypoint_->first, waypoint_->second,
        waypoint_tolerance_))
    {
      return {stop_command(), std::string("stopped")};
    }
    const double angle_to_target = std::atan2(y_error, x_error);
    double heading_error = angle_to_target - yaw_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
    const double angle_error_deg = py_degrees(heading_error);

    const Stamp curr_time = stamp_from_ns(now_ns);
    const double dt = compute_time_delta(curr_time, last_command_time_);
    Output out;
    if (std::fabs(angle_error_deg) > point_turn_threshold_) {
      point_turn_threshold_ = 20;  // reduce threshold while point turning
      if (last_movement_ == "ackerman" && dt < min_wait_) {
        // Stop while the steering modules transition to point-turn mode.
        out.drive = drive_command::point_turn_command(0.0);
      } else {
        const double turn_sign = angle_error_deg > 0.0 ? 1.0 : -1.0;
        out.drive = drive_command::point_turn_command(turn_sign * point_turn_velocity_);
        last_movement_ = "point_turn";
        last_command_time_ = curr_time;
      }
      out.movement = last_movement_;
      return out;
    }
    point_turn_threshold_ = 50;
    const double steer_angle_deg = py_degrees(heading_error);
    if (last_movement_ == "point_turn" && dt < min_wait_) {
      out.drive = drive_command::forward_heading_command(0.0, steer_angle_deg);
    } else {
      out.drive = drive_command::forward_heading_command(ackerman_velocity_, steer_angle_deg);
      last_movement_ = "ackerman";
      last_command_time_ = curr_time;
    }
    out.movement = last_movement_;
    return out;
  }

  static double compute_time_delta(const Stamp & cur, const Stamp & last)
  {
    // Python ints: exact integer differences, then float arithmetic.
    const int64_t dsec = static_cast<int64_t>(cur.sec) - static_cast<int64_t>(last.sec);
    const int64_t dns = static_cast<int64_t>(cur.nanosec) - static_cast<int64_t>(last.nanosec);
    return static_cast<double>(dsec) + static_cast<double>(dns) * 1e-9;
  }

  const std::string & last_movement() const {return last_movement_;}

private:
  double waypoint_tolerance_;
  double input_timeout_s_;
  bool stopped_{false};
  std::pair<double, double> robot_position_{0.0, 0.0};
  double yaw_{0.0};
  std::optional<double> last_pose_time_s_;
  std::optional<double> last_waypoint_time_s_;
  double point_turn_velocity_{0.4};
  double ackerman_velocity_{0.9};
  std::string last_movement_{"ackerman"};
  Stamp last_command_time_;
  double min_wait_;
  double point_turn_threshold_{40};
  std::optional<std::pair<double, double>> waypoint_;
};

}  // namespace autonomous_navigation_cpp
