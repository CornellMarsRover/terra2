// C++ port of cmr_controls/swerve_controller_node.py
// (console script `swerve_controller_node`).
//
// Drives the 4 swerve modules (moteus ids 1-4 = drive, 5-8 = steer) over an
// fdcanusb. Same node name, topics, QoS depths and (odd) sign/ID conventions
// as the Python original.
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <mjbots/moteus/moteus.h>

#include "cmr_controls_cpp/py_compat.hpp"
#include "cmr_msgs/msg/drive_command.hpp"
#include "cmr_controls_cpp/moteus_safe_bus.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moteus = mjbots::moteus;
using cmr_port::py_float_repr;
using cmr_port::py_round;

namespace
{

constexpr double WHEEL_RADIUS = 0.127;  // m
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * M_PI;
constexpr double SWERVE_RATIO = 50;  // swerve gearbox ratio
constexpr double DRIVE_RATIO = 26;   // drive gearbox ratio

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

class SwerveControllerNode : public rclcpp::Node
{
public:
  SwerveControllerNode()
  : Node("swerve_controller_node")
  {
    cmd_vel_sub_ = create_subscription<cmr_msgs::msg::DriveCommand>(
      "/cmd_vel", 10,
      std::bind(&SwerveControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    initialize_moteus();

    RCLCPP_INFO(get_logger(), "Swerve Controller Node has been started.");
  }

private:
  // Regular command velocity callback
  void cmd_vel_callback(const cmr_msgs::msg::DriveCommand::SharedPtr msg)
  {
    const double scale = msg->speed_rps * WHEEL_CIRCUMFERENCE / DRIVE_RATIO;
    double s[4], a[4];
    wheelAnglesAndSpeeds(msg->vx * scale, msg->vy * scale, msg->omega * scale, 1.0, 1.0, s, a);
    set_drive(s[0], s[1], s[2], s[3], a[0], a[1], a[2], a[3]);
  }

  // NOTE(port): unused on this branch (the Python method is also never called
  // now that the autonomy point-turn/ackerman callbacks are gone); kept for parity.
  void stop_wheels()
  {
    bus_->cycle([this]() {
        std::vector<moteus::CanFdFrame> frames;
        for (int id : kServoIds) {frames.push_back(bus_->ctrl(id, false).MakeStop());}
        return frames;
      });
  }

  void initialize_moteus()
  {
    // NOTE(port): moteus.Fdcanusb() in Python took the first fdcanusb it found.
    // Here can_port ("auto" by default) selects only an adapter that has been
    // locked and proven (by query) to carry this node's motors 1-8, so stop/rezero
    // and drive frames can never land on the arm's bus. If that adapter is held by
    // another node we log and wait for it rather than fall back to another one.
    declare_parameter<std::string>("can_port", "auto");
    bus_ = std::make_unique<cmr_controls_cpp::MotorBus>(
      get_logger(), get_parameter("can_port").as_string(),
      std::vector<int>(std::begin(kServoIds), std::end(kServoIds)));
    bus_->open_blocking();

    // reset servo positions (errors here propagate, as in Python)
    std::vector<moteus::CanFdFrame> frames;
    for (int id : kServoIds) {frames.push_back(bus_->ctrl(id, false).MakeStop());}
    bus_->cycle_or_throw(frames);
    frames.clear();
    for (int id : kServoIds) {
      // make_rezero() == set_output_nearest(position=0.0)
      moteus::OutputNearest::Command cmd;
      cmd.position = 0.0;
      frames.push_back(bus_->ctrl(id, false).MakeOutputNearest(cmd));
    }
    bus_->cycle_or_throw(frames);
  }

  moteus::CanFdFrame make_drive(int id, double velocity)
  {
    moteus::PositionMode::Command cmd;
    cmd.position = kNaN;
    cmd.velocity = velocity;
    cmd.accel_limit = drives_max_acc_;
    cmd.velocity_limit = drives_max_vel_;
    cmd.maximum_torque = drives_max_torque_;
    moteus::PositionMode::Format fmt;
    fmt.position = moteus::kFloat;
    fmt.velocity = moteus::kFloat;
    fmt.accel_limit = moteus::kFloat;
    fmt.velocity_limit = moteus::kFloat;
    fmt.maximum_torque = moteus::kFloat;
    return bus_->ctrl(id, false).MakePosition(cmd, &fmt);
  }

  moteus::CanFdFrame make_swerve(int id, double position)
  {
    moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.accel_limit = swerves_max_acc_;
    cmd.velocity_limit = swerves_max_vel_;
    cmd.maximum_torque = swerves_max_torque_;
    moteus::PositionMode::Format fmt;
    fmt.position = moteus::kFloat;
    fmt.velocity = moteus::kIgnore;  // velocity not passed in Python
    fmt.accel_limit = moteus::kFloat;
    fmt.velocity_limit = moteus::kFloat;
    fmt.maximum_torque = moteus::kFloat;
    return bus_->ctrl(id, false).MakePosition(cmd, &fmt);
  }

  void set_drive(
    double ws1, double ws2, double ws3, double ws4,
    double wa1, double wa2, double wa3, double wa4)
  {
    // NOTE(port): the ID <-> argument mapping is preserved exactly from Python:
    // drive 2 (BL) is never commanded (commented out upstream, so ws3 is unused),
    // id1<-ws2, id3<-ws4, id4<-ws1, and steer id5<-wa2, id6<-wa3, id7<-wa1, id8<-wa4.
    (void)ws3;
    bus_->cycle([&]() {
        return std::vector<moteus::CanFdFrame>{
          make_drive(1, ws2),
          make_drive(3, ws4),
          make_drive(4, ws1),
          make_swerve(5, wa2),
          make_swerve(6, wa3),
          make_swerve(7, wa1),
          make_swerve(8, wa4),
        };
      });
  }

  // helper to ensure wheels stay between 90 and -90 deg
  static void bound_wheels(double a, double s, double & a_out, double & s_out)
  {
    double a_bounded = a;
    if (std::abs(a) > 90) {
      if (a < 0) {
        a_bounded = 180 + a;
      } else {
        a_bounded = a - 180;
      }
      s *= -1;
    }
    a_out = a_bounded;
    s_out = -1 * s;
  }

  // Calculates wheel angles and speeds for a swerve module.
  // cmd_vel_callback passes L = W = 1.0.
  void wheelAnglesAndSpeeds(
    double Vx, double Vy, double omega, double L_, double W_, double s_out[4], double a_out[4])
  {
    const double R = std::sqrt(L_ * L_ + W_ * W_);

    Vx = py_round(Vx, 2);
    Vy = py_round(Vy, 2);
    omega = py_round(omega, 2);

    const double A = Vy - omega * (L_ / R);
    const double B = Vy + omega * (L_ / R);
    const double C = Vx - omega * (W_ / R);
    const double D = Vx + omega * (W_ / R);

    double ws1 = std::sqrt(B * B + C * C);
    double ws2 = std::sqrt(B * B + D * D);
    double ws3 = std::sqrt(A * A + D * D);
    double ws4 = std::sqrt(A * A + C * C);

    const double wa1 = std::atan2(B, C) * 180 / M_PI;
    const double wa2 = std::atan2(B, D) * 180 / M_PI;
    const double wa3 = std::atan2(A, D) * 180 / M_PI;
    const double wa4 = std::atan2(A, C) * 180 / M_PI;

    // normalize wheel speeds
    double max = ws1;
    if (ws2 > max) {max = ws2;}
    if (ws3 > max) {max = ws3;}
    if (ws4 > max) {max = ws4;}
    if (max > 1) {ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max;}

    double a1, s1, a2, s2, a3, s3, a4, s4;
    bound_wheels(wa1, ws1, a1, s1);
    bound_wheels(wa2, ws2, a2, s2);
    bound_wheels(wa3, ws3, a3, s3);
    bound_wheels(wa4, ws4, a4, s4);
    RCLCPP_INFO(
      get_logger(), "s1: %s a1: %s\ns2: %s a2: %s\ns3: %s a3: %s\n s4: %s a4: %s",
      py_float_repr(s1).c_str(), py_float_repr(a1).c_str(),
      py_float_repr(s2).c_str(), py_float_repr(a2).c_str(),
      py_float_repr(s3).c_str(), py_float_repr(a3).c_str(),
      py_float_repr(s4).c_str(), py_float_repr(a4).c_str());
    s1 *= DRIVE_RATIO / WHEEL_CIRCUMFERENCE;
    s2 *= DRIVE_RATIO / WHEEL_CIRCUMFERENCE;
    s3 *= DRIVE_RATIO / WHEEL_CIRCUMFERENCE;
    s4 *= DRIVE_RATIO / WHEEL_CIRCUMFERENCE;

    a1 *= SWERVE_RATIO / 360.0;
    a2 *= SWERVE_RATIO / 360.0;
    a3 *= SWERVE_RATIO / 360.0;
    a4 *= SWERVE_RATIO / 360.0;

    s_out[0] = -1 * s1;
    s_out[1] = s2;
    s_out[2] = s3;
    s_out[3] = -1 * s4;
    a_out[0] = a1;
    a_out[1] = a2;
    a_out[2] = a3;
    a_out[3] = a4;
  }

  // Drive constants
  const double swerves_max_torque_ = 6;
  const double swerves_max_vel_ = 120;
  const double swerves_max_acc_ = 120;
  const double drives_max_torque_ = 6;
  const double drives_max_vel_ = 300;
  const double drives_max_acc_ = 150;

  // 1, 2, 3, 4 = drives: front left, back left, front right, back right
  // 5, 6, 7, 8 = swerves: front left, back left, front right, back right
  static constexpr int kServoIds[] = {1, 2, 3, 4, 5, 6, 7, 8};
  std::unique_ptr<cmr_controls_cpp::MotorBus> bus_;

  rclcpp::Subscription<cmr_msgs::msg::DriveCommand>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SwerveControllerNode>();
    rclcpp::spin(node);
  } catch (const cmr_controls_cpp::ShutdownRequested &) {
    // Ctrl-C while waiting for the CAN adapter: exit cleanly.
  } catch (const std::exception & e) {
    // e.g. no fdcanusb present. Python died with a traceback here; log and
    // exit non-zero instead of std::terminate/abort.
    RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
