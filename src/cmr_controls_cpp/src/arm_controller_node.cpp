// C++ port of cmr_controls/arm_controller_node.py (console script `arm_controller_node`).
//
// Converts desired joint angles (degrees) into moteus position commands for the
// six arm joints on an fdcanusb.
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <tuple>
#include <thread>
#include <vector>

#include <mjbots/moteus/moteus.h>

#include "cmr_controls_cpp/moteus_safe_bus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace moteus = mjbots::moteus;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    // joint index -> (motor id, gear ratio, direction, offset)
    joint_motor_info_ = {
      {0, {0x09, 100, 1, 0.0}},
      {1, {0x0B, 100, -1, 0.0}},
      {2, {0x0A, 100, 1, 0.0}},
      {3, {0x0C, 50, 1, 0.0}},
      {4, {0x0D, 50, 1, 0.0}},
      {5, {0x0E, 50, 1, 0.0}},
    };

    // Subscription to the desired joint angles topic
    subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_angles/desired", 10,
      std::bind(&ArmControllerNode::arm_control_callback, this, std::placeholders::_1));
    calibration_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_angles/offsets", 10,
      std::bind(&ArmControllerNode::arm_offset_callback, this, std::placeholders::_1));

    initialize_moteus();

    // Initialize current joint angles to zero
    curr_positions_.fill(0.0);
    set_joints(curr_positions_);

    RCLCPP_INFO(get_logger(), "Arm Controller Node has been started.");
  }

private:
  struct MotorInfo
  {
    int id;
    double ratio;
    double swap;
    double offset;
  };

  double angle_to_motor_position(int joint_num, double angle) const
  {
    const auto & info = joint_motor_info_.at(joint_num);
    return (angle + info.offset) * (info.ratio / 360) * info.swap;
  }

  double motor_position_to_angle(int joint_num, double position) const
  {
    const auto & info = joint_motor_info_.at(joint_num);
    return position * (360 / info.ratio) * info.swap;
  }

  void arm_offset_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // NOTE(port): Python raises (IndexError / KeyError) and the node dies on a short
    // message or an out-of-range joint index; those messages are rejected here.
    if (msg->data.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Offset message needs [joint_index, offset]; ignoring");
      return;
    }
    const int i = static_cast<int>(msg->data[0]);  // int() truncates toward zero
    const double offset = msg->data[1];
    auto it = joint_motor_info_.find(i);
    if (it == joint_motor_info_.end()) {
      RCLCPP_ERROR(get_logger(), "Invalid joint index %d in offset message; ignoring", i);
      return;
    }
    it->second.offset = offset;
    std::array<double, 6> positions{};
    for (int j = 0; j < 6; ++j) {
      positions[j] = angle_to_motor_position(j, 0.0);
    }
    curr_positions_ = positions;
    set_joints(positions);
  }

  void arm_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      // NOTE(port): Python raises IndexError here, killing the node.
      RCLCPP_ERROR(
        get_logger(), "Expected 6 joint angles, got %zu; ignoring", msg->data.size());
      return;
    }
    std::array<double, 6> positions{};
    for (int i = 0; i < 6; ++i) {
      positions[i] = angle_to_motor_position(i, msg->data[i]);
    }
    curr_positions_ = positions;
    set_joints(positions);
  }

  void initialize_moteus()
  {
    // NOTE(port): moteus.Fdcanusb() in Python took the first fdcanusb it found.
    // Here can_port ("auto" by default) selects only an adapter that has been
    // locked and proven (by query) to carry this node's motors 0x09-0x0E, so
    // stop/rezero and position frames can never land on the drive bus. If that
    // adapter is held by another node we log and wait rather than fall back.
    declare_parameter<std::string>("can_port", "auto");
    bus_ = std::make_unique<cmr_controls_cpp::MotorBus>(
      get_logger(), get_parameter("can_port").as_string(), servo_order_);
    bus_->open_blocking();

    // reset servo positions (dict order 0x09, 0x0B, 0x0A, 0x0C, 0x0E, 0x0D);
    // make_stop() / make_rezero() default to query=False. Errors propagate (Python).
    std::vector<moteus::CanFdFrame> frames;
    for (int id : servo_order_) {frames.push_back(bus_->ctrl(id, false).MakeStop());}
    bus_->cycle_or_throw(frames);
    frames.clear();
    for (int id : servo_order_) {
      moteus::OutputNearest::Command cmd;  // make_rezero(rezero=0.0)
      cmd.position = 0.0;
      frames.push_back(bus_->ctrl(id, false).MakeOutputNearest(cmd));
    }
    bus_->cycle_or_throw(frames);
  }

  moteus::CanFdFrame make_joint(int id, double position)
  {
    moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.accel_limit = max_acc_;
    cmd.velocity_limit = max_vel_;
    cmd.maximum_torque = max_torque_;
    moteus::PositionMode::Format fmt;
    fmt.position = moteus::kFloat;
    fmt.velocity = moteus::kIgnore;  // velocity not passed in Python
    fmt.accel_limit = moteus::kFloat;
    fmt.velocity_limit = moteus::kFloat;
    fmt.maximum_torque = moteus::kFloat;
    return bus_->ctrl(id, true).MakePosition(cmd, &fmt);  // query=True
  }

  void set_joints(const std::array<double, 6> & p)
  {
    motors_free_ = false;
    max_vel_ = 6.0;
    max_acc_ = 5.0;
    max_torque_ = 0.7;

    std::vector<moteus::CanFdFrame> replies;
    const bool sent = bus_->cycle(
      [&]() {
        return std::vector<moteus::CanFdFrame>{
          make_joint(0x09, p[0]),
          make_joint(0x0B, p[1]),
          make_joint(0x0A, p[2]),
          make_joint(0x0C, p[3]),
          make_joint(0x0D, p[4]),
          make_joint(0x0E, p[5]),
        };
      }, &replies);
    if (!sent) {
      return;  // already logged by MotorBus
    }
    // NOTE(port): Python then indexes result[0..5] and discards the positions; with
    // fewer than 6 replies (any joint offline) that raises IndexError and kills the
    // node, including at startup. The replies are unused, so they are just ignored
    // here, with a warning when some are missing.
    if (replies.size() < 6) {
      RCLCPP_WARN(
        get_logger(), "Only %zu of 6 arm motors replied to the position command",
        replies.size());
    }
  }

  // Constants
  double max_torque_ = 0.7;
  double max_vel_ = 6.0;
  double max_acc_ = 5.0;
  bool motors_free_ = true;

  std::map<int, MotorInfo> joint_motor_info_;
  std::array<double, 6> curr_positions_{};

  const std::vector<int> servo_order_ = {0x09, 0x0B, 0x0A, 0x0C, 0x0E, 0x0D};
  std::unique_ptr<cmr_controls_cpp::MotorBus> bus_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr calibration_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ArmControllerNode>();
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
