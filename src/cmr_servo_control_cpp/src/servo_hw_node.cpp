// C++ port of cmr_servo_control/servo_hw_node.py (console script `servo_hw_node`).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cmr_servo_control_cpp/fd_can_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using cmr_servo_control_cpp::FdCanInterface;
using cmr_servo_control_cpp::ServoController;

namespace
{

const std::vector<std::string> SERVO_NAMES = {"ee13", "ee14", "ee8"};
// Logical names on the ROS side mapped to the physical servo IDs found by probing.
const std::map<std::string, int> SERVO_IDS = {{"ee13", 15}, {"ee14", 14}, {"ee8", 12}};

std::string py_list_repr(const std::vector<std::string> & items)
{
  std::string out = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i) {out += ", ";}
    out += "'" + items[i] + "'";
  }
  return out + "]";
}

std::string py_float_repr(double v)
{
  if (std::isnan(v)) {return "nan";}
  if (std::isinf(v)) {return v > 0 ? "inf" : "-inf";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

}  // namespace

class ServoHwNode : public rclcpp::Node
{
public:
  ServoHwNode()
  : Node("servo_hw_node")
  {
    declare_parameter<std::string>("can_port", "/dev/ttyACM0");
    declare_parameter<int>("baud", 115200);
    declare_parameter<int>("servo_can_id", 24);

    can_port_ = get_parameter("can_port").as_string();
    baud_ = static_cast<int>(get_parameter("baud").as_int());
    servo_can_id_ = static_cast<int>(get_parameter("servo_can_id").as_int());

    RCLCPP_INFO(get_logger(), "Opening CAN interface on %s at %d baud", can_port_.c_str(), baud_);

    fd_ = std::make_unique<FdCanInterface>(can_port_, baud_);
    fd_->rx_callback = [this](int can_id, const std::string & data_hex,
        const std::vector<std::string> & flags) {
        handle_rx_frame(can_id, data_hex, flags);
      };

    // NOTE(port): exclusive adapter lock first (before any byte is sent). A busy
    // adapter is waited on indefinitely (throttled warning, 1 s retry): the Python
    // node had no lock and would simply have shared the adapter and kept running.
    // Any other open failure propagates and exits, as the Python startup did.
    while (true) {
      try {
        fd_->open_port();
        break;
      } catch (const cmr_port::PortBusyError & e) {
        if (!rclcpp::ok()) {throw std::runtime_error("shutdown requested while waiting for CAN adapter");}
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "%s; retrying", e.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

    // startup_future.result(timeout=10.0)
    startup(FdCanInterface::Clock::now() + std::chrono::seconds(10));

    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "end_effector_joint_command", 10,
      std::bind(&ServoHwNode::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Servo hardware node ready and subscribed to end_effector_joint_command");
  }

  ~ServoHwNode() override
  {
    if (fd_) {
      RCLCPP_INFO(get_logger(), "Shutting down CAN interface");
    }
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (fd_) {
      try {
        fd_->close();
      } catch (const std::exception & exc) {
        RCLCPP_WARN(get_logger(), "Exception while closing CAN interface: %s", exc.what());
      }
    }
  }

private:
  void startup(FdCanInterface::Clock::time_point deadline)
  {
    fd_->open(deadline);
    fd_->configure_bus(deadline);
    running_ = true;
    read_thread_ = std::thread([this]() {
          try {
            fd_->read_loop(running_);
          } catch (const std::exception & exc) {
            // NOTE(port): Python's read_loop spins forever on EOF; here the
            // receive thread stops once the port reports an error.
            RCLCPP_ERROR(get_logger(), "CAN read loop stopped: %s", exc.what());
          }
        });
    for (const auto & name : SERVO_NAMES) {
      servos_.emplace_back(name, ServoController(fd_.get(), SERVO_IDS.at(name), 0, servo_can_id_));
    }
  }

  void handle_rx_frame(int can_id, const std::string & data_hex,
    const std::vector<std::string> & flags)
  {
    RCLCPP_INFO(
      get_logger(), "RX frame ID=0x%X data=%s flags=%s", static_cast<unsigned>(can_id),
      data_hex.c_str(), py_list_repr(flags).c_str());
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // {name: pos for name, pos in zip(msg.name, msg.position)} (last duplicate wins)
    std::map<std::string, double> positions;
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      positions[msg->name[i]] = msg->position[i];
    }
    if (positions.empty()) {
      return;
    }
    for (auto & [name, servo] : servos_) {
      auto it = positions.find(name);
      if (it == positions.end()) {
        continue;
      }
      const double pos = it->second;
      // NOTE(port): Python int(nan) raises ValueError (caught -> warn) but int(inf)
      // raises OverflowError, which is not caught and kills the node. Both warn here.
      if (!std::isfinite(pos)) {
        RCLCPP_WARN(
          get_logger(), "Invalid position for %s: %s", name.c_str(), py_float_repr(pos).c_str());
        continue;
      }
      const double truncated = std::trunc(pos);  // int() truncates toward zero
      // Only the low 12 bits reach the frame; fmod keeps that exact for any magnitude.
      const int64_t angle_bits = static_cast<int64_t>(std::fmod(truncated, 4096.0));
      RCLCPP_INFO(get_logger(), "Publishing %.0f° to %s", truncated, name.c_str());
      try {
        servo.go_to_position(angle_bits);
      } catch (const std::exception & exc) {
        // Python never retrieves this future's exception (asyncio logs it on GC).
        std::cerr << "Servo command for " << name << " failed: " << exc.what() << std::endl;
      }
    }
  }

  std::string can_port_;
  int baud_{115200};
  int servo_can_id_{24};

  std::unique_ptr<FdCanInterface> fd_;
  // Ordered like the Python dict (SERVO_NAMES order).
  std::vector<std::pair<std::string, ServoController>> servos_;
  std::atomic<bool> running_{false};
  std::thread read_thread_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ServoHwNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("servo_hw_node"), "%s", e.what());
      rclcpp::shutdown();
      return 1;
    }
    // Ctrl-C during startup: exit quietly.
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
