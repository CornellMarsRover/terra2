// C++ port of cmr_servo_control/arm_servo_control_node.py
// (console script `arm_servo_control_node`).
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "cmr_msgs/msg/controller_reading.hpp"
#include "cmr_servo_control_cpp/end_effector_servo_mapper.hpp"
#include "cmr_servo_control_cpp/fd_can_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using cmr_servo_control_cpp::EndEffectorServoMapper;
using cmr_servo_control_cpp::FdCanInterface;
using cmr_servo_control_cpp::ServoController;

class ArmServoControlNode : public rclcpp::Node
{
public:
  ArmServoControlNode()
  : Node("arm_servo_control_node")
  {
    declare_parameter<std::string>("can_port", "/dev/ttyACM0");
    declare_parameter<int>("baud", 115200);
    declare_parameter<int>("servo_can_id", 25);
    declare_parameter<std::string>("command_topic", "/arm_controller/cmd_buttons");

    can_port_ = get_parameter("can_port").as_string();
    baud_ = static_cast<int>(get_parameter("baud").as_int());
    servo_can_id_ = static_cast<int>(get_parameter("servo_can_id").as_int());
    command_topic_ = get_parameter("command_topic").as_string();

    fd_ = std::make_unique<FdCanInterface>(can_port_, baud_);

    RCLCPP_INFO(
      get_logger(), "Opening servo CAN interface on %s at %d baud", can_port_.c_str(), baud_);
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

    // NOTE(port): the Python node never reads the fdcanusb after startup, so the
    // "OK" reply to every `can std` command piles up in the asyncio StreamReader
    // (and eventually the tty buffer). Drain and discard it here instead.
    running_ = true;
    drain_thread_ = std::thread([this]() {
          try {
            fd_->drain_loop(running_);
          } catch (const std::exception & exc) {
            RCLCPP_WARN(get_logger(), "Servo CAN input drain stopped: %s", exc.what());
          }
        });

    subscription_ = create_subscription<cmr_msgs::msg::ControllerReading>(
      command_topic_, 10,
      std::bind(&ArmServoControlNode::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Arm servo control ready: topic=%s, default_servo_can_id=%d",
      command_topic_.c_str(), servo_can_id_);
  }

  ~ArmServoControlNode() override
  {
    if (fd_) {
      RCLCPP_INFO(get_logger(), "Shutting down servo CAN interface");
    }
    running_ = false;
    if (drain_thread_.joinable()) {
      drain_thread_.join();
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
    for (const auto & [name, servo_id] : cmr_servo_control_cpp::servo_ids()) {
      const auto it = cmr_servo_control_cpp::servo_can_ids().find(name);
      const int can_id =
        it != cmr_servo_control_cpp::servo_can_ids().end() ? it->second : servo_can_id_;
      servos_.emplace(name, ServoController(fd_.get(), servo_id, 0, can_id));
      RCLCPP_INFO(
        get_logger(), "Configured %s: board_can_id=%d, servo_id=%d", name.c_str(), can_id,
        servo_id);
    }
  }

  void command_callback(const cmr_msgs::msg::ControllerReading::SharedPtr msg)
  {
    const auto commands = mapper_.update(msg->button_array, msg->dpad);
    for (const auto & [name, angle] : commands) {
      auto it = servos_.find(name);
      if (it == servos_.end()) {
        RCLCPP_WARN(get_logger(), "No configured servo for %s", name.c_str());
        continue;
      }
      ServoController & servo = it->second;
      RCLCPP_INFO(
        get_logger(), "Commanding %s on board %d, servo %d -> %d deg", name.c_str(),
        servo.can_id(), servo.servo_id(), angle);
      try {
        servo.go_to_position(angle);
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(get_logger(), "Servo command failed for %s: %s", name.c_str(), exc.what());
      }
    }
  }

  std::string can_port_;
  int baud_{115200};
  int servo_can_id_{25};
  std::string command_topic_;

  EndEffectorServoMapper mapper_;
  std::unique_ptr<FdCanInterface> fd_;
  std::map<std::string, ServoController> servos_;
  std::atomic<bool> running_{false};
  std::thread drain_thread_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ArmServoControlNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("arm_servo_control_node"), "%s", e.what());
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
