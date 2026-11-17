// C++ port of cmr_rovernet/ccb_read.py (executable `cmr_read_node`).
//
// Polls the CCB over UART (/dev/ttyTHS1 @ 115200) for 14 motors: for each one
// it writes a 40-byte RECEIVE_DATA request and blocks for a 20-byte reply
// ('<BBBffffB'), updates the matching MotorData field of a cached
// MotorReadData and publishes it on /ccb/read.
//
// Differences from the Python (perf / robustness only):
//  * The serial port is opened once and kept; after an I/O error it is closed
//    and re-opened on the next cycle (Python crashed if the port was missing).
//  * The 1 kHz timer is non-reentrant and never queues: a cycle that overruns
//    (each read may block up to 1 s) simply causes the missed ticks to be
//    skipped; the next cycle starts on the next timer tick.
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cmr_msgs/msg/motor_read_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cmr_rovernet_cpp/rovernet_utils.hpp"
#include "cmr_rovernet_cpp/serial_port.hpp"

namespace cmr_rovernet_cpp {

class CCBReadPublisher : public rclcpp::Node
{
public:
  CCBReadPublisher()
  : rclcpp::Node("cmd_vel_publisher")
  {
    encoder_data_publisher_ =
      create_publisher<cmr_msgs::msg::MotorReadData>("/ccb/read", 10);

    motor_data_mapping_ = {
      {FRONT_LEFT, &stored_data_.front_left},
      {BACK_RIGHT, &stored_data_.back_right},
      {FRONT_RIGHT, &stored_data_.front_right},
      {BACK_LEFT, &stored_data_.back_left},
      {ARM_BASE, &stored_data_.base},
      {ARM_SHOULDER, &stored_data_.shoulder},
      {ARM_ELBOW, &stored_data_.elbow},
      {WRIST_ROTATE_1, &stored_data_.wrist_r1},
      // NOTE(port): as in the Python, WRIST_TILT (0x0D) replies land in the
      // `wrist_twist` field and WRIST_ROTATE_2 (0x0E) in `wrist_r2`.
      {WRIST_TILT, &stored_data_.wrist_twist},
      {WRIST_ROTATE_2, &stored_data_.wrist_r2},
      {FRONT_LEFT_SWERVE, &stored_data_.front_left_swerve},
      {BACK_RIGHT_SWERVE, &stored_data_.back_right_swerve},
      {FRONT_RIGHT_SWERVE, &stored_data_.front_right_swerve},
      {BACK_LEFT_SWERVE, &stored_data_.back_left_swerve},
    };

    // Request order identical to CCBReadPublisher.publish_msg.
    const uint8_t order[] = {
      BACK_RIGHT, FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, ARM_BASE, ARM_SHOULDER, ARM_ELBOW,
      WRIST_ROTATE_1, WRIST_ROTATE_2, WRIST_TILT, BACK_RIGHT_SWERVE, FRONT_RIGHT_SWERVE,
      FRONT_LEFT_SWERVE, BACK_LEFT_SWERVE};
    for (uint8_t motor : order) {
      // Python passes 10 for every numeric field.
      requests_.push_back(
        byte_command_converter(RECEIVE_DATA, motor, 10, 10, 10, 10, 10, 10));
    }

    open_port();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1), [this]() {publish_msg();});
  }

private:
  void open_port()
  {
    try {
      serial_port_.open(port_, baud_rate_);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Could not open %s: %s", port_.c_str(), exc.what());
    }
  }

  void publish_msg()
  {
    // Non-reentrant guard (the single-threaded executor already serialises
    // this callback; the flag also protects against multi-threaded executors).
    if (busy_.exchange(true)) {
      return;
    }
    struct Release
    {
      std::atomic<bool> & f;
      ~Release() {f.store(false);}
    } release{busy_};

    if (!serial_port_.is_open()) {
      open_port();
      if (!serial_port_.is_open()) {
        return;
      }
    }

    try {
      for (const auto & request : requests_) {
        serial_port_.write_all(request.data(), request.size());
        read_data();
      }
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Serial I/O error on %s: %s; reopening",
        port_.c_str(), exc.what());
      serial_port_.close();
    }
  }

  void read_data()
  {
    uint8_t data[kReplySize];
    const std::size_t n = serial_port_.read(data, kReplySize, std::chrono::seconds(1));
    process_data(data, n);
  }

  void process_data(const uint8_t * data, std::size_t size)
  {
    const auto reply = parse_ccb_reply(data, size);
    if (!reply) {
      RCLCPP_ERROR(
        get_logger(), "Error unpacking data: unpack requires a buffer of %zu bytes",
        kReplySize);
      return;
    }
    const auto it = motor_data_mapping_.find(reply->motor_id);
    if (it != motor_data_mapping_.end()) {
      auto & motor_data = *it->second;
      motor_data.mode = reply->mode;
      motor_data.position = reply->position;
      motor_data.velocity = reply->velocity;
      motor_data.torque = reply->torque;
      motor_data.current = reply->current;
    }
    // Published after every successfully decoded reply (14x per cycle).
    encoder_data_publisher_->publish(stored_data_);
  }

  const std::string port_ = "/dev/ttyTHS1";
  const int baud_rate_ = 115200;
  SerialPort serial_port_;
  std::atomic<bool> busy_{false};
  cmr_msgs::msg::MotorReadData stored_data_;
  std::map<uint8_t, cmr_msgs::msg::MotorData *> motor_data_mapping_;
  std::vector<CommandBytes> requests_;
  rclcpp::Publisher<cmr_msgs::msg::MotorReadData>::SharedPtr encoder_data_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmr_rovernet_cpp::CCBReadPublisher>());
  rclcpp::shutdown();
  return 0;
}
