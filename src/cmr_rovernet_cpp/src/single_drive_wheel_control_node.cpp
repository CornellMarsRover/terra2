// C++ port of cmr_rovernet/single_drive_wheel_control.py
// (executable `single_drive_wheel_control_node`,
// launch/single_drive_wheel.launch.py).
//
// Spins exactly one drive wheel forward while its face button is held:
// Triangle=front right(3), Circle=back right(4), X=back left(2),
// Square=front left(1).  One persistent fdcanusb transport.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmr_msgs/msg/controller_reading.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cmr_rovernet_cpp/drive_common.hpp"
#include "cmr_rovernet_cpp/moteus_bus.hpp"
#include "cmr_rovernet_cpp/node_config.hpp"
#include "cmr_rovernet_cpp/time_util.hpp"

namespace cmr_rovernet_cpp {
namespace {

struct FaceButton
{
  const char * name;
  std::size_t expanded_index;  // EXPANDED_BUTTONS
  int64_t legacy_value;        // LEGACY_BUTTON_VALUES
  int motor_id;                // BUTTON_TO_MOTOR
};

// Iteration order of the Python EXPANDED_BUTTONS / LEGACY_BUTTON_VALUES dicts.
constexpr FaceButton kButtons[] = {
  {"square", 4, 1, 1},          // front left drive
  {"x", 5, 256, 2},             // back left drive
  {"circle", 6, 65536, 4},      // back right drive
  {"triangle", 7, 16777216, 3},  // front right drive
};

const char * motor_label(int motor_id)
{
  switch (motor_id) {
    case 1: return "front left";
    case 2: return "back left";
    case 3: return "front right";
    case 4: return "back right";
    default: return "unknown";
  }
}

}  // namespace

class SingleDriveWheelControlNode : public rclcpp::Node
{
public:
  SingleDriveWheelControlNode()
  : rclcpp::Node("single_drive_wheel_control_node")
  {
    declare_parameter("config_path", "");
    ConfigLoadOptions load_opts;
    load_opts.log_loaded = false;
    const TomlTable config = load_node_config(*this, load_opts);

    declare_lenient(*this, "can_port", std::string("/dev/ttyACM0"));
    declare_lenient(*this, "timeout_s", 0.5);
    declare_lenient(*this, "drive_rps", 1.0);
    declare_lenient(*this, "drive_max_torque", 0.5);
    declare_lenient(*this, "drive_acceleration_limit", 5.0);
    declare_lenient(*this, "watchdog_timeout_s", 0.5);
    declare_lenient(*this, "command_timeout_s", 0.5);
    declare_lenient(*this, "refresh_rate_hz", 10.0);

    port_ = setting_string(*this, "can_port", config);
    timeout_s_ = setting_double(*this, "timeout_s", config);
    drive_rps_ = std::abs(setting_double(*this, "drive_rps", config));
    drive_max_torque_ = std::abs(setting_double(*this, "drive_max_torque", config));
    drive_acceleration_limit_ =
      std::abs(setting_double(*this, "drive_acceleration_limit", config));
    watchdog_timeout_s_ = std::max(0.25, setting_double(*this, "watchdog_timeout_s", config));
    command_timeout_s_ = setting_double(*this, "command_timeout_s", config);
    refresh_rate_hz_ = setting_double(*this, "refresh_rate_hz", config);

    BusOptions bus_opts;
    bus_opts.port_arg = port_;
    bus_opts.ids = drive::drive_ids();
    bus_opts.reply_timeout_s = timeout_s_;
    bus_opts.query_format = drive_gui_query_format();
    bus_ = std::make_unique<MoteusBus>(get_logger(), bus_opts);

    buttons_sub_ = create_subscription<cmr_msgs::msg::ControllerReading>(
      "/controller/drives/buttons", 10,
      [this](cmr_msgs::msg::ControllerReading::ConstSharedPtr msg) {
        controller_buttons_cb(*msg);
      });

    stop_drive_motors();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(refresh_rate_hz_, 1.0)),
      [this]() {timer_cb();});

    RCLCPP_INFO(
      get_logger(),
      "single_drive_wheel_control_node started on %s: drive_rps=%.2f, torque=%.2f. "
      "Triangle=front right, Circle=back right, X=back left, Square=front left.",
      port_.c_str(), drive_rps_, drive_max_torque_);
  }

  /// Python destroy_node().
  void shutdown()
  {
    RCLCPP_INFO(get_logger(), "Shutting down single_drive_wheel_control_node");
    try {
      bus_->stop_or_throw(drive::drive_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Single-wheel shutdown stop failed: %s", exc.what());
    }
  }

private:
  // NOTE(port): Python let CAN-port errors in the stop path kill the node.
  void stop_drive_motors()
  {
    try {
      bus_->stop_or_throw(drive::drive_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Single-wheel stop failed: %s", exc.what());
    }
  }

  static std::vector<const FaceButton *> pressed_face_buttons(const std::vector<int32_t> & buttons)
  {
    std::vector<const FaceButton *> pressed;
    if (buttons.size() >= 8) {
      for (const auto & b : kButtons) {
        if (buttons[b.expanded_index] != 0) {
          pressed.push_back(&b);
        }
      }
      return pressed;
    }
    if (buttons.size() >= 2) {
      const int64_t button_val = buttons[1];
      for (const auto & b : kButtons) {
        if (button_val == b.legacy_value) {
          pressed.push_back(&b);
        }
      }
    }
    return pressed;
  }

  void controller_buttons_cb(const cmr_msgs::msg::ControllerReading & msg)
  {
    const auto pressed = pressed_face_buttons(msg.button_array);
    last_cmd_at_ = mono_now();

    if (pressed.size() == 1) {
      selected_button_ = pressed[0];
      const int motor_id = selected_button_->motor_id;
      if (!last_motor_id_ || motor_id != *last_motor_id_) {
        RCLCPP_INFO(
          get_logger(), "%s pressed -> motor %d (%s) forward", selected_button_->name, motor_id,
          motor_label(motor_id));
      }
      last_motor_id_ = motor_id;
    } else {
      if (pressed.size() > 1) {
        std::ostringstream o;
        o << "[";
        for (std::size_t i = 0; i < pressed.size(); ++i) {
          o << (i ? ", " : "") << "'" << pressed[i]->name << "'";
        }
        o << "]";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Multiple wheel buttons pressed %s; stopping drive motors", o.str().c_str());
      }
      selected_button_ = nullptr;
      last_motor_id_.reset();
    }
  }

  void timer_cb()
  {
    if (selected_button_ == nullptr || mono_now() - last_cmd_at_ > command_timeout_s_) {
      stop_drive_motors();
      return;
    }

    const int motor_id = selected_button_->motor_id;
    try {
      bus_->ensure_open();
      const double signed_velocity = drive::forward_sign(motor_id) * drive_rps_;
      bus_->send(
        {make_drive_velocity_frame(
            bus_->controller(motor_id), signed_velocity, drive_max_torque_,
            drive_acceleration_limit_, std::max(0.25, watchdog_timeout_s_))});
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "motor %d (%s) velocity=%+.3f", motor_id,
        motor_label(motor_id), signed_velocity);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Single-wheel command failed: %s", exc.what());
    }
  }

  std::string port_;
  double timeout_s_ = 0.5;
  double drive_rps_ = 1.0;
  double drive_max_torque_ = 0.5;
  double drive_acceleration_limit_ = 5.0;
  double watchdog_timeout_s_ = 0.5;
  double command_timeout_s_ = 0.5;
  double refresh_rate_hz_ = 10.0;

  const FaceButton * selected_button_ = nullptr;
  double last_cmd_at_ = kNever;
  std::optional<int> last_motor_id_;

  std::unique_ptr<MoteusBus> bus_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr buttons_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cmr_rovernet_cpp::SingleDriveWheelControlNode>();
  int rc = 0;
  // Python `finally:` semantics: the final motor stop runs even if spin throws.
  try {
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "spin terminated by exception: %s", exc.what());
    rc = 1;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "spin terminated by unknown exception");
    rc = 1;
  }
  node->shutdown();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
