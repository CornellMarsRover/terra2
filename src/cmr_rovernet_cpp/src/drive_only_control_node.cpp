// C++ port of cmr_rovernet/drive_only_control.py
// (executable `drive_only_control_node`, launch/drive_only.launch.py).
//
// Drives the 4 drive motors (velocity mode) from the controller; steer
// motors are never commanded.  One persistent fdcanusb transport instead of
// one per timer tick.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cmr_msgs/msg/controller_reading.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cmr_rovernet_cpp/drive_common.hpp"
#include "cmr_rovernet_cpp/moteus_bus.hpp"
#include "cmr_rovernet_cpp/node_config.hpp"
#include "cmr_rovernet_cpp/time_util.hpp"

namespace cmr_rovernet_cpp {

using namespace drive;  // NOLINT

class DriveOnlyControlNode : public rclcpp::Node
{
public:
  DriveOnlyControlNode()
  : rclcpp::Node("drive_only_control_node")
  {
    declare_parameter("config_path", "");
    const TomlTable config = load_node_config(*this);

    declare_lenient(*this, "can_port", std::string("/dev/ttyACM0"));
    declare_lenient(*this, "timeout_s", 0.30);
    declare_lenient(*this, "drive_rps", 2.0);
    declare_lenient(*this, "half_speed_multiplier", HALF_SPEED_MULTIPLIER);
    declare_lenient(*this, "double_speed_multiplier", DOUBLE_SPEED_MULTIPLIER);
    declare_lenient(*this, "triple_speed_multiplier", TRIPLE_SPEED_MULTIPLIER);
    declare_lenient(*this, "trigger_pressed_threshold", TRIGGER_PRESSED_THRESHOLD);
    declare_lenient(*this, "drive_max_torque", 0.5);
    declare_lenient(*this, "drive_acceleration_limit", 5.0);
    declare_lenient(*this, "watchdog_timeout_s", 0.5);
    declare_lenient(*this, "controller_deadzone", 0.1);
    declare_lenient(*this, "command_timeout_s", 0.5);
    declare_lenient(*this, "refresh_rate_hz", 10.0);

    port_ = setting_string(*this, "can_port", config);
    timeout_s_ = setting_double(*this, "timeout_s", config);
    drive_rps_ = std::abs(setting_double(*this, "drive_rps", config));
    half_speed_multiplier_ = std::abs(setting_double(*this, "half_speed_multiplier", config));
    double_speed_multiplier_ = std::abs(setting_double(*this, "double_speed_multiplier", config));
    triple_speed_multiplier_ = std::abs(setting_double(*this, "triple_speed_multiplier", config));
    trigger_pressed_threshold_ = setting_int(*this, "trigger_pressed_threshold", config);
    drive_max_torque_ = std::abs(setting_double(*this, "drive_max_torque", config));
    drive_acceleration_limit_ =
      std::abs(setting_double(*this, "drive_acceleration_limit", config));
    watchdog_timeout_s_ = std::max(0.25, setting_double(*this, "watchdog_timeout_s", config));
    controller_deadzone_ = setting_double(*this, "controller_deadzone", config);
    command_timeout_s_ = setting_double(*this, "command_timeout_s", config);
    refresh_rate_hz_ = setting_double(*this, "refresh_rate_hz", config);

    BusOptions bus_opts;
    bus_opts.port_arg = port_;
    bus_opts.ids = drive_ids();
    bus_opts.reply_timeout_s = timeout_s_;
    bus_opts.query_format = drive_gui_query_format();
    bus_ = std::make_unique<MoteusBus>(get_logger(), bus_opts);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/controller/drives/axes", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        controller_cmd_vel_cb(*msg);
      });
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
      "drive_only_control_node started on %s: drive_rps=%.2f, drive_torque=%.2f, "
      "drive_accel=%.2f. Steer motors are not commanded by this node.",
      port_.c_str(), drive_rps_, drive_max_torque_, drive_acceleration_limit_);
  }

  /// Python destroy_node().
  void shutdown()
  {
    RCLCPP_INFO(get_logger(), "Shutting down drive_only_control_node");
    try {
      bus_->stop_or_throw(drive_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Drive-only shutdown stop failed: %s", exc.what());
    }
  }

private:
  // NOTE(port): in the Python, errors resolving/opening the CAN port inside
  // the stop path were not caught and killed the node (timer callback /
  // constructor exception).  Here they are logged and retried next tick.
  void stop_drive_motors()
  {
    try {
      bus_->stop_or_throw(drive_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Drive-only stop failed: %s", exc.what());
    }
  }

  void controller_cmd_vel_cb(const geometry_msgs::msg::TwistStamped & msg)
  {
    // NOTE(port): a Twist with a non-finite field is treated as not received
    // (Python clamped NaN to +1.0, i.e. full command).
    if (!twist_finite(msg.twist)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring non-finite /controller/drives/axes");
      return;
    }
    drive_axis_ = clamp(apply_deadzone(-msg.twist.linear.y / STICK_RANGE, controller_deadzone_));
    last_cmd_at_ = mono_now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "drive-only axis=%+.3f speed_rps=%+.3f", drive_axis_,
      speed_rps_);
  }

  void controller_buttons_cb(const cmr_msgs::msg::ControllerReading & msg)
  {
    const auto decoded = decode_controller_buttons(msg.button_array);
    if (!decoded) {
      return;
    }
    const bool estop_pressed = decoded->l1 && decoded->triangle;
    if (estop_pressed && !estop_latched_) {
      estop_latched_ = true;
      speed_rps_ = 0.0;
      drive_axis_ = 0.0;
      RCLCPP_WARN(get_logger(), "Drive-only estop latched from controller buttons");
      return;
    }
    if (estop_latched_ && !decoded->l1 && !decoded->triangle) {
      estop_latched_ = false;
      RCLCPP_INFO(get_logger(), "Drive-only estop released");
    }
    if (estop_latched_) {
      speed_rps_ = 0.0;
      last_cmd_at_ = mono_now();
      return;
    }
    speed_rps_ = manual_speed_rps(decoded->l1, decoded->r1, decoded->l2, decoded->r2);
    last_cmd_at_ = mono_now();
  }

  void timer_cb()
  {
    if (estop_latched_ || mono_now() - last_cmd_at_ > command_timeout_s_) {
      stop_drive_motors();
      return;
    }
    if (std::abs(drive_axis_) <= controller_deadzone_ ||
      std::abs(speed_rps_) <= COMMAND_EPSILON)
    {
      stop_drive_motors();
      return;
    }

    const double drive_command_rps = speed_rps_ * (drive_axis_ >= 0 ? 1.0 : -1.0);
    try {
      bus_->ensure_open();
      MotorTargets commands;
      std::vector<mm::CanFdFrame> frames;
      for (int motor_id : drive_ids()) {
        const double signed_velocity = forward_sign(motor_id) * drive_command_rps;
        commands.emplace_back(motor_id, signed_velocity);
        frames.push_back(
          make_drive_velocity_frame(
            bus_->controller(motor_id), signed_velocity, drive_max_torque_,
            drive_acceleration_limit_, std::max(0.25, watchdog_timeout_s_)));
      }
      bus_->send(frames);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "drive-only commands=%s",
        format_targets(commands).c_str());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Drive-only command failed: %s", exc.what());
    }
  }

  double manual_speed_rps(int64_t l1, int64_t r1, int64_t l2, int64_t r2) const
  {
    if (l1) {
      return drive_rps_ * triple_speed_multiplier_;
    }
    if (r1) {
      return -drive_rps_;
    }
    if (trigger_pressed(r2)) {
      return drive_rps_;
    }
    if (trigger_pressed(l2)) {
      return drive_rps_ * half_speed_multiplier_;
    }
    return 0.0;
  }

  bool trigger_pressed(int64_t trigger_byte) const
  {
    return trigger_byte >= std::max<int64_t>(TRIGGER_DEADBAND, trigger_pressed_threshold_);
  }

  std::string port_;
  double timeout_s_ = 0.3;
  double drive_rps_ = 2.0;
  double half_speed_multiplier_ = HALF_SPEED_MULTIPLIER;
  double double_speed_multiplier_ = DOUBLE_SPEED_MULTIPLIER;
  double triple_speed_multiplier_ = TRIPLE_SPEED_MULTIPLIER;
  int64_t trigger_pressed_threshold_ = TRIGGER_PRESSED_THRESHOLD;
  double drive_max_torque_ = 0.5;
  double drive_acceleration_limit_ = 5.0;
  double watchdog_timeout_s_ = 0.5;
  double controller_deadzone_ = 0.1;
  double command_timeout_s_ = 0.5;
  double refresh_rate_hz_ = 10.0;

  double drive_axis_ = 0.0;
  double speed_rps_ = 0.0;
  double last_cmd_at_ = kNever;
  bool estop_latched_ = false;

  std::unique_ptr<MoteusBus> bus_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr buttons_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cmr_rovernet_cpp::DriveOnlyControlNode>();
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
