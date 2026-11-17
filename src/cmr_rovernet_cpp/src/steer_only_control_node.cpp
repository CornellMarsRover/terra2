// C++ port of cmr_rovernet/steer_only_control.py
// (executable `steer_only_control_node`, launch/steer_only.launch.py).
//
// Commands only the 4 steer motors (position mode) from the controller
// sticks; drive motors are never commanded.  One persistent fdcanusb
// transport instead of one per timer tick.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
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

class SteerOnlyControlNode : public rclcpp::Node
{
public:
  SteerOnlyControlNode()
  : rclcpp::Node("steer_only_control_node")
  {
    declare_parameter("config_path", "");
    const TomlTable config = load_node_config(*this);

    declare_lenient(*this, "can_port", std::string("/dev/ttyACM0"));
    declare_lenient(*this, "timeout_s", 0.30);
    declare_lenient(*this, "steer_max_torque", 0.5);
    declare_lenient(*this, "steer_velocity_limit", 1.5);
    declare_lenient(*this, "watchdog_timeout_s", 0.5);
    declare_lenient(*this, "controller_deadzone", 0.1);
    declare_lenient(*this, "command_timeout_s", 0.5);
    declare_lenient(*this, "refresh_rate_hz", 10.0);
    declare_lenient(*this, "capture_steer_zero_on_start", true);

    port_ = setting_string(*this, "can_port", config);
    timeout_s_ = setting_double(*this, "timeout_s", config);
    steer_max_torque_ = std::abs(setting_double(*this, "steer_max_torque", config));
    steer_velocity_limit_ = std::abs(setting_double(*this, "steer_velocity_limit", config));
    watchdog_timeout_s_ = std::max(0.25, setting_double(*this, "watchdog_timeout_s", config));
    controller_deadzone_ = setting_double(*this, "controller_deadzone", config);
    command_timeout_s_ = setting_double(*this, "command_timeout_s", config);
    refresh_rate_hz_ = setting_double(*this, "refresh_rate_hz", config);
    capture_steer_zero_on_start_ = setting_bool(*this, "capture_steer_zero_on_start", config);

    steer_center_offsets_ = steer_center_offsets();

    BusOptions bus_opts;
    bus_opts.port_arg = port_;
    bus_opts.ids = steer_ids();
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

    initialize_hardware_state();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(refresh_rate_hz_, 1.0)),
      [this]() {timer_cb();});

    RCLCPP_INFO(
      get_logger(),
      "steer_only_control_node started on %s: steer_velocity_limit=%.2f, steer_torque=%.2f. "
      "Drive motors are not commanded by this node.",
      port_.c_str(), steer_velocity_limit_, steer_max_torque_);
  }

  /// Python destroy_node().
  void shutdown()
  {
    RCLCPP_INFO(get_logger(), "Shutting down steer_only_control_node");
    try {
      bus_->stop_or_throw(steer_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Steer-only shutdown stop failed: %s", exc.what());
    }
  }

private:
  // NOTE(port): Python let CAN-port errors in the stop path escape the timer
  // callback and kill the node; here they are logged and retried.
  void stop_steer_motors()
  {
    try {
      bus_->stop_or_throw(steer_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Steer-only stop failed: %s", exc.what());
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
    vx_ = clamp(apply_deadzone(-msg.twist.linear.y / STICK_RANGE, controller_deadzone_));
    vy_ = clamp(apply_deadzone(msg.twist.linear.x / STICK_RANGE, controller_deadzone_));
    omega_ = clamp(apply_deadzone(msg.twist.angular.x / STICK_RANGE, controller_deadzone_));
    last_cmd_at_ = mono_now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "steer-only cmd vx=%+.3f vy=%+.3f omega=%+.3f", vx_,
      vy_, omega_);
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
      vx_ = vy_ = omega_ = 0.0;
      RCLCPP_WARN(get_logger(), "Steer-only estop latched from controller buttons");
    } else if (estop_latched_ && !decoded->l1 && !decoded->triangle) {
      estop_latched_ = false;
      RCLCPP_INFO(get_logger(), "Steer-only estop released");
    }
  }

  void timer_cb()
  {
    if (!init_done_) {
      initialize_hardware_state();
      if (!init_done_) {
        return;
      }
    }
    if (estop_latched_ || mono_now() - last_cmd_at_ > command_timeout_s_) {
      stop_steer_motors();
      return;
    }
    if (std::max({std::abs(vx_), std::abs(vy_), std::abs(omega_)}) <= COMMAND_EPSILON) {
      stop_steer_motors();
      return;
    }

    try {
      const auto targets = calculate_steer_targets(vx_, vy_, omega_, steer_center_offsets_);
      bus_->ensure_open();
      std::vector<mm::CanFdFrame> frames;
      for (const auto & kv : targets) {
        frames.push_back(
          make_steer_position_frame(
            bus_->controller(kv.first), kv.second, steer_velocity_limit_, steer_max_torque_,
            std::max(0.25, watchdog_timeout_s_)));
      }
      bus_->send(frames);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "steer-only targets=%s",
        format_targets(targets).c_str());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Steer-only command failed: %s", exc.what());
    }
  }

  void initialize_hardware_state()
  {
    // NOTE(port): the adapter can be transiently busy (PortBusyError, e.g.
    // another node's auto-probe holds its lock).  Instead of silently
    // skipping initialization (steer query + initial stop) forever,
    // initialization is retried until the port has been opened; no motor is
    // commanded before that.  Python had no such
    // transient failure mode.
    try {
      bus_->ensure_open();
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s: CAN port not available yet: %s (retrying)",
        get_name(), exc.what());
      return;
    }
    init_done_ = true;
    try {
      if (capture_steer_zero_on_start_) {
        const std::string port = bus_->ensure_open();
        // NOTE(port): faithful to the Python, whose steer-zero capture never
        // captures anything.  It calls read_value() on the QueryResult
        // wrapper, whose `values` dict is keyed by register *name*, but
        // read_value() looks the value up by moteus.Register, so both
        // ABS_POSITION and POSITION always come back None (verified against
        // the Python node on a simulated fdcanusb that answers the queries).
        // The offsets therefore always stay at STEER_CENTER_OFFSETS (0.0) and
        // the log always reads "no valid steer positions".  The queries are
        // still sent, so bus traffic matches.  Actually capturing here (as an
        // earlier revision of this port did) would shift every steer target
        // by the motor's abs_position - or make it NaN and freeze steering -
        // which the Python never did.  Do not "fix" this without fixing and
        // re-validating the Python on the rover.
        for (int motor_id : steer_ids()) {
          try {
            (void)bus_->query(motor_id);
          } catch (const std::exception &) {
            // query_one(): any failure just marks the result not ok.
          }
        }
        RCLCPP_INFO(
          get_logger(), "Steer zero captured on %s: no valid steer positions",
          port.empty() ? "default transport" : port.c_str());
      }
      bus_->stop_or_throw(steer_ids());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Steer-only initialization error: %s", exc.what());
    }
  }

  std::string port_;
  double timeout_s_ = 0.3;
  double steer_max_torque_ = 0.5;
  double steer_velocity_limit_ = 1.5;
  double watchdog_timeout_s_ = 0.5;
  double controller_deadzone_ = 0.1;
  double command_timeout_s_ = 0.5;
  double refresh_rate_hz_ = 10.0;
  bool capture_steer_zero_on_start_ = true;

  double vx_ = 0.0;
  double vy_ = 0.0;
  double omega_ = 0.0;
  double last_cmd_at_ = kNever;
  bool estop_latched_ = false;
  bool init_done_ = false;
  std::map<int, double> steer_center_offsets_;

  std::unique_ptr<MoteusBus> bus_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr buttons_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cmr_rovernet_cpp::SteerOnlyControlNode>();
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
