// C++ port of cmr_rovernet/usama_control_testing.py
// (executable `usama_control_testing_node`; launched as `drivesnet` by
// rovernet.launch.py via config/drivesnet.toml).
//
// Shared swerve drive backend.  Two halves:
//  * controller adapter:
//      /controller/drives/axes    (TwistStamped, controller sticks)
//      /controller/drives/buttons (ControllerReading: speed, estop, D-pad
//                                  drive-mode selection)
//      -> /cmd_vel/teleop (cmr_msgs/DriveCommand), /cmd_vel/estop (Bool)
//  * backend: the mux-selected /cmd_vel (cmr_msgs/DriveCommand) is shaped for
//    the operator drive mode and sent to the moteus controllers.
// A worker thread evaluates the selected command every 1/refresh_rate_hz
// seconds and commands the 4 drive (velocity mode) + 4 steer (position mode)
// moteus controllers, or stops all 8 (idle / estop / drive-mode change).
//
// Perf change vs. Python: one fdcanusb transport is opened once and reused
// (Python created a new transport + controllers every cycle), and the 8
// commands of a cycle go out in a single fdcanusb cycle.
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cmr_msgs/msg/controller_reading.hpp>
#include <cmr_msgs/msg/drive_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cmr_rovernet_cpp/drive_common.hpp"
#include "cmr_rovernet_cpp/drive_modes.hpp"
#include "cmr_rovernet_cpp/moteus_bus.hpp"
#include "cmr_rovernet_cpp/node_config.hpp"
#include "cmr_rovernet_cpp/time_util.hpp"

namespace cmr_rovernet_cpp {

using namespace drive;  // NOLINT

namespace {

struct ManualCommandState
{
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  double speed_rps = 0.0;
};

struct SelectedCommandState
{
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  double speed_rps = 0.0;
  // Python default 0.0 on the monotonic clock: never fresh in practice.
  double updated_at = kNever;
};

struct SelectedCommand
{
  std::string mode;    // idle | estop | mode_change | swerve
  std::string source;  // idle | controller_estop | mode_change:<mode> | selected_cmd_vel
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  double speed_rps = 0.0;
  std::string drive_mode;  // swerve only

  SelectedCommand() = default;
  SelectedCommand(std::string mode_in, std::string source_in)
  : mode(std::move(mode_in)), source(std::move(source_in)) {}

  std::string repr() const
  {
    std::ostringstream o;
    o << "{'mode': '" << mode << "', 'source': '" << source << "'";
    if (mode == "swerve") {
      o << ", 'vx': " << format_double(vx) << ", 'vy': " << format_double(vy)
        << ", 'omega': " << format_double(omega)
        << ", 'speed_rps': " << format_double(speed_rps)
        << ", 'drive_mode': '" << drive_mode << "'";
    }
    o << "}";
    return o.str();
  }
};

}  // namespace

class UsamaControlRosNode : public rclcpp::Node
{
public:
  UsamaControlRosNode()
  : rclcpp::Node("usama_control_testing_node")
  {
    declare_parameter("config_path", "");
    const TomlTable config = load_node_config(*this);

    declare_lenient(*this, "can_port", std::string("/dev/ttyACM0"));
    declare_lenient(*this, "timeout_s", 0.30);
    declare_lenient(*this, "drive_rps", 4.0);
    declare_lenient(*this, "half_speed_multiplier", HALF_SPEED_MULTIPLIER);
    declare_lenient(*this, "double_speed_multiplier", DOUBLE_SPEED_MULTIPLIER);
    declare_lenient(*this, "triple_speed_multiplier", TRIPLE_SPEED_MULTIPLIER);
    declare_lenient(*this, "trigger_pressed_threshold", TRIGGER_PRESSED_THRESHOLD);
    declare_lenient(*this, "drive_max_torque", 2.0);
    declare_lenient(*this, "steer_max_torque", 3.0);
    declare_lenient(*this, "steer_velocity_limit", 6.0);
    declare_lenient(*this, "watchdog_timeout_s", 0.5);
    declare_lenient(*this, "controller_deadzone", 0.1);
    declare_lenient(*this, "command_timeout_s", 0.5);
    declare_lenient(*this, "refresh_rate_hz", 10.0);
    declare_lenient(*this, "capture_steer_zero_on_start", true);
    declare_lenient(
      *this, "drive_mode", std::string(drive_mode_value(DriveMode::TRANSLATION_ROTATION)));

    port_ = setting_string(*this, "can_port", config);
    timeout_s_ = setting_double(*this, "timeout_s", config);
    drive_rps_ = std::abs(setting_double(*this, "drive_rps", config));
    half_speed_multiplier_ = std::abs(setting_double(*this, "half_speed_multiplier", config));
    double_speed_multiplier_ = std::abs(setting_double(*this, "double_speed_multiplier", config));
    triple_speed_multiplier_ = std::abs(setting_double(*this, "triple_speed_multiplier", config));
    trigger_pressed_threshold_ = setting_int(*this, "trigger_pressed_threshold", config);
    drive_max_torque_ = std::abs(setting_double(*this, "drive_max_torque", config));
    steer_max_torque_ = std::abs(setting_double(*this, "steer_max_torque", config));
    steer_velocity_limit_ = std::abs(setting_double(*this, "steer_velocity_limit", config));
    watchdog_timeout_s_ = std::max(0.25, setting_double(*this, "watchdog_timeout_s", config));
    controller_deadzone_ = setting_double(*this, "controller_deadzone", config);
    command_timeout_s_ = setting_double(*this, "command_timeout_s", config);
    refresh_rate_hz_ = setting_double(*this, "refresh_rate_hz", config);
    capture_steer_zero_on_start_ = setting_bool(*this, "capture_steer_zero_on_start", config);
    const std::string configured_drive_mode = setting_string(*this, "drive_mode", config);
    try {
      drive_mode_ = parse_drive_mode(configured_drive_mode);
    } catch (const std::invalid_argument &) {
      RCLCPP_WARN(
        get_logger(), "Unknown drive_mode='%s'; using %s", configured_drive_mode.c_str(),
        drive_mode_value(DriveMode::TRANSLATION_ROTATION));
      drive_mode_ = DriveMode::TRANSLATION_ROTATION;
    }

    steer_center_offsets_ = steer_center_offsets();

    BusOptions bus_opts;
    bus_opts.port_arg = port_;
    bus_opts.resolve = true;
    bus_opts.ids = all_ids();
    bus_opts.reply_timeout_s = timeout_s_;
    bus_opts.query_format = drive_gui_query_format();
    bus_ = std::make_unique<MoteusBus>(get_logger(), bus_opts);

    teleop_publisher_ = create_publisher<cmr_msgs::msg::DriveCommand>("/cmd_vel/teleop", 10);
    estop_publisher_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel/estop", 10);

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
    selected_sub_ = create_subscription<cmr_msgs::msg::DriveCommand>(
      "/cmd_vel", 10,
      [this](cmr_msgs::msg::DriveCommand::ConstSharedPtr msg) {selected_cmd_vel_cb(*msg);});

    worker_ = std::thread([this]() {worker_loop();});

    RCLCPP_INFO(
      get_logger(),
      "Drive backend: controller adapter -> /cmd_vel/teleop; selected /cmd_vel -> Moteus");
    RCLCPP_INFO(
      get_logger(),
      "Using direct moteus command behavior on %s: drive_rps=%.2f, drive_torque=%.2f, "
      "steer_torque=%.2f, half=%.2fx, double=%.2fx, triple=%.2fx",
      port_.c_str(), drive_rps_, drive_max_torque_, steer_max_torque_, half_speed_multiplier_,
      double_speed_multiplier_, triple_speed_multiplier_);
    RCLCPP_INFO(
      get_logger(),
      "Drive mode=%s. D-pad: up=translation+rotation, right=Ackermann, down=point turn, "
      "left=steady heading",
      drive_mode_value(drive_mode_));
  }

  ~UsamaControlRosNode() override
  {
    if (!shutdown()) {
      // The abandoned worker still uses this object: it must not be destroyed.
      std::_Exit(1);
    }
  }

  /// Python destroy_node(): stop the worker (which sends a final stop) and
  /// wait for it, `self._worker.join(timeout=2.0)`.  Returns false if the
  /// worker did not finish in time (e.g. a stalled tty write): it is then
  /// detached and the caller must end the process without destroying the
  /// node (Python's daemon thread simply died with the interpreter).  The
  /// motors' own watchdog_timeout stops them in that case.
  bool shutdown()
  {
    if (!worker_.joinable()) {
      return !worker_abandoned_;
    }
    RCLCPP_INFO(get_logger(), "Shutting down usama_control_testing_node");
    std::unique_lock<std::mutex> lock(shutdown_mutex_);
    shutdown_requested_ = true;
    shutdown_cv_.notify_all();
    const bool done = shutdown_cv_.wait_for(
      lock, std::chrono::duration<double>(2.0), [this]() {return worker_done_;});
    lock.unlock();
    if (done) {
      worker_.join();
      return true;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Drive worker did not stop within 2.0 s (CAN adapter I/O stalled?); the final stop may "
      "not have been sent. Exiting without waiting for it.");
    worker_.detach();
    worker_abandoned_ = true;
    return false;
  }

private:
  void controller_cmd_vel_cb(const geometry_msgs::msg::TwistStamped & msg)
  {
    // NOTE(port): a Twist with a non-finite field is treated as not received:
    // nothing is stored and nothing is published on /cmd_vel/teleop (Python
    // clamped NaN to +1.0, i.e. published a full-scale teleop command).
    if (!twist_finite(msg.twist)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring non-finite /controller/drives/axes");
      return;
    }
    const double vx = apply_deadzone(-msg.twist.linear.y / STICK_RANGE, controller_deadzone_);
    const double vy = apply_deadzone(msg.twist.linear.x / STICK_RANGE, controller_deadzone_);
    const double omega = apply_deadzone(msg.twist.angular.x / STICK_RANGE, controller_deadzone_);
    {
      std::lock_guard<std::mutex> lock(lock_);
      manual_.vx = clamp(vx);
      manual_.vy = clamp(vy);
      manual_.omega = clamp(omega);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "controller cmd_vel vx=%+.3f vy=%+.3f omega=%+.3f",
      clamp(vx), clamp(vy), clamp(omega));
    publish_manual();
  }

  void controller_buttons_cb(const cmr_msgs::msg::ControllerReading & msg)
  {
    const auto decoded = decode_controller_buttons(msg.button_array);
    if (!decoded) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/controller/drives/buttons expected 2 or 8 entries, got %zu",
        msg.button_array.size());
      return;
    }
    const int64_t l1 = decoded->l1;
    const int64_t r1 = decoded->r1;
    const int64_t triangle = decoded->triangle;
    const int64_t l2 = decoded->l2;
    const int64_t r2 = decoded->r2;
    const bool estop_pressed = l1 && triangle;
    const std::optional<DriveMode> requested_mode = drive_mode_for_dpad(msg.dpad);
    double speed_rps = 0.0;
    {
      std::lock_guard<std::mutex> lock(lock_);
      if (estop_pressed && !estop_latched_) {
        estop_latched_ = true;
        manual_ = ManualCommandState();
        RCLCPP_WARN(get_logger(), "Drive estop latched from controller buttons");
        publish_estop(true);
        return;
      }
      if (estop_latched_ && !l1 && !triangle) {
        estop_latched_ = false;
        manual_ = ManualCommandState();
        RCLCPP_INFO(get_logger(), "Drive estop released");
        publish_estop(false);
      }
      if (estop_latched_) {
        manual_.speed_rps = 0.0;
        return;
      }
      if (requested_mode && *requested_mode != drive_mode_) {
        const DriveMode previous_mode = drive_mode_;
        drive_mode_ = *requested_mode;
        mode_change_pending_ = true;
        RCLCPP_INFO(
          get_logger(), "Drive mode changed: %s -> %s; stopping before new-mode commands",
          drive_mode_value(previous_mode), drive_mode_value(*requested_mode));
      }
      speed_rps = manual_speed_rps(l1, r1, l2, r2);
      manual_.speed_rps = speed_rps;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "controller buttons L1=%ld R1=%ld L2=%ld R2=%ld speed_rps=%+.3f",
      static_cast<long>(l1), static_cast<long>(r1), static_cast<long>(l2),
      static_cast<long>(r2), speed_rps);
    publish_manual();
  }

  void publish_estop(bool value)
  {
    std_msgs::msg::Bool out;
    out.data = value;
    estop_publisher_->publish(out);
  }

  // Controller adapter output: the operator's demand as a DriveCommand.
  void publish_manual()
  {
    cmr_msgs::msg::DriveCommand out;
    {
      std::lock_guard<std::mutex> lock(lock_);
      // NOTE(port): faithful to Python: vx is sent as abs(vx); the stick
      // direction only flips the sign of the drive speed (and the last sign
      // is remembered while the stick is inside the deadzone).
      out.speed_rps = manual_.speed_rps * manual_drive_axis_sign(manual_.vx);
      out.vx = std::abs(manual_.vx);
      out.vy = manual_.vy;
      out.omega = manual_.omega;
    }
    teleop_publisher_->publish(out);
  }

  void selected_cmd_vel_cb(const cmr_msgs::msg::DriveCommand & msg)
  {
    if (!std::isfinite(msg.vx) || !std::isfinite(msg.vy) || !std::isfinite(msg.omega) ||
      !std::isfinite(msg.speed_rps))
    {
      RCLCPP_ERROR(get_logger(), "Ignoring non-finite selected drive command");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(lock_);
      selected_.vx = clamp(msg.vx);
      selected_.vy = clamp(msg.vy);
      selected_.omega = clamp(msg.omega);
      const double speed_limit = drive_rps_ * triple_speed_multiplier_;
      selected_.speed_rps = clamp(msg.speed_rps, speed_limit);
      selected_.updated_at = mono_now();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "selected cmd_vel vx=%+.3f vy=%+.3f omega=%+.3f speed_rps=%+.3f",
      clamp(msg.vx), clamp(msg.vy), clamp(msg.omega), msg.speed_rps);
  }

  // stop_all_motors_async: stop DRIVE_IDS + STEER_IDS.
  // NOTE(port): Python collected per-motor stop failures in a status dict
  // that nobody read.  Here a stop that was not sent *and acknowledged by the
  // adapter* throws, so every caller logs it ("Drive worker error", "Drive
  // initialization error", "Drive shutdown stop failed") and the transport is
  // re-opened on the next cycle.
  void stop_all_motors()
  {
    bus_->stop_or_throw(all_ids());
  }

  bool wait_for_shutdown(double seconds)
  {
    std::unique_lock<std::mutex> lock(shutdown_mutex_);
    return shutdown_cv_.wait_for(
      lock, std::chrono::duration<double>(seconds), [this]() {return shutdown_requested_;});
  }

  bool shutdown_requested()
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    return shutdown_requested_;
  }

  void worker_loop()
  {
    initialize_hardware_state();
    const double refresh_period = 1.0 / std::max(refresh_rate_hz_, 1.0);

    while (!shutdown_requested()) {
      if (!init_done_) {
        initialize_hardware_state();
        if (!init_done_) {
          wait_for_shutdown(refresh_period);
          continue;
        }
      }
      try {
        const SelectedCommand command = select_command();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000, "selected drive command: %s",
          command.repr().c_str());
        if (command.mode == "idle" || command.mode == "estop" || command.mode == "mode_change") {
          if (last_source_ != command.source) {
            RCLCPP_INFO(get_logger(), "Drive source switched to %s", command.source.c_str());
          }
          stop_all_motors();
          last_source_ = command.source;
        } else {
          const auto targets = command_swerve_motors(command);
          if (command.source != last_source_) {
            RCLCPP_INFO(
              get_logger(), "Drive source switched to %s; targets={'drive': %s, 'steer': %s}",
              command.source.c_str(), format_targets(targets.drive).c_str(),
              format_targets(targets.steer).c_str());
          }
          last_source_ = command.source;
        }
      } catch (const std::exception & exc) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "Drive worker error: %s", exc.what());
      }
      // time.sleep(refresh_period), but wakes immediately on shutdown.
      wait_for_shutdown(refresh_period);
    }

    try {
      stop_all_motors();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Drive shutdown stop failed: %s", exc.what());
    }
    {
      std::lock_guard<std::mutex> lock(shutdown_mutex_);
      worker_done_ = true;
    }
    shutdown_cv_.notify_all();
  }

  SwerveTargets command_swerve_motors(const SelectedCommand & command)
  {
    const auto targets = calculate_swerve_targets(
      command.vx, command.vy, command.omega, command.speed_rps, steer_center_offsets_);
    bus_->ensure_open();
    const double watchdog = std::max(0.25, watchdog_timeout_s_);
    std::vector<mm::CanFdFrame> frames;
    frames.reserve(8);
    for (const auto & kv : targets.drive) {
      frames.push_back(
        make_drive_velocity_frame(
          bus_->controller(kv.first), kv.second, drive_max_torque_, 5.0, watchdog));
    }
    for (const auto & kv : targets.steer) {
      frames.push_back(
        make_steer_position_frame(
          bus_->controller(kv.first), kv.second, steer_velocity_limit_, steer_max_torque_,
          watchdog));
    }
    bus_->send(frames);
    return targets;
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
      stop_all_motors();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Drive initialization error: %s", exc.what());
    }
  }

  SelectedCommand select_command()
  {
    const double now = mono_now();
    SelectedCommandState selected;
    DriveMode drive_mode;
    {
      std::lock_guard<std::mutex> lock(lock_);
      if (estop_latched_) {
        return {"estop", "controller_estop"};
      }
      if (mode_change_pending_) {
        // One stop cycle before any command is accepted in the new mode.
        mode_change_pending_ = false;
        return {"mode_change", std::string("mode_change:") + drive_mode_value(drive_mode_)};
      }
      selected = selected_;
      drive_mode = drive_mode_;
    }

    const bool selected_active = (now - selected.updated_at) <= command_timeout_s_ &&
      (std::abs(selected.vx) > COMMAND_EPSILON || std::abs(selected.vy) > COMMAND_EPSILON ||
      std::abs(selected.omega) > COMMAND_EPSILON ||
      std::abs(selected.speed_rps) > COMMAND_EPSILON);
    if (selected_active) {
      return shape_command_for_drive_mode(selected_command(selected), drive_mode);
    }
    return {"idle", "idle"};
  }

  static SelectedCommand shape_command_for_drive_mode(
    const SelectedCommand & command, DriveMode drive_mode)
  {
    const auto shaped_motion =
      shape_motion_for_mode(drive_mode, command.vx, command.vy, command.omega);
    SelectedCommand shaped = command;
    shaped.drive_mode = drive_mode_value(drive_mode);
    shaped.vx = shaped_motion[0];
    shaped.vy = shaped_motion[1];
    shaped.omega = shaped_motion[2];
    return shaped;
  }

  // Caller holds lock_.
  double manual_drive_axis_sign(double vx)
  {
    if (std::abs(vx) > controller_deadzone_) {
      last_manual_drive_axis_sign_ = vx >= 0.0 ? 1.0 : -1.0;
    }
    return last_manual_drive_axis_sign_;
  }

  static SelectedCommand selected_command(const SelectedCommandState & selected)
  {
    SelectedCommand c{"swerve", "selected_cmd_vel"};
    c.vx = selected.vx;
    c.vy = selected.vy;
    c.omega = selected.omega;
    c.speed_rps = selected.speed_rps;
    return c;
  }

  bool trigger_pressed(int64_t trigger_byte) const
  {
    return trigger_byte >= std::max<int64_t>(TRIGGER_DEADBAND, trigger_pressed_threshold_);
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

  // Parameters
  std::string port_;
  double timeout_s_ = 0.3;
  double drive_rps_ = 4.0;
  double half_speed_multiplier_ = HALF_SPEED_MULTIPLIER;
  double double_speed_multiplier_ = DOUBLE_SPEED_MULTIPLIER;
  double triple_speed_multiplier_ = TRIPLE_SPEED_MULTIPLIER;
  int64_t trigger_pressed_threshold_ = TRIGGER_PRESSED_THRESHOLD;
  double drive_max_torque_ = 2.0;
  double steer_max_torque_ = 3.0;
  double steer_velocity_limit_ = 6.0;
  double watchdog_timeout_s_ = 0.5;
  double controller_deadzone_ = 0.1;
  double command_timeout_s_ = 0.5;
  double refresh_rate_hz_ = 10.0;
  bool capture_steer_zero_on_start_ = true;

  // State shared with ROS callbacks (guarded by lock_).
  std::mutex lock_;
  ManualCommandState manual_;
  SelectedCommandState selected_;
  bool estop_latched_ = false;
  DriveMode drive_mode_ = DriveMode::TRANSLATION_ROTATION;
  bool mode_change_pending_ = false;
  double last_manual_drive_axis_sign_ = 1.0;

  // Worker-thread-only state.
  std::string last_source_ = "idle";
  bool init_done_ = false;
  std::map<int, double> steer_center_offsets_;
  std::unique_ptr<MoteusBus> bus_;

  std::mutex shutdown_mutex_;
  std::condition_variable shutdown_cv_;
  bool shutdown_requested_ = false;
  bool worker_done_ = false;       // guarded by shutdown_mutex_
  bool worker_abandoned_ = false;  // main thread only
  std::thread worker_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr buttons_sub_;
  rclcpp::Subscription<cmr_msgs::msg::DriveCommand>::SharedPtr selected_sub_;
  rclcpp::Publisher<cmr_msgs::msg::DriveCommand>::SharedPtr teleop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cmr_rovernet_cpp::UsamaControlRosNode>();
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
  const bool worker_stopped = node->shutdown();
  if (!worker_stopped) {
    // The detached worker still references the node and its transport, so
    // neither may be destroyed: shut ROS down and leave without unwinding.
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::_Exit(1);
  }
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
