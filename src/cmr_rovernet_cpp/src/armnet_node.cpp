// C++ port of cmr_rovernet/armnet.py (executable `armnet_node`).
//
// Bridges arm commands to the arm moteus controllers over one persistent
// fdcanusb transport:
//  * /main_arm_controller/joint_trajectory (IK / MoveIt mode)
//  * /mini_arm_controller/cmd_pos          (mini-arm mode)
//  * /arm_controller/cmd_buttons           (mode switching)
// and optionally publishes /joint_states from moteus queries.
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmr_msgs/msg/controller_reading.hpp>
#include <cmr_msgs/msg/mini_arm_degree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "cmr_rovernet_cpp/drive_common.hpp"
#include "cmr_rovernet_cpp/moteus_bus.hpp"
#include "cmr_rovernet_cpp/node_config.hpp"
#include "cmr_rovernet_cpp/rovernet_utils.hpp"

namespace cmr_rovernet_cpp {
namespace {

constexpr double MAX_TORQUE = 0.7;
constexpr double MAX_ACCEL = 5.0;
constexpr double MINI_ARM_MAX_TORQUE = 0.7;
constexpr double MINI_ARM_VELOCITY_LIMIT = 10.0;
constexpr double MINI_ARM_ACCEL_LIMIT = 10.0;
constexpr std::array<double, 6> MINI_ARM_SOFT_GEAR_RATIO = {0.25, 0.3, 0.25, 0.125, 0.125, 0.125};
constexpr double MINI_ARM_DEADBAND = 0.002;
constexpr double PI = 3.141592653589793;

// JOINT_MOTOR_IDS (insertion order matters: it is the command order).
const std::vector<std::pair<std::string, int>> & joint_motor_ids()
{
  static const std::vector<std::pair<std::string, int>> ids = {
    {"base_joint", 9},
    {"shoulder_joint", 10},
    {"elbow_joint", 11},
    {"wrist_rotate", 12},
    {"wrist_twist", 13},
    {"wrist_rotate_two", 14},
  };
  return ids;
}

std::optional<int> motor_id_for(const std::string & joint_name)
{
  for (const auto & kv : joint_motor_ids()) {
    if (kv.first == joint_name) {
      return kv.second;
    }
  }
  return std::nullopt;
}

std::vector<int> all_arm_ids()
{
  std::vector<int> ids;
  for (const auto & kv : joint_motor_ids()) {
    ids.push_back(kv.second);
  }
  return ids;
}

std::string fmt_list(const std::vector<double> & v)
{
  std::ostringstream o;
  o << "[";
  for (std::size_t i = 0; i < v.size(); ++i) {
    o << (i ? ", " : "") << drive::format_double(v[i]);
  }
  o << "]";
  return o.str();
}

}  // namespace

class JSInputSubscriber : public rclcpp::Node
{
public:
  JSInputSubscriber()
  : rclcpp::Node("js_input_subscriber")
  {
    subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/main_arm_controller/joint_trajectory", 10,
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {listener_callback(*msg);});
    miniarm_subscription_ = create_subscription<cmr_msgs::msg::MiniArmDegree>(
      "/mini_arm_controller/cmd_pos", 10,
      [this](cmr_msgs::msg::MiniArmDegree::ConstSharedPtr msg) {listener_callback_mini(*msg);});
    button_subscription_ = create_subscription<cmr_msgs::msg::ControllerReading>(
      "/arm_controller/cmd_buttons", 10,
      [this](cmr_msgs::msg::ControllerReading::ConstSharedPtr msg) {
        listener_button_callback(*msg);
      });

    target_mini_arm_positions_.assign(joint_motor_ids().size(), 0.0);

    declare_parameter("config_path", "");
    ConfigLoadOptions load_opts;
    load_opts.expand_user = false;
    load_opts.check_exists = false;
    load_opts.log_loaded = false;
    const TomlTable node_config = load_node_config(*this, load_opts);
    auto get = [&](const std::string & key) -> const TomlValue * {
        const auto it = node_config.find(key);
        return it == node_config.end() ? nullptr : &it->second;
      };

    moveit_mode_ = get("start_in_moveit_mode") ? get("start_in_moveit_mode")->as_bool() : false;
    arm_can_port_ = get("can_port") ? get("can_port")->as_string() : "/dev/ttyACM1";
    publish_hardware_joint_states_ =
      get("publish_hardware_joint_states") ? get("publish_hardware_joint_states")->as_bool() :
      false;
    use_legacy_ik_command_mapping_ =
      get("use_legacy_ik_command_mapping") ? get("use_legacy_ik_command_mapping")->as_bool() :
      true;

    BusOptions bus_opts;
    bus_opts.port_arg = arm_can_port_;
    bus_opts.resolve = false;  // armnet passes can_port straight to moteus.Fdcanusb
    bus_opts.ids = all_arm_ids();
    bus_opts.reply_timeout_s = 0.2;  // query_moteus_sync(timeout_s=0.2)
    bus_opts.query_format = mm::Query::Format();  // moteus-python default resolution
    bus_ = std::make_unique<MoteusBus>(get_logger(), bus_opts);
    // NOTE(port): the Python node crashed if the port could not be opened.
    // Here a failure (including PortBusyError while drivesnet's auto-probe
    // briefly locks this adapter) is logged and retried by open_timer_ (and
    // by every command).  No frame is ever sent before ensure_open() holds
    // the exclusive device lock.
    try_open();
    open_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
          if (!bus_->is_open()) {
            try_open();
          }
        });

    if (publish_hardware_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      joint_state_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() {publish_joint_states();});
    }
    RCLCPP_INFO(get_logger(), "Arm moteus fdcanusb transport: %s", arm_can_port_.c_str());
    RCLCPP_INFO(
      get_logger(), "Arm hardware joint-state publishing: %s",
      publish_hardware_joint_states_ ? "True" : "False");
    RCLCPP_INFO(
      get_logger(), "Arm legacy IK command mapping: %s",
      use_legacy_ik_command_mapping_ ? "True" : "False");
    RCLCPP_INFO(get_logger(), "Arm initial mode: %s", moveit_mode_ ? "IK" : "mini arm");
  }

private:
  void try_open()
  {
    try {
      bus_->ensure_open();
    } catch (const PortBusyError & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Arm fdcanusb %s busy: %s (retrying)",
        arm_can_port_.c_str(), exc.what());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Could not open arm fdcanusb %s: %s (retrying)",
        arm_can_port_.c_str(), exc.what());
    }
  }

  static double convert_angle_to_custom_range(double angle, double max_range)
  {
    const double normalized_angle = (angle - PI) / (-2 * PI);
    const double offset = max_range / 2;
    return (normalized_angle * max_range) - offset;
  }

  std::pair<std::vector<double>, std::vector<double>> translate_to_electrical(
    const std::vector<double> & positions)
  {
    std::vector<double> output_pos = {
      -convert_angle_to_custom_range(positions[0], 100),
      -convert_angle_to_custom_range(positions[1], 100),
      -convert_angle_to_custom_range(positions[2], 100),
      convert_angle_to_custom_range(positions[3], 50),
      convert_angle_to_custom_range(positions[4], 50),
      convert_angle_to_custom_range(positions[5], 50),
    };
    std::vector<double> output_vels = {6, 6, 6, 6, 6, 6};
    RCLCPP_INFO(
      get_logger(), "%s, [6, 6, 6, 6, 6, 6]", fmt_list(output_pos).c_str());
    return {output_pos, output_vels};
  }

  static double translate_joint_to_electrical(const std::string & joint_name, double position)
  {
    if (joint_name == "base_joint" || joint_name == "shoulder_joint" ||
      joint_name == "elbow_joint")
    {
      return -convert_angle_to_custom_range(position, 100);
    }
    if (joint_name == "wrist_rotate" || joint_name == "wrist_twist" ||
      joint_name == "wrist_rotate_two")
    {
      return convert_angle_to_custom_range(position, 50);
    }
    throw std::invalid_argument("Unknown arm joint: " + joint_name);
  }

  static double motor_rotation_to_joint_angle(const std::string & joint_name, double position)
  {
    if (joint_name == "base_joint" || joint_name == "shoulder_joint" ||
      joint_name == "elbow_joint")
    {
      return position * 2.0 * PI / 100.0;
    }
    return -position * 2.0 * PI / 50.0;
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (const auto & kv : joint_motor_ids()) {
      msg.name.push_back(kv.first);
    }
    std::vector<double> positions;
    std::vector<double> velocities;

    try {
      for (const auto & kv : joint_motor_ids()) {
        const auto & joint_name = kv.first;
        const auto result = bus_->query(kv.second);
        if (!result) {
          throw std::runtime_error("TimeoutError()");
        }
        const int fault = result->values.fault;
        if (fault != 0) {
          RCLCPP_WARN(get_logger(), "%s moteus fault=%d", joint_name.c_str(), fault);
          return;
        }
        const double motor_position = result->values.position;
        double motor_velocity = result->values.velocity;
        if (std::isnan(motor_position)) {
          RCLCPP_WARN(get_logger(), "%s query missing position", joint_name.c_str());
          return;
        }
        if (std::isnan(motor_velocity)) {
          motor_velocity = 0.0;
        }
        positions.push_back(motor_rotation_to_joint_angle(joint_name, motor_position));
        velocities.push_back(motor_rotation_to_joint_angle(joint_name, motor_velocity));
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Could not publish moteus joint states: %s",
        exc.what());
      return;
    }

    msg.position = positions;
    msg.velocity = velocities;
    joint_state_pub_->publish(msg);
  }

  struct ArmCommand
  {
    std::string joint_name;
    double position;
    double velocity_limit;
    double maximum_torque;
    double accel_limit;
  };

  // send_arm_position for a batch of joints, sent in one fdcanusb cycle.
  void send_arm_positions(const std::vector<ArmCommand> & commands)
  {
    if (commands.empty()) {
      return;
    }
    try {
      bus_->ensure_open();
      std::vector<mm::CanFdFrame> frames;
      for (const auto & c : commands) {
        const int motor_id = *motor_id_for(c.joint_name);
        // rovernet_utils._send_moteus_command_async log line.
        RCLCPP_INFO(
          get_logger(),
          "[motor=%d] position=%s, velocity=nan, maximum_torque=%s, velocity_limit=%s, "
          "accel_limit=%s, feedforward_torque=0.0",
          motor_id, drive::format_double(c.position).c_str(),
          drive::format_double(c.maximum_torque).c_str(),
          drive::format_double(c.velocity_limit).c_str(),
          drive::format_double(c.accel_limit).c_str());
        frames.push_back(
          make_arm_position_frame(
            bus_->controller(motor_id), c.position, c.maximum_torque, c.velocity_limit,
            c.accel_limit));
      }
      bus_->send(frames);
    } catch (const std::exception & exc) {
      std::ostringstream joints;
      for (std::size_t i = 0; i < commands.size(); ++i) {
        joints << (i ? ", " : "") << commands[i].joint_name;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to command %s on %s: %s",
        joints.str().c_str(), arm_can_port_.c_str(), exc.what());
    }
  }

  std::pair<int64_t, int64_t> decode_button_combo(const cmr_msgs::msg::ControllerReading & msg)
  {
    const auto & buttons = msg.button_array;
    if (buttons.size() >= 8) {
      int64_t trigger_val = 0;
      int64_t button_val = 0;
      if (buttons[0]) {
        trigger_val = L1;
      } else if (buttons[1]) {
        trigger_val = R1;
      }
      if (buttons[4]) {
        button_val = SQUARE;
      } else if (buttons[5]) {
        button_val = X;
      } else if (buttons[6]) {
        button_val = CIRCLE;
      } else if (buttons[7]) {
        button_val = TRIANGLE;
      }
      return {trigger_val, button_val};
    }
    if (buttons.size() >= 2) {
      return {buttons[0], buttons[1]};
    }
    RCLCPP_WARN(get_logger(), "Ignoring arm button message with %zu entries.", buttons.size());
    return {0, 0};
  }

  std::vector<ArmCommand> iter_joint_commands(const trajectory_msgs::msg::JointTrajectory & msg)
  {
    if (msg.points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty JointTrajectory.");
      return {};
    }
    const auto & point = msg.points[0];
    if (msg.joint_names.size() != point.positions.size()) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring JointTrajectory with mismatched joint_names and positions.");
      return {};
    }

    std::vector<ArmCommand> commands;
    for (std::size_t index = 0; index < msg.joint_names.size(); ++index) {
      const auto & joint_name = msg.joint_names[index];
      if (!motor_id_for(joint_name)) {
        RCLCPP_WARN(get_logger(), "Ignoring unknown arm joint: %s", joint_name.c_str());
        continue;
      }
      double velocity_limit = 6;
      if (index < point.velocities.size() && point.velocities[index] > 0) {
        velocity_limit = std::min(point.velocities[index], 6.0);
      }
      commands.push_back(
        {joint_name, translate_joint_to_electrical(joint_name, point.positions[index]),
          velocity_limit, MAX_TORQUE, MAX_ACCEL});
    }

    std::ostringstream o;
    o << "[";
    for (std::size_t i = 0; i < commands.size(); ++i) {
      o << (i ? ", " : "") << "('" << commands[i].joint_name << "', "
        << drive::format_double(commands[i].position) << ", "
        << drive::format_double(commands[i].velocity_limit) << ")";
    }
    o << "]";
    RCLCPP_INFO(get_logger(), "JointTrajectory commands: %s", o.str().c_str());
    return commands;
  }

  void listener_callback(const trajectory_msgs::msg::JointTrajectory & msg)
  {
    if (!moveit_mode_) {
      return;
    }
    if (use_legacy_ik_command_mapping_) {
      if (msg.points.empty()) {
        RCLCPP_WARN(get_logger(), "Ignoring empty JointTrajectory.");
        return;
      }
      const auto & point = msg.points[0];
      if (point.positions.size() < 6) {
        RCLCPP_ERROR(get_logger(), "Ignoring JointTrajectory with fewer than 6 positions.");
        return;
      }
      const auto pv = translate_to_electrical(point.positions);
      std::vector<ArmCommand> commands;
      const auto & joints = joint_motor_ids();
      for (std::size_t i = 0; i < joints.size(); ++i) {
        commands.push_back({joints[i].first, pv.first[i], pv.second[i], MAX_TORQUE, MAX_ACCEL});
      }
      send_arm_positions(commands);
      return;
    }
    send_arm_positions(iter_joint_commands(msg));
  }

  void listener_callback_mini(const cmr_msgs::msg::MiniArmDegree & msg)
  {
    if (moveit_mode_) {
      return;
    }
    const std::vector<double> mini_arm_positions = {
      msg.base_angle, msg.shoulder_angle, msg.elbow_angle,
      msg.first_rotate_angle, msg.tilt_angle, msg.second_rotate_angle};

    if (!prev_mini_arm_positions_) {
      prev_mini_arm_positions_ = mini_arm_positions;
      target_mini_arm_positions_.assign(mini_arm_positions.size(), 0.0);
      RCLCPP_INFO(
        get_logger(), "Mini arm initialized at: %s", fmt_list(mini_arm_positions).c_str());
      return;
    }

    for (std::size_t index = 0; index < mini_arm_positions.size(); ++index) {
      double delta = mini_arm_positions[index] - (*prev_mini_arm_positions_)[index];
      if (std::abs(delta) < MINI_ARM_DEADBAND) {
        delta = 0.0;
      }
      target_mini_arm_positions_[index] += MINI_ARM_SOFT_GEAR_RATIO[index] * delta;
    }
    prev_mini_arm_positions_ = mini_arm_positions;
    std::cout << fmt_list(target_mini_arm_positions_) << std::endl;

    std::vector<ArmCommand> commands;
    const auto & joints = joint_motor_ids();
    for (std::size_t i = 0; i < joints.size() && i < target_mini_arm_positions_.size(); ++i) {
      commands.push_back(
        {joints[i].first, target_mini_arm_positions_[i], MINI_ARM_VELOCITY_LIMIT,
          MINI_ARM_MAX_TORQUE, MINI_ARM_ACCEL_LIMIT});
    }
    send_arm_positions(commands);
  }

  void listener_button_callback(const cmr_msgs::msg::ControllerReading & msg)
  {
    const auto [trigger_val, button_val] = decode_button_combo(msg);
    if (trigger_val == R1 && button_val == SQUARE) {
      moveit_mode_ = false;
      RCLCPP_INFO(get_logger(), "Mini arm mode: 0");
      std::cout << "Mini arm mode: 0" << std::endl;
    }
    if (trigger_val == R1 && button_val == CIRCLE) {
      moveit_mode_ = true;
      RCLCPP_INFO(get_logger(), "IK mode: 1");
      std::cout << "MoveIt arm mode: 1" << std::endl;
    }
    if (trigger_val == R1 && button_val == TRIANGLE) {
      moveit_mode_ = true;
      std::cout << "MoveIt arm mode: 1" << std::endl;
    }
  }

  bool moveit_mode_ = false;
  std::string arm_can_port_;
  bool publish_hardware_joint_states_ = false;
  bool use_legacy_ik_command_mapping_ = true;
  std::optional<std::vector<double>> prev_mini_arm_positions_;
  std::vector<double> target_mini_arm_positions_;
  std::unique_ptr<MoteusBus> bus_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  rclcpp::Subscription<cmr_msgs::msg::MiniArmDegree>::SharedPtr miniarm_subscription_;
  rclcpp::Subscription<cmr_msgs::msg::ControllerReading>::SharedPtr button_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr open_timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmr_rovernet_cpp::JSInputSubscriber>());
  rclcpp::shutdown();
  return 0;
}
