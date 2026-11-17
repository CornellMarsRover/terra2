// Port of autonomous_navigation/state_machine.py (executable `state_machine`,
// node `state_machine_node`): high-level autonomy mission logic with
// preplanned coarse waypoints.
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "autonomous_navigation_cpp/state_machine_core.hpp"

namespace autonomous_navigation_cpp
{

class StateMachineNode : public rclcpp::Node
{
public:
  explicit StateMachineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("state_machine_node", options)
  {
    // real vs sim flag
    const bool real = declare_parameter<bool>("real", true);

    // Load GPS waypoints. The YAML files are installed by the Python package
    // `autonomous_navigation`, so the default path points into its share dir.
    const std::string waypoints_file = real ?
      "config/waypoints_test_site_utah.yaml" : "config/sim_waypoints_condensed.yaml";
    const std::string default_path =
      (std::filesystem::path(ament_index_cpp::get_package_share_directory(
        "autonomous_navigation")) / waypoints_file).string();
    const std::string waypoints_path =
      declare_parameter<std::string>("waypoints_file", default_path);
    RCLCPP_INFO(get_logger(), "Waypoints file: %s", waypoints_path.c_str());
    std::vector<MissionWaypoint> waypoints = load_waypoints(waypoints_path);
    if (waypoints.empty()) {
      // NOTE(port): Python logs this, calls rclpy.shutdown() and returns from
      // __init__ (main() then dies with a traceback). Here the constructor
      // throws: the standalone executable exits 1 with a FATAL log, and a
      // component container reports a failed load.
      RCLCPP_ERROR(get_logger(), "No waypoints loaded. Shutting down.");
      throw std::runtime_error("No waypoints loaded. Shutting down.");
    }

    stop_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/stop", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/autonomy/pose/robot/global", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        core_->set_pose(m->twist.linear.x, m->twist.linear.y, m->twist.angular.z);
      });
    obj_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/autonomy/target_object/position", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        core_->on_object(m->linear.x, m->linear.y, m->linear.z);
      });
    global_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/autonomy/target/global", 10);
    local_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/autonomy/target/local", 10);
    name_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/target_object/name", 10);
    led_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/led", 10);

    StateMachineCore::Hooks hooks;
    hooks.log = [this](StateMachineCore::Level level, const std::string & s) {
        if (level == StateMachineCore::Level::Error) {
          RCLCPP_ERROR(get_logger(), "%s", s.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", s.c_str());
        }
      };
    hooks.publish_target_name = [this](const std::string & name) {
        std_msgs::msg::String m;
        m.data = name;
        name_pub_->publish(m);
      };
    hooks.blink_led = [this]() {
        std_msgs::msg::String m;
        m.data = "blink";
        led_pub_->publish(m);
      };
    core_ = std::make_unique<StateMachineCore>(real, std::move(waypoints), std::move(hooks));

    // Control loop @10Hz
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {control_loop();});
  }

private:
  std::vector<MissionWaypoint> load_waypoints(const std::string & path)
  {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      RCLCPP_ERROR(get_logger(), "Waypoints file not found: %s", path.c_str());
      return {};
    }
    YAML::Node data;
    try {
      data = YAML::LoadFile(path);
      RCLCPP_INFO(get_logger(), "Waypoints loaded successfully.");
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
      return {};
    }
    // data.get('waypoints', [])
    // NOTE(port): Python raises AttributeError if the document is not a
    // mapping; that case is treated as "no waypoints".
    if (!data.IsMap() || !data["waypoints"] || !data["waypoints"].IsSequence()) {
      return {};
    }
    std::vector<MissionWaypoint> out;
    std::size_t index = 0;
    for (const YAML::Node & entry : data["waypoints"]) {
      // NOTE(port): Python looks latitude/longitude up lazily (KeyError in
      // the middle of the mission when the bad waypoint becomes active);
      // every waypoint is validated up front instead.
      MissionWaypoint wp;
      try {
        if (!entry.IsMap() || !entry["latitude"] || !entry["longitude"]) {
          throw std::runtime_error("missing latitude/longitude");
        }
        wp.latitude = entry["latitude"].as<double>();
        wp.longitude = entry["longitude"].as<double>();
        if (entry["type"] && entry["type"].IsScalar()) {
          wp.type = entry["type"].as<std::string>();
        }
        // float(waypoint.get('threshold', 2.0)) for 'coordinate' waypoints
        if (wp.type && *wp.type == "coordinate" && entry["threshold"]) {
          wp.threshold = entry["threshold"].as<double>();
        }
      } catch (const std::exception & e) {
        throw std::runtime_error(
                "Invalid waypoint #" + std::to_string(index) + " in " + path + ": " + e.what());
      }
      out.push_back(wp);
      ++index;
    }
    return out;
  }

  void control_loop()
  {
    if (!core_->control_loop()) {
      RCLCPP_INFO_ONCE(get_logger(), "All waypoints reached.");
      std_msgs::msg::String m;
      m.data = "mission_complete";
      stop_pub_->publish(m);
      return;
    }
    // publish_targets(): the same [x, y] on the local and global topics.
    std_msgs::msg::Float32MultiArray msg;
    const Pt & target = core_->target_position();
    msg.data = {static_cast<float>(target.x), static_cast<float>(target.y)};
    local_pub_->publish(msg);
    global_pub_->publish(msg);
  }

  std::unique_ptr<StateMachineCore> core_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_pub_, name_pub_, led_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr global_pub_, local_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr obj_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::StateMachineNode)
