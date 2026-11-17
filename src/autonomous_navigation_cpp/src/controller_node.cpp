// Port of autonomous_navigation/controller.py (executable `controller`,
// node `controller_node`).
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cmr_msgs/msg/drive_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "autonomous_navigation_cpp/controller_core.hpp"

namespace autonomous_navigation_cpp
{

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("controller_node", options)
  {
    const bool real = declare_parameter<bool>("real", true);  // false in simulation
    const double waypoint_tolerance = declare_parameter<double>("waypoint_tolerance", 0.3);
    const double input_timeout_s = declare_parameter<double>("input_timeout_s", 1.0);
    drive_rps_ = std::fabs(declare_parameter<double>("drive_rps", 4.0));

    // Throws std::invalid_argument on an unusable timeout / tolerance.
    core_ = std::make_unique<ControllerCore>(
      real, waypoint_tolerance, input_timeout_s, now().nanoseconds());

    pose_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/autonomy/pose/robot/global", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        if (!core_->update_pose(
          m->twist.linear.x, m->twist.linear.y, m->twist.angular.z, now().nanoseconds()))
        {
          RCLCPP_ERROR(get_logger(), "Ignoring non-finite autonomy pose");
        }
      });
    waypoint_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/autonomy/path/next_waypoint", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        if (!core_->update_waypoint(m->data, now().nanoseconds())) {
          RCLCPP_ERROR(get_logger(), "Ignoring malformed autonomy waypoint");
        }
      });
    stop_sub_ = create_subscription<std_msgs::msg::String>(
      "/autonomy/stop", 10,
      // NOTE(port): any message latches stopped=true forever (controller.py:set_stop).
      [this](std_msgs::msg::String::ConstSharedPtr) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        core_->set_stop();
      });

    // The shared RoverNet drive node arbitrates this against manual input.
    drive_pub_ = create_publisher<cmr_msgs::msg::DriveCommand>("/cmd_vel/autonomy", 10);
    movement_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/move/move_type", 10);

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {follow_waypoint();});

    // Python's destroy_node() publishes a stop command; do the same right
    // before the context shuts down (Ctrl-C) and on destruction.
    context_ = get_node_base_interface()->get_context();
    shutdown_cb_ = context_->add_pre_shutdown_callback([this]() {final_stop();});
  }

  ~ControllerNode() override
  {
    if (context_) {
      context_->remove_pre_shutdown_callback(shutdown_cb_);
      if (context_->is_valid()) {
        final_stop();
      }
    }
  }

private:
  // The pre-shutdown callback runs on the signal-handling thread while the
  // executor may still be inside follow_waypoint(). Serialize them and latch
  // shutting_down_ so the zero command is guaranteed to be the last drive
  // command published.
  void final_stop()
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    shutting_down_.store(true);
    publish_drive(ControllerCore::stop_command());
  }

  void follow_waypoint()
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (shutting_down_.load()) {
      return;
    }
    // NOTE(port): one clock reading per tick; Python reads the clock again
    // for the mode-switch dwell timer a few microseconds later.
    const ControllerCore::Output out = core_->step(now().nanoseconds());
    publish_drive(out.drive);
    if (out.movement) {
      std_msgs::msg::String msg;
      msg.data = *out.movement;
      movement_pub_->publish(msg);
    }
  }

  void publish_drive(const drive_command::ChassisCommand & command)
  {
    cmr_msgs::msg::DriveCommand msg;
    msg.vx = command.linear_x;
    msg.omega = command.angular_z;
    msg.speed_rps = drive_rps_;
    drive_pub_->publish(msg);
  }

  double drive_rps_{4.0};
  std::unique_ptr<ControllerCore> core_;

  std::mutex cmd_mutex_;
  std::atomic<bool> shutting_down_{false};
  rclcpp::Context::SharedPtr context_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_sub_;
  rclcpp::Publisher<cmr_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::ControllerNode)
