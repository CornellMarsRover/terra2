// Port of autonomous_navigation/local_planner.py (executable `local_planner`,
// node `local_planner_node`).
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cmr_msgs/msg/ground_plane_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "autonomous_navigation_cpp/local_planner_core.hpp"

namespace autonomous_navigation_cpp
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("local_planner_node", options)
  {
    const bool visualize = declare_parameter<bool>("visualize", true);
    const bool real = declare_parameter<bool>("real", true);
    // NOTE(port): the Rerun visualization (plot_grid_rerun, streamed to
    // 192.168.1.101:9876) is not ported: there is no Rerun C++ SDK in this
    // workspace. This is the Python node's own "rerun not installed" path.
    if (visualize) {
      RCLCPP_WARN(get_logger(), "Rerun is unavailable; planner visualization disabled.");
    }

    core_ = std::make_unique<LocalPlannerCore>(
      real,
      [this](LocalPlannerCore::Level l, const std::string & s) {
        switch (l) {
          case LocalPlannerCore::Level::Info: RCLCPP_INFO(get_logger(), "%s", s.c_str()); break;
          case LocalPlannerCore::Level::Warn: RCLCPP_WARN(get_logger(), "%s", s.c_str()); break;
          case LocalPlannerCore::Level::Error: RCLCPP_ERROR(get_logger(), "%s", s.c_str()); break;
        }
      });

    target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/autonomy/target/local", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {core_->on_target(m->data);});
    pose_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/autonomy/pose/robot/global", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        if (!core_->on_pose(m->twist.linear.x, m->twist.linear.y, m->twist.angular.z)) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Ignoring non-finite robot pose");
        }
      });
    costmap_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/autonomy/costmap", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {core_->on_costmap(m->data);});
    // Ground plane is only used for visualization in Python; keep the
    // subscription so the graph is identical.
    ground_plane_sub_ = create_subscription<cmr_msgs::msg::GroundPlaneStamped>(
      "/camera/ground_plane", 10,
      [](cmr_msgs::msg::GroundPlaneStamped::ConstSharedPtr) {});

    path_debug_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/autonomy/path/plan", 10);
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/autonomy/path/next_waypoint", 10);

    path_check_timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(0.5), [this]() {core_->validate_path();});
    publish_timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(0.1), [this]() {publish_waypoint();});

    RCLCPP_INFO(get_logger(), "Planner Node initialized");
  }

private:
  void publish_waypoint()
  {
    std_msgs::msg::Float32MultiArray plan;
    const auto wp = core_->make_waypoint(&plan.data);
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(wp.begin(), wp.end());
    pub_->publish(msg);
    path_debug_pub_->publish(plan);
  }

  std::unique_ptr<LocalPlannerCore> core_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_, costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<cmr_msgs::msg::GroundPlaneStamped>::SharedPtr ground_plane_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_, path_debug_pub_;
  rclcpp::TimerBase::SharedPtr path_check_timer_, publish_timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::LocalPlannerNode)
