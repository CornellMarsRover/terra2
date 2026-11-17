// Port of autonomous_navigation/costmap.py (executable `costmap`, node `costmap_node`).
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cmr_msgs/msg/ground_plane_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "autonomous_navigation_cpp/costmap_core.hpp"

namespace autonomous_navigation_cpp
{

namespace
{

double read_scalar(const uint8_t * p, uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8: {int8_t v; std::memcpy(&v, p, 1); return v;}
    case PointField::UINT8: {uint8_t v; std::memcpy(&v, p, 1); return v;}
    case PointField::INT16: {int16_t v; std::memcpy(&v, p, 2); return v;}
    case PointField::UINT16: {uint16_t v; std::memcpy(&v, p, 2); return v;}
    case PointField::INT32: {int32_t v; std::memcpy(&v, p, 4); return v;}
    case PointField::UINT32: {uint32_t v; std::memcpy(&v, p, 4); return v;}
    case PointField::FLOAT32: {float v; std::memcpy(&v, p, 4); return v;}
    case PointField::FLOAT64: {double v; std::memcpy(&v, p, 8); return v;}
    default: return 0.0;
  }
}

}  // namespace

class CostmapNode : public rclcpp::Node
{
public:
  explicit CostmapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("costmap_node", options),
    real_(declare_parameter<bool>("real", true)),
    core_(real_)
  {
    if (real_) {
      ground_plane_sub_ = create_subscription<cmr_msgs::msg::GroundPlaneStamped>(
        "/camera/ground_plane", 10,
        [this](cmr_msgs::msg::GroundPlaneStamped::ConstSharedPtr m) {ground_plane_callback(*m);});
    }
    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/points", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {pointcloud_callback(*m);});
    pose_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/autonomy/pose/robot/global", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        north_ = m->twist.linear.x;
        west_ = m->twist.linear.y;
        yaw_ = m->twist.angular.z;
        c_ = std::cos(yaw_);
        s_ = std::sin(yaw_);
      });
    movement_sub_ = create_subscription<std_msgs::msg::String>(
      "/autonomy/move/move_type", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {last_movement_ = m->data;});
    object_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/autonomy/target_object/position", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        core_.set_object_position(m->linear.x, m->linear.y);
      });

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/autonomy/costmap", 10);

    // rclpy timers follow the node clock (sim time when use_sim_time=true).
    decay_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(20.0),
        [this]() {core_.decay_cost();});
    pub_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
        [this]() {publish_obstacles();});

    RCLCPP_INFO(get_logger(), "Costmap node initialized");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    core_.set_last_movement_point_turn(last_movement_ == "point_turn");
    if (!extract_xyz(msg)) {
      return;
    }
    // interpolate_pose(): real -> current R; sim -> R(-yaw).
    double c = c_, s = s_;
    if (!real_) {
      c = std::cos(-1.0 * yaw_);
      s = std::sin(-1.0 * yaw_);
    }
    core_.process_cloud(xyz_.data(), xyz_.size(), north_, west_, c, s);
  }

  // Equivalent of sensor_msgs_py.point_cloud2.read_points(msg, skip_nans=True)
  // followed by pt[0], pt[1], pt[2] (first three fields in msg.fields order).
  bool extract_xyz(const sensor_msgs::msg::PointCloud2 & msg)
  {
    using sensor_msgs::msg::PointField;
    xyz_.clear();
    // Flatten fields (count > 1 expands into sub-fields like numpy does).
    flat_.clear();
    for (const auto & f : msg.fields) {
      uint8_t sz = 0;
      switch (f.datatype) {
        case PointField::INT8: case PointField::UINT8: sz = 1; break;
        case PointField::INT16: case PointField::UINT16: sz = 2; break;
        case PointField::INT32: case PointField::UINT32: case PointField::FLOAT32: sz = 4; break;
        case PointField::FLOAT64: sz = 8; break;
        default: sz = 0;
      }
      if (sz == 0) {
        continue;
      }
      const uint32_t cnt = std::max<uint32_t>(1, f.count);
      for (uint32_t i = 0; i < cnt; ++i) {
        flat_.push_back({f.offset + i * sz, f.datatype, sz});
      }
    }
    if (flat_.size() < 3 || msg.point_step == 0) {
      return false;  // Python would IndexError on pt[2]
    }
    // NOTE(port): big-endian clouds are not byte-swapped (Jetson + ZED/Gazebo are LE).
    const std::size_t npts = static_cast<std::size_t>(msg.width) * msg.height;
    if (npts * msg.point_step > msg.data.size()) {
      return false;
    }
    xyz_.reserve(npts);
    const uint8_t * base = msg.data.data();
    const bool fast = flat_[0].type == PointField::FLOAT32 &&
      flat_[1].type == PointField::FLOAT32 && flat_[2].type == PointField::FLOAT32;
    // sensor_msgs_py only filters NaNs when `not cloud.is_dense`, and then on
    // every field (incl. packed rgb floats).
    const bool skip_nans = !msg.is_dense;
    for (std::size_t i = 0; i < npts; ++i) {
      const uint8_t * p = base + i * msg.point_step;
      if (skip_nans) {
        bool has_nan = false;
        for (const F & f : flat_) {
          if (f.type == PointField::FLOAT32) {
            float v; std::memcpy(&v, p + f.off, 4);
            if (std::isnan(v)) {has_nan = true; break;}
          } else if (f.type == PointField::FLOAT64) {
            double v; std::memcpy(&v, p + f.off, 8);
            if (std::isnan(v)) {has_nan = true; break;}
          }
        }
        if (has_nan) {
          continue;
        }
      }
      XYZ q;
      if (fast) {
        std::memcpy(&q.x, p + flat_[0].off, 4);
        std::memcpy(&q.y, p + flat_[1].off, 4);
        std::memcpy(&q.z, p + flat_[2].off, 4);
      } else {
        // NOTE(port): non-float32 xyz are narrowed to float32.
        q.x = static_cast<float>(read_scalar(p + flat_[0].off, flat_[0].type));
        q.y = static_cast<float>(read_scalar(p + flat_[1].off, flat_[1].type));
        q.z = static_cast<float>(read_scalar(p + flat_[2].off, flat_[2].type));
      }
      if (!skip_nans && (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z))) {
        // NOTE(port): Python would let NaNs from an is_dense cloud through and
        // create (nan, nan) dict keys; we drop them.
        continue;
      }
      xyz_.push_back(q);
    }
    return true;
  }

  void ground_plane_callback(const cmr_msgs::msg::GroundPlaneStamped & msg)
  {
    // Uses self.R / self.north / self.west (same as interpolate_pose in real mode).
    const std::size_t n = std::min(msg.x.size(), msg.y.size());
    std::vector<double> xs(n), ys(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double x = msg.x[i], y = msg.y[i];
      xs[i] = (c_ * x + (-s_) * y) + north_;
      ys[i] = (s_ * x + c_ * y) + west_;
    }
    if (!core_.set_ground_plane(std::move(xs), std::move(ys))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ground plane with %zu vertices ignored (invalid polygon)", n);
    }
  }

  void publish_obstacles()
  {
    std_msgs::msg::Float32MultiArray msg;
    core_.collect(msg.data);
    pub_->publish(msg);
  }

  bool real_;
  CostmapCore core_;
  std::string last_movement_{"ackerman"};
  double north_{0.0}, west_{0.0}, yaw_{0.0};
  double c_{1.0}, s_{0.0};

  struct F {uint32_t off; uint8_t type; uint8_t size;};
  std::vector<XYZ> xyz_;
  std::vector<F> flat_;

  rclcpp::Subscription<cmr_msgs::msg::GroundPlaneStamped>::SharedPtr ground_plane_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr object_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr decay_timer_, pub_timer_;
};

}  // namespace autonomous_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation_cpp::CostmapNode)
