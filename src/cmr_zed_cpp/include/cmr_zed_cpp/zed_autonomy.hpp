// Port of cmr_zed/zed_autonomy.py (executable `zed_autonomy`).
//
// 10 Hz: grab -> 720x404 XYZRGBA point cloud on /camera/points, then try three
// hit pixels for a horizontal plane and publish its bounds on
// /camera/ground_plane.
//
// The Python file also declares /zed/pose (TwistStamped) and a TF broadcaster,
// but positional tracking and the pose timer are commented out, so neither is
// ever published; only the publisher / broadcaster objects are kept.
#pragma once
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <cmr_msgs/msg/ground_plane_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "cmr_zed_cpp/zed_conversions.hpp"

namespace cmr_zed_cpp
{

class ZedAutonomy : public rclcpp::Node
{
public:
  explicit ZedAutonomy(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_autonomy", options)
  {
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points", 10);
    ground_publisher_ =
      create_publisher<cmr_msgs::msg::GroundPlaneStamped>("/camera/ground_plane", 10);
    pose_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/zed/pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    const sl::ERROR_CODE status = zed_.open(init_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Cannot open ZED camera: %s", sl::toString(status).c_str());
      throw std::runtime_error("ZED Camera open failed.");
    }

    // Resolution for retrieving the point cloud
    res_ = sl::Resolution(720, 404);
    // alloc() rather than assigning a temporary: sl::Mat copies are shallow.
    point_cloud_.alloc(res_, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);

    plane_parameters_.normal_similarity_threshold = 6;
    plane_parameters_.max_distance_threshold = 0.05f;

    pointcloud_timer_ =
      rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {publish_pointcloud();});
    RCLCPP_INFO(get_logger(), "ZedAutonomy node has been started.");
  }

  ~ZedAutonomy() override {zed_.close();}

private:
  static std::string stamp_repr(const builtin_interfaces::msg::Time & t)
  {
    // Mirrors Python's repr of builtin_interfaces.msg.Time in the log lines.
    return "builtin_interfaces.msg.Time(sec=" + std::to_string(t.sec) +
           ", nanosec=" + std::to_string(t.nanosec) + ")";
  }

  void publish_ground_plane()
  {
    static const unsigned int hits[3][2] = {{600, 600}, {300, 600}, {1000, 600}};
    for (const auto & hit : hits) {
      const sl::ERROR_CODE find_plane_status =
        zed_.findPlaneAtHit(sl::uint2(hit[0], hit[1]), ground_plane_, plane_parameters_);
      if (find_plane_status == sl::ERROR_CODE::SUCCESS &&
        ground_plane_.type == sl::PLANE_TYPE::HORIZONTAL)
      {
        auto msg = std::make_unique<cmr_msgs::msg::GroundPlaneStamped>();
        const auto bounds = ground_plane_.getBounds();
        msg->x.reserve(bounds.size());
        msg->y.reserve(bounds.size());
        for (const auto & b : bounds) {
          msg->x.push_back(b.x);
          msg->y.push_back(b.y);
        }
        // NOTE(port): frame_id left empty, as in the Python node.
        msg->header.stamp = now();
        ground_publisher_->publish(std::move(msg));
        ++ground_count_;
        RCLCPP_INFO(
          get_logger(), "Ground pub count: %d\n%s", ground_count_, stamp_repr(now()).c_str());
        return;
      }
    }
  }

  void publish_pointcloud()
  {
    if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
      zed_.retrieveMeasure(point_cloud_, sl::MEASURE::XYZRGBA, sl::MEM::CPU, res_);
      auto pc2_msg = cv_to_pointcloud2(sl_to_cv(point_cloud_), now(), "zed_camera_frame");
      ++pc_count_;
      RCLCPP_INFO(get_logger(), "PC count: %d\n%s", pc_count_, stamp_repr(now()).c_str());
      pointcloud_publisher_->publish(std::move(pc2_msg));
      publish_ground_plane();
    }
  }

  sl::Camera zed_;
  sl::Resolution res_;
  sl::Mat point_cloud_;
  sl::Plane ground_plane_;
  sl::PlaneDetectionParameters plane_parameters_;
  int pc_count_{0};
  int ground_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<cmr_msgs::msg::GroundPlaneStamped>::SharedPtr ground_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr pointcloud_timer_;
};

}  // namespace cmr_zed_cpp
