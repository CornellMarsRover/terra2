// Port of cmr_zed/threaded.py (executable `threaded`, node name `zed_autonomy`).
//
// A worker thread grabs from the ZED (HD720 @10 fps) and snapshots a 180x101
// XYZ cloud, the WORLD pose, a horizontal ground plane and the full-res left
// BGRA image. A 10 Hz timer republishes the latest snapshot on
// /camera/points, /camera/ground_plane, /zed/pose (+ TF map->zed_camera_frame)
// and /zed/image.
#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <cmr_msgs/msg/ground_plane_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "cmr_zed_cpp/zed_conversions.hpp"

namespace cmr_zed_cpp
{

class ZedThreaded : public rclcpp::Node
{
public:
  explicit ZedThreaded(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_autonomy", options)
  {
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points", 10);
    pose_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/zed/pose", 10);
    ground_publisher_ =
      create_publisher<cmr_msgs::msg::GroundPlaneStamped>("/camera/ground_plane", 10);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/zed/image", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.camera_fps = 10;
    sl::ERROR_CODE status = zed_.open(init_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Cannot open ZED camera: %s", sl::toString(status).c_str());
      throw std::runtime_error("ZED Camera open failed.");
    }

    // Ground plane parameters
    plane_parameters_.normal_similarity_threshold = 15;
    plane_parameters_.max_distance_threshold = 0.15f;

    // Positional tracking
    sl::PositionalTrackingParameters tracking_params;
    tracking_params.mode = sl::POSITIONAL_TRACKING_MODE::GEN_1;
    status = zed_.enablePositionalTracking(tracking_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "Failed to enable positional tracking: %s", sl::toString(status).c_str());
      zed_.close();
      throw std::runtime_error("Positional tracking initialization failed.");
    }

    pc_res_ = sl::Resolution(180, 101);
    // alloc() rather than assigning a temporary: sl::Mat copies are shallow.
    point_cloud_.alloc(pc_res_, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);

    running_ = true;
    worker_thread_ = std::thread(&ZedThreaded::zed_worker_loop, this);

    pointcloud_timer_ =
      rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {publish_pointcloud();});

    RCLCPP_INFO(get_logger(), "ZedAutonomy node has been started.");
  }

  ~ZedThreaded() override
  {
    running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    zed_.close();
  }

private:
  struct PoseData
  {
    sl::float3 translation;
    sl::float3 rotation;
  };

  void zed_worker_loop()
  {
    static const unsigned int hits[3][2] = {{600, 600}, {300, 600}, {1000, 600}};

    while (running_) {
      if (zed_.grab(runtime_params_) == sl::ERROR_CODE::SUCCESS) {
        // Use one timestamp for every product from this ZED grab so
        // downstream image/point-cloud synchronization is meaningful.
        const builtin_interfaces::msg::Time acquisition_stamp = now();

        // Retrieve outside the lock (only this thread touches the sl objects);
        // the snapshot swap below is what the publisher reads.
        zed_.retrieveMeasure(point_cloud_, sl::MEASURE::XYZ, sl::MEM::CPU, pc_res_);
        cv::Mat pc = sl_to_cv(point_cloud_).clone();

        const bool pose_ok =
          zed_.getPosition(current_pose_, sl::REFERENCE_FRAME::WORLD) ==
          sl::POSITIONAL_TRACKING_STATE::OK;
        // NOTE(port): Python stores a *reference* to self.current_pose once
        // tracking is OK, so later publishes see whatever the last
        // get_position() wrote even if that call was not OK. Mirrored by
        // refreshing the snapshot on every call after the first OK.
        PoseData pose{current_pose_.getTranslation(), current_pose_.getRotationVector()};

        std::optional<std::vector<sl::float3>> ground;
        for (const auto & hit : hits) {
          const sl::ERROR_CODE s =
            zed_.findPlaneAtHit(sl::uint2(hit[0], hit[1]), ground_plane_, plane_parameters_);
          if (s == sl::ERROR_CODE::SUCCESS && ground_plane_.type == sl::PLANE_TYPE::HORIZONTAL) {
            ground = ground_plane_.getBounds();
            break;
          }
        }

        zed_.retrieveImage(image_left_, sl::VIEW::LEFT);
        cv::Mat image = sl_to_cv(image_left_).clone();

        std::lock_guard<std::mutex> lk(pc_lock_);
        pc_data_ = std::move(pc);
        pc_stamp_ = acquisition_stamp;
        if (pose_ok) {
          have_pose_ = true;
          pose_stamp_ = acquisition_stamp;
        }
        if (have_pose_) {
          pose_data_ = pose;
        }
        if (ground) {
          ground_data_ = std::move(*ground);
          ground_stamp_ = acquisition_stamp;
          have_ground_ = true;
        }
        image_data_ = std::move(image);
        image_stamp_ = acquisition_stamp;
      }

      // Sleep to allow data access
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void publish_pointcloud()
  {
    cv::Mat pc_data, image_data;
    builtin_interfaces::msg::Time pc_stamp, ground_stamp, pose_stamp, image_stamp;
    std::vector<sl::float3> ground_data;
    PoseData pose_data{};
    bool have_ground = false, have_pose = false;
    {
      std::lock_guard<std::mutex> lk(pc_lock_);
      pc_data = pc_data_;  // refcounted header copy; worker swaps in new buffers
      pc_stamp = pc_stamp_;
      have_ground = have_ground_;
      if (have_ground) {
        ground_data = ground_data_;
        ground_stamp = ground_stamp_;
      }
      have_pose = have_pose_;
      pose_data = pose_data_;
      pose_stamp = pose_stamp_;
      image_data = image_data_;
      image_stamp = image_stamp_;
    }

    if (!pc_data.empty()) {
      pointcloud_publisher_->publish(cv_to_pointcloud2(pc_data, pc_stamp, "zed_camera_frame"));
    }
    if (have_ground) {
      publish_ground_plane(ground_data, ground_stamp);
    }
    if (have_pose) {
      publish_pose(pose_data, pose_stamp);
      publish_transform(pose_stamp);
    }
    if (!image_data.empty()) {
      auto image_msg = cv_to_image_msg(image_data, "bgra8");
      image_msg->header.stamp = image_stamp;
      image_msg->header.frame_id = "zed_camera_frame";
      image_publisher_->publish(std::move(image_msg));
    }
  }

  void publish_pose(const PoseData & pose, const builtin_interfaces::msg::Time & stamp)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "zed_camera_frame";
    msg->twist.linear.x = pose.translation.x;
    msg->twist.linear.y = pose.translation.y;
    msg->twist.linear.z = pose.translation.z;
    msg->twist.angular.x = pose.rotation.x;
    msg->twist.angular.y = pose.rotation.y;
    msg->twist.angular.z = pose.rotation.z;
    pose_publisher_->publish(std::move(msg));
  }

  void publish_ground_plane(
    const std::vector<sl::float3> & bounds, const builtin_interfaces::msg::Time & stamp)
  {
    auto msg = std::make_unique<cmr_msgs::msg::GroundPlaneStamped>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "zed_camera_frame";
    msg->x.reserve(bounds.size());
    msg->y.reserve(bounds.size());
    for (const auto & b : bounds) {
      msg->x.push_back(b.x);
      msg->y.push_back(b.y);
    }
    ground_publisher_->publish(std::move(msg));
  }

  void publish_transform(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "zed_camera_frame";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 1.1;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);
  }

  // ZED objects: touched only by the worker thread after construction.
  sl::Camera zed_;
  sl::RuntimeParameters runtime_params_;
  sl::PlaneDetectionParameters plane_parameters_;
  sl::Resolution pc_res_;
  sl::Mat point_cloud_;
  sl::Pose current_pose_;
  sl::Plane ground_plane_;
  sl::Mat image_left_;

  // Snapshot shared with the publish timer (Python's pc_lock).
  std::mutex pc_lock_;
  cv::Mat pc_data_;
  builtin_interfaces::msg::Time pc_stamp_;
  bool have_pose_{false};
  PoseData pose_data_{};
  builtin_interfaces::msg::Time pose_stamp_;
  bool have_ground_{false};
  std::vector<sl::float3> ground_data_;
  builtin_interfaces::msg::Time ground_stamp_;
  cv::Mat image_data_;
  builtin_interfaces::msg::Time image_stamp_;

  std::atomic<bool> running_{false};
  std::thread worker_thread_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<cmr_msgs::msg::GroundPlaneStamped>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr pointcloud_timer_;
};

}  // namespace cmr_zed_cpp
