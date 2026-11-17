// Port of cmr_zed/zed_camera_publisher.py (executable `zed_publisher_node`).
//
// 10 Hz: grab -> publish full-res left/right BGRA images on /zed/image_left,
// /zed/image_right. On request, answers plane equations (/zed/plane/pixel ->
// /zed/plane/equation) and 3-D points (/zed/point/pixel -> /zed/point/coordinate).
#pragma once
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "cmr_zed_cpp/zed_conversions.hpp"

namespace cmr_zed_cpp
{

using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::Int32MultiArray;

class ZEDCameraPublisher : public rclcpp::Node
{
public:
  explicit ZEDCameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_camera_publisher", options)
  {
    publisher_left_ = create_publisher<sensor_msgs::msg::Image>("/zed/image_left", 10);
    publisher_right_ = create_publisher<sensor_msgs::msg::Image>("/zed/image_right", 10);
    plane_publisher_ = create_publisher<Float32MultiArray>("/zed/plane/equation", 10);
    plane_request_ = create_subscription<Int32MultiArray>(
      "/zed/plane/pixel", 10,
      [this](Int32MultiArray::ConstSharedPtr msg) {plane_request_callback(*msg);});
    coordinate_publisher_ = create_publisher<Float32MultiArray>("/zed/point/coordinate", 10);
    coordinate_request_ = create_subscription<Int32MultiArray>(
      "/zed/point/pixel", 10,
      [this](Int32MultiArray::ConstSharedPtr msg) {point_request_callback(*msg);});

    // Initialize ZED camera
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.camera_fps = 15;

    if (zed_.open(init_params) != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to open ZED camera. Exiting.");
      // NOTE(port): Python calls rclpy.shutdown() and keeps going until
      // enable_positional_tracking fails and exit() kills the process. A
      // component cannot exit() its container, so fail construction instead.
      throw std::runtime_error("ZED Camera open failed.");
    }

    // Positional tracking (default params; Python passes an identity
    // sl.Transform as _init_pos, which is the default initial_world_transform).
    sl::PositionalTrackingParameters tracking_parameters;
    sl::ERROR_CODE err = zed_.enablePositionalTracking(tracking_parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "Enable positional tracking : %s. Exit program.", sl::toString(err).c_str());
      zed_.close();
      throw std::runtime_error("Positional tracking initialization failed.");
    }

    // Spatial mapping (MESH)
    sl::SpatialMappingParameters mapping_parameters;
    mapping_parameters.map_type = sl::SPATIAL_MAP_TYPE::MESH;
    err = zed_.enableSpatialMapping(mapping_parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "Enable spatial mapping : %s. Exit program.", sl::toString(err).c_str());
      zed_.close();
      throw std::runtime_error("Spatial mapping initialization failed.");
    }

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.100), [this]() {publish_images();});
  }

  ~ZEDCameraPublisher() override
  {
    std::lock_guard<std::mutex> lk(zed_mutex_);
    zed_.close();
  }

private:
  void publish_images()
  {
    std::lock_guard<std::mutex> lk(zed_mutex_);
    if (zed_.grab(runtime_params_) == sl::ERROR_CODE::SUCCESS) {
      zed_.retrieveImage(image_left_, sl::VIEW::LEFT);
      zed_.retrieveImage(image_right_, sl::VIEW::RIGHT);

      // NOTE(port): like the Python node, header (stamp/frame_id) is left empty.
      publisher_left_->publish(cv_to_image_msg(sl_to_cv(image_left_), "bgra8"));
      publisher_right_->publish(cv_to_image_msg(sl_to_cv(image_right_), "bgra8"));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to grab images from ZED camera.");
    }
  }

  void plane_request_callback(const Int32MultiArray & msg)
  {
    std::lock_guard<std::mutex> lk(zed_mutex_);
    std::vector<float> message_data;
    // NOTE(port): Python leaves tracking_state unbound if the very first
    // grab() fails (UnboundLocalError) and otherwise reuses the previous
    // iteration's value. Same reuse here, starting from OFF instead of crashing.
    sl::POSITIONAL_TRACKING_STATE tracking_state = sl::POSITIONAL_TRACKING_STATE::OFF;

    for (size_t i = 0; i < msg.data.size(); i += 3) {
      if (i + 2 >= msg.data.size()) {
        break;  // NOTE(port): Python raises IndexError on a trailing partial triple.
      }
      const float index = static_cast<float>(msg.data[i]);  // Aruco marker index
      const sl::uint2 coord(
        static_cast<unsigned int>(msg.data[i + 1]), static_cast<unsigned int>(msg.data[i + 2]));

      sl::Plane plane;
      sl::Pose pose;
      sl::PlaneDetectionParameters plane_parameters;

      if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
        tracking_state = zed_.getPosition(pose);
      }

      if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
        if (zed_.findPlaneAtHit(coord, plane, plane_parameters) == sl::ERROR_CODE::SUCCESS) {
          // (Python also computes roll/pitch/yaw from the normal but never uses them.)
          const sl::float4 eq = plane.getPlaneEquation();  // (a,b,c,d)
          message_data.push_back(index);
          message_data.insert(message_data.end(), {eq.x, eq.y, eq.z, eq.w});
        }
      }
    }

    if (!message_data.empty()) {
      auto out = std::make_unique<Float32MultiArray>();
      out->data = std::move(message_data);
      plane_publisher_->publish(std::move(out));
    }
  }

  void point_request_callback(const Int32MultiArray & msg)
  {
    std::lock_guard<std::mutex> lk(zed_mutex_);
    std::vector<float> message_data;

    for (size_t i = 0; i < msg.data.size(); i += 3) {
      if (i + 2 >= msg.data.size()) {
        break;  // NOTE(port): Python raises IndexError on a trailing partial triple.
      }
      const float index = static_cast<float>(msg.data[i]);  // Aruco marker index

      if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
        zed_.retrieveMeasure(point_cloud_, sl::MEASURE::XYZ);
        // NOTE(port): Python ignores get_value's error code (its len(point[1]) < 3
        // check can never trigger for an F32_C4 cloud); out-of-range pixels give NaN here.
        sl::float4 point;
        point.x = point.y = point.z = point.w = std::numeric_limits<float>::quiet_NaN();
        point_cloud_.getValue(
          static_cast<size_t>(msg.data[i + 1]), static_cast<size_t>(msg.data[i + 2]), &point);
        RCLCPP_INFO(get_logger(), "X=%.3f m, Y=%.3f m, Z=%.3f m", point.x, point.y, point.z);
        message_data.push_back(index);
        message_data.insert(message_data.end(), {point.x, point.y, point.z});
      }
    }

    if (!message_data.empty()) {
      auto out = std::make_unique<Float32MultiArray>();
      out->data = std::move(message_data);
      coordinate_publisher_->publish(std::move(out));
    }
  }

  // Callbacks may run concurrently in a multi-threaded component container.
  std::mutex zed_mutex_;
  sl::Camera zed_;
  sl::RuntimeParameters runtime_params_;
  sl::Mat image_left_;
  sl::Mat image_right_;
  sl::Mat point_cloud_;  // reused instead of a fresh sl.Mat() per request

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr plane_publisher_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr coordinate_publisher_;
  rclcpp::Subscription<Int32MultiArray>::SharedPtr plane_request_;
  rclcpp::Subscription<Int32MultiArray>::SharedPtr coordinate_request_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_zed_cpp
