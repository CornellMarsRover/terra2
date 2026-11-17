// Port of usb_camera_publisher/publish_images.py (executable `image_publisher`).
//
// One capture thread per /dev/videoX; each publishes raw frames on
// /camera_<id>/image_raw (sensor_msgs/Image, bgr8, depth 10).
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "usb_camera_publisher_cpp/image_msg.hpp"

namespace usb_camera_publisher_cpp
{

class MultiCameraImagePublisher : public rclcpp::Node
{
public:
  explicit MultiCameraImagePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("multi_camera_image_publisher", options)
  {
    RCLCPP_INFO(get_logger(), "Initializing Multi-Camera Image Publisher...");

    // === Configuration === (hard-coded in publish_images.py main())
    camera_ids_ = {0, 2, 4, 6};   // /dev/videoX devices
    width_ = 320;                 // resolution = (320, 240)
    height_ = 240;
    image_format_ = "bgr8";
    publish_rate_ = 10;           // Hz

    for (int cam_id : camera_ids_) {
      const std::string topic_name = "/camera_" + std::to_string(cam_id) + "/image_raw";
      cam_publishers_[cam_id] = create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    }
    // Threads are started after every publisher exists so the map is never
    // mutated while a capture thread reads it.
    for (int cam_id : camera_ids_) {
      capture_threads_.emplace_back(&MultiCameraImagePublisher::capture_loop, this, cam_id);
    }
  }

  ~MultiCameraImagePublisher() override {stop_all();}

private:
  void capture_loop(int cam_id)
  {
    cv::VideoCapture cap(cam_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap.set(cv::CAP_PROP_FPS, publish_rate_);

    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/video%d", cam_id);
      return;
    }
    RCLCPP_INFO(get_logger(), "Started capture thread for camera %d", cam_id);

    const auto rate = std::chrono::duration<double>(1.0 / publish_rate_);
    const std::string frame_id = "camera_" + std::to_string(cam_id);
    auto & pub = cam_publishers_.at(cam_id);
    cv::Mat frame;

    while (running_ && cap.isOpened()) {
      if (!cap.read(frame)) {
        RCLCPP_WARN(get_logger(), "Camera %d: Failed to read frame.", cam_id);
        std::this_thread::sleep_for(rate);
        continue;
      }
      try {
        pub->publish(to_image_msg(frame, image_format_, now(), frame_id));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Camera %d: Error publishing frame - %s", cam_id, e.what());
      }
      std::this_thread::sleep_for(rate);
    }

    cap.release();
    RCLCPP_INFO(get_logger(), "Camera %d: Video capture stopped.", cam_id);
  }

  void stop_all()
  {
    running_ = false;
    for (auto & t : capture_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    capture_threads_.clear();
    RCLCPP_INFO(get_logger(), "All camera capture threads stopped.");
  }

  std::vector<int> camera_ids_;
  int width_{0};
  int height_{0};
  std::string image_format_;
  int publish_rate_{0};

  std::atomic<bool> running_{true};
  std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> cam_publishers_;
  std::vector<std::thread> capture_threads_;
};

}  // namespace usb_camera_publisher_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_publisher_cpp::MultiCameraImagePublisher)
