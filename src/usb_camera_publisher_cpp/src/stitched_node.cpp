// Port of usb_camera_publisher/stitched.py (executable `stitched`).
//
// Captures 4 USB cameras and publishes one 2x2 mosaic on
// /camera/stitched_image at 10 Hz. Listens on /node_management/shutdown for
// the string "stitched_publisher".
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "usb_camera_publisher_cpp/image_msg.hpp"

namespace usb_camera_publisher_cpp
{

class MultiCameraStitchedPublisher : public rclcpp::Node
{
public:
  explicit MultiCameraStitchedPublisher(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("multi_camera_stitched_publisher", options)
  {
    // Configuration hard-coded in stitched.py main()
    camera_ids_ = {0, 2, 4, 6};
    width_ = 320;
    height_ = 240;
    image_format_ = "bgr8";
    publish_rate_ = 10;

    RCLCPP_INFO(get_logger(), "Initializing Multi-Camera Stitched Publisher...");
    name_ = "stitched_publisher";
    shutdown_listener_ = create_subscription<std_msgs::msg::String>(
      "/node_management/shutdown", 30,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {shutdown_callback(*msg);});

    camera_labels_ = {{0, "FRONT"}, {2, "LEFT"}, {4, "BACK"}, {6, "RIGHT"}};
    // (row, col) in the 2x2 grid
    camera_positions_ = {{0, {0, 0}}, {4, {0, 1}}, {2, {1, 0}}, {6, {1, 1}}};

    for (int cid : camera_ids_) {
      frames_[cid] = cv::Mat();  // None
    }

    for (int cid : camera_ids_) {
      auto cap = std::make_unique<cv::VideoCapture>(cid);
      cap->set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      cap->set(cv::CAP_PROP_FPS, publish_rate_);
      if (!cap->isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open /dev/video%d", cid);
      }
      caps_[cid] = std::move(cap);
    }
    for (int cid : camera_ids_) {
      threads_.emplace_back(&MultiCameraStitchedPublisher::capture_loop, this, cid);
    }

    stitched_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/stitched_image", 10);

    // Node clock (honours use_sim_time) like rclpy's create_timer.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_),
      [this]() {publish_stitched_image();});
  }

  ~MultiCameraStitchedPublisher() override {stop_all();}

private:
  void capture_loop(int cid)
  {
    cv::VideoCapture & cap = *caps_.at(cid);
    while (running_ && cap.isOpened()) {
      cv::Mat frame;  // fresh buffer: the publish timer may still hold the last one
      if (!cap.read(frame)) {
        {
          std::lock_guard<std::mutex> lk(frames_mutex_);
          frames_[cid] = cv::Mat();  // None => placeholder
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      {
        std::lock_guard<std::mutex> lk(frames_mutex_);
        frames_[cid] = frame;
      }
      // Avoid busy looping at max speed
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    cap.release();
    RCLCPP_INFO(get_logger(), "Camera %d capture loop stopped.", cid);
  }

  void publish_stitched_image()
  {
    std::map<int, cv::Mat> frames;
    {
      std::lock_guard<std::mutex> lk(frames_mutex_);
      frames = frames_;  // header copies only
    }

    const int width = width_, height = height_;
    // Draw straight into the outgoing message buffer.
    cv::Mat stitched;
    auto msg = make_image_msg(height * 2, width * 2, CV_8UC3, image_format_, stitched);
    stitched.setTo(cv::Scalar::all(0));

    cv::Mat resized;
    for (int cid : camera_ids_) {
      const auto [row, col] = camera_positions_.at(cid);
      const int y0 = row * height;
      const int x0 = col * width;
      cv::Mat subimage = stitched(cv::Rect(x0, y0, width, height));

      const cv::Mat & frame = frames[cid];
      if (!frame.empty()) {
        // Ensure the frame is the correct size, then rotate 180 deg into place
        cv::resize(frame, resized, cv::Size(width, height));
        cv::rotate(resized, subimage, cv::ROTATE_180);
      } else {
        // Placeholder (already black)
        cv::putText(
          subimage, "NO IMAGE AVAILABLE",
          cv::Point(static_cast<int>(width * 0.1), static_cast<int>(height * 0.5)),
          cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      }

      // Label (bright green, top-left)
      auto it = camera_labels_.find(cid);
      const std::string camera_label =
        it != camera_labels_.end() ? it->second : "Cam " + std::to_string(cid);
      cv::putText(
        subimage, camera_label, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // Black separator lines
    cv::line(stitched, cv::Point(width, 0), cv::Point(width, height * 2), cv::Scalar(0, 0, 0), 2);
    cv::line(stitched, cv::Point(0, height), cv::Point(width * 2, height), cv::Scalar(0, 0, 0), 2);

    msg->header.stamp = now();
    msg->header.frame_id = "stitched_image";
    stitched_pub_->publish(std::move(msg));
  }

  void stop_all()
  {
    running_ = false;
    for (auto & t : threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    threads_.clear();
    RCLCPP_INFO(get_logger(), "All camera capture threads stopped.");
  }

  void shutdown_callback(const std_msgs::msg::String & msg)
  {
    if (msg.data == name_) {
      RCLCPP_WARN(get_logger(), "Shutdown requested for %s. Exiting...", name_.c_str());
      stop_all();
      if (timer_) {
        timer_->cancel();
      }
      // NOTE(port): the Python node calls rclpy.shutdown(); this shuts down the
      // node's context, which in a component container stops the container too.
      rclcpp::shutdown(get_node_base_interface()->get_context());
    }
  }

  std::vector<int> camera_ids_;
  int width_{0};
  int height_{0};
  std::string image_format_;
  int publish_rate_{0};
  std::string name_;

  std::map<int, std::string> camera_labels_;
  std::map<int, std::pair<int, int>> camera_positions_;

  std::mutex frames_mutex_;
  std::map<int, cv::Mat> frames_;
  std::map<int, std::unique_ptr<cv::VideoCapture>> caps_;
  std::vector<std::thread> threads_;
  std::atomic<bool> running_{true};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shutdown_listener_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace usb_camera_publisher_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_publisher_cpp::MultiCameraStitchedPublisher)
