// Port of usb_camera_publisher/bev.py (executable `bev`).
//
// Four fisheye USB cameras -> undistort (cv::fisheye maps) -> rotate 180 deg
// -> publish /camera_<id>/image_rect, and a 10 Hz 300x300 bird's-eye mosaic
// on /birdseye/image_raw.
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "usb_camera_publisher_cpp/image_msg.hpp"

namespace usb_camera_publisher_cpp
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(double deg) {return deg * kPi / 180.0;}

// Returns a 3x3 rotation matrix Rz(yaw) * Ry(pitch) * Rx(roll), degrees in.
// float32 like the numpy original.
cv::Matx33f rotation_matrix_from_ypr(double yaw_deg, double pitch_deg, double roll_deg)
{
  const double yaw = radians(yaw_deg);
  const double pitch = radians(pitch_deg);
  const double roll = radians(roll_deg);

  const cv::Matx33f Rx(
    1.f, 0.f, 0.f,
    0.f, static_cast<float>(std::cos(roll)), static_cast<float>(-std::sin(roll)),
    0.f, static_cast<float>(std::sin(roll)), static_cast<float>(std::cos(roll)));
  const cv::Matx33f Ry(
    static_cast<float>(std::cos(pitch)), 0.f, static_cast<float>(std::sin(pitch)),
    0.f, 1.f, 0.f,
    static_cast<float>(-std::sin(pitch)), 0.f, static_cast<float>(std::cos(pitch)));
  const cv::Matx33f Rz(
    static_cast<float>(std::cos(yaw)), static_cast<float>(-std::sin(yaw)), 0.f,
    static_cast<float>(std::sin(yaw)), static_cast<float>(std::cos(yaw)), 0.f,
    0.f, 0.f, 1.f);
  return Rz * Ry * Rx;
}

// image -> ground(z=0) homography: inv(K * [r1 r2 t]).
cv::Matx33f compute_image_to_ground_homography(
  const cv::Matx33f & K, const cv::Matx33f & R, const cv::Vec3f & t)
{
  const cv::Matx33f Rt(
    R(0, 0), R(0, 1), t[0],
    R(1, 0), R(1, 1), t[1],
    R(2, 0), R(2, 1), t[2]);
  const cv::Matx33f H_ground_to_image = K * Rt;
  return H_ground_to_image.inv();
}

// ground metres -> mosaic pixels, origin at mosaic centre, +y -> +v.
cv::Matx33f compute_ground_to_mosaic_transform(double m_per_pix, int w, int h)
{
  const float pix_per_m = static_cast<float>(1.0 / m_per_pix);
  const float cx = static_cast<float>(w / 2.0);
  const float cy = static_cast<float>(h / 2.0);
  return cv::Matx33f(
    pix_per_m, 0.f, cx,
    0.f, pix_per_m, cy,
    0.f, 0.f, 1.f);
}

struct CameraPose
{
  cv::Vec3f translation;
  double yaw;
  double pitch;
  double roll;
};

}  // namespace

class BirdseyePublisher : public rclcpp::Node
{
public:
  explicit BirdseyePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("birdseye_publisher", options)
  {
    // === Camera intrinsics & distortion (example) ===
    // NOTE(port): principal point (1239, 576) is far outside the 640x480 frame
    // the maps are built for; kept verbatim from bev.py.
    K_ = cv::Matx33f(
      1102.3128f, 0.0f, 1239.31013f,
      0.0f, 880.524776f, 576.711558f,
      0.0f, 0.0f, 1.0f);
    D_ = cv::Vec4f(-0.19872862f, 1.00295506f, -2.21112719f, 1.71318476f);

    resolution_ = cv::Size(640, 480);
    publish_rate_ = 10;

    camera_ids_ = {0, 2, 4, 6};  // front, left, back, right

    for (int cam_id : camera_ids_) {
      rect_publishers_[cam_id] = create_publisher<sensor_msgs::msg::Image>(
        "/camera_" + std::to_string(cam_id) + "/image_rect", 10);
    }

    H_matrices_ = compute_all_homographies();

    for (int cam_id : camera_ids_) {
      capture_threads_.emplace_back(&BirdseyePublisher::capture_loop, this, cam_id);
    }

    birdseye_pub_ = create_publisher<sensor_msgs::msg::Image>("/birdseye/image_raw", 10);

    // Node clock (honours use_sim_time) like rclpy's create_timer.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_),
      [this]() {publish_birdseye();});

    RCLCPP_INFO(get_logger(), "BirdseyePublisher initialized.");
  }

  ~BirdseyePublisher() override {stop_all();}

private:
  void capture_loop(int cam_id)
  {
    cv::VideoCapture cap(cam_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, resolution_.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, resolution_.height);
    cap.set(cv::CAP_PROP_FPS, publish_rate_);

    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/video%d", cam_id);
      return;
    }

    const auto rate = std::chrono::duration<double>(1.0 / publish_rate_);
    // NOTE(port): bev.py also builds a pinhole cv2.initUndistortRectifyMap()
    // (mapx, mapy) that is never used; only the fisheye maps are built here.
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(
      K_, D_, cv::Matx33f::eye(), K_, resolution_, CV_16SC2, map1, map2);

    const std::string frame_id = "camera_" + std::to_string(cam_id);
    auto & pub = rect_publishers_.at(cam_id);
    cv::Mat frame, undistorted;

    while (running_ && cap.isOpened()) {
      if (!cap.read(frame)) {
        RCLCPP_WARN(get_logger(), "Camera %d: failed to read frame.", cam_id);
        std::this_thread::sleep_for(rate);
        continue;
      }

      // Undistort
      cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

      // Rotate image 180 degrees (fresh buffer each frame: the mosaic timer
      // keeps a reference to it, so it must not be overwritten in place).
      cv::Mat rotated;
      cv::rotate(undistorted, rotated, cv::ROTATE_180);

      // Store for stitching
      {
        std::lock_guard<std::mutex> lk(frames_mutex_);
        if (frames_.find(cam_id) == frames_.end()) {
          frame_order_.push_back(cam_id);  // python dict insertion order
        }
        frames_[cam_id] = rotated;
      }

      // Publish the rectified image
      try {
        pub->publish(to_image_msg(rotated, "bgr8", now(), frame_id));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "Error publishing rectified image from camera %d: %s", cam_id, e.what());
      }

      std::this_thread::sleep_for(rate);
    }

    cap.release();
    RCLCPP_INFO(get_logger(), "Camera %d: capture loop stopped.", cam_id);
  }

  std::map<int, cv::Matx33f> compute_all_homographies() const
  {
    // 3x3 metre area => 300x300 pixels => 0.01 m/pixel
    const double m_per_pix = 0.01;
    const cv::Matx33f M_ground_to_mosaic = compute_ground_to_mosaic_transform(m_per_pix, 300, 300);

    const std::map<int, CameraPose> camera_poses = {
      {0, {cv::Vec3f(0.35f, 0.0f, 0.42f), 0.0, -11.0, 0.0}},    // front
      {2, {cv::Vec3f(0.0f, 0.35f, 0.42f), 90.0, -11.0, 0.0}},   // left
      {4, {cv::Vec3f(-0.35f, 0.0f, 0.42f), 180.0, -11.0, 0.0}}, // back
      {6, {cv::Vec3f(0.0f, -0.35f, 0.42f), -90.0, -11.0, 0.0}}, // right
    };

    std::map<int, cv::Matx33f> H;
    for (const auto & [cam_id, pose] : camera_poses) {
      const cv::Matx33f R_cam = rotation_matrix_from_ypr(pose.yaw, pose.pitch, pose.roll);
      const cv::Matx33f H_img_to_ground =
        compute_image_to_ground_homography(K_, R_cam, pose.translation);
      H[cam_id] = M_ground_to_mosaic * H_img_to_ground;
    }
    return H;
  }

  void publish_birdseye()
  {
    std::vector<std::pair<int, cv::Mat>> frames;
    {
      std::lock_guard<std::mutex> lk(frames_mutex_);
      // We need frames from all cameras
      if (frames_.size() < camera_ids_.size()) {
        return;
      }
      for (int cid : frame_order_) {
        frames.emplace_back(cid, frames_[cid]);
      }
    }

    const int mosaic_w = 300, mosaic_h = 300;
    cv::Mat mosaic;
    auto msg = make_image_msg(mosaic_h, mosaic_w, CV_8UC3, "bgr8", mosaic);
    mosaic.setTo(cv::Scalar(255, 255, 255));

    // Black square for the robot: 70 cm => 70 px at 0.01 m/px
    // NOTE(port): 0.70 / 0.01 == 69.999... in IEEE double, so int() gives 69
    // (not the 70 the Python comment claims); same truncation here.
    const int robot_size_px = static_cast<int>(0.70 / 0.01);
    const int center_x = mosaic_w / 2, center_y = mosaic_h / 2;
    cv::rectangle(
      mosaic,
      cv::Point(center_x - robot_size_px / 2, center_y - robot_size_px / 2),
      cv::Point(center_x + robot_size_px / 2, center_y + robot_size_px / 2),
      cv::Scalar(0, 0, 0), cv::FILLED);

    cv::Mat warped, all_bright, mask;
    for (const auto & [cam_id, frame] : frames) {
      cv::warpPerspective(
        frame, warped, cv::Mat(H_matrices_.at(cam_id)), cv::Size(mosaic_w, mosaic_h),
        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
      // mask = np.any(warped < 250, axis=-1)  ==  NOT(all channels >= 250)
      cv::inRange(warped, cv::Scalar(250, 250, 250), cv::Scalar(255, 255, 255), all_bright);
      cv::bitwise_not(all_bright, mask);
      warped.copyTo(mosaic, mask);
    }

    msg->header.stamp = now();
    msg->header.frame_id = "birdseye";
    birdseye_pub_->publish(std::move(msg));
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
    RCLCPP_INFO(get_logger(), "All capture threads stopped.");
  }

  cv::Matx33f K_;
  cv::Vec4f D_;
  cv::Size resolution_;
  int publish_rate_{0};
  std::vector<int> camera_ids_;

  std::map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rect_publishers_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr birdseye_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<int, cv::Matx33f> H_matrices_;

  std::mutex frames_mutex_;
  std::map<int, cv::Mat> frames_;
  std::vector<int> frame_order_;

  std::atomic<bool> running_{true};
  std::vector<std::thread> capture_threads_;
};

}  // namespace usb_camera_publisher_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_publisher_cpp::BirdseyePublisher)
