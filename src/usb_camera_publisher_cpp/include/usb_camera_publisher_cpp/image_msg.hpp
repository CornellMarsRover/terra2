// Small helpers shared by the usb_camera_publisher_cpp nodes.
#pragma once

#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace usb_camera_publisher_cpp
{

// Equivalent of `CvBridge().cv2_to_imgmsg(mat, encoding)` + header assignment,
// but filling a std::unique_ptr message so rclcpp can hand it over
// intra-process without another copy.
inline sensor_msgs::msg::Image::UniquePtr to_image_msg(
  const cv::Mat & mat, const std::string & encoding,
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id;
  cv_bridge::CvImage(header, encoding, mat).toImageMsg(*msg);
  return msg;
}

// Allocate an Image message of the given geometry and return a cv::Mat that
// aliases its data buffer, so OpenCV can draw/write straight into the message.
inline sensor_msgs::msg::Image::UniquePtr make_image_msg(
  int rows, int cols, int cv_type, const std::string & encoding, cv::Mat & view)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  const size_t elem = CV_ELEM_SIZE(cv_type);
  msg->height = static_cast<uint32_t>(rows);
  msg->width = static_cast<uint32_t>(cols);
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(cols * elem);
  msg->data.resize(static_cast<size_t>(rows) * msg->step);
  view = cv::Mat(rows, cols, cv_type, msg->data.data(), msg->step);
  return msg;
}

}  // namespace usb_camera_publisher_cpp
