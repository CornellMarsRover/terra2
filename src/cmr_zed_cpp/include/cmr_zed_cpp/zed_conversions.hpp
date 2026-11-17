// sl::Mat <-> cv::Mat / ROS message helpers shared by the cmr_zed_cpp nodes.
#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sl/Camera.hpp>

namespace cmr_zed_cpp
{

inline int cv_type_of(sl::MAT_TYPE t)
{
  switch (t) {
    case sl::MAT_TYPE::F32_C1: return CV_32FC1;
    case sl::MAT_TYPE::F32_C2: return CV_32FC2;
    case sl::MAT_TYPE::F32_C3: return CV_32FC3;
    case sl::MAT_TYPE::F32_C4: return CV_32FC4;
    case sl::MAT_TYPE::U8_C1: return CV_8UC1;
    case sl::MAT_TYPE::U8_C2: return CV_8UC2;
    case sl::MAT_TYPE::U8_C3: return CV_8UC3;
    case sl::MAT_TYPE::U8_C4: return CV_8UC4;
    default: break;
  }
  throw std::runtime_error("unsupported sl::MAT_TYPE");
}

// Zero-copy cv::Mat view of a CPU sl::Mat (valid while the sl::Mat is alive
// and not re-allocated by the next retrieve*).
inline cv::Mat sl_to_cv(const sl::Mat & m)
{
  return cv::Mat(
    static_cast<int>(m.getHeight()), static_cast<int>(m.getWidth()),
    cv_type_of(m.getDataType()), m.getPtr<sl::uchar1>(sl::MEM::CPU),
    m.getStepBytes(sl::MEM::CPU));
}

// cv2_to_imgmsg(mat, encoding): one copy, straight into the message buffer.
// `header` is left default (empty stamp / frame_id) unless the caller sets it.
inline sensor_msgs::msg::Image::UniquePtr cv_to_image_msg(
  const cv::Mat & src, const std::string & encoding)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->height = static_cast<uint32_t>(src.rows);
  msg->width = static_cast<uint32_t>(src.cols);
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(src.cols * src.elemSize());
  msg->data.resize(static_cast<size_t>(msg->height) * msg->step);
  cv::Mat dst(src.rows, src.cols, src.type(), msg->data.data(), msg->step);
  src.copyTo(dst);
  return msg;
}

// Python convert_sl_mat_to_pointcloud2 / convert_array_to_pointcloud2:
// F32_C4 organised cloud, fields x,y,z,rgb (FLOAT32), point_step 16,
// is_dense False, data = tobytes() (tightly packed rows).
inline sensor_msgs::msg::PointCloud2::UniquePtr cv_to_pointcloud2(
  const cv::Mat & pc, const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  if (pc.type() != CV_32FC4) {
    throw std::runtime_error("Expecting 4 channels (X, Y, Z, RGBA).");
  }
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = static_cast<uint32_t>(pc.rows);
  msg->width = static_cast<uint32_t>(pc.cols);
  msg->is_bigendian = false;
  msg->point_step = 16;
  msg->row_step = msg->point_step * msg->width;
  msg->is_dense = false;

  const char * names[] = {"x", "y", "z", "rgb"};
  for (uint32_t i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg->fields.push_back(f);
  }

  msg->data.resize(static_cast<size_t>(msg->row_step) * msg->height);
  cv::Mat dst(pc.rows, pc.cols, CV_32FC4, msg->data.data(), msg->row_step);
  pc.copyTo(dst);
  return msg;
}

}  // namespace cmr_zed_cpp
