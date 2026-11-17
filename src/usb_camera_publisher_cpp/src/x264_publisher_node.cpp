// Port of usb_camera_publisher/publisher.py (executable `publisher`).
//
// One GStreamer pipeline per USB camera:
//   v4l2src -> MJPEG 640x480 -> jpegdec -> [videocrop] -> x264enc -> h264parse -> appsink
// Each H.264 access unit is published as foxglove_msgs/CompressedVideo on
// camera_<id>/h264 (relative name, as in Python) followed by a CameraInfo on
// camera_<id>/camera_info built from <calib_dir>/camera<id>_calib.yaml.
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <foxglove_msgs/msg/compressed_video.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace usb_camera_publisher_cpp
{

namespace
{
bool file_exists(const std::string & path)
{
  std::FILE * f = std::fopen(path.c_str(), "r");
  if (f) {
    std::fclose(f);
    return true;
  }
  return false;
}

// calib.get('crop', {}).get(side, 0)
int crop_value(const YAML::Node & crop, const char * side)
{
  if (crop && crop.IsMap() && crop[side]) {
    return crop[side].as<int>();
  }
  return 0;
}
}  // namespace

class X264MultiCameraPublisher : public rclcpp::Node
{
public:
  explicit X264MultiCameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("x264_multi_camera_publisher", options)
  {
    // Hard-coded in publisher.py main()
    camera_ids_ = {0, 2, 4, 6, 8, 10};
    calib_dir_ = "/home/cmr/cmr/terra2/calib_dir";

    RCLCPP_INFO(get_logger(), "Initializing Multi-Camera Publisher with manual calibration…");

    for (int cam_id : camera_ids_) {
      auto & cam = cameras_[cam_id];
      cam.node = this;
      cam.cam_id = cam_id;
      cam.frame_id = "camera_" + std::to_string(cam_id);
      cam.video_pub = create_publisher<foxglove_msgs::msg::CompressedVideo>(
        "camera_" + std::to_string(cam_id) + "/h264", 10);
      cam.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_" + std::to_string(cam_id) + "/camera_info", 10);

      const std::string yaml_path = calib_dir_ + "/camera" + std::to_string(cam_id) + "_calib.yaml";
      if (!file_exists(yaml_path)) {
        RCLCPP_ERROR(get_logger(), "Calibration file not found: %s", yaml_path.c_str());
        continue;
      }

      YAML::Node calib;
      try {
        calib = YAML::LoadFile(yaml_path);
      } catch (const std::exception & e) {
        // NOTE(port): yaml.safe_load would raise and kill the Python node here.
        RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s", yaml_path.c_str(), e.what());
        continue;
      }

      // verify required keys (Python only logs the first missing one and
      // carries on anyway)
      for (const char * key : {"image_width", "image_height", "camera_matrix",
          "distortion_coefficients", "rectification_matrix", "projection_matrix"})
      {
        if (!calib[key]) {
          RCLCPP_ERROR(get_logger(), "%s missing in %s", key, yaml_path.c_str());
          break;
        }
      }

      const YAML::Node crop = calib["crop"];
      cam.crop_left = crop_value(crop, "left");
      cam.crop_right = crop_value(crop, "right");
      cam.crop_top = crop_value(crop, "top");
      cam.crop_bottom = crop_value(crop, "bottom");

      // Pre-build the (per-frame constant) CameraInfo once instead of
      // re-reading the YAML tree for every H.264 frame.
      try {
        build_camera_info(calib, cam);
        cam.info_ok = true;
      } catch (const std::exception & e) {
        // NOTE(port): in Python the KeyError surfaces inside every new-sample
        // callback (after the h264 frame was published), so camera_info is never
        // published for that camera. Same observable result here.
        RCLCPP_ERROR(
          get_logger(), "[cam%d] invalid calibration, camera_info disabled: %s", cam_id, e.what());
      }
      cam.has_calib = true;
    }

    gst_init(nullptr, nullptr);

    for (int cam_id : camera_ids_) {
      if (cameras_[cam_id].has_calib) {
        start_camera_pipeline(cameras_[cam_id]);
      }
    }

    RCLCPP_INFO(get_logger(), "All camera pipelines initialized.");
  }

  ~X264MultiCameraPublisher() override
  {
    RCLCPP_INFO(get_logger(), "Stopping pipelines…");
    for (auto & [id, cam] : cameras_) {
      (void)id;
      if (cam.pipeline) {
        gst_element_set_state(cam.pipeline, GST_STATE_NULL);
        gst_object_unref(cam.pipeline);
        cam.pipeline = nullptr;
      }
    }
    RCLCPP_INFO(get_logger(), "Stopped.");
  }

private:
  struct Camera
  {
    X264MultiCameraPublisher * node{nullptr};
    int cam_id{0};
    std::string frame_id;
    bool has_calib{false};
    bool info_ok{false};
    int crop_left{0}, crop_right{0}, crop_top{0}, crop_bottom{0};
    sensor_msgs::msg::CameraInfo info_template;
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr video_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    GstElement * pipeline{nullptr};
  };

  static std::vector<double> data_of(const YAML::Node & n)
  {
    return n["data"].as<std::vector<double>>();
  }

  template<size_t N>
  static void copy_fixed(const std::vector<double> & src, std::array<double, N> & dst)
  {
    if (src.size() != N) {
      throw std::runtime_error(
              "expected " + std::to_string(N) + " values, got " + std::to_string(src.size()));
    }
    std::copy(src.begin(), src.end(), dst.begin());
  }

  static void build_camera_info(const YAML::Node & calib, Camera & cam)
  {
    auto & info = cam.info_template;
    info.header.frame_id = cam.frame_id;
    const int orig_w = calib["image_width"].as<int>();
    const int orig_h = calib["image_height"].as<int>();
    info.width = static_cast<uint32_t>(orig_w - cam.crop_left - cam.crop_right);
    info.height = static_cast<uint32_t>(orig_h - cam.crop_top - cam.crop_bottom);
    info.distortion_model =
      calib["distortion_model"] ? calib["distortion_model"].as<std::string>() : "plumb_bob";
    info.d = data_of(calib["distortion_coefficients"]);
    copy_fixed(data_of(calib["camera_matrix"]), info.k);
    copy_fixed(data_of(calib["rectification_matrix"]), info.r);
    copy_fixed(data_of(calib["projection_matrix"]), info.p);
    // no binning, no ROI so Foxglove sees a full-frame model
    info.binning_x = 0;
    info.binning_y = 0;
    info.roi.x_offset = 0;
    info.roi.y_offset = 0;
    info.roi.width = 0;
    info.roi.height = 0;
    info.roi.do_rectify = false;
  }

  void start_camera_pipeline(Camera & cam)
  {
    const std::string appsink_name = "appsink_cam" + std::to_string(cam.cam_id);

    std::string crop_args;
    const std::pair<const char *, int> sides[] = {
      {"left", cam.crop_left}, {"right", cam.crop_right},
      {"top", cam.crop_top}, {"bottom", cam.crop_bottom}};
    for (const auto & [side, val] : sides) {
      if (val) {
        crop_args += (crop_args.empty() ? "" : " ") + std::string(side) + "=" + std::to_string(val);
      }
    }

    // NOTE(port): publisher.py always emits "videoconvert ! {videocrop_str} ! ".
    // All calib_dir/*.yaml files have a crop: section, so that is fine in
    // practice, but a YAML without crop (or with all-zero crop) would produce
    // "videoconvert !  ! videoconvert", which Gst.parse_launch rejects with
    // "syntax error" (verified with gst-launch-1.0, GStreamer 1.20) and the
    // Python node would crash. The empty link is dropped here; otherwise the
    // pipeline string is identical.
    const std::string videocrop_str = crop_args.empty() ? "" : "videocrop " + crop_args + " ! ";

    const std::string pipeline_str =
      "v4l2src device=/dev/video" + std::to_string(cam.cam_id) + " ! "
      "image/jpeg, width=640, height=480 ! "
      "jpegdec ! videoconvert ! " +
      videocrop_str +
      "videoconvert ! "
      "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
      "h264parse config-interval=1 ! "
      "capsfilter caps=\"video/x-h264,stream-format=(string)byte-stream\" ! "
      "appsink name=" + appsink_name + " emit-signals=true sync=false max-buffers=1 drop=true";

    RCLCPP_INFO(get_logger(), "[cam%d] %s", cam.cam_id, pipeline_str.c_str());

    GError * err = nullptr;
    GstElement * pipe = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err != nullptr || pipe == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "[cam%d] parse_launch failed%s%s", cam.cam_id,
        err ? ": " : "", err ? err->message : "");
      if (err) {
        g_error_free(err);
      }
      if (pipe) {
        gst_object_unref(pipe);
      }
      return;
    }

    GstElement * appsink = gst_bin_get_by_name(GST_BIN(pipe), appsink_name.c_str());
    // Native appsink callbacks instead of the "new-sample" signal: same
    // semantics, invoked on the streaming thread, without GLib signal
    // marshalling. No GLib main loop is needed for appsink delivery (the
    // Python per-camera MainLoops only dispatched the shared default context).
    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = &X264MultiCameraPublisher::on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks, &cam, nullptr);
    gst_object_unref(appsink);

    gst_element_set_state(pipe, GST_STATE_PLAYING);
    cam.pipeline = pipe;
  }

  static GstFlowReturn on_new_sample(GstAppSink * appsink, gpointer user_data)
  {
    auto * cam = static_cast<Camera *>(user_data);
    GstSample * sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
      return GST_FLOW_ERROR;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstMapInfo mi;
    if (!buf || !gst_buffer_map(buf, &mi, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    // publish the (cropped) H.264 frame
    const builtin_interfaces::msg::Time now = cam->node->now();
    auto vid = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
    vid->timestamp = now;
    vid->format = "h264";
    vid->frame_id = cam->frame_id;
    vid->data.assign(mi.data, mi.data + mi.size);
    gst_buffer_unmap(buf, &mi);
    gst_sample_unref(sample);
    cam->video_pub->publish(std::move(vid));

    // publish adjusted CameraInfo
    if (cam->info_ok) {
      auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(cam->info_template);
      info->header.stamp = now;
      cam->info_pub->publish(std::move(info));
    }
    return GST_FLOW_OK;
  }

  std::vector<int> camera_ids_;
  std::string calib_dir_;
  // std::map nodes are address-stable: the GStreamer callbacks hold Camera*.
  std::map<int, Camera> cameras_;
};

}  // namespace usb_camera_publisher_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_publisher_cpp::X264MultiCameraPublisher)
