#include "cmr_cv/camera_node.hpp"

#include <opencv2/imgproc.hpp>

#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/external/tomlcpp.hxx"

using namespace std::chrono_literals;
namespace cmr
{

CameraNode::CameraNode(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
}

// NOLINTNEXTLINE(readability-function-size)
bool CameraNode::configure_settings(const std::unique_ptr<toml::Table> settings)
{
    const auto [frame_id_ok, frame_id] = settings->getString("frame_id");
    if (!frame_id_ok) {
        CMR_LOG(
            WARN,
            "No value configured for 'frame_id'; using default value of 'camera'.");
        m_frame_id = "camera";
    } else {
        m_frame_id = frame_id;
    }

    const auto [width_ok, width] = settings->getInt("image_width");
    if (!width_ok) {
        CMR_LOG(
            WARN,
            "No value configured for 'image_width'; using default value of 320.");
        m_image_width = 320;
    } else {
        m_image_width = static_cast<int>(width);
    }

    const auto [height_ok, height] = settings->getInt("image_height");
    if (!width_ok) {
        CMR_LOG(
            WARN,
            "No value configured for 'image_height'; using default value of 240.");
        m_image_height = 240;
    } else {
        m_image_height = static_cast<int>(height);
    }

    const auto [fps_ok, fps] = settings->getDouble("fps");
    if (!fps_ok) {
        CMR_LOG(WARN, "No value configured for 'fps'; using default value of 10.0.");
        m_fps = 10.0;
    } else {
        m_fps = fps;
    }

    const auto [id_ok, cam_id] = settings->getString("camera_id");
    if (!id_ok) {
        CMR_LOG(WARN,
                "No value configured for 'camera_id'; using default value of '0'.");
        m_camera_id = "0";
    } else {
        m_camera_id = cam_id;
    }

    return true;
}

void CameraNode::initialize_publishers()
{
    // create publishers with QoS configured for sensor data
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
        qos_profile);

    m_camera_info_pub =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);

    m_image_pub =
        this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
}

bool CameraNode::configure(const std::shared_ptr<toml::Table>& table)
{
    if (!configure_settings(table->getTable("node"))) {
        return false;
    }

    initialize_publishers();

    m_cam_info_manager = std::make_shared<cmr_cv::CameraInfoManager>(this);
    m_cam_info_manager->load_camera_info(
        "file:///cmr/terra/src/cmr_cv/config/cameras_info.yaml");

    return true;
}

bool CameraNode::activate()
{
    m_camera_info_pub->on_activate();
    m_image_pub->on_activate();

    m_cap.open(m_camera_id);
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, m_image_width);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_image_height);
    // use MJPG format for capture format
    m_cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    m_last_frame = std::chrono::steady_clock::now();

    m_timer = this->create_wall_timer((1 / m_fps) * 1000ms,
                                      std::bind(&CameraNode::image_callback, this));

    return true;
}

bool CameraNode::deactivate()
{
    m_timer->cancel();

    m_cap.release();

    m_camera_info_pub->on_deactivate();
    m_image_pub->on_deactivate();

    return true;
}

bool CameraNode::cleanup() { return true; }

sensor_msgs::msg::Image CameraNode::convert_frame_to_image(cv::Mat& frame) const
{
    sensor_msgs::msg::Image ros_image;

    // Ensure we output the target image size
    if (frame.rows != m_image_width || frame.cols != m_image_height) {
        cv::resize(frame, frame, cv::Size(m_image_width, m_image_height));
    }

    // Copy over the data
    ros_image.height = static_cast<unsigned int>(frame.rows);
    ros_image.width = static_cast<unsigned int>(frame.cols);
    ros_image.encoding = "bgr8";
    ros_image.is_bigendian = 0u;
    ros_image.step = static_cast<unsigned int>(
        static_cast<unsigned int>(frame.cols) * frame.elemSize());
    auto size = ros_image.step * static_cast<unsigned int>(frame.rows);

    if (frame.isContinuous()) {
        // Since the data is all stored continuously, we can just copy the whole
        // range);
        ros_image.data = std::vector(frame.data, frame.data + size);
    } else {
        // Performs a row-by-row copy
        ros_image.data.resize(size);
        auto* ros_data_ptr = reinterpret_cast<uchar*>((ros_image.data).data());
        auto* cv_data_ptr = frame.data;
        for (int i = 0; i < frame.rows; ++i) {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += frame.step;
        }
    }

    return ros_image;
}

void CameraNode::image_callback()
{
    m_cap >> m_frame;

    auto now = std::chrono::steady_clock::now();
    if (!m_frame.empty()) {
        m_last_frame = now;

        if (!m_is_flipped) {
            m_image_msg = convert_frame_to_image(m_frame);
        } else {
            // flip the frame if configured
            cv::flip(m_frame, m_flipped_frame, 1);
            m_image_msg = convert_frame_to_image(m_flipped_frame);
        }

        auto timestamp = this->get_clock()->now();

        // construct and publish camera info
        sensor_msgs::msg::CameraInfo cam_info_msg{
            m_cam_info_manager->get_camera_info()};
        cam_info_msg.header.stamp = timestamp;
        cam_info_msg.header.frame_id = m_frame_id;

        m_camera_info_pub->publish(cam_info_msg);

        // construct and publish image
        m_image_msg.header.stamp = timestamp;
        m_image_msg.header.frame_id = m_frame_id;

        m_image_pub->publish(m_image_msg);
    }
}

}  // namespace cmr
