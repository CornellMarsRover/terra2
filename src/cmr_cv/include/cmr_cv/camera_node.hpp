#pragma once
#include <image_transport/camera_publisher.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include "cmr_cv/external/camera_info_manager.hpp"
#include "cmr_fabric/fabric_node.hpp"

namespace cmr
{

/**
 * `CameraNode`
 *
 * The camera node reads from a USB camera and outputs the image data as
 * sensor_msgs::Image messages on the "image_raw" topic. It also publishes
 * camera information on the "camera_info" topic using a CameraInfoManager from
 * the ROS2 Perception library.
 *
 * The configuration options for this node are as follows:
 * - "frame_id" -> a string, the frame ID to be passed in the headers of messages
 * published by this node (default: "camera").
 * - "image_width" -> an integer, the width of the image in pixels (default: 1280)
 * - "image_height" -> an integer, the height of the image in pixels (default: 720)
 * - "fps" -> a double, the number of frames per second to take from the camera
 * (default: 10.0)
 *
 * Under the hood, this node uses the OpenCV image capturing library to read from
 * the camera, and then converts this data into sensor_msgs::Image messages before
 * sending them onto the "image_raw" topic. For camera information, see the online
 * documentation for the CameraInfoManager provided by ROS2 Perception.
 */
class CameraNode : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `CameraNode`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit CameraNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    rclcpp::TimerBase::SharedPtr m_timer;
    cv::Mat m_frame;
    cv::Mat m_flipped_frame;
    cv::VideoCapture m_cap;

    bool m_is_flipped = false;

    std::string m_frame_id;
    int m_image_width = 0;
    int m_image_height = 0;
    double m_fps = 0;
    std::string m_camera_id;

    std::chrono::steady_clock::time_point m_last_frame;

    std::shared_ptr<cmr_cv::CameraInfoManager> m_cam_info_manager;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>>
        m_camera_info_pub;

    sensor_msgs::msg::Image m_image_msg;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>>
        m_image_pub;

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    bool configure_settings(const std::unique_ptr<toml::Table> settings);

    void initialize_publishers();

    sensor_msgs::msg::Image convert_frame_to_image(cv::Mat& frame) const;

    void image_callback();
};

}  // namespace cmr