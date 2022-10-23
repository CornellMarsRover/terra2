#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/arm_joint_effort.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
namespace cmr
{
class JoystickDirectControl : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::Subscription<cmr_msgs::msg::JoystickReading>>
        m_joystick_sub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_base_rotate_effort_pub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_shoulder_effort_pub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_elbow_effort_pub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_second_rotate_effort_pub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_third_tilt_effort_pub;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::ArmJointEffort>>
        m_third_rotate_effort_pub;
    std::shared_ptr<double> m_sensitivity;

  public:
    /**
     * Constructs a `JoystickDirectControl`, optionally passing in config parameters
     * for testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit JoystickDirectControl(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    bool update_arm_position(const cmr_msgs::msg::JoystickReading msg);
};

}  // namespace cmr