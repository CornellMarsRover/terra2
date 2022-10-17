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
        m_arm_effort_pub;

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