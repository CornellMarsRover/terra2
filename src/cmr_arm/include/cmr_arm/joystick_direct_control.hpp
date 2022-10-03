#pragma once
#include <cmr_msgs/msg/detail/joystick_reading__struct.hpp>

#include "cmr_fabric/fabric_node.hpp"
namespace cmr
{

/**
 * `JoystickDirectControl`
 *
 * TODO
 */
class JoystickDirectControl : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::Subscription<cmr_msgs::msg::JoystickReading>>
        m_joystick_sub;

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

    void update_arm_position();
};

}  // namespace cmr