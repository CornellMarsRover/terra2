#pragma once

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
#include "cmr_utils/thread_wrapper.hpp"

namespace cmr
{

/**
 * `Joystick`
 *
 * TODO
 */
class Joystick : public cmr::fabric::FabricNode
{
    std::string m_device_name;

    int m_js;

    std::shared_ptr<rclcpp::WallTimer<std::function<void()>>> m_buffer_timer;

  public:
    /**
     * Constructs a `Joystick`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit Joystick(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

    /**
     * Current state of an axis.
     */
    struct AxisState {
        short x, y;
    };

  private:
    std::array<AxisState, 3> m_axis_state{};

    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::JoystickReading>>
        m_joystick_pub;

    void joystick_callback(std::array<AxisState, 3>& axis_state) const;

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

}  // namespace cmr