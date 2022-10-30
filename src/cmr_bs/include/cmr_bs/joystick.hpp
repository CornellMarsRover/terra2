#pragma once

#include "cmr_fabric/fabric_node.hpp"
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

    std::unique_ptr<JThread> m_js_thread;

    std::atomic_bool m_loop_flag = false;

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

  private:
    void joystick_callback() const;

    void joystick_loop();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

}  // namespace cmr