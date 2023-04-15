#pragma once

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "cmr_fabric/fabric_node.hpp"

constexpr auto initial_lat = 42.44524544760542;
constexpr auto initial_lon = -76.48259290304085;

namespace cmr
{

/**
 * `GpsFake`
 *
 * TODO
 */
class GpsFake : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `GpsFake`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit GpsFake(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>>
        m_pub;

    std::shared_ptr<fabric::GenericLifecycleTimer> m_timer;

    double m_speed = 10.6833;  // in meters per second
    double m_lat = initial_lat, m_lon = initial_lon;

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    void timer_cb();
};

}  // namespace cmr