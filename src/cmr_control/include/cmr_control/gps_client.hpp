#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "gps.h"

// undefine these macros introduced by gps.h because they are used as variable
// names by sensor_msgs.
#undef STATUS_FIX
#undef STATUS_NO_FIX
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace cmr
{

/**
 * `GpsClient` connects to a GPS server exposed by gpsd and outputs the GPS
 * fixes as `sensor_msgs::msg::NavSatFix` messages on the /gps/fix topic.
 *
 * Configuration:
 * - frame_id (string) is the frame in which the NavSatFix messages should be
 * published.
 * - gps_server (string) is the address of the GPS server, usually localhost
 * - gps_port (string) is the the port of the GPS server
 * - verbise (bool, default: false) is true if additional logging output should be
 * provided.
 */
class GpsClient : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `GpsClient`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit GpsClient(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    std::string m_frame_id;
    std::string m_gps_server;
    std::string m_gps_port;
    int m_gps_handle = 0;
    bool m_verbose = false;
    struct gps_data_t m_gps_data {
    };
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>>
        m_pub;
    std::unique_ptr<fabric::GenericLifecycle> m_timer;

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    void timer_cb();
};

}  // namespace cmr