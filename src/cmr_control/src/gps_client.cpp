#include "cmr_control/gps_client.hpp"

#include <cmr_utils/external/tomlcpp.hxx>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
namespace cmr
{

GpsClient::GpsClient(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
}

// NOLINTNEXTLINE(readability-function-size)
bool GpsClient::configure(const std::shared_ptr<toml::Table>& table)
{
    const auto node_settings = table->getTable("node");

    const auto [fi_ok, fi] = node_settings->getString("frame_id");
    if (!fi_ok) {
        CMR_LOG(WARN,
                "Frame ID string configuration parameter not set. Using default "
                "value of ''.");
        m_frame_id = "";
    } else {
        m_frame_id = fi;
    }

    const auto [gs_ok, gs] = node_settings->getString("gps_server");
    if (!gs_ok) {
        CMR_LOG(
            ERROR,
            "Invalid value for string configuration parameter gps_server. Stop.");
        return false;
    }
    m_gps_server = gs;

    const auto [gp_ok, gp] = node_settings->getInt("gps_port");
    if (!gp_ok) {
        CMR_LOG(ERROR,
                "Invalid value for integer configuration parameter gps_port. Stop.");
        return false;
    }
    m_gps_port = std::to_string(gp);

    const auto [v_ok, v] = node_settings->getBool("verbose");
    if (!v_ok) {
        CMR_LOG(WARN,
                "Invalid value for boolean configuration parameter verbose. Using "
                "default value of false.");
        m_verbose = false;
    } else {
        m_verbose = v;
    }

    m_timer = create_lifecycle_timer(5s, std::function([this]() { timer_cb(); }));
    m_pub = create_lifecycle_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 10);

    return true;
}

bool GpsClient::activate()
{
    m_gps_handle = gps_open(m_gps_server.c_str(), m_gps_port.c_str(), &m_gps_data);
    if (m_gps_handle != 0) {
        CMR_LOG(ERROR, "Cannot open GPS at %s:%s; is gpsd running?",
                m_gps_server.c_str(), m_gps_port.c_str());
        return false;
    }

    gps_stream(&m_gps_data, WATCH_ENABLE | WATCH_JSON, nullptr);

    m_pub->on_activate();
    m_timer->activate();

    CMR_LOG(INFO, "GPS node is activated.");

    return true;
}

bool GpsClient::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool GpsClient::cleanup()
{
    // undo the effects of configure here

    return true;
}

void GpsClient::timer_cb()
{
    // wait for GPS data, block for up to 5 seconds
    if (!gps_waiting(&m_gps_data, static_cast<int>(5 * 1e6))) {
        if (m_verbose) {
            CMR_LOG(DEBUG, "Received no GPS data for 5 seconds.");
        }
        return;
    }

    if (gps_read(&m_gps_data, nullptr, 0) == -1) {
        CMR_LOG(WARN, "GPS read error, retrying in 5s...");
        return;
    }

    auto lat = m_gps_data.fix.latitude;
    auto lon = m_gps_data.fix.longitude;
    auto alt = m_gps_data.fix.altitude;

    if (!std::isfinite(lat) || !std::isfinite(lon)) {
        CMR_LOG(WARN, "GPS read error, retrying in 5s... (%f, %f)", lat, lon);
        CMR_LOG(DEBUG, "NaN value reported for latitude or longitude.");
        return;
    }

    if (!std::isfinite(alt)) {
        alt = std::numeric_limits<double>::quiet_NaN();
    }

    sensor_msgs::msg::NavSatFix msg;

    msg.header.frame_id = m_frame_id;
    msg.header.stamp = this->get_clock()->now();

    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = alt;
    m_pub->publish(msg);
}

}  // namespace cmr
