#include "cmr_demo/gps_fake.hpp"

using namespace std::chrono_literals;

inline double meters_to_degrees(double meters) { return meters * (0.00001 / 1.11); }

constexpr auto timer_period = 5;  // seconds

namespace cmr
{

GpsFake::GpsFake(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
}

bool GpsFake::configure(const std::shared_ptr<toml::Table>&)
{
    m_pub =
        this->create_lifecycle_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 10);
    m_timer = this->create_lifecycle_timer((timer_period)*1s,
                                           std::function([this]() { timer_cb(); }));
    return true;
}

bool GpsFake::activate()
{
    // reset
    m_lat = initial_lat;
    m_lon = initial_lon;

    m_pub->on_activate();
    m_timer->activate();

    return true;
}

bool GpsFake::deactivate()
{
    m_pub->on_deactivate();
    m_timer->deactivate();
    return true;
}

bool GpsFake::cleanup()
{
    // undo the effects of configure here

    return true;
}

void GpsFake::timer_cb()
{
    m_lat += meters_to_degrees(m_speed * timer_period);

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->get_clock()->now();
    msg.latitude = m_lat;
    msg.longitude = m_lon;
    m_pub->publish(msg);
}

}  // namespace cmr
