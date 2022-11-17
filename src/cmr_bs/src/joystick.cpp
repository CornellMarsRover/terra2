#include "cmr_bs/joystick.hpp"

// For joystick callback function
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <array>
#include <cmr_utils/cmr_debug.hpp>
#include <memory>

#include "cmr_msgs/msg/joystick_reading.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/string_utils.hpp"

using namespace std::chrono_literals;
constexpr auto invalid_fd = -1;
namespace cmr
{

Joystick::Joystick(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config), m_js(-1)
{
}

std::optional<js_event> read_event(int fd)
{
    js_event event{};
    const auto bytes = read(fd, &event, sizeof(event));

    if (bytes == sizeof(event)) {
        return event;
    }

    /* Error, could not read full event. */
    return {};
}

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(const js_event& event,
                      std::array<Joystick::AxisState, 3>& axis_state)
{
    const size_t axis = event.number / 2;

    // Each axis has an X and Y component. So for axis 0, the X component is 0
    // and the Y component is 1.
    // therefore, if the event number is even, it is the X component, and if it
    // is odd, it is the Y component.

    // So the axis numbers are 0 to 5
    // Control 0, X = 0, Y = 1
    // Control 1, X = 2, Y = 3
    // Control 2, X = 4, Y = 5

    if (axis < 3) {
        if (event.number % 2 == 0) {
            axis_state.at(axis).x = event.value;
        } else {
            axis_state.at(axis).y = event.value;
        }
    }

    return axis;
}

// Used to read values from the joystick and send inputs in the form of messages
void Joystick::joystick_callback(std::array<AxisState, 3>& axis_state) const
{
    auto message = cmr_msgs::msg::JoystickReading();
    if (const auto event = read_event(m_js); event) {
        switch (event->type) {
            case JS_EVENT_BUTTON:
                // Add 3 so buttons have different control ids than axes
                message.control_id = event->number + 3;
                message.axis_id = 0;
                message.magnitude = event->value != 0 ? 1 : 0;
                CMR_LOG(INFO, "Button %u %s\n", event->number + 3,
                        event->value != 0 ? "pressed" : "released");
                m_joystick_pub->publish(message);
                break;
            case JS_EVENT_AXIS: {
                size_t axis = get_axis_state(*event, axis_state);
                message.control_id = static_cast<int>(axis);
                message.axis_id = 0;
                message.magnitude = static_cast<int>(axis_state.at(axis).x / 327.67);
                m_joystick_pub->publish(message);
                message.axis_id = 1;
                message.magnitude = static_cast<int>(axis_state.at(axis).y / 327.67);
                m_joystick_pub->publish(message);
                if (axis < 3) {
                    CMR_LOG(INFO, "Axis %zu at (%6d, %6d)\n", axis,
                            static_cast<int>(axis_state.at(axis).x / 327.67),
                            static_cast<int>(axis_state.at(axis).y / 327.67));
                }
                break;
            }
            case JS_EVENT_INIT:
                /* Ignore init events. */
                break;
            default:
                CMR_ASSERT_MSG(false, "Unknown event type: %d", event->type);
                break;
        }
    }
}

bool Joystick::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto buffer_size = static_cast<uint64_t>(10);
    m_joystick_pub = this->create_publisher<cmr_msgs::msg::JoystickReading>(
        "js_input", buffer_size);

    const auto node_settings = table->getTable("node");
    const auto [ok, device] = node_settings->getString("device");
    if (ok) {
        m_device_name = device;
    } else {
        CMR_LOG(ERROR, "Could not find device name in config file");
    }
    return ok;
}

bool Joystick::activate()
{
    CMR_LOG(INFO, "About to open file %s", m_device_name.c_str());
    m_js = open(m_device_name.c_str(), O_RDONLY);
    if (m_js == invalid_fd) {
        CMR_LOG(ERROR, "Could not open joystick %s", m_device_name.c_str());
        return false;
    }

    m_joystick_pub->on_activate();

    m_buffer_timer = create_wall_timer(
        5ms, std::function([this]() { joystick_callback(m_axis_state); }));

    CMR_LOG(INFO, "Joystick activated");
    return true;
}

bool Joystick::deactivate()
{
    m_joystick_pub->on_deactivate();
    m_buffer_timer->cancel();
    const auto err = close(m_js);
    if (err != 0) {
        CMR_LOG(ERROR,
                "Could not close joystick %s with errno %d. "
                "Continuing with deactivation as normal",
                m_device_name.c_str(), errno);
    }
    m_js = invalid_fd;
    return true;
}

bool Joystick::cleanup() { return true; }

}  // namespace cmr
