#include "cmr_bs/joystick.hpp"

// For joystick callback function
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <unistd.h>

#include <array>
#include <cmr_utils/cmr_debug.hpp>
#include <memory>

using namespace std::chrono_literals;
namespace cmr
{

Joystick::Joystick(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
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
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    uint8_t axes{};

    if (ioctl(fd, JSIOCGAXES, &axes) == -1) {
        return 0;
    }

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    uint8_t buttons{};
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1) {
        return 0;
    }

    return buttons;
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

    if (axis < 3) {
        if (event.number % 2 == 0) {
            axis_state.at(axis).x = event.value;
        } else {
            axis_state.at(axis).y = event.value;
        }
    }

    return axis;
}

void Joystick::joystick_callback(std::array<AxisState, 3>& axis_state) const
{
    // std::array<AxisState, 3> axis_state = {};
    if (const auto event = read_event(m_js); event) {
        /* This loop will exit if the controller is unplugged. */
        // event.value().type;
        switch (event->type) {
            case JS_EVENT_BUTTON:

                CMR_LOG(INFO, "Button %u %s\n", event->number + 3,
                        event->value != 0 ? "pressed" : "released");
                break;
            case JS_EVENT_AXIS: {
                size_t axis = get_axis_state(*event, axis_state);
                if (axis < 3) {
                    CMR_LOG(INFO, "Axis %zu at (%6d, %6d)\n", axis,
                            (int)(axis_state.at(axis).x / 327.67),
                            (int)(axis_state.at(axis).y / 327.67));
                }
                break;
            }
            default:
                /* Ignore init events. */
                break;
        }
    }

    fflush(stdout);
}

void Joystick::joystick_loop()
{
    std::array<AxisState, 3> axis_state{};
    while (m_loop_flag) {
        joystick_callback(axis_state);
    }
    m_js = close(m_js);
}

bool Joystick::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    auto buffer_size = static_cast<uint64_t>(100);
    m_joystick_pub = this->create_publisher<cmr_msgs::msg::JoystickReading>(
        "js_input", buffer_size);

    const auto node_settings = table->getTable("node");
    const auto [ok, device] = node_settings->getString("device");
    if (ok) {
        m_device_name = device;
    }
    return ok;
}

bool Joystick::activate()
{
    // do any last-minute things before activation here
    // it should be quick
    m_joystick_pub->on_activate();

    m_js = open(m_device_name.c_str(), O_RDONLY);
    if (m_js == -1) {
        perror("Could not open joystick");
        return false;
    }
    // m_loop_flag = true;

    // m_js_thread = std::make_unique<JThread>(([this]() { joystick_loop(); }));

    m_buffer_timer = create_wall_timer(
        5ms, std::function([this]() { joystick_callback(m_axis_state); }));

    return true;
}

bool Joystick::deactivate()
{
    // undo the effects of activate here

    m_joystick_pub->on_deactivate();
    m_js = close(m_js);
    m_buffer_timer->cancel();
    // m_loop_flag = false;
    return true;
}

bool Joystick::cleanup()
{
    // undo the effects of configure here

    return true;
}

}  // namespace cmr
