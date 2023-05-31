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
#include "cmr_utils/external/tomlcpp.hxx"
#include "cmr_utils/monad.hpp"
#include "cmr_utils/string_utils.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
// Constants
constexpr auto invalid_fd = -1;
constexpr double joystick_max = std::numeric_limits<int16_t>::max();
namespace cmr
{

void Joystick::js_event_button_publ(std::optional<js_event> event,
                                    cmr_msgs::msg::JoystickReading message) const
{
    message.control_id = event->number + 3;
    message.axis_id = cmr_msgs::msg::JoystickReading::X_AXIS_ID;
    message.magnitude = event->value != 0 ? 1 : 0;
    CMR_LOG(INFO, "Button %u %s\n", event->number + 3,
            event->value != 0 ? "pressed" : "released");
    m_joystick_pub->publish(message);
}

Joystick::Joystick(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config), m_js(invalid_fd)
{
}
/**
 * @brief Read an event being produced by the controller and return the js_event
 * that occurred.
 *
 * @param fd the connected joystick file
 * @return the js_event event representing the event on the joystick
 */
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
 * @brief Keeps track of the current axis state.
 *
 * This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * @return the axis that the event indicated.
 */
uint8_t get_control_state(const js_event& event,
                          std::array<Joystick::AxisState, 3>& axis_state)
{
    const uint8_t axis = event.number / 2;

    // Each axis has an X and Y component. So for axis 0, the X component is 0
    // and the Y component is 1.
    // therefore, if the event number is even, it is the X component, and if it
    // is odd, it is the Y component.

    // So the axis numbers are 0 to 5
    // Control 0, X = 0, Y = 1
    // Control 1, X = 2, Y = 3
    // Control 2, X = 4, Y = 5
    CMR_ASSERT_MSG(axis < axis_state.size(),
                   "Invalid axis detected from controller input")
    if (event.number % 2 == 0) {
        axis_state.at(axis).x = event.value;
    } else {
        axis_state.at(axis).y = event.value;
    }

    return axis;
}

void Joystick::arm_callback(std::array<AxisState, 3>& axis_state) const
{
    auto message = cmr_msgs::msg::JoystickReading();
    if (const auto event = read_event(m_js); event) {
        switch (event->type) {
            case JS_EVENT_BUTTON:
                js_event_button_publ(event, message);
                break;
            case JS_EVENT_AXIS: {
                size_t control = get_control_state(*event, axis_state);

                auto rotate_val =
                    axis_state.at(control).x / (joystick_max / m_max_arm_speed);
                auto translate_val =
                    axis_state.at(control).y / (joystick_max / m_max_arm_speed);

                message.control_id = static_cast<int>(control);
                message.axis_id = cmr_msgs::msg::JoystickReading::X_AXIS_ID;
                message.magnitude = rotate_val;
                m_joystick_pub->publish(message);

                message.axis_id = cmr_msgs::msg::JoystickReading::Y_AXIS_ID;
                if (control == 1) {
                    message.control_id = 0;
                    message.axis_id = cmr_msgs::msg::JoystickReading::Z_AXIS_ID;
                }
                message.magnitude = translate_val;
                m_joystick_pub->publish(message);
                // Used to display the values of each moved axis as CMR_LOGs
                CMR_LOG(INFO, "Control %zu at (x, y) = (%6f, %6f)\n", control,
                        rotate_val, translate_val);
                break;
            }
            case JS_EVENT_INIT:
                /* Ignore init events. */
                break;
            default:
                CMR_LOG(WARN, "Unknown event type: %d", event->type);
                break;
        }
    }
}

void Joystick::pan_tilt_cam_callback(std::array<AxisState, 3>& axis_state) const
{
    auto message = std_msgs::msg::Int32();
    if (const auto event = read_event(m_js); event) {
        switch (event->type) {
            case JS_EVENT_BUTTON:
                js_event_button_publ(event, message);
                break;
            case JS_EVENT_AXIS: {
                size_t control = get_control_state(*event, axis_state);

                auto rotate_val =
                    axis_state.at(control).x / (joystick_max / m_max_arm_speed);
                auto translate_val =
                    axis_state.at(control).y / (joystick_max / m_max_arm_speed);

                message.control_id = static_cast<int>(control);
                message.axis_id = cmr_msgs::msg::JoystickReading::X_AXIS_ID;
                message.magnitude = rotate_val;
                m_joystick_pub->publish(message);

                message.axis_id = cmr_msgs::msg::JoystickReading::Y_AXIS_ID;
                if (control == 1) {
                    message.control_id = 0;
                    message.axis_id = cmr_msgs::msg::JoystickReading::Z_AXIS_ID;
                }
                message.magnitude = translate_val;
                m_joystick_pub->publish(message);
                // Used to display the values of each moved axis as CMR_LOGs
                CMR_LOG(INFO, "Control %zu at (x, y) = (%6f, %6f)\n", control,
                        rotate_val, translate_val);
                break;
            }
            case JS_EVENT_INIT:
                /* Ignore init events. */
                break;
            default:
                CMR_LOG(WARN, "Unknown event type: %d", event->type);
                break;
        }
    }
}

void Joystick::drives_callback(std::array<AxisState, 3>& axis_state)
{
    CMR_LOG(INFO, "Running");
    auto message = geometry_msgs::msg::TwistStamped();
    if (const auto event = read_event(m_js); event) {
        /* This loop will exit if the controller is unplugged. */
        switch (event->type) {
            case JS_EVENT_AXIS: {
                const size_t control = get_control_state(*event, axis_state);
                /*assuming x, y, and z start at 0*/
                if (static_cast<int>(control) == 0 ||
                    static_cast<int>(control) == 1) {
                    // Scaling for both angular and linear is 0-2
                    message.twist.linear.x =
                        (axis_state.at(0).y / -(joystick_max / m_max_speed));
                    message.twist.linear.y = 0;
                    message.twist.linear.z = 0;
                    message.twist.angular.x = 0;
                    message.twist.angular.y = 0;
                    message.twist.angular.z =
                        (axis_state.at(1).x / -(joystick_max / m_max_speed));
                    message.header.frame_id = "base_link";
                    m_last_drives_twist = message;
                    m_got_first_message = true;
                } else {
                    CMR_LOG(INFO, "Control numbers invalid");
                }
                break;
            }
            default:
                /* Ignore init events. */
                break;
        }
    }
}

void Joystick::drives_publish_callback() const
{
    if (m_got_first_message) {
        m_drives_pub->publish(m_last_drives_twist);
        // Used to display the values of the controller as a CMR_LOG
        CMR_LOG(INFO, "Linear: %6f, Angular: %6f",
                m_last_drives_twist.twist.linear.x,
                m_last_drives_twist.twist.angular.z);
    }
}

bool Joystick::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto buffer_size = static_cast<uint64_t>(10);
    m_joystick_pub =
        this->create_lifecycle_publisher<cmr_msgs::msg::JoystickReading>(
            "js_input", buffer_size);
    m_drives_pub =
        this->create_lifecycle_publisher<geometry_msgs::msg::TwistStamped>(
            "/drives_controller/cmd_vel", buffer_size);

    /*
    device - name of the device in the Linux system
    type - type of controller (arm or drives)
    arm_max_speed - the max speed the arm can move
    max_speed - the max speed the rover can drive
    */
    const auto node_settings = table->getTable("node");
    const auto [ok, device] = node_settings->getString("device");
    const auto [ok_type, type] = node_settings->getString("type");
    if (ok && ok_type) {
        m_device_name = device;
        m_type_name = type;
    } else {
        CMR_LOG(ERROR, "Could not find device name in config file");
        return false;
    }
    m_max_arm_speed =
        monad::value_or(node_settings->getDouble("arm_max_speed"), 10.0);
    m_max_speed = monad::value_or(node_settings->getDouble("max_speed"), 2.5);
    return true;
}

bool Joystick::activate()
{
    // opens the joystick to be read
    m_js = open(m_device_name.c_str(), O_RDONLY | O_NONBLOCK);
    if (m_js == invalid_fd) {
        CMR_LOG(ERROR, "Could not open joystick %s", m_device_name.c_str());
        return false;
    }

    if (m_type_name == "armcontroller") {
        m_buffer_timer = create_lifecycle_timer(
            5ms, std::function([this]() { arm_callback(m_axis_state); }));
    } else if (m_type_name == "drivescontroller") {
        m_drives_publish_timer = create_lifecycle_timer(
            5ms, std::function([this]() { drives_publish_callback(); }));
        m_buffer_timer = create_lifecycle_timer(
            5ms, std::function([this]() { drives_callback(m_axis_state); }));

        CMR_LOG(INFO, "Drives Callback If Statement has run");
    } else {
        CMR_LOG(ERROR, "Unknown device type in config file");
        return false;
    }

    // Activates the publishers needed
    m_joystick_pub->on_activate();
    m_drives_pub->on_activate();
    m_buffer_timer->activate();
    if (m_drives_publish_timer) {
        m_drives_publish_timer->activate();
    }

    CMR_LOG(INFO, "Node is active");
    // Checks which device is being used to initialize the wall timer
    return true;
}

bool Joystick::deactivate()
{
    // Deactivates the publishers and timers
    m_joystick_pub->on_deactivate();
    m_drives_pub->on_deactivate();
    m_buffer_timer->deactivate();
    if (m_drives_publish_timer) {
        m_drives_publish_timer->deactivate();
    }

    // Closes the joystick file being read
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

void const_declarations() {}
