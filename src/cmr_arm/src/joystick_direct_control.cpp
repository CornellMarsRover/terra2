#include "cmr_arm/joystick_direct_control.hpp"

#include "cmr_utils/cmr_debug.hpp"
using cmr_msgs::msg::JoystickReading;
namespace cmr
{

JoystickDirectControl::JoystickDirectControl(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config),
      m_sensitivity(0),
      m_sens_scaler(0),
      m_is_activated(false)
{
}

static auto get_arm_control_msg(const JoystickReading& msg, double sensitivity)
{
    auto message = std_msgs::msg::Float64MultiArray();
    message.data.resize(7, 0.0);
    // logic to send effort values to certain motors based on control value
    switch (msg.control_id) {
        case JoystickReading::MAIN_JOYSTICK_ID:
            if (msg.axis_id == JoystickReading::HORIZONTAL_AXIS_ID) {
                message.data[0] = msg.magnitude * sensitivity;
            } else if (msg.axis_id == JoystickReading::VERTICAL_AXIS_ID) {
                message.data[1] = msg.magnitude * sensitivity;
            }
            break;
        case JoystickReading::SECOND_JOYSTICK_ID:
            if (msg.axis_id == JoystickReading::HORIZONTAL_AXIS_ID) {
                message.data[2] = msg.magnitude * sensitivity;
            } else if (msg.axis_id == JoystickReading::VERTICAL_AXIS_ID) {
                message.data[3] = msg.magnitude * sensitivity;
            }
            break;
        case JoystickReading::THIRD_BUTTON_ID:
            message.data[4] = msg.magnitude * sensitivity;
            break;
        case JoystickReading::FOURTH_BUTTON_ID:
            message.data[4] = -msg.magnitude * sensitivity;
            break;
        case JoystickReading::FIFTH_BUTTON_ID:
            message.data[5] = msg.magnitude * sensitivity;
            break;
        case JoystickReading::SIXTH_BUTTON_ID:
            message.data[5] = -msg.magnitude * sensitivity;
            break;
        case JoystickReading::SEVENTH_BUTTON_ID:
            message.data[6] = msg.magnitude * sensitivity;
            break;
        case JoystickReading::EIGHTH_BUTTON_ID:
            message.data[6] = -msg.magnitude * sensitivity;
            break;
    }
    return message;
}

void JoystickDirectControl::update_arm_position(const JoystickReading& msg)
{
    if (!m_is_activated) {
        return;
    }
    if (msg.control_id == JoystickReading::SENS_SLIDER_ID) {
        m_sensitivity = msg.magnitude * m_sens_scaler;
    } else {
        const auto message = get_arm_control_msg(msg, m_sensitivity);
        m_arm_control_pub->publish(message);
    }
}

bool JoystickDirectControl::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto node_settings = table->getTable("node");

    const auto [ok_buffer_size, buffer_size] = node_settings->getInt("buffer_size");
    const auto [ok_sensi_scaler, sensi_scaler] =
        node_settings->getDouble("sensitivity_scaler");
    const auto [ok_start_sens, start_sens] =
        node_settings->getDouble("starting_sensitivity");
    if (!ok_buffer_size || !ok_sensi_scaler || !ok_start_sens) {
        return false;
    }
    m_sens_scaler = sensi_scaler;
    auto buf_size = static_cast<uint64_t>(buffer_size);
    m_joystick_sub = this->create_subscription<cmr_msgs::msg::JoystickReading>(
        "js_input", buf_size, [this](const JoystickReading& joy_msg) {
            return update_arm_position(joy_msg);
        });
    m_arm_control_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", buf_size);
    m_sensitivity = start_sens;
    return true;
}

bool JoystickDirectControl::activate()
{
    m_arm_control_pub->on_activate();
    m_is_activated = true;
    return true;
}

bool JoystickDirectControl::deactivate()
{
    m_arm_control_pub->on_deactivate();
    m_is_activated = false;
    return true;
}

bool JoystickDirectControl::cleanup()
{
    m_joystick_sub.reset();
    m_arm_control_pub.reset();
    return true;
}

}  // namespace cmr
