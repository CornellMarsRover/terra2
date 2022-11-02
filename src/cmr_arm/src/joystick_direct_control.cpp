#include "cmr_arm/joystick_direct_control.hpp"
using cmr_msgs::msg::JoystickReading;
namespace cmr
{

JoystickDirectControl::JoystickDirectControl(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    m_sensitivity = 0.0;
    m_sens_scaler = 0.0;
    m_is_activated = false;
}

void JoystickDirectControl::update_arm_position(const JoystickReading& msg)
{
    if (!m_is_activated) {
        return;
    }
    const int control = msg.control_id;
    const int axis = msg.axis_id;
    const double magnitude = msg.magnitude;
    auto message = std_msgs::msg::Float64MultiArray();
    message.data.resize(7, 0.0);
    // logic to send effort values to certain motors based on control value
    if (control == JoystickReading::MAIN_JOYSTICK_ID) {
        if (axis == JoystickReading::HORIZONTAL_AXIS_ID) {
            message.data[0] = magnitude * m_sensitivity;
        }
        if (axis == JoystickReading::VERTICAL_AXIS_ID) {
            message.data[1] = magnitude * m_sensitivity;
        }
    } else if (control == JoystickReading::SENS_SLIDER_ID) {
        m_sensitivity = magnitude * m_sens_scaler;
    } else if (control == JoystickReading::SECOND_JOYSTICK_ID) {
        if (axis == JoystickReading::HORIZONTAL_AXIS_ID) {
            message.data[2] = magnitude * m_sensitivity;
        }
        if (axis == JoystickReading::VERTICAL_AXIS_ID) {
            message.data[3] = magnitude * m_sensitivity;
        }
    } else if (control == JoystickReading::THIRD_BUTTON_ID) {
        message.data[4] = magnitude * m_sensitivity;
    } else if (control == JoystickReading::FOURTH_BUTTON_ID) {
        message.data[4] = -1.0 * magnitude * m_sensitivity;
    } else if (control == JoystickReading::FIFTH_BUTTON_ID) {
        message.data[5] = magnitude * m_sensitivity;
    } else if (control == JoystickReading::SIXTH_BUTTON_ID) {
        message.data[5] = -1.0 * magnitude * m_sensitivity;
    } else if (control == JoystickReading::SEVENTH_BUTTON_ID) {
        message.data[6] = magnitude * m_sensitivity;
    } else if (control == JoystickReading::EIGHTH_BUTTON_ID) {
        message.data[6] = -1.0 * magnitude * m_sensitivity;
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
        "/arm_control/command", buf_size);
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
