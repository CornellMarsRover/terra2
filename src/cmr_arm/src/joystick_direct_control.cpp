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
    // control int corresponds to specific control type on joystick hardware
    const int control = msg.control_id;
    // generalized axis value for controls with multiple axes
    const int axis = msg.axis_id;
    // magnitude is important for continuous controls such as the joystick. values
    // range 0-100 float. buttons will have discrete values of 1.0, -1.0.
    const double magnitude = msg.magnitude;
    // sensitivity is updated whenever slider is moved. affects magnitude of
    // motor effort via a multiplier.
    auto message = cmr_msgs::msg::ArmJointEffort();
    message.effort = magnitude * m_sensitivity;
    // logic to send effort values to certain motors based on control value
    if (control == JoystickReading::MAIN_JOYSTICK_ID) {
        if (axis == JoystickReading::HORIZONTAL_AXIS_ID) {
            m_base_rotate_effort_pub->publish(message);
        }
        if (axis == JoystickReading::VERTICAL_AXIS_ID) {
            m_shoulder_effort_pub->publish(message);
        }
    } else if (control == JoystickReading::SENS_SLIDER_ID) {
        m_sensitivity = magnitude * m_sens_scaler;
    } else if (control == JoystickReading::SECOND_JOYSTICK_ID) {
        if (axis == JoystickReading::HORIZONTAL_AXIS_ID) {
            m_third_tilt_effort_pub->publish(message);
        }
        if (axis == JoystickReading::VERTICAL_AXIS_ID) {
            m_third_rotate_effort_pub->publish(message);
        }
    } else if (control == JoystickReading::THIRD_BUTTON_ID) {
        m_elbow_effort_pub->publish(message);
    } else if (control == JoystickReading::FOURTH_BUTTON_ID) {
        m_second_rotate_effort_pub->publish(message);
    }
    return;
}

bool JoystickDirectControl::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto node_settings = table->getTable("node");

    const auto [ok_buffer_size, buffer_size] = node_settings->getInt("buffer_size");
    const auto [ok_sensi_scaler, sensi_scaler] =
        node_settings->getInt("sensitivity_scaler");
    if (!ok_buffer_size || !ok_sensi_scaler) {
        return false;
    }
    m_sens_scaler = static_cast<double>(sensi_scaler);
    auto buf_size = static_cast<uint64_t>(buffer_size);
    m_joystick_sub = this->create_subscription<cmr_msgs::msg::JoystickReading>(
        "/js_input", buf_size, [this](const JoystickReading& joy_msg) {
            return update_arm_position(joy_msg);
        });
    m_base_rotate_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "/base_rotate/effort", buf_size);
    m_shoulder_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "/shoulder/effort", buf_size);
    m_elbow_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "/elbow/effort", buf_size);
    m_second_rotate_effort_pub =
        this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
            "/second_rotate/effort", buf_size);
    m_third_tilt_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "/third_tilt/effort", buf_size);
    m_third_rotate_effort_pub =
        this->create_publisher<cmr_msgs::msg::ArmJointEffort>("/third_rotate/effort",
                                                              buf_size);
    m_sensitivity = 50 * m_sens_scaler;
    return true;
}

bool JoystickDirectControl::activate()
{
    m_base_rotate_effort_pub->on_activate();
    m_shoulder_effort_pub->on_activate();
    m_elbow_effort_pub->on_activate();
    m_second_rotate_effort_pub->on_activate();
    m_third_tilt_effort_pub->on_activate();
    m_third_rotate_effort_pub->on_activate();
    m_is_activated = true;
    return true;
}

bool JoystickDirectControl::deactivate()
{
    m_base_rotate_effort_pub->on_deactivate();
    m_shoulder_effort_pub->on_deactivate();
    m_elbow_effort_pub->on_deactivate();
    m_second_rotate_effort_pub->on_deactivate();
    m_third_tilt_effort_pub->on_deactivate();
    m_third_rotate_effort_pub->on_deactivate();
    m_is_activated = false;
    return true;
}

bool JoystickDirectControl::cleanup()
{
    m_joystick_sub.reset();
    m_base_rotate_effort_pub.reset();
    m_shoulder_effort_pub.reset();
    m_elbow_effort_pub.reset();
    m_second_rotate_effort_pub.reset();
    m_third_tilt_effort_pub.reset();
    m_third_rotate_effort_pub.reset();
    return true;
}

}  // namespace cmr
