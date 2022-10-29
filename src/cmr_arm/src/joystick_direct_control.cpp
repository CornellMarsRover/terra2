#include "cmr_arm/joystick_direct_control.hpp"

namespace cmr
{

JoystickDirectControl::JoystickDirectControl(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool JoystickDirectControl::update_arm_position(
    const cmr_msgs::msg::JoystickReading msg)
{
    // control integer corresponds to specific buttons or joysticks on thrustmaster.
    // i.e. main joystick corresponds to control = 0.
    int control = msg.control;
    // generalized axis value for controls with multiple axes. main joystick
    // horizontal = 0, vertical = 1.
    int axis = msg.axis;
    // magnitude is important for continuous controls such as the joystick. values
    // range 0-100 float. buttons will have discrete values of 1.0, -1.0, 0.0 for
    // pressing/unpressing.
    double magnitude = msg.magnitude;
    // sensitivity is updated whenever slider is moved. affects magnitude of
    // motor effort via a multiplier.
    double sens = *m_sensitivity;

    auto message = cmr_msgs::msg::ArmJointEffort();
    message.effort = magnitude * sens;
    // logic to send effort values to certain motors based on control value
    if (control == 0) {   // main joystick movement detected
        if (axis == 0) {  // horizontal movement on joystick detected
            m_base_rotate_effort_pub->publish(message);
        }
        if (axis == 1) {  // vertical movement on joystick detected
            m_shoulder_effort_pub->publish(message);
        }
    } else if (control == 1) {  // sensitivity slider has changed
        *m_sensitivity = magnitude * 0.01;
    } else if (control == 2) {
        if (axis == 0) {  // horizontal movement on joystick detected
            m_third_tilt_effort_pub->publish(message);
        }
        if (axis == 1) {  // vertical movement on joystick detected
            m_third_rotate_effort_pub->publish(message);
        }
    } else if (control == 3) {
        m_elbow_effort_pub->publish(message);
    } else if (control == 4) {
        m_second_rotate_effort_pub->publish(message);
    }
    return true;
}

bool JoystickDirectControl::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto node_settings = table->getTable("node");

    // const auto [ok_arm1, arm_segment1_length] =
    //     node_settings->getDouble("arm_segment1_length");
    // const auto [_, buffer_size] = node_settings->getInt("buffer_size");
    // Ex.const auto node_settings = table->getTable("node");

    m_joystick_sub = this->create_subscription<cmr_msgs::msg::JoystickReading>(
        "topic", 100,
        std::bind(&JoystickDirectControl::update_arm_position, this,
                  std::placeholders::_1));
    m_base_rotate_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "base_rotate/effort", 100);
    m_shoulder_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "shoulder/effort", 100);
    m_elbow_effort_pub =
        this->create_publisher<cmr_msgs::msg::ArmJointEffort>("elbow/effort", 100);
    m_second_rotate_effort_pub =
        this->create_publisher<cmr_msgs::msg::ArmJointEffort>("second_rotate/effort",
                                                              100);
    m_third_tilt_effort_pub = this->create_publisher<cmr_msgs::msg::ArmJointEffort>(
        "third_tilt/effort", 100);
    m_third_rotate_effort_pub =
        this->create_publisher<cmr_msgs::msg::ArmJointEffort>("third_rotate/effort",
                                                              100);
    *m_sensitivity = 0.5;
    return true;
}

bool JoystickDirectControl::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool JoystickDirectControl::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool JoystickDirectControl::cleanup()
{
    // undo the effects of configure here
    // m_joystick_sub.reset();
    // m_base_rotate_effort_pub.reset();
    // m_shoulder_effort_pub.reset();
    // m_elbow_effort_pub.reset();
    // m_second_rotate_effort_pub.reset();
    // m_third_tilt_effort_pub.reset();
    // m_third_rotate_effort_pub.reset();
    // m_sensitivity.reset();
    return true;
}

}  // namespace cmr
