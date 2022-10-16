#include "cmr_arm/joystick_direct_control.hpp"

namespace cmr
{

JoystickDirectControl::JoystickDirectControl(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool JoystickDirectControl::update_arm_position() { return true; }

bool JoystickDirectControl::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto node_settings = table->getTable("node");
    // const auto [ok_arm1, arm_segment1_length] =
    //     node_settings->getDouble("arm_segment1_length");
    // const auto [ok_arm2, arm_segment2_length] =
    //     node_settings->getDouble("arm_segment2_length");
    // const auto [ok_arm3, arm_segment3_length] =
    //     node_settings->getDouble("arm_segment3_length");
    // Ex.const auto node_settings = table->getTable("node");
    m_joystick_sub = nullptr;
    // m_joystick_sub = create_subscription<typename MessageT>(
    //    "joystick_reading", 10, update_arm_position());
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

    return true;
}

}  // namespace cmr
