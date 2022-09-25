#include "cmr_arm/joystick_forward_kinematics.hpp"

namespace cmr
{

JoystickForwardKinematics::JoystickForwardKinematics(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool JoystickForwardKinematics::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool JoystickForwardKinematics::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool JoystickForwardKinematics::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool JoystickForwardKinematics::cleanup()
{
    // undo the effects of configure here

    return true;
}

}  // namespace cmr
