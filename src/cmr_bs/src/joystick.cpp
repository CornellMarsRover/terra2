#include "cmr_bs/joystick.hpp"

namespace cmr
{

Joystick::Joystick(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool Joystick::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool Joystick::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool Joystick::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool Joystick::cleanup()
{
    // undo the effects of configure here

    return true;
}

}  // namespace cmr
