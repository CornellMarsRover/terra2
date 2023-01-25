#include "cmr_nav/autonomy_control_node.hpp"

namespace cmr
{

AutonomyControlNode::AutonomyControlNode(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool AutonomyControlNode::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool AutonomyControlNode::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool AutonomyControlNode::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool AutonomyControlNode::cleanup()
{
    // undo the effects of configure here

    return true;
}

} // namespace cmr
