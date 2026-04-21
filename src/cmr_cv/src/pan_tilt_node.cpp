#include "cmr_cv/pan_tilt_node.hpp"

namespace cmr
{

PanTiltNode::PanTiltNode(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    // declare parameters here
}

bool PanTiltNode::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool PanTiltNode::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool PanTiltNode::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool PanTiltNode::cleanup()
{
    // undo the effects of configure here

    return true;
}

} // namespace cmr
