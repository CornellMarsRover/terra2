#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace cmr
{

/**
 * @brief A cmr::PrintAction class that wraps nav2_msgs::action::Wait
 */
class PrintAction : public BT::SyncActionNode
{
  public:
    /**
     * @brief A constructor for cmr::PrintAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    PrintAction(const std::string& xml_tag_name, const std::string& action_name,
                const BT::NodeConfiguration& conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     */
    void on_tick();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    // NOLINTNEXTLINE(readability-identifier-naming)
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message", "", "message to print")};
    }

    virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }
    virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }
    virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

    BT::NodeStatus tick() override
    {
        on_tick();
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace cmr
