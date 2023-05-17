#pragma once

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace cmr
{
/**
 * @brief The WhileNode has only two children and returns SUCCESS if one of
 * the children do
 *
 *  The first child is consider the condition, if it fails or succeeds, the
 *  node will terminate and have the same status
 *
 *  If the first child is running, the second child will be ticked. If the second
 *  returns success, the node terminates, otherwise the node just remains running.
 *
 *
 */
class WhileNode : public BT::ControlNode
{
  public:
    /**
     * @brief A constructor for cmr::WhileNode
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    WhileNode(const std::string& name, const BT::NodeConfiguration& conf);

    /**
     * @brief A destructor for cmr::WhileNode
     */
    ~WhileNode() override = default;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    // NOLINTNEXTLINE(readability-identifier-naming)
    static BT::PortsList providedPorts() { return {}; }

  private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief The other (optional) override required by a BT action to reset node
     * state
     */
    void halt() override;

    static const int cond_node_index = 0;
    static const int body_node_index = 1;
};

}  // namespace cmr