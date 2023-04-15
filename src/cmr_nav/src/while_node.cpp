#include "cmr_nav/while_node.hpp"

#include <string>
namespace cmr
{

WhileNode::WhileNode(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ControlNode::ControlNode(name, conf)
{
}

BT::NodeStatus WhileNode::tick()
{
    if (children_nodes_.size() != 2) {
        throw BT::BehaviorTreeException("While Node '" + name() +
                                        "' must only have 2 children.");
    }

    setStatus(BT::NodeStatus::RUNNING);
    TreeNode* condition = children_nodes_[cond_node_index];
    TreeNode* body = children_nodes_[body_node_index];
    BT::NodeStatus cond_status = condition->executeTick();

    if (cond_status != BT::NodeStatus::RUNNING) {
        halt();
        return cond_status;
    }

    BT::NodeStatus body_status = body->executeTick();
    if (body_status == BT::NodeStatus::SUCCESS) {
        halt();
        return body_status;
    }

    return BT::NodeStatus::RUNNING;
}

void WhileNode::halt() { ControlNode::halt(); }

}  // namespace cmr

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<cmr::WhileNode>("While"); }
