#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/tree_node.h>

class ArucoAction : public BT::ActionNodeBase
{
  public:
    BT::NodeStatus executeTick() override;
};