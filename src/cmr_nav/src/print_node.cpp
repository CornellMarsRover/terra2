#include <string>
#include <memory>

#include "cmr_nav/print_node.hpp"

namespace cmr
{
std::string g_message;
PrintAction::PrintAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  std::string message;
  getInput("message", message);
  g_message = message;
}

void PrintAction::on_tick()
{
    RCLCPP_INFO(node_->get_logger(), "Message: %s. \n", g_message.c_str());
}

}  // namespace cmr

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<cmr::PrintAction>(name, "print", config);
    };

  factory.registerBuilder<cmr::PrintAction>("Print", builder);
}