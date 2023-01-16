#include "cmr_nav/print_node.hpp"

#include <memory>
#include <string>

namespace cmr
{
std::string g_message;
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
PrintAction::PrintAction(const std::string& xml_tag_name,
                         const std::string&,
                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf)
{
    std::string message;
    getInput("message", message);
    g_message = message;
}

void PrintAction::on_tick()
{
    RCLCPP_INFO(rclcpp::get_logger("Print Logger"), "Message: %s. \n",
                g_message.c_str());
}

}  // namespace cmr

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<cmr::PrintAction>(name, "print", config);
    };

    factory.registerBuilder<cmr::PrintAction>("Print", builder);
}