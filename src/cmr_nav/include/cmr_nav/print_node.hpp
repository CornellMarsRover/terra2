#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace cmr
{

/**
 * @brief A cmr::PrintAction class that wraps nav2_msgs::action::Wait
 */
class PrintAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>
{
public:
  /**
   * @brief A constructor for cmr::PrintAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  PrintAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("message", "", "message to print")
      });
  }
};

}  // namespace cmr

