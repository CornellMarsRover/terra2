#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <string>

namespace cmr::fabric
{

/**
 * Provides functions to easily invoke lifecycle transitions for a node.
 */
class LifecycleManager
{
public:
  LifecycleManager();

  /**
  * @brief Tell a node to load its configuration. Moves the node to the Inactive state if successful.
  * @param node_name The name of the node to configure. If the node is not found, the function has no effect.
  */
  void configure(const std::string & node_name) noexcept;

  /**
  * @brief Tell a node to activate. Moves the node to the Active state if successful.
  * @param node_name The name of the node to activate. If the node is not found, the function has no effect.
  */
  void activate(const std::string & node_name) noexcept;

  /**
   * @brief Tell a node to deactivate. Moves the node to the Inactive state if successful.
   * @param node_name The name of the node to deactivate. If the node is not found, the function has no effect.
   */
  void deactivate(const std::string & node_name) noexcept;

  /**
   * @brief Tell a node to shutdown. Moves the node to the Finalized state if successful.
   * @param node_name The name of the node to shutdown. If the node is not found, the function has no effect.
   */
  void destroy(const std::string & node_name) noexcept;

private:
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
};

}
