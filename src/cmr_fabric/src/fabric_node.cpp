#include <chrono>
#include "cmr_fabric/fabric_node.hpp"

using namespace std::chrono_literals;

namespace cmr::fabric
{

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto config_path = get_parameter("config_path").as_string();
  if (config_path.empty()) {
    RCLCPP_ERROR(get_logger(), "No config_path parameter specified");
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
  }

  auto toml = toml::parseFile(config_path);

  if (!toml.table) {
    RCLCPP_ERROR(get_logger(), "Failed to parse config file: %s", toml.errmsg.c_str());
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
  }

  auto config = toml.table;

  auto faultHandlingSettings = config->getTable("fault_handling");
  auto [ok, restart_attempts] = faultHandlingSettings->getInt("restart_attempts");
  if (!ok) {
    RCLCPP_ERROR(
      get_logger(), "Failed to parse fault_handling.restart_attempts: %s", toml.errmsg.c_str());
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
  }
  set_parameter(rclcpp::Parameter("restart_attempts", restart_attempts));

  return onConfigure(std::move(config)) ?
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS :
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  // TODO notify dependency manager that this node is activating
  return onActivate() ?
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS :
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // TODO notify dependency manager that this node is deactivating
  return onDeactivate() ?
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS :
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // TODO notify dependency manager that this node is shutting down.
  // We treat on_cleanup and on_shutdown in the same way, we just invoke onShutdown()
  return onShutdown() ?
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS :
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}


rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  // TODO notify dependency manager that this node is shutting down
  return onShutdown() ?
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS :
         rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_error(
  const rclcpp_lifecycle::State &)
{
  int num_restarts = get_parameter("num_restarts").as_int();
  RCLCPP_INFO(get_logger(), "Num restarts: %d", num_restarts);

  if (num_restarts < get_parameter("restart_attempts").as_int()) {
    scheduleRestart();
  } else {
    // reset the counter in case the user wants to try and enable this again later
    set_parameter(rclcpp::Parameter("num_restarts", 0));
    RCLCPP_ERROR(
      get_logger(),
      "Node will not attempt to restart because it has restarted the maximum amount of times.");
  }
  // returning SUCCESS will tell ROS2 to move the node into the Unconfigured state.
  return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void FabricNode::scheduleRestart()
{
  // make sure the fault handler is available before doing anything else
  while (!recover_fault_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the fault handler. Not scheduling restart.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Fault handler not available, waiting again...");
  }

  auto num_restarts = get_parameter("num_restarts").as_int();
  set_parameter(rclcpp::Parameter("num_restarts", num_restarts + 1));

  // Best effort to send the restart request; don't block and wait for the response though though
  // because if it fails, this is emblematic of a more serious problem.
  auto req = std::make_shared<cmr_msgs::srv::RecoverFault::Request>();
  req->node_name = get_name();
  recover_fault_client->async_send_request(req);
}

}
