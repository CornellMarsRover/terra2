#include "cmr_fabric/fabric_node.hpp"


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

  return onConfigure(std::move(toml.table)) ?
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

  if (num_restarts < get_parameter("restart_attempts").as_int()) {
    // TODO notify fault handler that this node should be restarted
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Node will not attempt to restart because it has restarted the maximum amount of times.");
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
  }
}

}
