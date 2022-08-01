#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "cmr_utils/tomlcpp.hpp"

namespace cmr::fabric
{

class FabricNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FabricNode(std::string name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    declare_parameter("config_path", "");
    declare_parameter("restart_attempts", 0);
    declare_parameter("restart_delay", 0);
    declare_parameter("num_restarts", 0);
  }

  virtual ~FabricNode() = default;

  virtual bool onConfigure(std::shared_ptr<toml::Table>)
  {
    return true;
  }

  virtual bool onActivate()
  {
    return true;
  }

  virtual bool onDeactivate()
  {
    return true;
  }

  virtual bool onShutdown()
  {
    return true;
  }

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  override;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  override;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  override;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  override;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  override;

  rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(const rclcpp_lifecycle::State &)
  override;

};

}
