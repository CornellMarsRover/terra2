#pragma once
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/get_node_state.hpp"
#include "cmr_msgs/srv/reconfigure_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{
class LifecycleManager : public rclcpp::Node
{
  private:
    /**
     * A service that gets requests from the dependency manager and fault handler to
     * activate a node. Does this forwarding the request to ROS's lifecycle node
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> m_activate_srv;
    /**
     * A service that gets requests from the dependency manager to deactive a node.
     * Handles this by forwarding a reqeust to ROS's lifecycle node
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> m_deactivate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReconfigureNode>>
        m_reconfigure_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::GetNodeState>>
        m_get_node_state_srv;

  public:
    explicit LifecycleManager(const std::string& node_name = "lifecycle_manager",
                              const std::string& node_namespace = "fabric");
};
}  // namespace cmr::fabric