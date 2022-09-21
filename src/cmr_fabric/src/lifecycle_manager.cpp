#include "cmr_fabric/lifecycle_helpers.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief A node which provides a service interface for `lifecycle_helpers`
 *
 * ## Usage:
 * - `/<effective_namespace>/name/activate` - Activates a node
 * [cmr_msgs/srv/ActivateNode]
 * - `/<effective_namespace>/name/deactivate` - Deactivates a node
 * [cmr_msgs/srv/DeactivateNode]
 * - `/<effective_namespace>/name/cleanup` - Cleans up (unconfigures) a node
 * [cmr_msgs/srv/DeactivateNode]
 */
class LifecycleManager : public rclcpp::Node
{
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> m_activate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> m_deactivate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> m_cleanup_srv;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

  public:
    LifecycleManager() : rclcpp::Node("lifecycle")
    {
        m_callback_group =
            create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        m_activate_srv = create_service<cmr_msgs::srv::ActivateNode>(
            get_effective_namespace() + "/" + get_name() + "/activate",
            [](std::shared_ptr<cmr_msgs::srv::ActivateNode::Request> req,
               std::shared_ptr<cmr_msgs::srv::ActivateNode::Response> resp) {
                resp->success = cmr::fabric::activate_node(req->node_name);
                return resp;
            },
            rmw_qos_profile_services_default, m_callback_group);

        m_deactivate_srv = create_service<cmr_msgs::srv::DeactivateNode>(
            get_effective_namespace() + "/" + get_name() + "/deactivate",
            [](std::shared_ptr<cmr_msgs::srv::DeactivateNode::Request> req,
               std::shared_ptr<cmr_msgs::srv::DeactivateNode::Response> resp) {
                resp->success = cmr::fabric::deactivate_node(req->node_name);
                return resp;
            },
            rmw_qos_profile_services_default, m_callback_group);

        m_cleanup_srv = create_service<cmr_msgs::srv::DeactivateNode>(
            get_effective_namespace() + "/" + get_name() + "/cleanup",
            [](std::shared_ptr<cmr_msgs::srv::DeactivateNode::Request> req,
               std::shared_ptr<cmr_msgs::srv::DeactivateNode::Response> resp) {
                resp->success = cmr::fabric::cleanup_node(req->node_name);
                return resp;
            },
            rmw_qos_profile_services_default, m_callback_group);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto lifecycle_manager = std::make_shared<LifecycleManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lifecycle_manager);
    executor.spin();
    // use multithreaded executor to allow multiple requests at once
    // also allow an activation to depend on another activation which is a node
    // running on the same thread as the first node
    rclcpp::shutdown();
    return 0;
}