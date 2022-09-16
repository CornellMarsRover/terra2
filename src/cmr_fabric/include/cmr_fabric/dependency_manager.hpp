#pragma once
#include <unordered_map>
#include <unordered_set>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/notify_deactivate.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{
/**
 * The dependency manager provides a service interface for nodes to acquire their
 * dependencies.
 *
 * When a node acquires its dependencies, we make sure that all nodes it depends on
 * are already running. When it releases its dependencies, will allow the dependent
 * nodes to deactivate if there are no other nodes that depend on them.
 *
 * The acquire service is created on `/<namespace>/acquire` and takes messages of
 * `cmr_msgs::srv::AcquireDependency`.
 * The release service is created on `/<namespace>/release` and takes messages of
 * `cmr_msgs::srv::ReleaseDependency`.
 */
class DependencyManager : public rclcpp::Node
{
  private:
    /**
     * maps names of acquired nodes to the set of names of the nodes that it depends
     * on
     */
    std::unordered_map<std::string, std::unordered_set<std::string>> m_users;

    /**
     * @brief This is the reverse of `m_users`. It maps names of nodes to the set of
     * names of nodes that depend on it
     *
     */
    std::unordered_map<std::string, std::unordered_set<std::string>> m_dependers;

    /** set of nodes that the dependency manager started; nodes in this set will
     * be disabled when they no longer have any users
     */
    std::unordered_set<std::string> m_started_as_deps;

    /**
     * @brief A service on `/<effective_namespace>/acquire` that allows other nodes
     * to indicate that they can only start running after their dependencies are
     * running
     *
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::AcquireDependency>>
        m_acquire_dependency_srv;

    /**
     * @brief A service on `/<effective_namespace>/release` that allows other nodes
     * to know that they are no longer needed by a particular dependency
     *
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReleaseDependency>>
        m_release_dependency_srv;

    /**
     * A service that listens on `/<effective_namespace>/notify_deactivate` that
     * allows a node that is deactivating which is depndened upon by other nodes to
     * deactive those other nodes
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>>
        m_notify_deactivate_srv;

    rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
    create_acquire_dependency_service();

    const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>&
    release_dependency_callback(
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>& request,
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>& response);

    const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>&
    notify_deactivate_callback(
        const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request>& request,
        const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>& response);

    rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
    create_release_dependency_service();

    rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
    create_notify_deactivate_service();

    bool activate_dependency(const std::string& target);

    bool deactivate_dependency(const std::string& target);

  public:
    explicit DependencyManager(const std::string& node_name = "dependency_manager",
                               const std::string& node_namespace = "fabric");
};
}  // namespace cmr::fabric