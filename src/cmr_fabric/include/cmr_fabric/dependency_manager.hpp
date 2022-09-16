#include <unordered_map>
#include <unordered_set>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{
/**
 * @brief TODO(@fad35)
 *
 */
class DependencyManager : public rclcpp::Node
{
  private:
    /**
     * maps names of acquired nodes to the names of the nodes that depend on
     * them
     */
    std::unordered_map<std::string, std::unordered_set<std::string>> m_users;

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

    rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
    create_acquire_dependency_service();

    const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>&
    release_dependency_callback(
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>& request,
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>& response);

    rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
    create_release_dependency_service();

    bool activate_dependency(const std::string& target);

    bool deactivate_dependency(const std::string& target);

  public:
    explicit DependencyManager(const std::string& node_name = "dependency_manager",
                               const std::string& node_namespace = "fabric");
};
}  // namespace cmr::fabric