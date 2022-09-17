#pragma once
#include <unordered_map>
#include <unordered_set>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/notify_deactivate.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cmr::fabric
{

class DependencyHandler
{
  private:
    std::unordered_map<std::string, bool> m_dependencies;
    std::unordered_set<std::string> m_dependers;
    bool m_started_as_dep;

    std::reference_wrapper<rclcpp_lifecycle::LifecycleNode> m_node;

    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::AcquireDependency>>
        m_acquire_dependency_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReleaseDependency>>
        m_release_dependency_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>>
        m_notify_deactivate_srv;

  public:
    /**
     * @brief Construct a new Dependency Handler object
     *
     * @param node the parent node for this handler. IT MUST NOT OUTLIVE the
     * DependencyHandler
     */
    explicit DependencyHandler(rclcpp_lifecycle::LifecycleNode& node);

    template <typename It>
    void set_dependencies(It begin, It end)
    {
        m_dependencies.clear();
        for (; begin != end; ++begin) {
            m_dependencies.insert(std::make_pair(*begin, false));
        }
    }

    /**
     * @brief Attempts to acquire all dependencies.
     *
     * Blocking. If this function return false, some dependencies may have been
     * enabled, but can be correctly cleaned up with a subsequent call to
     * `release_all_dependencies`.
     *
     * This function will fail once the first dependency fails to be acquired.
     *
     * @return true if all dependencies were acquired successfully.
     */
    bool acquire_all_dependencies();

    /**
     * @brief Attempts to release all dependencies.
     *
     * Blocking. If this function return false, some dependencies may have been
     * disabled, which is kept track of internally. Therefore a call to
     * `activate_all_dependencies` will correctly undo this function.
     *
     * This function will continue to try and release all other depedencies after a
     * dependency fails to be released.
     *
     * @return true if all dependencies were deactivated successfully.
     */
    bool release_all_dependencies();

    /**
     * @brief Attempts to notify all dependers that this node is being deactivated
     *
     * @return true if all dependers were notified successfully
     */
    bool notify_deactivate();

  private:
    rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
    create_acquire_dependency_service(rclcpp_lifecycle::LifecycleNode& node);

    rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
    create_release_dependency_service(rclcpp_lifecycle::LifecycleNode& node);

    rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
    create_notify_deactivate_service(rclcpp_lifecycle::LifecycleNode& node);
};
}  // namespace cmr::fabric