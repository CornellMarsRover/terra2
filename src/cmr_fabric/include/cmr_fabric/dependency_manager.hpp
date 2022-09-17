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

/** The reason why a node is deactivated */
enum class DeactivationReason : uint8_t {
    /** Node has been deactivated due to an error transition */
    Error,
    /** Node has been deactivated  */
    Manual,
};

/**
 * @brief The DependencyHandler class is responsible for managing the dependencies.
 *
 * A node that contains a dependency manager becomes part of the dependency DAG. Each
 * node in the DAG should be on a separate thread or be in a separate process. This
 * is because `DependencyHandler` requests block the calling thread. I debated with
 * using asynchronous requests, however synchronous was cleaner, especially because
 * we want to make sure that a request finished before we move on so that all
 * dependencies are active before the dependender node is activated. We also want to
 * make sure we get a response so that we can correctly react if something fails.
 * The DependencyHandler is a move away from a centralized dependency manager. This
 * was done to deal with larger dependency graphs. With a centralized manager, we
 * dependency paths longer than length 1 become challenging because of the
 * synchronous services used. If a service activates a node, and that activation
 * triggers the activation of another node (and thus a call to the centralized
 * dependency manager), then a centralized dependency manager could deadlock as it
 * waits for itself. This problem could be handled with multithreaded exeuctors or
 * asynchonous services and a scheduler, however a decentralized approach seemed much
 * simpler. Furthermore, if asynchronous services are desired, a decentralized
 * approach would be mucher easier to implement that.
 *
 * ## Usage
 *
 * The DependencyHandler exposes the following service interface:
 *
 * * `/<node>/acquire` - Acquires `<node>` as a dependency for the caller
 * (`cmr_msgs::srv::AcquireDependency`)
 *
 * * `/<node>/release` - Releases `<node>` as a dependency for the caller. If
 * `<node>` was started by an `acquire` call and no longer has any active nodes
 * depending on it, `<node>` will deactivate itself.
 * (`cmr_msgs::srv::ReleaseDependency`)
 *
 * * `/<node>/notify_deactivate` - Notifies `<node>` that one of its dependencies has
 * just deactivated. This is essentially `release` but in the other direction.
 * (`cmr_msgs::srv::NotifyDeactivate`)
 *
 * To use the DependencyHandler, a node should have a `DependencyHandler` has a
 * member, and initialize it in the node's constructor.
 *
 * The DependencyHandler will hold a reference to the node that is passed to it
 * during construction. Thus, the DependencyHandler should not outlive it.
 *
 * Vocabulary Note: I tend to use `depender` to mean the node that depends on the
 * current node and `dependency` to mean a node the current node
 * depends on.
 *
 *
 */
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
     * DependencyHandler and it must be a `FabricNode`, specifically it needs to have
     * `composition_ns` and `restart_delay` parameters
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
    bool notify_deactivate(DeactivationReason reason, int32_t restart_delay = 0);

  private:
    rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
    create_acquire_dependency_service(rclcpp_lifecycle::LifecycleNode& node);

    rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
    create_release_dependency_service(rclcpp_lifecycle::LifecycleNode& node);

    rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
    create_notify_deactivate_service(rclcpp_lifecycle::LifecycleNode& node);
};
}  // namespace cmr::fabric