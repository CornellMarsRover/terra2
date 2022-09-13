#pragma once

#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/external/tomlcpp.hxx"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cmr::fabric
{

/**
 * @brief The FabricNode is the base class for All CMR nodes
 *
 * Derived classes must implement the `configure`, `activate`, `deactivate`,
 * and `cleanup` methods.
 *
 * This node sets up the ROS 2 lifecycle services to communicate with the CMR
 * Dependency and Lifecycle manager nodes
 *
 */
class FabricNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
    FabricNode();

    ~FabricNode() override = default;

    /**
     * @brief Returns the vector of dependencies defined by this node. There is no
     * guarantee that everything in this list is a valid dependency (i.e. that they
     * each exist as a node).
     *
     * @return std::vector<std::string> list of dependency names
     */
    std::vector<std::string> get_dependencies() { return m_dependencies; }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(
        const rclcpp_lifecycle::State &) override;

  private:
    /** The name of the namespace? */
    std::string m_composition_namespace;
    std::shared_ptr<rclcpp::Client<cmr_msgs::srv::RecoverFault>>
        m_recover_fault_client;
    std::vector<std::string> m_dependencies;

    /**
     * Schedules a restart by sending a request to the lifecycle manager.
     *
     * @return true if the restart was scheduled successfully
     */
    bool schedule_restart();

    /**
     * Derived class hook for configuring the node.
     *
     * > callback will be called to allow the node to load its configuration and
     * > conduct any required setup.
     * >
     * > The configuration of a node will typically involve those tasks that must be
     * > performed once during the node’s life time, such as obtaining permanent
     * > memory buffers and setting up topic publications/subscriptions that do not
     * > change.
     * >
     * > The node uses this to set up any resources it must hold throughout its life
     * > (irrespective of if it is active or inactive). As examples, such resources
     * > may include topic publications and subscriptions, memory that is held
     * > continuously, and initialising configuration parameters.
     *
     * @return true on success, false otherwise which will cause transition to
     * `ErrorProcessing`
     */
    virtual bool configure(const std::shared_ptr<toml::Table> &) = 0;

    /**
     * Derived  class hook for activation.
     *
     * > This method is expected to do any final preparations to start executing.
     * > This may include acquiring resources that are only held while the node is
     * > actually active, such as access to hardware. Ideally, no preparation that
     * > requires significant time (such as lengthy hardware initialisation) should
     * > be performed in this callback.
     *
     * @see `FabricNode::deactivate`
     * @return true on success, false otherwise which will cause transition to
     * `ErrorProcessing`
     */
    virtual bool activate() = 0;

    /**
     * Derived class hook for deactivation.
     *
     * > This method is expected to do any cleanup to start executing, and should
     * > reverse the `activate()` changes
     *
     * @see `FabricNode::activate`
     * @return true on success, false otherwise which will cause transition to
     * `ErrorProcessing`
     */
    virtual bool deactivate() = 0;

    /**
     * Derived class hook for cleanup.
     *
     * > This method is expected to clear all state and return the node to a
     * > functionally equivalent state as when first created. If the cleanup cannot
     * > be successfully achieved it will transition to `ErrorProcessing`
     *
     * This should essentially undo `configure()`
     *
     * @return true on success, false otherwise which will cause transition to
     * `ErrorProcessing`
     */
    virtual bool cleanup() = 0;
};

}  // namespace cmr::fabric
