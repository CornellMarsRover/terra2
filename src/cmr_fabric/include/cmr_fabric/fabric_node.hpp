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

  protected:
    /**
     * @brief A node can internally call panic() when it has entered an invalid state
     * during its execution. This function must only be called within a callback; if
     * a node's state is illegal outside of a callback, return false in the lifecycle
     * function instead.
     */
    void panic();

  private:
    /** The name of the namespace? */
    std::string m_composition_namespace;
    std::shared_ptr<rclcpp::Client<cmr_msgs::srv::RecoverFault>>
        m_recover_fault_client;
    std::vector<std::string> m_dependencies;

    /** TODO(@fad35) */
    void schedule_restart();

    /** Derived class hook for configuring the node */
    virtual bool configure(const std::shared_ptr<toml::Table> &) = 0;

    /** Derived  class hook for activation */
    virtual bool activate() = 0;

    /** Derived class hook for deactivation */
    virtual bool deactivate() = 0;

    /** Derived class hook for cleanup */
    virtual bool cleanup() = 0;
};

}  // namespace cmr::fabric
