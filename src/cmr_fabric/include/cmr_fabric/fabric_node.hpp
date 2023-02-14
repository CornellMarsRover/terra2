#pragma once

#include "cmr_fabric/lifecycle_client.hpp"
#include "cmr_fabric/lifecycle_helpers.hpp"
#include "cmr_fabric/lifecycle_servers.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rclcpp_action
{
// Forward declare the ActionServer class
template <typename ActionT>
class Server;
}  // namespace rclcpp_action

namespace toml
{
// Forward declare the toml::Table class
class Table;
}  // namespace toml

namespace cmr::fabric
{

// Forward declare the ActionServerPolicy class
template <typename ActionT>
class ActionServerPolicy;

/** A struct that provides semantic typing when an argument is a path */
struct FabricConfigPath {
    std::string path;
};

/**
 * @brief Configuration parameters for a FabricNode
 *
 */
struct FabricNodeConfig {
    std::string node_name;
    std::string composition_namespace;
    /** Either a path to a config file or the data of a config file */
    std::variant<FabricConfigPath, std::string> toml_config;
};

/**
 * @brief The FabricNode is the base class for All CMR nodes
 *
 * Derived classes must implement the `configure`, `activate`, `deactivate`,
 * and `cleanup` methods.
 *
 * This node sets up the ROS 2 lifecycle services to communicate with the CMR
 * Dependency and Lifecycle manager nodes.
 *
 * When a FabricNode is activated or deactivated, it communicates with the dependency
 * manager to ensure all dependencies are running and on deactivation, notifies the
 * dependency manager that those dependents are no needed by this node. The
 * dependency manager will activate all dependents via the lifecycle manager.
 *
 * If an error occurs in a state transition or if the `error_transition` method is
 * invoked, the FabricNode communicates with the fault handler to notify it of the
 * error. The fault handler will then schedule this node for restart.
 *
 * When a Fabric Node is created, it is passed a configuration file in
 * TOML. It must contain the following section:
 *
 * ```
 * [fault_handling]
 * restart_attempts = <int>
 * restart_delay = <int>
 * ```
 *
 * The `restart_attempts` is the number of times a Fabric Node will attempt to
 * restart when an error occurs. The `restart_delay` is the number of seconds to wait
 * before restarting the node when it fails. A fabric node is initially in the
 * inactive state after finishing its configuration. It then must be activated which
 * can be done from the topic interface provided by `LifecycleManager` or from C++
 * via `cmr::fabric::activate_node()`.
 *
 * Upon activation, the node will communicate with `cmr::fabric::DependencyHandler`
 * to first activate any dependencies. The same process will happen
 * in reverse on deactivation to deactivate any nodes that were activated for the
 * sole purpose of being the dependency.
 *
 * When an error occurs, `FabricNode::schedule_restart()` should be called, which
 * will schedule the node for restart by `cmr::fabric::FaultHandler`. When using the
 * `cmr::fabric::LifecycleServer` which can be created by factory methods
 * `create_lifecycle_timer()`, `create_lifecycle_subscription()`, etc., all
 * exceptions thrown in the Callback will be caught and trigger a call to
 * `FabricNode::schedule_restart()`. Furthermore, any assertions called within
 * the server callback functions will also trigger a node restart.
 *
 * A node restart shuts down all nodes that depend on the faulting node and restarts
 * all of them in reverse topological order to maintain the constraint that
 * a node is never active for a nontrivial period of time without its dependencies.
 *
 * ## ROS Params:
 *
 * - `composition_ns` - the namespace to run the node in, which will be the namespace
 * the node uses for the `~/recover_fault` service to communicate with
 * `cmr::fabric::FaultHandler`
 * - `config_path` - the path to the TOML config file
 * - `config_data` - the contents of the TOML config file. Will be overriden by
 * `config_path` if both parameters are specefied
 *
 * ## TOML Params:
 *
 * - `restart_attempts` - See above
 * - `restart_delay` - See above
 * - `dependencies` - List of node names that the node depends upon
 *
 */
class FabricNode : public rclcpp_lifecycle::LifecycleNode
{
  private:
    std::unique_ptr<class DependencyHandler> m_dependency_manager;

    /** The name of the namespace */
    std::string m_composition_namespace;
    /**
     * A client which communicates to the fault handler to restart us if we
     * encounter an error
     */
    std::shared_ptr<rclcpp::Client<cmr_msgs::srv::RecoverFault>>
        m_recover_fault_client;

    bool m_processing_fault = false;

  public:
    explicit FabricNode(const std::optional<FabricNodeConfig>& config = std::nullopt,
                        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    explicit FabricNode(std::optional<FabricNodeConfig>&& config = std::nullopt)
        : FabricNode(config)
    {
    }

    // friend function to allow it to access the error handling method while still
    // not adding excess dependencies on rclcpp_action
    template <typename ActionT>
    friend std::unique_ptr<LifecycleServer<ActionServerPolicy<ActionT>>>
    create_lifecycle_action_server(
        FabricNode& node, const std::string& action_name,
        const typename rclcpp_action::Server<ActionT>::GoalCallback& goal_callback,
        const typename rclcpp_action::Server<ActionT>::CancelCallback&
            cancel_callback,
        const typename rclcpp_action::Server<ActionT>::AcceptedCallback&
            accepted_callback);

    // definition in source file to allow forward declaration of DependencyManager
    ~FabricNode() override;

    FabricNode(const FabricNode&) = delete;
    FabricNode& operator=(const FabricNode&) = delete;
    FabricNode(FabricNode&&) = delete;
    FabricNode& operator=(FabricNode&&) = delete;

  private:
    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(
        const rclcpp_lifecycle::State&) override;

    /**
     * @brief Cleans up any state when an error occurs
     *
     * @param current_state the state befre the error
     * @return true if the error was handled successfully
     */
    bool cleanup_on_error(const rclcpp_lifecycle::State& current_state);

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
    virtual bool configure(const std::shared_ptr<toml::Table>&) = 0;

    /**
     * Derived  class hook for activation.
     *
     * > This method is expected to do any final preparations to start executing.
     * > This may include acquiring resources that are only held while the node is
     * > actually active, such as access to hardware. Ideally, no preparation that
     * > requires significant time (such as lengthy hardware initialisation) should
     * > be performed in this callback.
     *
     * This method MUST have the strong error guarantee. If it returns false, or an
     * exception is thrown `deactivate()` will NOT be called.
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

  protected:
    /**
     * @brief Transitions us to the error state and schedules a restart.
     *
     * The transition is made manually, as ROS 2 does not support transitions to
     * ErrorProcessing from a PrimaryState
     */
    void error_transition();

    /**
     * Schedules a restart by sending a request to the fault handler.
     * The restart will be scheduled `t` seconds after the fault handler receives our
     * request where `t` is the value of the `restart_delay` parameter.
     *
     * @return true if the restart was scheduled successfully
     */
    bool schedule_restart();

    /**
     * @defgroup LifecycleFactory
     * Creates lifecycle versions of clients and servers that communicate with
     * ROS such as publishers, subscribers, timers, and services.
     * @{
     */

    /**
     * @brief Create a lifecycle subscription object that can be activated and
     * deactivated and will call the fabric node error transition when an exception
     * is thrown from the callback
     *
     * @tparam MessageT type of message to receive
     * @tparam FuncT callback function
     * @param topic topic to subscribe to
     * @param qos quality of service settings
     * @param callback callback function
     * @param options optional subscription options
     * @return a lifecycle subscription object
     */
    template <typename MessageT, typename FuncT>
    auto create_lifecycle_subscription(
        const std::string& topic, const rclcpp::QoS& qos, FuncT&& callback,
        const rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>&
            options =
                rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>())
    {
        typename LifecycleSubscription<MessageT>::config_t config(topic, qos,
                                                                  options);
        return std::make_unique<LifecycleSubscription<MessageT>>(
            *this, [this]() { return this->error_transition(); }, config,
            std::forward<FuncT>(callback));
    }

    /**
     * @brief Create a lifecycle publisher object that can be activated and
     * deactivated.
     *
     * This is a wrapper of `LifecycleNode::create_publisher()` to provide more
     * consitent naming with the other lifecycle creation methods.
     *
     * @tparam MessageT type of message to publish
     * @tparam AllocatorT allocator type
     * @param topic topic to publish to
     * @param qos quality of service settings
     * @param options optional publisher options
     * @return a lifecycle publisher object
     */
    template <typename MessageT>
    auto create_lifecycle_publisher(const std::string& topic_name,
                                    const rclcpp::QoS& qos)
    {
        return create_publisher<MessageT>(topic_name, qos);
    }

    /**
     * @brief Create a lifecycle client object that can be activated and
     * deactivated
     *
     * @tparam ServiceT type of service to create
     * @param service_name name of the service
     * @param qos_profile quality of service settings
     * @return a lifecycle client object
     */
    template <typename ServiceT>
    auto create_lifecycle_client(
        const std::string& service_name,
        const rmw_qos_profile_t& qos_profile = rmw_qos_profile_services_default)
    {
        rcl_client_options_t client_options = rcl_client_get_default_options();
        client_options.qos = qos_profile;
        return std::make_unique<LifecycleClient<ServiceT>>(
            get_node_base_interface().get(), get_node_graph_interface(),
            service_name, client_options);
    }

    /**
     * @brief Create a lifecycle service listener that can be activated and
     * deactivated and will call the fabric node error transition when an exception
     * is thrown from the callback
     *
     * This listener will not execute the callback until it is activated or after it
     * is deactivated
     *
     * @tparam ServiceT
     * @tparam FuncT
     * @param service_name name of the service
     * @param callback callback function
     * @param qos_profile quality of service settings
     * @param group optional execution group to add the service to
     * @return a lifecycle service listener object
     */
    template <typename ServiceT, typename FuncT>
    auto create_lifecycle_service(
        const std::string& service_name, FuncT&& callback,
        const rmw_qos_profile_t& qos_profile = rmw_qos_profile_services_default,
        const rclcpp::CallbackGroup::SharedPtr group = nullptr)
    {
        ServiceConfig config(service_name, qos_profile, group);
        return std::make_unique<LifecycleService<ServiceT>>(
            *this, [this]() { return this->error_transition(); }, config,
            std::forward<FuncT>(callback));
    }

    /**
     * @brief Create a lifecycle timer object that can be activated and deactivated
     * and will call the fabric node error transition when an exception is thrown
     *
     * @tparam CallbackT callback type
     * @tparam RepT duration type
     * @tparam PeriodT duration period
     * @param period period for the timer
     * @param callback callback to execute for the timer. Should return `void` and
     * take no arguments or a `TimerBase&`
     * @return a lifecycle timer object
     */
    template <typename CallbackT, typename RepT, typename PeriodT>
    auto create_lifecycle_timer(const std::chrono::duration<RepT, PeriodT>& period,
                                CallbackT&& callback)
    {
        typename WallTimerServerPolicy<RepT, PeriodT>::Config config = {period};
        return std::make_unique<LifecycleTimer<RepT, PeriodT>>(
            *this, [this]() { return this->error_transition(); }, config,
            std::forward<CallbackT>(callback));
    }

    /** @} */
};

}  // namespace cmr::fabric
