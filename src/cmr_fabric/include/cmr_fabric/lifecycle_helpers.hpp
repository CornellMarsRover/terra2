#pragma once
#include <string>

namespace cmr::fabric
{

/**
 * A LifecycleState is Fabric's representation of the state of a node.
 * This is a typed wrapper around ROS 2's lifecycle state which is represented as an
 * integer.
 *
 * [Read more about ROS 2's lifecycle
 * states](https://design.ros2.org/articles/node_lifecycle.html)
 */
enum class LifecycleState : uint8_t {
    /** An error ocurred while trying to read a node's state **/
    Unknown,
    /** Node has been created, but its persisted configuration has not been loaded.
     */
    Unconfigured,
    /** Node has been configured, but it's not doing anything. */
    Inactive,
    /** Node is actively having effects. */
    Active,
    /** Node has shut down, and can no longer be configured or activated. */
    Finalized,
    /** Node is transitioning from Inactive to Active */
    Activating,
    /** Node is transitioning from Active to Inactive */
    Deactivating,
    /** Node is transitioning from Unconfigured to Inactive */
    Configuring,
    /** Node is transitioning from Inactive to Unconfigured */
    CleaningUp,
    /** Node is transitioning from to Finalized */
    ShuttingDown,
    /** Node is transitiong to Unconfigured due to an error in a state transition
       callback */
    ErrorProcessing,
};

/**
 * Uses the `LifecycleNode` exposed services to activate `node_name`.
 *
 * If `node_name` is already active, this function does nothing and returns true.
 * This will block the caller thread.
 */
bool activate_node(const std::string& node_name);

/**
 * Uses the `LifecycleNode` exposed services to deactivate `node_name`.
 *
 * If `node_name` is already inactive, this function does nothing and returns true.
 * This will block the caller thread.
 */
bool deactivate_node(const std::string& node_name);

/**
 * Uses the `LifecycleNode` exposed services to cleanup `node_name`.
 *
 * If `node_name` is already unconfigured, this function does nothing and returns
 * true.
 * This will block the caller thread.
 */
bool cleanup_node(const std::string& node_name);

/**
 * @brief Get the LifecycleState of a lifecycle node.
 * This will block the caller thread.
 */
LifecycleState get_lifecycle_state(const std::string& node_name);
}  // namespace cmr::fabric