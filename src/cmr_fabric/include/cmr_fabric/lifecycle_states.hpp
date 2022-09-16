#pragma once

#include "cmr_msgs/msg/state.hpp"

namespace cmr::fabric
{
/** A LifecycleState is Fabric's representation of the state of a node. */
enum class LifecycleState {
    /** Unknown state, indicates that something might be wrong. **/
    Unknown,
    /** Node has been created, but its persisted configuration has not been loaded.
     */
    Unconfigured,
    /** Node has been configured, but it's not doing anything. */
    Inactive,
    /** Node is actively having effects. */
    Active,
    /** Node has shut down, and can no longer be configured or activated. */
    Finalized
};

/** The reason why a node is deactivated */
enum class DeactivationReason : uint8_t {
    /** Node has been deactivated due to an error transition */
    Error,
    /** Node has been deactivated  */
    Manual,
};
}  // namespace cmr::fabric
