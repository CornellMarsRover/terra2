#pragma once
#include <string>

namespace cmr::fabric
{
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
 */
bool cleanup_node(const std::string& node_name);
}  // namespace cmr::fabric