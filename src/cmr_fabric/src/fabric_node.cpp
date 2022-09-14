#include "cmr_fabric/fabric_node.hpp"

#include <chrono>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

namespace cmr::fabric
{

// we generate an ugle name for the fabric node because they should be named
// appropriately during the launch process
FabricNode::FabricNode()
    : rclcpp_lifecycle::LifecycleNode(
          "fabric_untitled_" +
          std::to_string(
              std::chrono::system_clock::now().time_since_epoch().count()))
{
    declare_parameter("config_path", "");
    declare_parameter("composition_ns", "");
    declare_parameter("restart_attempts", 0);
    declare_parameter("restart_delay", 0);
    declare_parameter("num_restarts", 0);

    m_composition_namespace = get_parameter("composition_ns").as_string();
    m_recover_fault_client = this->create_client<cmr_msgs::srv::RecoverFault>(
        m_composition_namespace + "/recover_fault");
}

// Basic guarantee. This is ok because if this fails, entire node is reset anyway
bool set_config_params(FabricNode &node, const toml::Result &toml)
{
    const auto table = toml.table;

    if (!table) {
        RCLCPP_ERROR(node.get_logger(), "Failed to parse config file: %s",
                     toml.errmsg.c_str());
        return false;
    }

    const auto fault_handling_settings = table->getTable("fault_handling");
    const auto [ok, restart_attempts] =
        fault_handling_settings->getInt("restart_attempts");
    if (!ok) {
        RCLCPP_ERROR(node.get_logger(),
                     "Failed to parse fault_handling.restart_attempts: %s",
                     toml.errmsg.c_str());
        return false;
    }
    node.set_parameter(rclcpp::Parameter("restart_attempts", restart_attempts));

    const auto [delay_ok, restart_delay] =
        fault_handling_settings->getInt("restart_delay");
    if (!delay_ok) {
        RCLCPP_ERROR(node.get_logger(),
                     "Failed to parse fault_handling.restart_delay: %s",
                     toml.errmsg.c_str());
        return false;
    }
    node.set_parameter(rclcpp::Parameter("restart_delay", restart_delay));
    return true;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_configure(
    const rclcpp_lifecycle::State &)
{
    const auto config_path = get_parameter("config_path").as_string();
    if (config_path.empty()) {
        CMR_LOG(ERROR, "No config_path parameter specified");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    const auto toml = toml::parseFile(config_path);

    if (!set_config_params(*this, toml)) {
        // error logging done in set_config_params
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    const auto config = toml.table;

    const auto dependencies = config->getArray("dependencies");
    if (dependencies) {
        this->m_dependencies = *dependencies->getStringVector();
        m_activated_dependencies = std::vector<bool>(m_dependencies.size(), false);
    }

    return configure(config)
               ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
               : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

bool FabricNode::activate_dependencies()
{
    CMR_ASSERT(m_dependencies.size() == m_activated_dependencies.size());
    for (size_t i = 0; i < m_dependencies.size(); ++i) {
        const auto &dep_name = m_dependencies[i];
        auto request = std::make_shared<cmr_msgs::srv::AcquireDependency::Request>();
        request->dependent = get_name();
        request->target = dep_name;
        const auto response = cmr::send_request<cmr_msgs::srv::AcquireDependency>(
            m_composition_namespace + "/acquire", request);
        if (!response) {
            CMR_LOG(ERROR, "Failed to acquire dependency %s", dep_name.c_str());
            return false;
        } else {
            m_activated_dependencies[i] = true;
        }
    }
    return true;
}

bool FabricNode::deactivate_dependencies()
{
    CMR_ASSERT(m_dependencies.size() == m_activated_dependencies.size());
    bool success = true;
    for (size_t i = 0; i < m_dependencies.size(); ++i) {
        if (!m_activated_dependencies[i]) {
            continue;
        }
        const auto &dep_name = m_dependencies[i];
        auto request = std::make_shared<cmr_msgs::srv::ReleaseDependency::Request>();
        request->dependent = get_name();
        request->target = dep_name;
        const auto response = cmr::send_request<cmr_msgs::srv::ReleaseDependency>(
            m_composition_namespace + "/release", request);
        if (!response) {
            CMR_LOG(ERROR, "Failed to release dependency %s", dep_name.c_str());
            success = false;
        } else {
            m_activated_dependencies[i] = false;
        }
    }
    return success;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_activate(
    const rclcpp_lifecycle::State &)
{
    if (!activate_dependencies()) {
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return activate() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                      : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    if (!deactivate_dependencies()) {
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return deactivate() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                        : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    return cleanup() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                     : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    // We treat shutdown and cleanup the same way; just invoke cleanup()
    return cleanup() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                     : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

// Performs the necessary cleanup for `node` during the `ErrorProcessing` state
bool FabricNode::cleanup_on_error(const rclcpp_lifecycle::State &current_state)
{
    using namespace lifecycle_msgs::msg;  // NOLINT(google-build-using-namespace)
    constexpr auto success =
        rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    constexpr auto error = rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    switch (current_state.id()) {
        case State::PRIMARY_STATE_UNCONFIGURED:
            return true;
        case State::PRIMARY_STATE_ACTIVE: {
            const auto deactive_code = on_deactivate(current_state);
            if (deactive_code == error) {
                return false;
            }
            return on_cleanup(current_state) == success;
        }
        case State::TRANSITION_STATE_ACTIVATING:
            return deactivate_dependencies() && on_cleanup(current_state) == success;
        case State::TRANSITION_STATE_DEACTIVATING:
        case State::PRIMARY_STATE_INACTIVE:
            return on_cleanup(current_state) == success;
        default:
            CMR_LOG(ERROR, "Reached on_error from state %d", current_state.id());
            return false;
    }
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_error(
    const rclcpp_lifecycle::State &current_state)
{
    const auto cleanup_success = cleanup_on_error(current_state);
    if (cleanup_success) {
        // returning SUCCESS will tell ROS2 to move the node into the Unconfigured
        // state.
        // returning ERROR will tell ROS2 to move the node into the Finalized state
        // and prep for destruction
        return schedule_restart()
                   ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                   : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

bool FabricNode::schedule_restart()
{
    const int64_t num_restarts = get_parameter("num_restarts").as_int();
    CMR_LOG(INFO, "Scheduling restart");

    if (num_restarts >= get_parameter("restart_attempts").as_int()) {
        // reset the counter in case the user wants to try and enable this again
        // later
        set_parameter(rclcpp::Parameter("num_restarts", 0));
        CMR_LOG(ERROR,
                "Node will not attempt to restart because it has restarted "
                "the maximum amount of times.");
        return false;
    }

    // make sure the fault handler is available before doing anything else
    while (!m_recover_fault_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            CMR_LOG(ERROR,
                    "Interrupted while waiting for the fault handler. Not "
                    "scheduling restart.");
            return false;
        }
        CMR_LOG(INFO, "Fault handler not available, waiting again...");
    }

    set_parameter(rclcpp::Parameter("num_restarts", num_restarts + 1));

    // Best effort to send the restart request; don't block and wait for the
    // response though though because if it fails, this is emblematic of a more
    // serious problem.
    auto req = std::make_shared<cmr_msgs::srv::RecoverFault::Request>();
    req->node_name = get_name();
    m_recover_fault_client->async_send_request(req);
    // do we want to wait for the response?
    return true;
}
}  // namespace cmr::fabric
