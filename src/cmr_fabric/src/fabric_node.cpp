#include "cmr_fabric/fabric_node.hpp"

#include <chrono>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "cmr_utils/cmr_error.hpp"
#include "cmr_utils/services.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

namespace cmr::fabric
{

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto config_path = get_parameter("config_path").as_string();
    if (config_path.empty()) {
        CMR_LOG(ERROR, "No config_path parameter specified");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    auto toml = toml::parseFile(config_path);

    if (!toml.table) {
        CMR_LOG(ERROR, "Failed to parse config file: %s", toml.errmsg.c_str());
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    auto config = toml.table;

    auto fault_handling_settings = config->getTable("fault_handling");
    auto [ok, restart_attempts] =
        fault_handling_settings->getInt("restart_attempts");
    if (!ok) {
        CMR_LOG(ERROR, "Failed to parse fault_handling.restart_attempts: %s",
                toml.errmsg.c_str());
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    set_parameter(rclcpp::Parameter("restart_attempts", restart_attempts));

    auto [delay_ok, restart_delay] =
        fault_handling_settings->getInt("restart_delay");
    if (!delay_ok) {
        CMR_LOG(ERROR, "Failed to parse fault_handling.restart_delay: %s",
                toml.errmsg.c_str());
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    set_parameter(rclcpp::Parameter("restart_delay", restart_delay));

    auto dependencies = config->getArray("dependencies");
    if (dependencies) {
        this->m_dependencies = *dependencies->getStringVector();
    }

    return configure(config)
               ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
               : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_activate(
    const rclcpp_lifecycle::State &)
{
    for (const auto &dep_name : this->get_dependencies()) {
        auto request = std::make_shared<cmr_msgs::srv::AcquireDependency::Request>();
        request->dependent = get_name();
        request->target = dep_name;
        auto response = cmr::send_request<cmr_msgs::srv::AcquireDependency>(
            m_composition_ns + "/acquire", request);
        if (!response) {
            CMR_LOG(ERROR, "Failed to acquire dependency %s", dep_name.c_str());
            return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
        }
    }
    return activate() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                      : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    for (const auto &dep_name : this->get_dependencies()) {
        auto request = std::make_shared<cmr_msgs::srv::ReleaseDependency::Request>();
        request->dependent = get_name();
        request->target = dep_name;
        auto response = cmr::send_request<cmr_msgs::srv::ReleaseDependency>(
            m_composition_ns + "/release", request);
        if (!response) {
            CMR_LOG(ERROR, "Failed to acquire dependency %s", dep_name.c_str());
            return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
        }
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

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_error(
    const rclcpp_lifecycle::State &)
{
    schedule_restart();
    // returning SUCCESS will tell ROS2 to move the node into the Unconfigured
    // state.
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void FabricNode::schedule_restart()
{
    int64_t num_restarts = get_parameter("num_restarts").as_int();

    if (num_restarts >= get_parameter("restart_attempts").as_int()) {
        // reset the counter in case the user wants to try and enable this again
        // later
        set_parameter(rclcpp::Parameter("num_restarts", 0));
        CMR_LOG(ERROR,
                "Node will not attempt to restart because it has restarted "
                "the maximum amount of times.");
        return;
    }

    // make sure the fault handler is available before doing anything else
    while (!m_recover_fault_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            CMR_LOG(ERROR,
                    "Interrupted while waiting for the fault handler. Not "
                    "scheduling restart.");
            return;
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
}

void FabricNode::panic()
{
    this->trigger_transition(
        rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE));
    this->schedule_restart();
}

}  // namespace cmr::fabric
