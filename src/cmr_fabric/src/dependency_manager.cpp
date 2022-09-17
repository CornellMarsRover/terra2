#include "cmr_fabric/dependency_manager.hpp"

#include "cmr_fabric/lifecycle_manager.hpp"
#include "cmr_fabric/lifecycle_states.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_utils/string_utils.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cmr::fabric
{

// NOLINTNEXTLINE
using namespace lifecycle_msgs::msg;

static bool transition_to_active(rclcpp_lifecycle::LifecycleNode& node)
{
    bool success = true;
    const auto current_state = node.get_current_state().id();
    if (current_state == State::PRIMARY_STATE_ACTIVE) {
        return true;
    }
    if (current_state == State::PRIMARY_STATE_UNCONFIGURED) {
        success = node.trigger_transition(
                          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)
                      .id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }
    // inactive or finalized
    return success &&
           node.trigger_transition(
                   lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)
                   .id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

static bool transition_to_inactive(rclcpp_lifecycle::LifecycleNode& node)
{
    const auto current_state = node.get_current_state().id();
    if (current_state == State::PRIMARY_STATE_INACTIVE ||
        current_state == State::PRIMARY_STATE_UNCONFIGURED) {
        return true;
    }
    // active or finalized
    return node.trigger_transition(
                   lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
               .id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
DependencyHandler::create_release_dependency_service(
    rclcpp_lifecycle::LifecycleNode& node)
{
    return node.create_service<cmr_msgs::srv::ReleaseDependency>(
        cmr::build_string(node.get_name(), "/release"),
        [this, &node](
            const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request> request,
            const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>
                response) {
            const auto depender = request->depender;
            m_dependers.erase(depender);
            if (m_dependers.empty() && m_started_as_dep) {
                response->success = transition_to_inactive(node);
            } else {
                response->success = true;
            }
        });
}

rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
DependencyHandler::create_acquire_dependency_service(
    rclcpp_lifecycle::LifecycleNode& node)
{
    return node.create_service<cmr_msgs::srv::AcquireDependency>(
        cmr::build_string(node.get_name(), "/acquire"),
        [&node, this](
            const std::shared_ptr<cmr_msgs::srv::AcquireDependency::Request> request,
            std::shared_ptr<cmr_msgs::srv::AcquireDependency::Response> response) {
            const auto depender = request->depender;
            if (node.get_current_state().id() == State::PRIMARY_STATE_UNCONFIGURED) {
                m_started_as_dep = true;
            }
            response->success = transition_to_active(node);
            if (response->success) {
                m_dependers.insert(depender);
            }
        });
}

/**
 * @brief Notify all the nodes in `dependers` that `node` is being deactivated
 *
 * @param node the node being deactivated
 * @param dependers the nodes that depend on `node`. All nodes successfully notified
 * will be removed from this set
 *
 * @return true if all nodes were successfully notified
 */
static bool notify_deactivation_to_dependenders(
    rclcpp_lifecycle::LifecycleNode& node,
    std::unordered_set<std::string>& dependers, DeactivationReason reason,
    int32_t restart_delay)
{
    const auto req = std::make_shared<cmr_msgs::srv::NotifyDeactivate::Request>();
    req->node_name = node.get_name();
    req->deactivation_reason = static_cast<uint8_t>(reason);
    req->restart_delay = restart_delay;
    bool success = true;
    std::vector<std::string> notified_nodes;
    for (const auto& depender : dependers) {
        RCLCPP_INFO(node.get_logger(), "Notifying %s of deactivation of %s",
                    depender.c_str(), node.get_name());
        auto opt = cmr::send_request<cmr_msgs::srv::NotifyDeactivate>(
            depender + "/notify_deactivate", req);
        const auto notify_success = opt && opt.value()->success;
        if (!notify_success) {
            RCLCPP_ERROR(node.get_logger(),
                         "Failed to notify %s about deactivation of %s",
                         depender.c_str(), node.get_name());
        } else {
            notified_nodes.push_back(depender);
        }
        success &= notify_success;
    }
    for (const auto& depender : notified_nodes) {
        dependers.erase(depender);
    }
    RCLCPP_INFO(node.get_logger(), "Notified dependenders success: %d", success);
    return success;
}

/**
 * @brief Determines if we should ask the fault handler to restart outselves
 *
 * @param reason the reason why we are being deactivated. If this is an error, we
 * will restart
 * @param restart_delay the delay before restarting, taking into account the delay of
 * our dependencies
 * @param node
 *
 * @return true if we don't need to restart or the scheduling was successful
 */
static bool trigger_restart_if_necessary(DeactivationReason reason,
                                         int32_t restart_delay,
                                         rclcpp_lifecycle::LifecycleNode& node)
{
    const auto comp_namespace = node.get_parameter("composition_ns").as_string();
    if (reason == DeactivationReason::Error) {
        auto req = std::make_shared<cmr_msgs::srv::RecoverFault::Request>();
        req->node_name = node.get_name();
        req->restart_delay = restart_delay;
        const auto resp = cmr::send_request<cmr_msgs::srv::RecoverFault>(
            "/" + comp_namespace + "/recover_fault", req);
        return resp.has_value();
    }
    return true;
}

rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
DependencyHandler::create_notify_deactivate_service(
    rclcpp_lifecycle::LifecycleNode& node)
{
    return node.create_service<cmr_msgs::srv::NotifyDeactivate>(
        cmr::build_string("/", node.get_name(), "/notify_deactivate"),
        [this, &node](
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request> request,
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>
                response) {
            const auto node_name = request->node_name;
            if (auto it = m_dependencies.find(node_name);
                it != m_dependencies.end()) {
                // setthing this to false prevents us trying to release this
                // dependency
                it->second = false;
                const auto node_restart_delay = static_cast<int32_t>(
                    node.get_parameter("restart_delay").as_int());
                response->success =
                    notify_deactivation_to_dependenders(
                        node, m_dependers,
                        static_cast<DeactivationReason>(
                            request->deactivation_reason),
                        request->restart_delay + node_restart_delay) &&
                    transition_to_inactive(node) &&
                    trigger_restart_if_necessary(
                        static_cast<DeactivationReason>(
                            request->deactivation_reason),
                        request->restart_delay + node_restart_delay, node);
            } else {
                response->success = false;
                RCLCPP_WARN(
                    node.get_logger(),
                    "Received deactivation notification from a non-dependency");
            }
        });
}

bool DependencyHandler::acquire_all_dependencies()
{
    for (auto& [dep_name, dep_acquired] : m_dependencies) {
        auto request = std::make_shared<cmr_msgs::srv::AcquireDependency::Request>();
        request->depender = m_node.get().get_name();
        RCLCPP_INFO(m_node.get().get_logger(),
                    "Sending activation to %s/acquire from %s", dep_name.c_str(),
                    m_node.get().get_name());
        const auto response = cmr::send_request<cmr_msgs::srv::AcquireDependency>(
            dep_name + "/acquire", request);
        if (!response) {
            RCLCPP_ERROR(m_node.get().get_logger(),
                         "Failed to acquire dependency %s", dep_name.c_str());
            return false;
        } else {
            dep_acquired = true;
        }
    }
    return true;
}
bool DependencyHandler::release_all_dependencies()
{
    bool success = true;
    for (auto& [dep_name, dep_acquired] : m_dependencies) {
        if (!dep_acquired) {
            continue;
        }
        auto request = std::make_shared<cmr_msgs::srv::ReleaseDependency::Request>();
        request->depender = m_node.get().get_name();
        const auto response = cmr::send_request<cmr_msgs::srv::ReleaseDependency>(
            dep_name + "/release", request);
        if (!response) {
            RCLCPP_ERROR(m_node.get().get_logger(),
                         "Failed to release dependency %s", dep_name.c_str());
            success = false;
        } else {
            dep_acquired = false;
        }
    }
    return success;
}

bool DependencyHandler::notify_deactivate(DeactivationReason reason,
                                          int32_t restart_delay)
{
    return notify_deactivation_to_dependenders(m_node.get(), m_dependers, reason,
                                               restart_delay);
}

DependencyHandler::DependencyHandler(rclcpp_lifecycle::LifecycleNode& node)
    : m_dependencies({}), m_dependers({}), m_started_as_dep(false), m_node(node)
{
    m_acquire_dependency_srv = create_acquire_dependency_service(node);
    m_release_dependency_srv = create_release_dependency_service(node);
    m_notify_deactivate_srv = create_notify_deactivate_service(node);
}
}  // namespace cmr::fabric
