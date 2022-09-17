#include "cmr_fabric/dependency_manager.hpp"

#include "cmr_fabric/lifecycle_manager.hpp"
#include "cmr_fabric/lifecycle_states.hpp"
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
/*
rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
DependencyHandler::create_notify_deactivate_service(
    rclcpp_lifecycle::LifecycleNode& node)
{
    return node.create_service<cmr_msgs::srv::NotifyDeactivate>(
        cmr::build_string(node.get_namespace(), "/", node.get_name(),
                          "/notify_deactivate"),
        [this, &node](
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request> request,
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>
                response) {
            const auto node_name = request->node_name;
            if (m_dependencies.find(node_name) != m_dependencies.end()) {
                std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request> req;
                for (const auto& depender : m_dependers) {
                    cmr::send_request<cmr_msgs::srv::NotifyDeactivate>(
                        depender + "/notify_deactivate", node_name);
                        const std::shared_ptr<typename SrvT::Request> request)
                }
            }
        });
}*/

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

DependencyHandler::DependencyHandler(rclcpp_lifecycle::LifecycleNode& node)
    : m_dependencies({}), m_dependers({}), m_started_as_dep(false), m_node(node)
{
    m_acquire_dependency_srv = create_acquire_dependency_service(node);
    m_release_dependency_srv = create_release_dependency_service(node);
}
}  // namespace cmr::fabric
