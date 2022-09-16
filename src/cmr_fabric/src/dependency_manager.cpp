#include "cmr_fabric/dependency_manager.hpp"

#include "cmr_fabric/lifecycle_states.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cmr::fabric
{
rclcpp::Service<cmr_msgs::srv::AcquireDependency>::SharedPtr
DependencyManager::create_acquire_dependency_service()
{
    const auto acquire_dep_callback =
        [this](
            const std::shared_ptr<cmr_msgs::srv::AcquireDependency::Request>&
                request,
            std::shared_ptr<cmr_msgs::srv::AcquireDependency::Response> response) {
            const auto [dependent, target] = *request;

            CMR_LOG(INFO, "Acquiring dependency for %s of %s", dependent.c_str(),
                    target.c_str());
            if (m_users.find(target) == m_users.end()) {
                const auto activate_response = activate_dependency(target);

                if (!activate_response) {
                    // failed to activate
                    CMR_LOG(ERROR, "Failed to activate dependency %s",
                            target.c_str());
                    response->success = false;
                    return response;
                }

                CMR_LOG(INFO, "Adding %s to m_started_as_deps and m_users",
                        target.c_str());
                m_started_as_deps.insert(target);
                m_users[target] = std::unordered_set<std::string>();
            }

            m_users[target].insert(dependent);
            m_dependers[dependent].insert(target);
            CMR_LOG(INFO, "Added %s to m_users[%s]", dependent.c_str(),
                    target.c_str());

            response->success = true;
            return response;
        };

    return this->create_service<cmr_msgs::srv::AcquireDependency>(
        get_effective_namespace() + "/acquire", acquire_dep_callback);
}

const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>&
DependencyManager::release_dependency_callback(
    const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>& request,
    const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>& response)
{
    const auto [dependent, target] = *request;

    CMR_LOG(DEBUG, "Releasing dependency %s for node %s...", target.c_str(),
            dependent.c_str());

    if (m_users.find(target) == m_users.end()) {
        // target node was never acquired; just return success in
        // this case although this suggests misuse of the dependency
        // manager
        CMR_LOG(WARN,
                "attempting to release node %s, but it was "
                "never acquired",
                target.c_str());
        response->success = true;
        return response;
    }

    m_users[target].erase(dependent);
    m_dependers[dependent].erase(target);

    if (m_users[target].empty()) {
        // no more users of the target node; deactivate it if it was
        // started as a dependency
        m_users.erase(target);
        if (m_started_as_deps.find(target) != m_started_as_deps.end()) {
            const auto deactivate_response = deactivate_dependency(target);
            if (!deactivate_response) {
                // failed to deactivate
                response->success = false;
                return response;
            }
            m_started_as_deps.erase(target);
        }
    }
    response->success = true;
    return response;
}

rclcpp::Service<cmr_msgs::srv::ReleaseDependency>::SharedPtr
DependencyManager::create_release_dependency_service()
{
    const auto release_dep_callback =
        [this](const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>& req,
               const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>&
                   resp) { return release_dependency_callback(req, resp); };

    return this->create_service<cmr_msgs::srv::ReleaseDependency>(
        get_effective_namespace() + "/release", release_dep_callback);
}

/*
 * The following two helpers call the lifecycle management services exposed
 * by dependencies directly. This code looks very similar to how Fabric's
 * lifecycle manager node performs activation and deactivation, which
 * understandably begs the question: why not just use the lifecycle manager's
 * activate and deactivate services? The reason is because the dependency manager
 * is invoked during the lifecycle manager's activate and deactivate callbacks;
 * if we were to call them from within the dependency manager, we would enter
 * into a deadlock because the lifecycle manager's services are busy waiting for
 * the dependency manager, but the dependency manager is busy waiting for the
 * lifecycle manager. So we just have to cut out the middleman here, even if it
 * means some regrettable code duplication.
 */

bool DependencyManager::activate_dependency(const std::string& target)
{
    // check if node is configured
    CMR_LOG(INFO, "Request to activate dependency %s", target.c_str());
    const auto first_response = cmr::send_request<lifecycle_msgs::srv::GetState>(
        "/" + target + "/get_state",
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (first_response &&
        first_response.value()->current_state.id ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        // configure first
        CMR_LOG(INFO, "configuring dependency %s", target.c_str());
        const auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id =
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        const auto response = cmr::send_request<lifecycle_msgs::srv::ChangeState>(
            "/" + target + "/change_state", request);
        if (!response || !response.value()->success) {
            return false;
        }
    }
    // we can activate now
    CMR_LOG(INFO, "activating dependency %s", target.c_str());
    const auto activate_request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    activate_request->transition.id =
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    const auto activate_response =
        cmr::send_request<lifecycle_msgs::srv::ChangeState>(
            "/" + target + "/change_state", activate_request);
    return activate_response && activate_response.value()->success;
}

bool DependencyManager::deactivate_dependency(const std::string& target)
{
    // check if node is configured
    const auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    const auto response = cmr::send_request<lifecycle_msgs::srv::GetState>(
        "/" + target + "/get_state", request);
    if (response && response.value()->current_state.id ==
                        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        // This suggests that something is wrong, but since the node is
        // deactivating anyway, this isn't real cause for alarm enough for an
        // error log.
        CMR_LOG(WARN,
                "Attempted to deactivate dependency %s, but it was already "
                "inactive.",
                target.c_str());
        return true;
    }
    // we can deactivate now
    CMR_LOG(INFO, "deactivating dependency %s", target.c_str());
    const auto activate_request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    activate_request->transition.id =
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    const auto activate_response =
        cmr::send_request<lifecycle_msgs::srv::ChangeState>(
            "/" + target + "/change_state", activate_request);
    return activate_response && activate_response.value()->success;
}

const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>&
DependencyManager::notify_deactivate_callback(
    const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request>&,
    const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>& response)
{
    // TODO(sev47)

    return response;
}

rclcpp::Service<cmr_msgs::srv::NotifyDeactivate>::SharedPtr
DependencyManager::create_notify_deactivate_service()
{
    const auto notify_dep_callback =
        [this](
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Request>& req,
            const std::shared_ptr<cmr_msgs::srv::NotifyDeactivate::Response>& resp) {
            return notify_deactivate_callback(req, resp);
        };

    return this->create_service<cmr_msgs::srv::NotifyDeactivate>(
        get_effective_namespace() + "/notify_deactivate", notify_dep_callback);
}

DependencyManager::DependencyManager(const std::string& node_name,
                                     const std::string& node_namespace)
    : Node(node_name, node_namespace)
{
    m_acquire_dependency_srv = create_acquire_dependency_service();
    m_release_dependency_srv = create_release_dependency_service();
    m_notify_deactivate_srv = create_notify_deactivate_service();
    CMR_LOG(INFO, "dependency manager initialized");
}
}  // namespace cmr::fabric
