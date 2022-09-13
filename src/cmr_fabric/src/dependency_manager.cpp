#include <cstdio>
#include <unordered_map>
#include <unordered_set>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief TODO(@fad35)
 *
 */
class DependencyManager : public rclcpp::Node
{
  private:
    // maps names of acquired nodes to the names of the nodes that depend on
    // them
    std::unordered_map<std::string, std::unordered_set<std::string>> m_users;

    // set of nodes that the dependency manager started; nodes in this set will
    // be disabled when they no longer have any users
    std::unordered_set<std::string> m_started_as_deps;

    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::AcquireDependency>>
        m_acquire_dependency_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReleaseDependency>>
        m_release_dependency_srv;

    auto create_acquire_dependency_service()
    {
        const auto acquire_dep_callback =
            [this](const std::shared_ptr<cmr_msgs::srv::AcquireDependency::Request>&
                       request,
                   std::shared_ptr<cmr_msgs::srv::AcquireDependency::Response>
                       response) {
                const auto [target, dependent] = *request;

                if (m_users.find(target) == m_users.end()) {
                    const auto activate_response = activate_dependency(target);

                    if (!activate_response) {
                        // failed to activate
                        response->success = false;
                        return response;
                    }

                    m_started_as_deps.emplace(target);
                    m_users[target] = std::unordered_set<std::string>();
                }

                m_users[target].emplace(dependent);

                response->success = true;
                return response;
            };

        return this->create_service<cmr_msgs::srv::AcquireDependency>(
            get_effective_namespace() + "/acquire", acquire_dep_callback);
    }

    auto release_dependency_callback(
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>& request,
        const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>& response)
    {
        const auto [target, dependent] = *request;

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

        if (m_users[target].empty()) {
            // no more users of the target node; deactivate it if it was
            // started as a dependency
            m_users.erase(target);
            if (m_started_as_deps.find(target) != m_started_as_deps.end()) {
                /*auto request =
                    std::make_shared<cmr_msgs::srv::DeactivateNode::Request>();
                request->node_name = target;*/
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

    auto create_release_dependency_service()
    {
        const auto release_dep_callback =
            [this](const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>&
                       req,
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

    bool activate_dependency(const std::string& target)
    {
        // check if node is configured
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
            const auto response =
                cmr::send_request<lifecycle_msgs::srv::ChangeState>(
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
        return !(activate_response && activate_response.value()->success);
    }

    bool deactivate_dependency(const std::string& target)
    {
        // check if node is configured
        const auto request =
            std::make_shared<lifecycle_msgs::srv::GetState::Request>();
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

  public:
    DependencyManager() : Node("dependency_manager", "fabric")
    {
        m_acquire_dependency_srv = create_acquire_dependency_service();
        m_release_dependency_srv = create_release_dependency_service();
        CMR_LOG(INFO, "dependency manager initialized");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<DependencyManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
