#include <cstdio>
#include <unordered_map>
#include <unordered_set>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "cmr_utils/cmr_error.hpp"
#include "cmr_utils/services.hpp"
#include "rclcpp/rclcpp.hpp"

class DependencyManager : public rclcpp::Node
{
  public:
    DependencyManager() : Node("dependency_manager", "fabric") {}

  private:
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::AcquireDependency>>
        acquire_dependency_service;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReleaseDependency>>
        release_dependency_service;

    // maps names of acquired nodes to the names of the nodes that depend on
    // them
    std::unordered_map<std::string, std::unordered_set<std::string>> users;

    // set of nodes that the dependency manager started; nodes in this set will
    // be disabled when they no longer have any users
    std::unordered_set<std::string> started_as_deps;

    void init()
    {
        createAcquireDependencyService();
        createReleaseDependencyService();
        RCLCPP_INFO(get_logger(), "dependency manager initialized");
    }

    void createAcquireDependencyService()
    {
        auto acquire_dep_callback =
            [this](const std::shared_ptr<cmr_msgs::srv::AcquireDependency::Request>
                       request,
                   std::shared_ptr<cmr_msgs::srv::AcquireDependency::Response>) {
                auto target = request->target;
                auto dependent = request->dependent;
                auto response =
                    std::make_shared<cmr_msgs::srv::AcquireDependency::Response>();

                CMR_LOG(DEBUG, "Acquiring dependency %s for node %s...",
                        target.c_str(), dependent.c_str());

                if (users.find(target) == users.end()) {
                    auto request =
                        std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
                    request->node_name = target;

                    auto activate_response =
                        cmr::sendRequest<cmr_msgs::srv::ActivateNode>(
                            "/fabric/activate_node", request);

                    if (!activate_response) {
                        // failed to activate
                        response->success = false;
                        return response;
                    }

                    started_as_deps.emplace(target);
                    users[target] = std::unordered_set<std::string>();
                }

                users[target].emplace(dependent);

                response->success = true;
                return response;
            };

        this->create_service<cmr_msgs::srv::AcquireDependency>(
            get_effective_namespace() + "/acquire", acquire_dep_callback);
    }

    void createReleaseDependencyService()
    {
        auto release_dep_callback =
            [this](const std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Request>
                       request,
                   std::shared_ptr<cmr_msgs::srv::ReleaseDependency::Response>
                       response) {
                auto target = request->target;
                auto dependent = request->dependent;

                CMR_LOG(DEBUG, "Releasing dependency %s for node %s...",
                        target.c_str(), dependent.c_str());

                if (users.find(target) == users.end()) {
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

                users[target].erase(dependent);

                if (users[target].empty()) {
                    // no more users of the target node; deactivate it if it was
                    // started as a dependency
                    users.erase(target);
                    if (started_as_deps.find(target) != started_as_deps.end()) {
                        auto request = std::make_shared<
                            cmr_msgs::srv::DeactivateNode::Request>();
                        request->node_name = target;
                        auto deactivate_response =
                            cmr::sendRequest<cmr_msgs::srv::DeactivateNode>(
                                "/fabric/deactivate_node", request);
                        if (!deactivate_response) {
                            // failed to deactivate
                            response->success = false;
                            return response;
                        }
                        started_as_deps.erase(target);
                    }
                }
                response->success = true;
                return response;
            };
        this->create_service<cmr_msgs::srv::ReleaseDependency>(
            get_effective_namespace() + "/release", release_dep_callback);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    printf("The dependency_manager node.\n");

    rclcpp::shutdown();
    return 0;
}
