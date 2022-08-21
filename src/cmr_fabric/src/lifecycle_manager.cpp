#include <chrono>
#include <cstdio>

#include "cmr_fabric/lifecycle_states.hpp"
#include "cmr_msgs/msg/state.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_msgs/srv/get_node_state.hpp"
#include "cmr_msgs/srv/reconfigure_node.hpp"
#include "cmr_utils/services.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node
{
  public:
    LifecycleManager() : rclcpp::Node("lifecycle_manager", "fabric") { initialize(); }

  private:
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> activateSrv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> deactivateSrv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReconfigureNode>> reconfigureSrv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::GetNodeState>> getNodeStateSrv;

    void initialize()
    {
        createActivateService();
        createDeactivateService();
        createReconfigureService();
        createStateService();
        RCLCPP_INFO(get_logger(), "lifecycle manager initialized");
    }

    void createActivateService()
    {
        auto activateCb =
            [this](const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request> request,
                   std::shared_ptr<cmr_msgs::srv::ActivateNode::Response> response) {
                // check if node is configured
                auto state = callGetStateClient(request->node_name);
                if (state == cmr::fabric::LifecycleState::Unconfigured) {
                    // configure first
                    RCLCPP_INFO(get_logger(), "configuring node %s",
                                request->node_name.c_str());
                    auto success = callChangeStateClient(
                        request->node_name,
                        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                    if (!success) {
                        response->success = false;
                        return;
                    }
                }
                // we can activate now
                RCLCPP_INFO(get_logger(), "activating node %s",
                            request->node_name.c_str());
                auto result = callChangeStateClient(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                response->success = result;
            };
        activateSrv = this->create_service<cmr_msgs::srv::ActivateNode>(
            get_effective_namespace() + "/activate_node", activateCb);
    }

    void createDeactivateService()
    {
        auto deactivateCb =
            [this](const std::shared_ptr<cmr_msgs::srv::DeactivateNode::Request> request,
                   std::shared_ptr<cmr_msgs::srv::DeactivateNode::Response> response) {
                // check if node is active
                auto state = callGetStateClient(request->node_name.c_str());
                if (state != cmr::fabric::LifecycleState::Active) {
                    RCLCPP_WARN(
                        get_logger(),
                        "could not deactivate node %s because it is not currently active",
                        request->node_name.c_str());
                    response->success = false;
                    return;
                }
                // we can deactivate now
                RCLCPP_INFO(get_logger(), "deactivating node %s",
                            request->node_name.c_str());
                auto result = callChangeStateClient(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                response->success = result;
            };
        deactivateSrv = this->create_service<cmr_msgs::srv::DeactivateNode>(
            get_effective_namespace() + "/deactivate_node", deactivateCb);
    }

    void createReconfigureService()
    {
        auto reconfigureCb =
            [this](const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Request> request,
                   std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Response> response) {
                // we have to go through four transitions: deactivate, cleanup, configure,
                // activate
                auto state = callGetStateClient(request->node_name.c_str());
                if (state == cmr::fabric::LifecycleState::Active) {
                    RCLCPP_INFO(get_logger(), "deactivating node %s",
                                request->node_name.c_str());
                    auto success = callChangeStateClient(
                        request->node_name,
                        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                    if (!success) {
                        response->success = false;
                        return;
                    }
                }
                // cleanup the node
                auto success = callChangeStateClient(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
                if (!success) {
                    response->success = false;
                    return;
                }
                // configure the node
                RCLCPP_INFO(get_logger(), "configuring node %s",
                            request->node_name.c_str());
                success = callChangeStateClient(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                if (!success) {
                    response->success = false;
                    return;
                }
                // activate the node
                RCLCPP_INFO(get_logger(), "activating node %s",
                            request->node_name.c_str());
                success = callChangeStateClient(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                response->success = success;
            };
        reconfigureSrv = this->create_service<cmr_msgs::srv::ReconfigureNode>(
            get_effective_namespace() + "/reconfigure_node", reconfigureCb);
    }

    void createStateService()
    {
        auto getNodeStateCb =
            [this](const std::shared_ptr<cmr_msgs::srv::GetNodeState::Request> request,
                   std::shared_ptr<cmr_msgs::srv::GetNodeState::Response> response) {
                auto result = callGetStateClient(request->node_name);
                response->state.id = static_cast<uint8_t>(result);
            };
        getNodeStateSrv = this->create_service<cmr_msgs::srv::GetNodeState>(
            get_effective_namespace() + "/get_node_state", getNodeStateCb);
    }

    bool callChangeStateClient(const std::string& targetNode, uint8_t transition)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto response = cmr::sendRequest<lifecycle_msgs::srv::ChangeState>(
            "/" + targetNode + "/change_state", request);
        return response->success;
    }

    cmr::fabric::LifecycleState callGetStateClient(const std::string& targetNode)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto response = cmr::sendRequest<lifecycle_msgs::srv::GetState>(
            "/" + targetNode + "/get_state", request);

        auto rosStateId = response->current_state.id;
        switch (rosStateId) {
            case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                return cmr::fabric::LifecycleState::Unconfigured;
            case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
                return cmr::fabric::LifecycleState::Inactive;
            case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
                return cmr::fabric::LifecycleState::Active;
            case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
                return cmr::fabric::LifecycleState::Finalized;
            default:
                return cmr::fabric::LifecycleState::Unknown;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LifecycleManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
