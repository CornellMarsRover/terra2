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
    LifecycleManager() : rclcpp::Node("lifecycle_manager") { initialize(); }

  private:
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> m_activate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> m_deactivate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReconfigureNode>>
        m_reconfigure_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::GetNodeState>>
        m_get_node_state_srv;

    void initialize()
    {
        create_activate_service();
        create_deactivate_service();
        create_reconfigure_service();
        create_state_service();
        RCLCPP_INFO(get_logger(), "lifecycle manager initialized");
    }

    void create_activate_service()
    {
        auto activate_cb =
            [this](
                const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request>& request,
                const std::shared_ptr<cmr_msgs::srv::ActivateNode::Response>&
                    response) {
                // check if node is configured
                auto state = call_get_state_client(request->node_name);
                if (state == cmr::fabric::LifecycleState::Unconfigured) {
                    // configure first
                    RCLCPP_INFO(get_logger(), "configuring node %s",
                                request->node_name.c_str());
                    auto success = call_change_state_client(
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
                auto result = call_change_state_client(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                response->success = result;
            };
        m_activate_srv = this->create_service<cmr_msgs::srv::ActivateNode>(
            get_effective_namespace() + "/activate", activate_cb);
    }

    void create_deactivate_service()
    {
        auto deactivate_cb =
            [this](const std::shared_ptr<cmr_msgs::srv::DeactivateNode::Request>&
                       request,
                   const std::shared_ptr<cmr_msgs::srv::DeactivateNode::Response>&
                       response) {
                // check if node is active
                auto state = call_get_state_client(request->node_name.c_str());
                if (state != cmr::fabric::LifecycleState::Active) {
                    RCLCPP_WARN(get_logger(),
                                "could not deactivate node %s because it is not "
                                "currently active",
                                request->node_name.c_str());
                    response->success = false;
                    return;
                }
                // we can deactivate now
                RCLCPP_INFO(get_logger(), "deactivating node %s",
                            request->node_name.c_str());
                auto result = call_change_state_client(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                response->success = result;
            };
        m_deactivate_srv = this->create_service<cmr_msgs::srv::DeactivateNode>(
            get_effective_namespace() + "/deactivate", deactivate_cb);
    }

    void reconfigure_callback(
        const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Request>& request,
        const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Response>& response)
    {
        // we have to go through four transitions: deactivate, cleanup,
        // configure, activate
        const auto state = call_get_state_client(request->node_name.c_str());
        if (state == cmr::fabric::LifecycleState::Active) {
            RCLCPP_INFO(get_logger(), "deactivating node %s",
                        request->node_name.c_str());
            auto success = call_change_state_client(
                request->node_name,
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            if (!success) {
                response->success = false;
                return;
            }
        }
        // cleanup the node
        auto success = call_change_state_client(
            request->node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        if (!success) {
            response->success = false;
            return;
        }
        // configure the node
        RCLCPP_INFO(get_logger(), "configuring node %s", request->node_name.c_str());
        success = call_change_state_client(
            request->node_name,
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (!success) {
            response->success = false;
            return;
        }
        // activate the node
        RCLCPP_INFO(get_logger(), "activating node %s", request->node_name.c_str());
        success = call_change_state_client(
            request->node_name,
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        response->success = success;
    }

    void create_reconfigure_service()
    {
        const auto reconfigure_cb =
            [this](const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Request>&
                       request,
                   const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Response>&
                       response) { return reconfigure_callback(request, response); };

        m_reconfigure_srv = this->create_service<cmr_msgs::srv::ReconfigureNode>(
            get_effective_namespace() + "/reconfigure", reconfigure_cb);
    }

    void create_state_service()
    {
        auto get_node_state_cb =
            [this](
                const std::shared_ptr<cmr_msgs::srv::GetNodeState::Request>& request,
                const std::shared_ptr<cmr_msgs::srv::GetNodeState::Response>&
                    response) {
                const auto result = call_get_state_client(request->node_name);
                response->state.id = static_cast<uint8_t>(result);
            };
        m_get_node_state_srv = this->create_service<cmr_msgs::srv::GetNodeState>(
            get_effective_namespace() + "/get_node_state", get_node_state_cb);
    }

    bool call_change_state_client(const std::string& targetNode, uint8_t transition)
    {
        const auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        const auto response = cmr::send_request<lifecycle_msgs::srv::ChangeState>(
            "/" + targetNode + "/change_state", request);
        return response->success;
    }

    cmr::fabric::LifecycleState call_get_state_client(const std::string& targetNode)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto response = cmr::send_request<lifecycle_msgs::srv::GetState>(
            "/" + targetNode + "/get_state", request);

        auto ros_state_id = response->current_state.id;
        switch (ros_state_id) {
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
