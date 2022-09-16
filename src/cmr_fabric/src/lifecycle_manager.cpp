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

static bool call_change_state_client(const std::string& targetNode,
                                     uint8_t transition)
{
    const auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    const auto response = cmr::send_request<lifecycle_msgs::srv::ChangeState>(
        "/" + targetNode + "/change_state", request);
    return response && response.value()->success;
}

static cmr::fabric::LifecycleState call_get_state_client(
    const std::string& targetNode)
{
    const auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    const auto response = cmr::send_request<lifecycle_msgs::srv::GetState>(
        "/" + targetNode + "/get_state", request);

    if (!response) {
        return cmr::fabric::LifecycleState::Unknown;
    }

    const auto ros_state_id = response.value()->current_state.id;
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

static auto create_activate_service(rclcpp::Node& node)
{
    const auto activate_cb =
        [&node](
            const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request>& request,
            const std::shared_ptr<cmr_msgs::srv::ActivateNode::Response>& response) {
            // check if node is configured
            const auto state = call_get_state_client(request->node_name);
            if (state == cmr::fabric::LifecycleState::Unconfigured) {
                // configure first
                RCLCPP_INFO(node.get_logger(), "configuring node %s",
                            request->node_name.c_str());
                const auto success = call_change_state_client(
                    request->node_name,
                    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                if (!success) {
                    response->success = false;
                    return;
                }
            }
            // we can activate now
            RCLCPP_INFO(node.get_logger(), "activating node %s",
                        request->node_name.c_str());
            const auto result = call_change_state_client(
                request->node_name,
                lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            response->success = result;
        };
    return node.create_service<cmr_msgs::srv::ActivateNode>(
        node.get_effective_namespace() + "/activate", activate_cb);
}

static auto create_deactivate_service(rclcpp::Node& node)
{
    const auto deactivate_cb =
        [&node](
            const std::shared_ptr<cmr_msgs::srv::DeactivateNode::Request>& request,
            const std::shared_ptr<cmr_msgs::srv::DeactivateNode::Response>&
                response) {
            // check if node is active
            const auto state = call_get_state_client(request->node_name.c_str());
            if (state != cmr::fabric::LifecycleState::Active) {
                RCLCPP_WARN(node.get_logger(),
                            "could not deactivate node %s because it is not "
                            "currently active",
                            request->node_name.c_str());
                response->success = false;
                return;
            }
            // we can deactivate now
            RCLCPP_INFO(node.get_logger(), "deactivating node %s",
                        request->node_name.c_str());
            const auto result = call_change_state_client(
                request->node_name,
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            response->success = result;
        };
    return node.create_service<cmr_msgs::srv::DeactivateNode>(
        node.get_effective_namespace() + "/deactivate", deactivate_cb);
}

static auto reconfigure_callback(
    const rclcpp::Node& node,
    const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Request>& request,
    const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Response>& response)
{
    // we have to go through four transitions: deactivate, cleanup,
    // configure, activate
    const auto state = call_get_state_client(request->node_name.c_str());
    if (state == cmr::fabric::LifecycleState::Active) {
        RCLCPP_INFO(node.get_logger(), "deactivating node %s",
                    request->node_name.c_str());
        const auto success = call_change_state_client(
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
    RCLCPP_INFO(node.get_logger(), "configuring node %s",
                request->node_name.c_str());
    success = call_change_state_client(
        request->node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (!success) {
        response->success = false;
        return;
    }
    // activate the node
    RCLCPP_INFO(node.get_logger(), "activating node %s", request->node_name.c_str());
    success = call_change_state_client(
        request->node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    response->success = success;
}

static auto create_reconfigure_service(rclcpp::Node& node)
{
    const auto reconfigure_cb =
        [&node](
            const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Request>& request,
            const std::shared_ptr<cmr_msgs::srv::ReconfigureNode::Response>&
                response) { return reconfigure_callback(node, request, response); };

    return node.create_service<cmr_msgs::srv::ReconfigureNode>(
        node.get_effective_namespace() + "/reconfigure", reconfigure_cb);
}

static auto create_state_service(rclcpp::Node& node)
{
    const auto get_node_state_cb =
        [](const std::shared_ptr<cmr_msgs::srv::GetNodeState::Request>& request,
           const std::shared_ptr<cmr_msgs::srv::GetNodeState::Response>& response) {
            const auto result = call_get_state_client(request->node_name);
            response->state.id = static_cast<uint8_t>(result);
        };
    return node.create_service<cmr_msgs::srv::GetNodeState>(
        node.get_effective_namespace() + "/get_node_state", get_node_state_cb);
}
class LifecycleManager : public rclcpp::Node
{
  private:
    /**
     * A service that gets requests from the dependency manager and fault handler to
     * activate a node. Does this forwarding the request to ROS's lifecycle node
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> m_activate_srv;
    /**
     * A service that gets requests from the dependency manager to deactive a node.
     * Handles this by forwarding a reqeust to ROS's lifecycle node
     */
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::DeactivateNode>> m_deactivate_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ReconfigureNode>>
        m_reconfigure_srv;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::GetNodeState>>
        m_get_node_state_srv;

  public:
    LifecycleManager() : rclcpp::Node("lifecycle_manager")
    {
        m_activate_srv = create_activate_service(*this);
        m_deactivate_srv = create_deactivate_service(*this);
        m_reconfigure_srv = create_reconfigure_service(*this);
        m_get_node_state_srv = create_state_service(*this);
        RCLCPP_INFO(get_logger(), "lifecycle manager initialized");
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
