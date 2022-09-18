#include "cmr_fabric/lifecycle_helpers.hpp"

#include "cmr_msgs/msg/state.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/deactivate_node.hpp"
#include "cmr_utils/monad.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_utils/string_utils.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{

static bool call_change_state_client(const std::string& target_node,
                                     uint8_t transition)
{
    const auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    const auto response = cmr::send_request<lifecycle_msgs::srv::ChangeState>(
        cmr::build_string("/", target_node, "/change_state"), request);
    return response && response.value()->success;
}

LifecycleState get_lifecycle_state(const std::string& node_name)
{
    using lifecycle_msgs::msg::State;
    const auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    const auto response = cmr::send_request<lifecycle_msgs::srv::GetState>(
        cmr::build_string("/", node_name, "/get_state"), request);
    switch (monad::map(response, [](const auto& resp) {
                return resp->current_state.id;
            }).value_or(State::PRIMARY_STATE_UNKNOWN)) {
        case State::PRIMARY_STATE_UNCONFIGURED:
            return LifecycleState::Unconfigured;
        case State::PRIMARY_STATE_INACTIVE:
            return LifecycleState::Inactive;
        case State::PRIMARY_STATE_ACTIVE:
            return LifecycleState::Active;
        case State::PRIMARY_STATE_FINALIZED:
            return LifecycleState::Finalized;
        case State::TRANSITION_STATE_ACTIVATING:
            return LifecycleState::Activating;
        case State::TRANSITION_STATE_DEACTIVATING:
            return LifecycleState::Deactivating;
        case State::TRANSITION_STATE_CONFIGURING:
            return LifecycleState::Configuring;
        case State::TRANSITION_STATE_CLEANINGUP:
            return LifecycleState::CleaningUp;
        case State::TRANSITION_STATE_SHUTTINGDOWN:
            return LifecycleState::ShuttingDown;
        case State::TRANSITION_STATE_ERRORPROCESSING:
            return LifecycleState::ErrorProcessing;
        default:
            return cmr::fabric::LifecycleState::Unknown;
    }
}

bool activate_node(const std::string& node_name)
{
    auto state = get_lifecycle_state(node_name);
    if (state == LifecycleState::Active) {
        return true;
    } else if (state == LifecycleState::Unconfigured) {
        // configure first
        CMR_LOG(INFO, "configuring node %s", node_name.c_str());
        const auto success = call_change_state_client(
            node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (!success) {
            return false;
        }
        state = get_lifecycle_state(node_name);
    }

    if (state != LifecycleState::Inactive) {
        CMR_LOG(ERROR, "Trying to activate a node that's not inactive %s",
                node_name.c_str());
        return false;
    }
    // we can activate now
    CMR_LOG(INFO, "activating node %s", node_name.c_str());
    const auto result = call_change_state_client(
        node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    // if an error occurs during activation, ROS doesn't seem to tell us, so we check
    // the state again
    return result && get_lifecycle_state(node_name) == LifecycleState::Active;
}

bool deactivate_node(const std::string& node_name)
{
    // check if node is active
    const auto state = get_lifecycle_state(node_name);
    if (state == cmr::fabric::LifecycleState::Inactive) {
        return true;
    } else if (state != cmr::fabric::LifecycleState::Active) {
        CMR_LOG(WARN,
                "could not deactivate node %s because it is not "
                "currently active",
                node_name.c_str());
        return false;
    }
    // we can deactivate now
    CMR_LOG(INFO, "deactivating node %s", node_name.c_str());
    const auto result = call_change_state_client(
        node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    return result;
}

bool cleanup_node(const std::string& node_name)
{
    CMR_LOG(INFO, "Unconfiguring node %s", node_name.c_str());
    if (get_lifecycle_state(node_name) == LifecycleState::Unconfigured) {
        return true;
    }
    if (!deactivate_node(node_name)) {
        return false;
    }
    const auto result = call_change_state_client(
        node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    return result;
}
}  // namespace cmr::fabric
