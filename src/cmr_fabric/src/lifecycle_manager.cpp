#include "cmr_fabric/lifecycle_manager.hpp"

#include <chrono>

#include "cmr_fabric/lifecycle_states.hpp"
#include "cmr_msgs/msg/state.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_utils/string_utils.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cmr::fabric
{
using namespace std::chrono_literals;

static bool call_change_state_client(const std::string& targetNode,
                                     uint8_t transition)
{
    const auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    const auto response = cmr::send_request<lifecycle_msgs::srv::ChangeState>(
        cmr::build_string("/", targetNode, "/change_state"), request);
    return response && response.value()->success;
}

static LifecycleState get_lifecycle_state(const std::string& targetNode)
{
    const auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    const auto response = cmr::send_request<lifecycle_msgs::srv::GetState>(
        cmr::build_string("/", targetNode, "/get_state"), request);

    if (!response) {
        return LifecycleState::Unknown;
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

bool activate_node(const std::string& node_name)
{
    const auto state = get_lifecycle_state(node_name);
    if (state == LifecycleState::Finalized || state == LifecycleState::Unknown) {
        CMR_LOG(ERROR, "Trying to activate a finalized or transitioning node %s",
                node_name.c_str());
        return false;
    } else if (state == LifecycleState::Active) {
        return true;
    } else if (state == LifecycleState::Unconfigured) {
        // configure first
        CMR_LOG(INFO, "configuring node %s", node_name.c_str());
        const auto success = call_change_state_client(
            node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (!success) {
            return false;
        }
    }
    // we can activate now
    CMR_LOG(INFO, "activating node %s", node_name.c_str());
    const auto result = call_change_state_client(
        node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    return result;
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
