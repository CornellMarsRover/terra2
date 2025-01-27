#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cmr::fabric
{

template <typename T>
struct ActionDefaultReturnConstruct : ServerDefaultArgumentConstructor<T> {
};

template <>
struct ActionDefaultReturnConstruct<rclcpp_action::GoalResponse> {
    const inline static auto val = rclcpp_action::GoalResponse::REJECT;
};

template <>
struct ActionDefaultReturnConstruct<rclcpp_action::CancelResponse> {
    const inline static auto val = rclcpp_action::CancelResponse::REJECT;
};

/** Configuration for a LifecycleActionServer */
struct ActionConfig {
    std::string action_name;
};

/**
 * @brief A class that provides an action server to LifecycleServer
 *
 * @tparam ActionT
 */
template <typename ActionT>
class ActionServerPolicy
{
  public:
    using server_t = rclcpp_action::Server<ActionT>;
    using config_t = ActionConfig;

    using goal_cb_t = typename server_t::GoalCallback;
    using cancel_cb_t = typename server_t::CancelCallback;
    using accepted_cb_t = typename server_t::AcceptedCallback;

    // 3) Inside the class, you can refer to the template at namespace scope:
    template <typename T>
    using make_return_t = ActionDefaultReturnConstruct<T>;

    // Create your server:
    template <typename>
    static auto make_server(rclcpp_lifecycle::LifecycleNode& node_base,
                            goal_cb_t goal_callback,
                            cancel_cb_t cancel_cb,
                            accepted_cb_t accpted_cb,
                            const config_t config)
    {
        return rclcpp_action::create_server<ActionT>(
            &node_base,
            config.action_name,
            goal_callback,
            cancel_cb,
            accpted_cb);
    }
};

/**
 * @brief A class that provides a managed Action Server that can be enabled and
 * disabled and wraps all errors in the appropriate error handling transition
 * function of the fabric node
 *
 * @tparam ActionT type of the action
 */
template <typename ActionT>
using LifecycleActionServer = LifecycleServer<ActionServerPolicy<ActionT>>;

/**
 * @brief Create a lifecycle action server object
 *
 * @tparam ActionT type of the action
 * @param node owning node which must not outlive the action server
 * @param action_name
 * @param goal_callback callback when a goal is received
 * @param cancel_callback callback when a goal is cancelled
 * @param accepted_callback callback when a goal is accepted from the `goal_callback`
 * @return auto
 * @ingroup LifecycleFactory
 */
template <typename ActionT>
std::unique_ptr<LifecycleActionServer<ActionT>> create_lifecycle_action_server(
    FabricNode& node,
    const std::string& action_name,
    const typename rclcpp_action::Server<ActionT>::GoalCallback& goal_callback,
    const typename rclcpp_action::Server<ActionT>::CancelCallback& cancel_callback,
    const typename rclcpp_action::Server<ActionT>::AcceptedCallback& accepted_callback)
{
    ActionConfig config{action_name};
    return std::make_unique<LifecycleActionServer<ActionT>>(
        node,
        [&node]() { node.error_transition(); },
        config,
        goal_callback,
        cancel_callback,
        accepted_callback);
}

}  // namespace cmr::fabric
