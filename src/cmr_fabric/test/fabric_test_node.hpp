#pragma once
#include <cmr_utils/services.hpp>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_fabric/lifecycle_actions.hpp"
#include "cmr_fabric/lifecycle_servers.hpp"
#include "cmr_msgs/action/test_target_position.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @brief this node provides a simple test of dependency management and fault
 * handling. It provides one topic, /kill, which, will cause the node to trigger the
 * fault handler
 *
 */
class FabricTestNode : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>> m_sub;
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::ActivateNode>>
        m_fail_activate_srv;

    bool m_fail_activate = false;

    cmr::fabric::LifecycleSubscription<std_msgs::msg::Bool>::ptr_t m_test_sub;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>>
        m_test_pub, m_timer_pub;
    cmr::fabric::LifecycleClient<cmr_msgs::srv::ActivateNode>::ptr_t m_test_client;
    cmr::fabric::LifecycleService<cmr_msgs::srv::ActivateNode>::ptr_t m_test_service;
    cmr::fabric::LifecycleTimer<int64_t, std::milli>::ptr_t m_test_timer;
    // std::unique_ptr<cmr::fabric::GenericLifecycle> m_test_timer;
    cmr::fabric::LifecycleActionServer<cmr_msgs::action::TestTargetPosition>::ptr_t
        m_test_action;

    using target_pos_t = cmr_msgs::action::TestTargetPosition;
    using target_pos_goal_t = target_pos_t::Goal;
    using target_pos_handle_t = rclcpp_action::ServerGoalHandle<target_pos_t>;

  public:
    explicit FabricTestNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt)
        : FabricNode(config),
          m_test_sub(create_lifecycle_subscription<std_msgs::msg::Bool>(
              get_name() + std::string("/test_sub"), 10,
              [this](const std_msgs::msg::Bool& msg) { m_test_pub->publish(msg); })),
          m_test_pub(create_publisher<std_msgs::msg::Bool>(
              get_name() + std::string("/test_pub"), 10)),
          m_timer_pub(create_publisher<std_msgs::msg::Bool>(
              get_name() + std::string("/test_timer_pub"), 10)),
          m_test_client(create_lifecycle_client<cmr_msgs::srv::ActivateNode>(
              get_name() + std::string("/test_client"))),
          m_test_service(create_lifecycle_service<cmr_msgs::srv::ActivateNode>(
              get_name() + std::string("/test_service"),
              [this](
                  const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request>
                      request,
                  std::shared_ptr<cmr_msgs::srv::ActivateNode::Response> response) {
                  if (request->node_name == "bad_node_name") {
                      throw CmrTestException(
                          "Testing exception with: bad node name");
                  }
                  auto resp = cmr::send_request<cmr_msgs::srv::ActivateNode>(
                      get_name() + std::string("/test_client"), request);
                  response->success = resp.has_value() && (*resp)->success;
                  return response;
              })),
          m_test_timer(create_lifecycle_timer(std::chrono::milliseconds(10),
                                              [this]() {
                                                  auto msg = std_msgs::msg::Bool();
                                                  msg.data = true;
                                                  m_timer_pub->publish(msg);
                                              })),
          m_test_action(cmr::fabric::create_lifecycle_action_server<
                        cmr_msgs::action::TestTargetPosition>(
              *this, get_name() + std::string("/test_action"),
              [](const rclcpp_action::GoalUUID&,
                 std::shared_ptr<const target_pos_goal_t>) {
                  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
              },
              [](std::shared_ptr<target_pos_handle_t>) {
                  return rclcpp_action::CancelResponse::ACCEPT;
              },

              [this](std::shared_ptr<target_pos_handle_t> handle) {
                  auto msg = std_msgs::msg::Bool();
                  msg.data = true;
                  m_test_pub->publish(msg);
                  auto result = std::make_shared<target_pos_t::Result>();
                  result->success = true;
                  handle->succeed(result);
              }))

    {
        m_fail_activate_srv = this->create_service<cmr_msgs::srv::ActivateNode>(
            get_name() + std::string("/fail_activate"),
            [this](const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request>,
                   std::shared_ptr<cmr_msgs::srv::ActivateNode::Response> res) {
                CMR_LOG(INFO, "Got request to fail activation");
                m_fail_activate = true;
                res->success = true;
                return res;
            });
    }

    bool configure(const std::shared_ptr<toml::Table>&) override
    {
        m_sub = this->create_service<cmr_msgs::srv::ActivateNode>(
            get_name() + std::string("/kill"),
            [this](const std::shared_ptr<cmr_msgs::srv::ActivateNode::Request> req,
                   std::shared_ptr<cmr_msgs::srv::ActivateNode::Response> res) {
                CMR_RETRY_ON_ERR(CMR_LOG(INFO, "Got kill message: %s",
                                         req->node_name.empty() ? "true" : "false");
                                 CMR_ASSERT_MSG(false, "Killed by kill message");)
                res->success = true;
                return res;
            });
        return true;
    }

    bool activate() override
    {
        CMR_LOG(INFO, "Activating fabric test node: composition_ns is %s",
                get_parameter("composition_ns").as_string().c_str());
        CMR_LOG(INFO, "Activation success: %d", !m_fail_activate);
        if (!m_fail_activate) {
            m_test_pub->on_activate();
            m_test_sub->activate();
            m_test_client->activate();
            m_test_service->activate();
            m_test_timer->activate();
            m_test_action->activate();
            m_timer_pub->on_activate();
        }
        return !m_fail_activate;
    }

    bool deactivate() override
    {
        m_test_pub->on_deactivate();
        m_test_sub->deactivate();
        m_test_client->deactivate();
        m_test_service->deactivate();
        m_test_timer->deactivate();
        m_test_action->deactivate();
        m_timer_pub->on_deactivate();
        return true;
    }

    bool cleanup() override
    {
        m_fail_activate = false;
        return true;
    }
};