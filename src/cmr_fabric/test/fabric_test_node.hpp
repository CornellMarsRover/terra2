#pragma once
#include "cmr_fabric/fabric_node.hpp"
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

  public:
    explicit FabricTestNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt)
        : FabricNode(config)
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
        return !m_fail_activate;
    }

    bool deactivate() override { return true; }

    bool cleanup() override
    {
        m_fail_activate = false;
        return true;
    }
};