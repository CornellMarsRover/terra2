#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "std_msgs/msg/bool.hpp"

class FabricTestNode : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> m_sub;

  public:
    explicit FabricTestNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt)
        : FabricNode(config)
    {
    }

    bool configure(const std::shared_ptr<toml::Table>&) override
    {
        m_sub = this->create_subscription<std_msgs::msg::Bool>(
            get_name() + std::string("/kill"), 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                CMR_RETRY_ON_ERR(CMR_LOG(INFO, "Got kill message: %s",
                                         msg->data ? "true" : "false");
                                 CMR_ASSERT_MSG(false, "Killed by kill message");)
            });
        return true;
    }

    bool activate() override
    {
        CMR_LOG(INFO, "Activating fabric test node: composition_ns is %s",
                get_parameter("composition_ns").as_string().c_str());
        return true;
    }

    bool deactivate() override { return true; }

    bool cleanup() override { return true; }
};