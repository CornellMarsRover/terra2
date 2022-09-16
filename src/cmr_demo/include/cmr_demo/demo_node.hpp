#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "std_msgs/msg/bool.hpp"

class DemoNode : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> m_sub;

  public:
    explicit DemoNode(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};