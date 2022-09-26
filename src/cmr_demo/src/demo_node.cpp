#include "cmr_demo/demo_node.hpp"

#include <cstdio>
#include <sstream>
#include <thread>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

DemoNode::DemoNode(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    declare_parameter("test", "");
}

bool DemoNode::configure(const std::shared_ptr<toml::Table>& table)
{
    const auto node_settings = table->getTable("node");
    const auto [ok, test] = node_settings->getString("test");
    m_sub = create_subscription<std_msgs::msg::Bool>(
        get_name() + std::string("/kill"), 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            // NOLINTNEXTLINE(bugprone-lambda-function-name)
            CMR_RETRY_ON_ERR(RCLCPP_INFO(get_logger(), "Got kill message: %s",
                                         msg->data ? "true" : "false");
                             CMR_ASSERT_MSG(false, "Killed by kill message");)
        });
    if (ok) {
        set_parameter(rclcpp::Parameter("test", test));
    }
    return ok;
}

bool DemoNode::activate()
{
    printf("Activating...\n");
    CMR_LOG(INFO, "test is %s", get_parameter("test").as_string().c_str());
    CMR_LOG(INFO, "composition_ns is %s",
            get_parameter("composition_ns").as_string().c_str());
    return true;
}

bool DemoNode::deactivate()
{
    printf("Deactivating...\n");
    return true;
}

bool DemoNode::cleanup()
{
    printf("Shutting down...\n");
    return true;
}
