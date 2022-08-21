#include <cstdio>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/cmr_error.hpp"
#include "rclcpp/rclcpp.hpp"

class DemoNode : public cmr::fabric::FabricNode
{
  public:
    DemoNode() : cmr::fabric::FabricNode::FabricNode("demo")
    {
        declare_parameter("test", "");
    }

    bool onConfigure(std::shared_ptr<toml::Table> table) override
    {
        auto nodeSettings = table->getTable("node");
        auto [ok, test] = nodeSettings->getString("test");
        if (!ok) {
            return false;
        }
        set_parameter(rclcpp::Parameter("test", test));
        return true;
    }

    bool onActivate() override
    {
        printf("Activating...\n");
        CMR_LOG(INFO, "test is %s", get_parameter("test").as_string().c_str());
        return true;
    }

    bool onDeactivate() override
    {
        printf("Deactivating...\n");
        return true;
    }

    bool onShutdown() override
    {
        printf("Shutting down...\n");
        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoNode>();

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
