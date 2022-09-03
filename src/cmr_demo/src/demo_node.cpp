#include <cstdio>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DemoNode : public cmr::fabric::FabricNode
{
  public:
    DemoNode() : cmr::fabric::FabricNode::FabricNode()
    {
        declare_parameter("test", "");
    }

    bool configure(const std::shared_ptr<toml::Table>& table) override
    {
        auto node_settings = table->getTable("node");
        auto [ok, test] = node_settings->getString("test");
        if (!ok) {
            return false;
        }
        set_parameter(rclcpp::Parameter("test", test));
        return true;
    }

    bool activate() override
    {
        printf("Activating...\n");
        CMR_LOG(INFO, "test is %s", get_parameter("test").as_string().c_str());
        CMR_LOG(INFO, "composition_ns is %s",
                get_parameter("composition_ns").as_string().c_str());
        // auto timer_cb = [this]() {
        //     this->panic();
        //     m_timer->cancel();
        // };
        // m_timer = this->create_wall_timer(1s, timer_cb);
        return true;
    }

    bool deactivate() override
    {
        printf("Deactivating...\n");
        return true;
    }

    bool cleanup() override
    {
        printf("Shutting down...\n");
        return true;
    }

  private:
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoNode>();

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
