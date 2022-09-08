#include <cstdio>
#include <sstream>
#include <thread>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DemoNode : public cmr::fabric::FabricNode
{
    std::shared_ptr<rclcpp::WallTimer<std::function<void()>>> m_timer;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> m_sub;

  public:
    DemoNode() : cmr::fabric::FabricNode::FabricNode()
    {
        using namespace std::chrono_literals;
        declare_parameter("test", "");

        std::function<void()> callback = []() {
            std::stringstream ss;
            ss << std::this_thread::get_id();
            /*RCLCPP_INFO(this->get_logger(), "Hello World! from pid %s",
                        ss.str().c_str());*/
        };
        m_timer = this->create_wall_timer(1s, callback);
        // For testing Fabric
        m_sub = this->create_subscription<std_msgs::msg::Bool>(
            get_name() + std::string{"/panic"}, 10,
            [this](std_msgs::msg::Bool::SharedPtr) {
                RCLCPP_WARN(this->get_logger(), "Node got kill msg!");
                this->panic();
            });
    }

    bool configure(const std::shared_ptr<toml::Table>& table) override
    {
        const auto node_settings = table->getTable("node");
        const auto [ok, test] = node_settings->getString("test");
        if (ok) {
            set_parameter(rclcpp::Parameter("test", test));
        }
        return ok;
    }

    bool activate() override
    {
        printf("Activating...\n");
        CMR_LOG(INFO, "test is %s", get_parameter("test").as_string().c_str());
        CMR_LOG(INFO, "composition_ns is %s",
                get_parameter("composition_ns").as_string().c_str());
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
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoNode>();

    std::stringstream ss;
    ss << std::this_thread::get_id();
    RCLCPP_INFO(node->get_logger(), "Starting demo node on thread: pid %s",
                ss.str().c_str());
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
