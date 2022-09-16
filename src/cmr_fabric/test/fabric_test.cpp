#include "cmr_fabric/dependency_manager.hpp"
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_fabric/fault_handler.hpp"
#include "cmr_fabric/lifecycle_manager.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_utils/services.hpp"
#include "fabric_test_node.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/get_state.hpp"

constexpr auto config_a = R"(
name = "test_a"
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 10
)";

// constexpr auto config_b = R"(
// name = "test_b"
// dependencies = ["test_a"]

// [fault_handling]
// restart_attempts = 2
// restart_delay = 0
// )";

TEST(FabricTest, activateTest)
{
    std::atomic<bool> end_test = false;
    std::thread t([&end_test]() {
        auto lfm = std::make_shared<cmr::fabric::LifecycleManager>("lfm", "test1");
        auto dm = std::make_shared<cmr::fabric::DependencyManager>("dm", "test1");
        auto fh = std::make_shared<cmr::fabric::FaultHandler>("fh", "test1");
        cmr::fabric::FabricNodeConfig config{"test_1a", "test1",
                                             std::string(config_a)};
        auto node_config = std::make_optional(config);
        auto node_a = std::make_shared<FabricTestNode>(node_config);

        while (!end_test) {
            rclcpp::spin_some(lfm);
            rclcpp::spin_some(dm);
            rclcpp::spin_some(fh);
            rclcpp::spin_some(node_a->get_node_base_interface());
        }
    });

    auto state_req = cmr::send_request<lifecycle_msgs::srv::GetState>(
        "test_1a/get_state",
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    ASSERT_TRUE(state_req.has_value());
    ASSERT_EQ(state_req.value()->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    auto req = std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
    req->node_name = "test_1a";
    auto resp =
        cmr::send_request<cmr_msgs::srv::ActivateNode>("test1/activate", req);
    ASSERT_TRUE(resp.has_value());
    ASSERT_TRUE(resp.value()->success);

    auto req2 = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto resp2 =
        cmr::send_request<lifecycle_msgs::srv::GetState>("test_1a/get_state", req2);
    ASSERT_TRUE(resp2.has_value());
    ASSERT_EQ(resp2.value()->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    t.join();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}
