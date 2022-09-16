#include <cstdio>

#include "cmr_fabric/dependency_manager.hpp"
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_fabric/fault_handler.hpp"
#include "cmr_fabric/lifecycle_manager.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_utils/string_utils.hpp"
#include "fabric_test_node.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

constexpr auto config_a = R"(
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 4
)";

constexpr auto config_b = R"(
dependencies = [%s]

[fault_handling]
restart_attempts = 2
restart_delay = 0
)";

/**
 * @brief Create threads for each of the dependency manager, fault handler, and
 * lifecycle manager
 *
 * @param end_test
 * @param test_namespace
 * @return auto
 */
static auto create_base_threads(const std::atomic<bool>& end_test,
                                const char* test_namespace)
{
    std::thread lifecycle_manager_thread([&end_test, test_namespace]() {
        auto lifecycle_manager =
            std::make_shared<cmr::fabric::LifecycleManager>("lfm", test_namespace);
        while (!end_test) {
            rclcpp::spin_some(lifecycle_manager);
        }
    });
    std::thread dep_manager_thread([&end_test, test_namespace]() {
        auto dep_manager =
            std::make_shared<cmr::fabric::DependencyManager>("dm", test_namespace);
        while (!end_test) {
            rclcpp::spin_some(dep_manager);
        }
    });
    std::thread fault_handler_thread([&end_test, test_namespace]() {
        auto fault_handler =
            std::make_shared<cmr::fabric::FaultHandler>("fh", test_namespace);
        while (!end_test) {
            rclcpp::spin_some(fault_handler);
        }
    });
    return std::make_tuple(std::move(lifecycle_manager_thread),
                           std::move(dep_manager_thread),
                           std::move(fault_handler_thread));
}

/**
 * @brief Create a thread for a FabricTestNode
 *
 * @param end_test
 * @param config
 * @return auto
 */
static auto create_test_thread(const std::atomic<bool>& end_test,
                               const cmr::fabric::FabricNodeConfig& config)
{
    std::thread test_thread([&end_test, &config]() {
        auto test_node =
            std::make_shared<FabricTestNode>(std::make_optional(config));
        while (!end_test) {
            rclcpp::spin_some(test_node->get_node_base_interface());
        }
    });
    return test_thread;
}

/**
 * @brief Get the state object
 *
 * @param node_name
 * @return auto
 * @throw if service does not get a response
 */
static auto get_state(const char* node_name)
{
    auto state_req = cmr::send_request<lifecycle_msgs::srv::GetState>(
        cmr::build_string("/", node_name, "/get_state"),
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    return state_req.value();
}

/**
 * @brief Activate a node via the lifecycle manager
 *
 * @param node_name
 * @param namespace_name
 * @return auto
 */
static auto activate_node(const std::string& node_name, const char* namespace_name)
{
    auto req = std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
    req->node_name = node_name;
    auto resp = cmr::send_request<cmr_msgs::srv::ActivateNode>(
        cmr::build_string("/", namespace_name, "/activate"), req);
    return resp.has_value() && *resp;
}

// Kill a test node via the kill topic
static auto kill_node(const char* node_name)
{
    auto node = std::make_shared<rclcpp::Node>(
        "kill_node_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    auto pub = node->create_publisher<std_msgs::msg::Bool>(
        cmr::build_string("/", node_name, "/kill"), 10);
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pub->publish(msg);
}

TEST(FabricTest, activateTest)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "activate_test";
    constexpr auto node_name = "test_1a";

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);
    cmr::fabric::FabricNodeConfig config{node_name, test_namespace,
                                         std::string(config_a)};
    auto test_thread = create_test_thread(end_test, config);

    ASSERT_EQ(get_state(node_name)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    ASSERT_TRUE(activate_node(node_name, test_namespace));

    ASSERT_EQ(get_state(node_name)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread.join();
}

TEST(FabricTest, restartOnKillTest)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "restart_on_kill_test";
    constexpr auto node_name = "test_2a";

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);
    cmr::fabric::FabricNodeConfig config{node_name, test_namespace,
                                         std::string(config_a)};
    auto test_thread = create_test_thread(end_test, config);
    ASSERT_TRUE(activate_node(node_name, test_namespace));

    ASSERT_EQ(get_state(node_name)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    kill_node(node_name);

    ASSERT_EQ(get_state(node_name)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    std::this_thread::sleep_for(5s);

    ASSERT_EQ(get_state(node_name)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread.join();
}

TEST(FabricTest, startDependency)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "dependency_test";
    constexpr auto node_name_a = "test_3a";
    constexpr auto node_name_b = "test_3b";

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);
    cmr::fabric::FabricNodeConfig config_dependee{node_name_a, test_namespace,
                                                  std::string(config_a)};
    auto test_thread = create_test_thread(end_test, config_dependee);
    std::array<char, 1024> depender_toml = {};
    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  cmr::build_string('"', node_name_a, '"').c_str());

    cmr::fabric::FabricNodeConfig config_depender{node_name_b, test_namespace,
                                                  std::string(depender_toml.data())};

    auto test_thread2 = create_test_thread(end_test, config_depender);
    ASSERT_TRUE(activate_node(node_name_b, test_namespace));

    ASSERT_EQ(get_state(node_name_b)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(get_state(node_name_a)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread.join();
    test_thread2.join();
}

// WIP
auto create_dependee(std::atomic<bool>& end_test, const std::string& node_name,
                     const char* namespace_name)
{
    cmr::fabric::FabricNodeConfig config{node_name, namespace_name,
                                         std::string(config_a)};
    return create_test_thread(end_test, config);
}

// WIP
template <typename... Args>
void add_to_stream(std::stringstream& stream, const std::string& node_name,
                   Args&&... rest)
{
    stream << '"' << node_name << '"' << ',';
    if constexpr (sizeof...(rest) > 0) {
        add_to_stream(stream, std::forward<Args>(rest)...);
    }
}

// WIP
template <typename... Dependees>
auto create_depender(std::atomic<bool>& end_test, const std::string& node_name,
                     const char* namespace_name, Dependees&&... dependee_names)
{
    std::array<char, 1024> depender_toml = {};
    std::stringstream ss;
    add_to_stream(ss, std::forward<Dependees>(dependee_names)...);
    CMR_LOG(INFO, "Deps length: %zd", ss.str().length());
    CMR_LOG(INFO, "Dependencies: %s", ss.str().c_str());
    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  ss.str().c_str());

    CMR_LOG(INFO, "Config: %s", depender_toml.data());

    cmr::fabric::FabricNodeConfig config_depender{node_name, namespace_name,
                                                  std::string(depender_toml.data())};

    auto test_thread2 = create_test_thread(end_test, config_depender);
    return test_thread2;
}

// This needs to be fixed
// Currently not supported
// TODO(@sev47)
// NOLINTNEXTLINE
TEST(FabricTest, DISABLED_startDependencyChain)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "dependency_chain_test";
    constexpr auto node_name_a = "test_6a";
    constexpr auto node_name_b = "test_6b";
    constexpr auto node_name_c = "test_6c";
    constexpr auto node_name_d = "test_6d";
    constexpr auto node_name_e = "test_6e";

    /*        c -> a
     *       /
     * e -> d
     *       \
     *        b
     */

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);

    cmr::fabric::FabricNodeConfig config_dependee{node_name_a, test_namespace,
                                                  std::string(config_a)};
    auto test_thread_a = create_test_thread(end_test, config_dependee);

    cmr::fabric::FabricNodeConfig config_dependee_b{node_name_b, test_namespace,
                                                    std::string(config_a)};
    auto test_thread_b = create_test_thread(end_test, config_dependee_b);

    std::array<char, 1024> depender_toml = {};

    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  cmr::build_string('"', node_name_a, '"').c_str());
    cmr::fabric::FabricNodeConfig config_depender_c{
        node_name_c, test_namespace, std::string(depender_toml.data())};
    auto test_thread_c = create_test_thread(end_test, config_depender_c);

    std::snprintf(
        depender_toml.data(), sizeof(depender_toml), config_b,
        cmr::build_string('"', node_name_c, "\", \"", node_name_b, "\"").c_str());
    cmr::fabric::FabricNodeConfig config_depender_d{
        node_name_d, test_namespace, std::string(depender_toml.data())};
    auto test_thread_d = create_test_thread(end_test, config_depender_d);

    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  cmr::build_string('"', node_name_d, "\"").c_str());
    cmr::fabric::FabricNodeConfig config_depender_e{
        node_name_e, test_namespace, std::string(depender_toml.data())};
    auto test_thread_e = create_test_thread(end_test, config_depender_e);

    ASSERT_TRUE(activate_node(node_name_e, test_namespace));

    ASSERT_EQ(get_state(node_name_b)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(get_state(node_name_a)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(get_state(node_name_c)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(get_state(node_name_d)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(get_state(node_name_e)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread_a.join();
    test_thread_b.join();
    test_thread_c.join();
    test_thread_d.join();
    test_thread_e.join();
}

TEST(FabricTest, killDependender)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "kill_dependency_test";
    constexpr auto node_name_a = "test_4a";
    constexpr auto node_name_b = "test_4b";

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);
    cmr::fabric::FabricNodeConfig config_dependee{node_name_a, test_namespace,
                                                  std::string(config_a)};
    auto test_thread = create_test_thread(end_test, config_dependee);
    std::array<char, 1024> depender_toml = {};
    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  cmr::build_string('"', node_name_a, '"').c_str());

    cmr::fabric::FabricNodeConfig config_depender{node_name_b, test_namespace,
                                                  std::string(depender_toml.data())};

    auto test_thread2 = create_test_thread(end_test, config_depender);
    ASSERT_TRUE(activate_node(node_name_b, test_namespace));

    kill_node(node_name_b);

    ASSERT_EQ(get_state(node_name_b)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    ASSERT_EQ(get_state(node_name_a)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::this_thread::sleep_for(5s);

    ASSERT_EQ(get_state(node_name_b)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(get_state(node_name_a)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread.join();
    test_thread2.join();
}

// This needs to be fixed
// currently not supported
// TODO(@sev47)
TEST(FabricTest, DISABLED_killDependendent)
{
    std::atomic<bool> end_test = false;
    constexpr auto test_namespace = "kill_dependent_test";
    constexpr auto node_name_a = "test_5a";
    constexpr auto node_name_b = "test_5b";

    auto [lifecycle_manager_thread, dep_manager_thread, fault_handler_thread] =
        create_base_threads(end_test, test_namespace);
    cmr::fabric::FabricNodeConfig config_dependee{node_name_a, test_namespace,
                                                  std::string(config_a)};
    auto test_thread = create_test_thread(end_test, config_dependee);
    std::array<char, 1024> depender_toml = {};
    std::snprintf(depender_toml.data(), sizeof(depender_toml), config_b,
                  cmr::build_string('"', node_name_a, '"').c_str());

    cmr::fabric::FabricNodeConfig config_depender{node_name_b, test_namespace,
                                                  std::string(depender_toml.data())};

    auto test_thread2 = create_test_thread(end_test, config_depender);
    ASSERT_TRUE(activate_node(node_name_b, test_namespace));

    kill_node(node_name_a);

    ASSERT_EQ(get_state(node_name_b)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    ASSERT_EQ(get_state(node_name_a)->current_state.id,
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    end_test = true;
    lifecycle_manager_thread.join();
    dep_manager_thread.join();
    fault_handler_thread.join();
    test_thread.join();
    test_thread2.join();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}
