#include <cstdio>

#include "cmr_fabric/dependency_manager.hpp"
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_fabric/fault_handler.hpp"
#include "cmr_fabric/lifecycle_helpers.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_utils/string_utils.hpp"
#include "fabric_test_node.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/get_state.hpp"

#ifndef BUILD_TESTS
#error "This test must be compiled in test mode"
#endif

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

constexpr auto config_c = R"(
dependencies = [%s]

[fault_handling]
restart_attempts = 2
restart_delay = 2
)";

struct NodeConfig {
    const char* node_name;
    const char* test_namespace;
};

// NOLINTNEXTLINE
#define MAKE_END_TEST_FLAG()            \
    std::atomic<bool> end_test = false; \
    ALWAYS(&end_test) { end_test = true; }

constexpr auto system_now = std::chrono::system_clock::now;
using cmr::fabric::get_lifecycle_state;
using cmr::fabric::LifecycleState;

/**
 * @brief Create threads for each of the dependency manager, fault handler, and
 * lifecycle manager
 *
 * @param end_test
 * @param test_namespace
 * @return tuple of fault handler thread and the shared pointer to the fault handler
 */
static auto create_base_threads(const std::atomic<bool>& end_test,
                                const char* test_namespace)
{
    auto fault_handler =
        std::make_shared<cmr::fabric::FaultHandler>("fh", test_namespace);
    std::thread fault_handler_thread([&end_test, fault_handler]() {
        while (!end_test) {
            rclcpp::spin_some(fault_handler);
        }
    });
    return std::make_tuple(std::move(fault_handler_thread), fault_handler);
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
    std::thread test_thread([&end_test, config = std::make_optional(config)]() {
        auto test_node = std::make_shared<FabricTestNode>(config);
        while (!end_test) {
            rclcpp::spin_some(test_node->get_node_base_interface());
        }
    });
    return test_thread;
}

/**
 * @brief Creates a FabricNode config by formatting a toml string
 *
 * @tparam Formats
 * @param node_config node name and namespace
 * @param config the configuration string, with format specifiers such as `%d`
 * @param formats optional arguments to format into `config`
 */
template <typename... Formats>
static auto create_config(NodeConfig node_config, const char* const config,
                          Formats&&... formats)
{
    std::array<char, 1024> config_buffer = {};
#pragma GCC diagnostic push
#pragma clang diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma clang diagnostic ignored "-Wformat-security"
    std::snprintf(config_buffer.data(), sizeof(config_buffer), config,
                  std::forward<Formats>(formats)...);
#pragma GCC diagnostic pop
#pragma clang diagnostic pop
    return cmr::fabric::FabricNodeConfig{node_config.node_name,
                                         node_config.test_namespace,
                                         {std::string(config_buffer.data())}};
}

// Kill a test node via the kill topic
static auto kill_node(const char* node_name)
{
    const auto request = std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
    request->node_name = "really_i_should_make_new_service_type_for_this";
    auto response = cmr::send_request<cmr_msgs::srv::ActivateNode>(
        cmr::build_string("/", node_name, "/kill"), request);
    return response && (*response)->success;
}

// Create a dependee node (based off `config_a`)
static auto create_dependee(std::atomic<bool>& end_test, NodeConfig config)
{
    cmr::fabric::FabricNodeConfig config_struct{
        config.node_name, config.test_namespace, std::string(config_a)};
    return create_test_thread(end_test, config_struct);
}

/**
 * @brief Creates a list of strings by inserting string between quotes, and
 * separating them with commas
 *
 * ## Example
 *
 * ```C++
 * std::stringstream ss;
 * const auto name = "test";
 * add_to_stream(ss, 10, "hello", name);
 * ss.str() == "\"10\",\"hello\",\"test\",";
 * ```
 *
 * @tparam Args
 * @param stream
 * @param node_name
 * @param rest
 */
template <typename... Args>
void add_to_stream(std::stringstream& stream, const std::string& node_name,
                   Args&&... rest)
{
    stream << '"' << node_name << '"' << ',';
    if constexpr (sizeof...(rest) > 0) {
        add_to_stream(stream, std::forward<Args>(rest)...);
    }
}

/**
 * @brief Create a test node that depends on the specified nodes, using a specified
 * format string
 *
 * @tparam Dependees
 * @param end_test the end test flag
 * @param config_str the configuration string, with format specifiers
 * @param node_config node_name and test_namespace
 * @param dependee_names the names of the dependees
 */
template <typename... Dependees>
auto create_custom_depender(std::atomic<bool>& end_test, const char* config_str,
                            NodeConfig node_config, Dependees&&... dependee_names)
{
    std::stringstream ss;
    add_to_stream(ss, std::forward<Dependees>(dependee_names)...);
    return create_test_thread(
        end_test, create_config(node_config, config_str, ss.str().c_str()));
}

/**
 * @brief Like `create_custom_depender`, but with a default configuration string
 *
 */
template <typename... Dependees>
auto create_depender(std::atomic<bool>& end_test, NodeConfig node_config,
                     Dependees&&... dependee_names)
{
    return create_custom_depender(end_test, config_b, node_config,
                                  std::forward<Dependees>(dependee_names)...);
}

/**
 * @brief Ends the test by joining all functions after signaling the end test flag
 */
template <typename... Threads>
void join_all(std::atomic<bool>& end_test, Threads&... threads)
{
    end_test = true;
    (threads.join(), ...);
}

// NOLINTNEXTLINE
TEST(FabricTest, activateTest)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "activate_test";
    constexpr auto node_name = "test_1a";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);

    auto test_thread = create_dependee(end_test, {node_name, test_namespace});

    ASSERT_EQ(get_lifecycle_state(node_name), LifecycleState::Unconfigured);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name));

    ASSERT_EQ(get_lifecycle_state(node_name), LifecycleState::Active);

    join_all(end_test, test_thread, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, restartOnKillTest)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "restart_on_kill_test";
    constexpr auto node_name = "test_2a";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_test_thread(
        end_test, create_config({node_name, test_namespace}, config_a));
    ASSERT_TRUE(cmr::fabric::activate_node(node_name));

    ASSERT_EQ(get_lifecycle_state(node_name), LifecycleState::Active);

    const auto base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_TRUE(kill_node(node_name));

    ASSERT_EQ(get_lifecycle_state(node_name), LifecycleState::Unconfigured);

    fh->test_mock_check_time_and_wait(base_time + 5s);

    ASSERT_EQ(get_lifecycle_state(node_name), LifecycleState::Active);

    join_all(end_test, test_thread, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, startDependency)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "dependency_test";
    constexpr auto node_name_a = "test_3a";
    constexpr auto node_name_b = "test_3b";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});

    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);
    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, startDependencyChain)
{
    MAKE_END_TEST_FLAG();
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

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);

    auto test_thread_a = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread_b = create_dependee(end_test, {node_name_b, test_namespace});
    auto test_thread_c =
        create_depender(end_test, {node_name_c, test_namespace}, node_name_a);
    auto test_thread_d = create_depender(end_test, {node_name_d, test_namespace},
                                         node_name_b, node_name_c);
    auto test_thread_e =
        create_depender(end_test, {node_name_e, test_namespace}, node_name_d);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name_e));

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_d), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_e), LifecycleState::Active);

    join_all(end_test, test_thread_a, test_thread_b, test_thread_c, test_thread_d,
             test_thread_e, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, killDependender)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "kill_dependency_test";
    constexpr auto node_name_a = "test_4a";
    constexpr auto node_name_b = "test_4b";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);
    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));

    const auto base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_TRUE(kill_node(node_name_b));

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Unconfigured);

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Inactive);

    fh->test_mock_check_time_and_wait(base_time + 5s);

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, killDependendent)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "kill_dependent_test";
    constexpr auto node_name_a = "test_5a";
    constexpr auto node_name_b = "test_5b";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);
    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));

    ASSERT_TRUE(kill_node(node_name_a));

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Inactive);

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Unconfigured);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, restartDependendent)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "restart_dependent_test";
    constexpr auto node_name_a = "test_7a";
    constexpr auto node_name_b = "test_7b";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));

    const auto base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_TRUE(kill_node(node_name_a));
    fh->test_mock_check_time_and_wait(base_time + 5s);

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread);
}

// NOLINTNEXTLINE
TEST(FabricTest, killDependencyChain)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "kill_chain_test";
    constexpr auto node_name_a = "test_8a";
    constexpr auto node_name_b = "test_8b";
    constexpr auto node_name_c = "test_8c";
    constexpr auto node_name_d = "test_8d";
    constexpr auto node_name_e = "test_8e";

    /*        c -> a
     *       /
     * e -> d
     *       \
     *        b
     */

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);

    auto test_thread_a = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread_b = create_dependee(end_test, {node_name_b, test_namespace});
    auto test_thread_c = create_custom_depender(
        end_test, config_c, {node_name_c, test_namespace}, node_name_a);
    auto test_thread_d = create_custom_depender(
        end_test, config_c, {node_name_d, test_namespace}, node_name_b, node_name_c);
    auto test_thread_e = create_custom_depender(
        end_test, config_c, {node_name_e, test_namespace}, node_name_d);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name_a));
    ASSERT_TRUE(cmr::fabric::activate_node(node_name_e));

    auto base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_TRUE(kill_node(node_name_c));

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_d), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_e), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Unconfigured);

    fh->test_mock_check_time_and_wait(base_time + 4500ms);

    ASSERT_EQ(get_lifecycle_state(node_name_e), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_d), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Active);

    fh->test_mock_check_time_and_wait(base_time + 4500ms + 2s);

    ASSERT_EQ(get_lifecycle_state(node_name_e), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_d), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Active);

    base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_TRUE(kill_node(node_name_d));

    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_e), LifecycleState::Inactive);
    ASSERT_EQ(get_lifecycle_state(node_name_d), LifecycleState::Unconfigured);
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    join_all(end_test, test_thread_a, test_thread_b, test_thread_c, test_thread_d,
             test_thread_e, fault_handler_thread);
}

TEST(FabricTest, doubleDependency)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "double_dependency_test";
    constexpr auto node_name_a = "test_9a";
    constexpr auto node_name_b = "test_9b";
    constexpr auto node_name_c = "test_9c";

    /*
        b
         \
          a
         /
        c
    */

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);
    auto test_thread3 =
        create_depender(end_test, {node_name_c, test_namespace}, node_name_a);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));
    ASSERT_TRUE(cmr::fabric::activate_node(node_name_c));

    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    ASSERT_TRUE(cmr::fabric::cleanup_node(node_name_b));
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);

    ASSERT_TRUE(cmr::fabric::cleanup_node(node_name_c));
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Inactive);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread,
             test_thread3);
}

TEST(FabricTest, transitionFailure)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "transition_fail_test";
    constexpr auto node_name_a = "test_10a";
    constexpr auto node_name_b = "test_10b";
    constexpr auto node_name_c = "test_10c";

    // c -> b -> a

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);
    auto test_thread3 =
        create_depender(end_test, {node_name_c, test_namespace}, node_name_b);

    auto req = std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
    req->node_name = node_name_a;
    auto resp = cmr::send_request<cmr_msgs::srv::ActivateNode>(
        std::string("/") + node_name_b + std::string("/fail_activate"), req);

    const auto base_time = system_now();
    fh->test_mock_base_check_time(base_time);
    ASSERT_FALSE(cmr::fabric::activate_node(node_name_c));
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Unconfigured);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Unconfigured);

    fh->test_mock_check_time_and_wait(base_time + 10s);
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Active);
    ASSERT_EQ(get_lifecycle_state(node_name_c), LifecycleState::Active);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread,
             test_thread3);
}

TEST(FabricTest, nonFailureDeactivation)
{
    MAKE_END_TEST_FLAG();
    constexpr auto test_namespace = "non_fail_deactivate_test";
    constexpr auto node_name_a = "test_11a";
    constexpr auto node_name_b = "test_11b";

    auto [fault_handler_thread, fh] = create_base_threads(end_test, test_namespace);
    auto test_thread = create_dependee(end_test, {node_name_a, test_namespace});
    auto test_thread2 =
        create_depender(end_test, {node_name_b, test_namespace}, node_name_a);

    ASSERT_TRUE(cmr::fabric::activate_node(node_name_b));
    ASSERT_TRUE(cmr::fabric::cleanup_node(node_name_a));
    ASSERT_EQ(get_lifecycle_state(node_name_a), LifecycleState::Unconfigured);
    ASSERT_EQ(get_lifecycle_state(node_name_b), LifecycleState::Inactive);

    join_all(end_test, test_thread, test_thread2, fault_handler_thread);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}
