#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cmr_bs/joystick.hpp>
#include <cmr_fabric/fault_handler.hpp>
#include <cmr_fabric/lifecycle_helpers.hpp>
#include <cmr_fabric/test_utils.hpp>
#include <cmr_msgs/msg/joystick_reading.hpp>
#include <cmr_utils/thread_wrapper.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

using namespace cmr::fabric;

/**
 * @brief Create threads for each of the dependency manager, fault handler, and
 * lifecycle manager
 *
 * @param end_test
 * @param test_namespace
 * @return tuple of fault handler thread and the shared pointer to the fault
 * handler's base and check clock
 */
static auto create_base_threads(const std::atomic<bool>& end_test,
                                const char* test_namespace)
{
    auto fault_handler =
        std::make_shared<cmr::fabric::FaultHandler>("fh", test_namespace);
    JThread fault_handler_thread([&end_test, fault_handler]() {
        while (!end_test) {
            rclcpp::spin_some(fault_handler);
        }
    });
    return std::make_tuple(std::move(fault_handler_thread));
}
constexpr auto pipe_name = "/cmr/cmr_test_pipe";
constexpr auto test_toml = R"(
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 2

[node]
device="%s"
)";

// NOLINTNEXTLINE(readability-function-size)
TEST(JoystickTest, basicTest)
{
    // boilerplate stuff
    std::atomic<bool> end_test = false;
    ALWAYS(&end_test) { end_test = true; };
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control1";
    if (std::filesystem::exists(pipe_name)) {
        unlink(pipe_name);
    }
    ASSERT_EQ(mkfifo(pipe_name, 0666), 0);
    // always unlink the pipe when we're done
    ALWAYS() { unlink(pipe_name); };
    auto fault_handler_tuple = create_base_threads(end_test, test_namespace);

    // insert the pipe_name into the toml config string
    std::array<char, 512> toml_buffer = {};
    snprintf(toml_buffer.data(), sizeof(toml_buffer), test_toml, pipe_name);

    // create the joystick node in a new thread
    cmr::fabric::FabricNodeConfig config{node_name, test_namespace,
                                         toml_buffer.data()};
    auto test_thread = create_test_thread<cmr::Joystick>(end_test, config);

    // activate the node
    ASSERT_TRUE(cmr::fabric::activate_node(node_name));
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");

    // write the message
    js_event event{};
    event.value = 1000;
    event.type = JS_EVENT_AXIS;
    event.number = 2;

    // open the pipe for writing only
    const auto fd = open(pipe_name, O_WRONLY);
    ALWAYS(fd) { close(fd); };  // close the file descriptor when we're done

    // write the joystick event message
    ASSERT_EQ(write(fd, &event, sizeof(event)), static_cast<int>(sizeof(event)));
    bool call = false;

    // create the topic subscriber
    // the subscriber will contain the assert logic  to check the messages
    // it receives
    auto sub = node->create_subscription<cmr_msgs::msg::JoystickReading>(
        test_namespace + std::string("/js_input"), 10,
        [&call](cmr_msgs::msg::JoystickReading::SharedPtr ptr) {
            // assert that the message is correct
            ASSERT_EQ(ptr->control_id, 1);
            ASSERT_GT(ptr->magnitude, 0);
            call = true;
        });

    // wait for the message to be received
    const auto start = std::chrono::steady_clock::now();
    while (!call) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            FAIL() << "Timed out waiting for message";
        }
    }

    // check that the subscriber callback was called
    ASSERT_TRUE(call);
    end_test = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}