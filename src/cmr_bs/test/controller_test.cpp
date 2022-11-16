#include <fcntl.h>
#include <gmock/gmock.h>
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
constexpr auto pipe_name = "/tmp/cmr_test_pipe";
constexpr auto test_toml = R"(
dependencies = []

[fault_handling]
restart_attempts = 2
restart_delay = 2

[node]
device="%s"
)";

class JsInputMockSubscriber
{
  public:
    /**
     * @brief Construct callback for a js_input subscriber
     * @param control_id
     * @param axis_id
     * @param magnitude
     */
    MOCK_METHOD(void, callback, (int, int, double));
};

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace testing;

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
    int ret = mkfifo(pipe_name, 0666);
    ASSERT_EQ(ret, 0) << "Failed to create pipe " << ret << " : " << errno;
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

    std::atomic<bool> activated = false;
    JThread activate_thread(
        [&activated]() { activated = cmr::fabric::activate_node(node_name); });

    // open the pipe for writing only, this will block until a reader opens it
    const auto fd = open(pipe_name, O_WRONLY);
    ALWAYS(fd) { close(fd); };  // close the file descriptor when we're done

    // // activate the node
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");

    auto calls = 0;
    JsInputMockSubscriber mock;
    // create the topic subscriber
    // the subscriber will contain the assert logic  to check the messages
    // it receives
    auto sub = node->create_subscription<cmr_msgs::msg::JoystickReading>(
        "js_input", 10,
        [&mock, &calls](cmr_msgs::msg::JoystickReading::SharedPtr ptr) {
            mock.callback(ptr->control_id, ptr->axis_id, ptr->magnitude);
            ++calls;
        });

    EXPECT_CALL(mock, callback(Eq(1), _, Ge(1))).Times(1);
    EXPECT_CALL(mock, callback(Eq(1), _, DoubleEq(0))).Times(1);

    // write the message
    js_event event{};
    event.value = 1000;
    event.type = JS_EVENT_AXIS;
    event.number = 2;
    // write the joystick event message
    ASSERT_EQ(write(fd, &event, sizeof(event)), static_cast<int>(sizeof(event)));

    // wait for the message to be received
    const auto start = std::chrono::steady_clock::now();
    while (calls < 2) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            FAIL() << "Timed out waiting for message";
        }
    }

    // check that the subscriber callback was called
    ASSERT_EQ(calls, 2);
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