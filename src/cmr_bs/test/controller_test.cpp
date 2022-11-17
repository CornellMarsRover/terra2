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

/**
 * @brief Create all of the test threads and return them in a tuple
 */
auto create_test_threads(const char* node_name, std::atomic<bool>& end_test,
                         const char* test_namespace)
{
    auto fault_handler_tuple = create_base_threads(end_test, test_namespace);

    // insert the pipe_name into the toml config string
    std::array<char, 512> toml_buffer = {};
    snprintf(toml_buffer.data(), sizeof(toml_buffer), test_toml, pipe_name);

    // create the joystick node in a new thread
    cmr::fabric::FabricNodeConfig config{node_name, test_namespace,
                                         toml_buffer.data()};
    auto test_thread = create_test_thread<cmr::Joystick>(end_test, config);

    JThread activate_thread(
        [node_name]() { cmr::fabric::activate_node(node_name); });

    return std::make_tuple(std::move(fault_handler_tuple), std::move(test_thread),
                           std::move(activate_thread));
}

/**
 * @brief Performs all the boilerplate for creating a joystick test
 *
 * @param test_namespace the namespace to run the test in
 * @param node_name the name of the node to create
 * @param mock_func the function to call to mock the callbacks for the subscriber
 * @param send_func the function to call to send the joystick input to the joystick
 * node
 */
// NOLINTNEXTLINE(readability-function-size)
auto test_wrapper(const char* test_namespace, const char* node_name,
                  std::function<void(JsInputMockSubscriber&, InSequence&)> mock_func,
                  std::function<int(int)> send_func)
{
    // boilerplate stuff
    std::atomic<bool> end_test = false;
    ALWAYS(&end_test) { end_test = true; };
    if (std::filesystem::exists(pipe_name)) {
        unlink(pipe_name);
    }
    int ret = mkfifo(pipe_name, 0666);
    ASSERT_EQ(ret, 0) << "Failed to create pipe " << ret << " : " << errno;
    // always unlink the pipe when we're done
    ALWAYS() { unlink(pipe_name); };

    auto test_threads = create_test_threads(node_name, end_test, test_namespace);

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

    {
        InSequence s;
        mock_func(mock, s);
    }
    const int call_count = send_func(fd);
    // wait for the message to be received
    const auto start = std::chrono::steady_clock::now();
    while (calls < call_count) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            FAIL() << "Timed out waiting for message";
        }
    }

    // check that the subscriber callback was called
    ASSERT_EQ(calls, call_count);
    end_test = true;
}

/**
 * @brief Writes a joystick message to the fd
 *
 * @param fd
 * @param event
 * @return true if entire event was written
 */
auto write_event(int fd, const js_event& event)
{
    return write(fd, &event, sizeof(event)) == static_cast<int>(sizeof(event));
}

// NOLINTNEXTLINE(readability-function-size)
TEST(JoystickTest, basicTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control1";
    bool fail_send = false;
    test_wrapper(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            EXPECT_CALL(mock, callback(Eq(1), Eq(0), Ge(1)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(1), DoubleEq(0)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(0), Ge(1)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(1), Ge(1)));

            EXPECT_CALL(mock, callback(Eq(0), Eq(0), Ge(1)));
            EXPECT_CALL(mock, callback(Eq(0), Eq(1), DoubleEq(0)));
            EXPECT_CALL(mock, callback(Eq(0), Eq(0), Ge(1)));
            EXPECT_CALL(mock, callback(Eq(0), Eq(1), Gt(4)));
        },
        [&fail_send](int fd) -> int {
            // write the message
            js_event event{};
            event.value = 1000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = 1000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;
            fail_send |= !write_event(fd, event);

            event.value = 1000;
            event.type = JS_EVENT_AXIS;
            event.number = 0;
            fail_send |= !write_event(fd, event);

            event.value = 3000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;
            fail_send |= !write_event(fd, event);

            return 8;
        });
    ASSERT_FALSE(fail_send);
}

// NOLINTNEXTLINE(readability-function-size)
TEST(JoystickTest, negativeTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control2";
    bool fail_send = false;
    test_wrapper(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            EXPECT_CALL(mock, callback(Eq(1), Eq(0), Le(-1)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(1), DoubleEq(0)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(0), Le(-1)));
            EXPECT_CALL(mock, callback(Eq(1), Eq(1), Le(-1)));

            EXPECT_CALL(mock, callback(Eq(2), Eq(0), Le(-3)));
            EXPECT_CALL(mock, callback(Eq(2), Eq(1), DoubleEq(0)));
            EXPECT_CALL(mock, callback(Eq(2), Eq(0), Le(-3)));
            EXPECT_CALL(mock, callback(Eq(2), Eq(1), Lt(-4)));
        },
        [&fail_send](int fd) -> int {
            // write the message
            js_event event{};
            event.value = -1000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = -1000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;
            fail_send |= !write_event(fd, event);

            event.value = -9000;
            event.type = JS_EVENT_AXIS;
            event.number = 4;
            fail_send |= !write_event(fd, event);

            event.value = -9000;
            event.type = JS_EVENT_AXIS;
            event.number = 5;
            fail_send |= !write_event(fd, event);

            return 8;
        });
    ASSERT_FALSE(fail_send);
}

// NOLINTNEXTLINE(readability-function-size)
TEST(JoystickTest, buttonTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control2";
    bool fail_send = false;
    test_wrapper(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            EXPECT_CALL(mock, callback(Eq(5), Eq(0), DoubleEq(1)));
            EXPECT_CALL(mock, callback(Eq(5), Eq(0), DoubleEq(0)));
            EXPECT_CALL(mock, callback(Eq(28), Eq(0), DoubleEq(1)));
            EXPECT_CALL(mock, callback(Eq(28), Eq(0), DoubleEq(0)));
        },
        [&fail_send](int fd) -> int {
            // write the message
            js_event event{};
            event.value = 1;
            event.type = JS_EVENT_BUTTON;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = 0;
            event.type = JS_EVENT_BUTTON;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = 25;
            event.type = JS_EVENT_BUTTON;
            event.number = 25;
            fail_send |= !write_event(fd, event);

            event.value = 0;
            event.type = JS_EVENT_BUTTON;
            event.number = 25;
            fail_send |= !write_event(fd, event);

            return 4;
        });
    ASSERT_FALSE(fail_send);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}