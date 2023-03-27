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
#include <geometry_msgs/msg/twist_stamped.hpp>
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
type="%s"
arm_max_speed = 10
max_speed = 2.5
)";

/** Joystick controller type config */
enum class JoystickType { Arm, Drives };

constexpr auto joystick_type_to_string(JoystickType type)
{
    switch (type) {
        case JoystickType::Arm:
            return "armcontroller";
        case JoystickType::Drives:
            return "drivescontroller";
        default:
            throw std::invalid_argument("Invalid joystick type");
    }
}

class JsInputMockSubscriber
{
  public:
    /**
     * @brief Construct callback for a js_input subscriber
     * @param control_id
     * @param axis_id
     * @param magnitude
     */
    MOCK_METHOD(void, arm_callback, (int, int, double));

    /**
     * @brief Callback for a cmd_vel subscriber
     * @param linear_x
     * @param linear_y
     * @param linear_z
     * @param angular_x
     * @param angular_y
     * @param angular_z
     * @param frame_id
     */
    // NOLINTNEXTLINE
    MOCK_METHOD(void, drives_callback,
                (double, double, double, double, double, double, std::string));
};
// NOLINTNEXTLINE(google-build-using-namespace)
using namespace testing;

/**
 * @brief Create all of the test threads and return them in a tuple
 */
auto create_test_threads(const char* node_name, std::atomic<bool>& end_test,
                         const char* test_namespace, JoystickType test_type)
{
    auto fault_handler_tuple = create_base_threads(end_test, test_namespace);

    // insert the pipe_name into the toml config string
    std::array<char, 512> toml_buffer = {};
    snprintf(toml_buffer.data(), sizeof(toml_buffer), test_toml, pipe_name,
             joystick_type_to_string(test_type));

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
 * @brief Get the test subscriber object
 *
 * @tparam TestType
 * @param node ros node which owns the subscriber
 * @param mock mock object
 * @param callback_calls reference which is incremented when a callback is executed
 * @return auto
 */
template <JoystickType TestType>
auto get_test_subscriber(std::shared_ptr<rclcpp::Node>& node,
                         JsInputMockSubscriber& mock, int& callback_calls)
{
    if constexpr (TestType == JoystickType::Arm) {
        return node->create_subscription<cmr_msgs::msg::JoystickReading>(
            "js_input", 10,
            [&mock, &callback_calls](cmr_msgs::msg::JoystickReading::SharedPtr ptr) {
                mock.arm_callback(ptr->control_id, ptr->axis_id, ptr->magnitude);
                ++callback_calls;
            });
    } else if constexpr (TestType == JoystickType::Drives) {
        return node->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/drives_controller/cmd_vel", 10,
            [&mock,
             &callback_calls](geometry_msgs::msg::TwistStamped::SharedPtr ptr) {
                mock.drives_callback(ptr->twist.linear.x, ptr->twist.linear.y,
                                     ptr->twist.linear.z, ptr->twist.angular.x,
                                     ptr->twist.angular.y, ptr->twist.angular.z,
                                     ptr->header.frame_id);
                ++callback_calls;
            });
    } else {
        throw std::invalid_argument("Invalid joystick type");
    }
}

/**
 * @brief Wait for all of the expected messages to be received
 *
 * @param num_calls reference to number of messages received
 * @param node node to spin
 * @param num_expected number of messages expected
 * @return true if all expected messages were received
 */
inline bool wait_for_all_expected_msgs(int& num_calls,
                                       std::shared_ptr<rclcpp::Node>& node,
                                       int num_expected)
{
    // wait for the message to be received
    const auto start = std::chrono::steady_clock::now();
    while (num_calls < num_expected) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Performs all the boilerplate for creating a joystick test
 *
 * @tparam TestType the type of joystick to test
 * @param test_namespace the namespace to run the test in
 * @param node_name the name of the node to create
 * @param mock_func the function to call to mock the callbacks for the subscriber
 * @param send_func the function to call to send the joystick input to the
 * joystick node
 */
// NOLINTNEXTLINE(readability-function-size)
template <JoystickType TestType>
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

    auto test_threads =
        create_test_threads(node_name, end_test, test_namespace, TestType);

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
    auto sub = get_test_subscriber<TestType>(node, mock, calls);

    {
        InSequence s;
        mock_func(mock, s);
    }
    const int call_count = send_func(fd);
    ASSERT_TRUE(wait_for_all_expected_msgs(calls, node, call_count))
        << "Timed out waiting for message";

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

// NOLINTNEXTLINE(readability-function-size,cppcoreguidelines-*)
TEST(JoystickTest, basicTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control1";
    bool fail_send = false;
    test_wrapper<JoystickType::Arm>(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            InSequence s;
            //                         control_id, axis_id, magnitude
            // control_id = 1, axis_id = 1 -> control_id = 0, axis_id = 2
            EXPECT_CALL(mock, arm_callback(Eq(1), Eq(0), Ge(1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(2), DoubleEq(0))).Times(1);

            EXPECT_CALL(mock, arm_callback(Eq(1), Eq(0), Ge(1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(2), Ge(1))).Times(1);

            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(0), Ge(1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(1), DoubleEq(0))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(0), Ge(1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(1), Gt(4))).Times(1);
        },
        [&fail_send](int fd) -> int {
            // write the message
            // control_id = event.number / 2
            // axis_id = event.number % 2
            // publishes for each axis of a control
            js_event event{};
            event.value = 10000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = 10000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;
            fail_send |= !write_event(fd, event);

            event.value = 10000;
            event.type = JS_EVENT_AXIS;
            event.number = 0;
            fail_send |= !write_event(fd, event);

            event.value = 30000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;
            fail_send |= !write_event(fd, event);

            return 8;
        });
    ASSERT_FALSE(fail_send);
}

// NOLINTNEXTLINE(readability-function-size,cppcoreguidelines-*)
TEST(JoystickTest, negativeTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control2";
    bool fail_send = false;
    test_wrapper<JoystickType::Arm>(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            InSequence s;
            //                         control_id, axis_id, magnitude
            // control_id = 1, axis_id = 1 -> control_id = 0, axis_id = 2
            EXPECT_CALL(mock, arm_callback(Eq(1), Eq(0), Le(-1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(2), DoubleEq(0))).Times(1);

            EXPECT_CALL(mock, arm_callback(Eq(1), Eq(0), Le(-1))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(0), Eq(2), Le(-1))).Times(1);

            EXPECT_CALL(mock, arm_callback(Eq(2), Eq(0), Le(-3))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(2), Eq(1), DoubleEq(0))).Times(1);

            EXPECT_CALL(mock, arm_callback(Eq(2), Eq(0), Le(-3))).Times(1);
            EXPECT_CALL(mock, arm_callback(Eq(2), Eq(1), Lt(-2))).Times(1);
        },
        [&fail_send](int fd) -> int {
            // write the message
            js_event event{};
            event.value = -10000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;
            fail_send |= !write_event(fd, event);

            event.value = -10000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;
            fail_send |= !write_event(fd, event);

            event.value = -10000;
            event.type = JS_EVENT_AXIS;
            event.number = 4;
            fail_send |= !write_event(fd, event);

            event.value = -10000;
            event.type = JS_EVENT_AXIS;
            event.number = 5;
            fail_send |= !write_event(fd, event);

            return 8;
        });
    ASSERT_FALSE(fail_send);
}

// NOLINTNEXTLINE(readability-function-size,cppcoreguidelines-*)
TEST(JoystickTest, buttonTest)
{
    constexpr auto test_namespace = "ctrl_test";
    constexpr auto node_name = "test_control2";
    bool fail_send = false;
    test_wrapper<JoystickType::Arm>(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            InSequence s;
            EXPECT_CALL(mock, arm_callback(Eq(5), Eq(0), DoubleEq(1)));
            EXPECT_CALL(mock, arm_callback(Eq(5), Eq(0), DoubleEq(0)));
            EXPECT_CALL(mock, arm_callback(Eq(28), Eq(0), DoubleEq(1)));
            EXPECT_CALL(mock, arm_callback(Eq(28), Eq(0), DoubleEq(0)));
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

// NOLINTNEXTLINE(readability-function-size,cppcoreguidelines-*)
TEST(JoystickTest, drivesTest)
{
    constexpr auto test_namespace = "drives_test";
    constexpr auto node_name = "test_control3";
    bool fail_send = false;
    test_wrapper<JoystickType::Drives>(
        test_namespace, node_name,
        [](auto& mock, auto& /*sequence*/) {
            InSequence s;

            // linear
            EXPECT_CALL(mock,
                        drives_callback(Gt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), DoubleEq(0), Eq("base_link")))
                .Times(2);
            EXPECT_CALL(mock,
                        drives_callback(Lt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), DoubleEq(0), Eq("base_link")))
                .Times(2);

            // reset
            EXPECT_CALL(mock, drives_callback(DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                              DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                              Eq("base_link")))
                .Times(1);

            // angular
            EXPECT_CALL(mock, drives_callback(DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                              DoubleEq(0), DoubleEq(0), Gt(0),
                                              Eq("base_link")))
                .Times(2);
            EXPECT_CALL(mock, drives_callback(DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                              DoubleEq(0), DoubleEq(0), Lt(0),
                                              Eq("base_link")))
                .Times(2);

            // linear and angular
            EXPECT_CALL(mock,
                        drives_callback(Gt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), Lt(0), Eq("base_link")))
                .Times(1);

            EXPECT_CALL(mock,
                        drives_callback(Gt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), Gt(0), Eq("base_link")))
                .Times(1);

            EXPECT_CALL(mock,
                        drives_callback(Lt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), Gt(0), Eq("base_link")))
                .Times(1);

            EXPECT_CALL(mock,
                        drives_callback(Lt(0), DoubleEq(0), DoubleEq(0), DoubleEq(0),
                                        DoubleEq(0), Lt(0), Eq("base_link")))
                .Times(1);
        },
        [&fail_send](int fd) -> int {
            // write the message
            js_event event{};
            event.value = -30000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;  // control 0, axis 1
            fail_send |= !write_event(fd, event);

            event.value = -20000;
            event.type = JS_EVENT_AXIS;
            event.number = 0;                      // control 0, axis 0
            fail_send |= !write_event(fd, event);  // should do nothing

            event.value = 20000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;  // control 0, axis 1
            fail_send |= !write_event(fd, event);

            event.value = 17000;
            event.type = JS_EVENT_AXIS;
            event.number = 0;                      // control 0, axis 0
            fail_send |= !write_event(fd, event);  // should do nothing

            // reset
            event.value = 0;
            event.type = JS_EVENT_AXIS;
            event.number = 1;  // control 0, axis 1
            fail_send |= !write_event(fd, event);

            // angular
            event.value = -25000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;  // control 1, axis 0
            fail_send |= !write_event(fd, event);

            event.value = 25000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;                      // control 1, axis 1
            fail_send |= !write_event(fd, event);  // should change nothing

            event.value = 20000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;  // control 1, axis 0
            fail_send |= !write_event(fd, event);

            event.value = -20000;
            event.type = JS_EVENT_AXIS;
            event.number = 3;                      // control 1, axis 1
            fail_send |= !write_event(fd, event);  // should change nothing

            // linear and angular
            event.value = -20000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;  // control 0, axis 1
            fail_send |= !write_event(fd, event);

            event.value = -20000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;  // control 1, axis 0
            fail_send |= !write_event(fd, event);

            event.value = 20000;
            event.type = JS_EVENT_AXIS;
            event.number = 1;  // control 0, axis 1
            fail_send |= !write_event(fd, event);

            event.value = 17000;
            event.type = JS_EVENT_AXIS;
            event.number = 2;  // control 1, axis 0
            fail_send |= !write_event(fd, event);

            return 13;
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