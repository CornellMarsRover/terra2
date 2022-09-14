#include <cstdio>

#include "cmr_msgs/action/test_target_position.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * The TestController is a node that listens for messages published to the
 * /test/set_target topic of type geometry_msgs::Point. It then creates a goal to
 * move the robot to the specified point via an action. This node will log feedback
 * updates of position and notify the user when the robot has reached to goal.
 */
class TestControllerClient : public rclcpp::Node
{
    std::shared_ptr<rclcpp_action::Client<cmr_msgs::action::TestTargetPosition>>
        m_action_client;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Point>>
        m_target_listener;

    using target_pos_t = cmr_msgs::action::TestTargetPosition;
    using target_pos_goal_t = target_pos_t::Goal;
    using target_pos_handle_t = rclcpp_action::ClientGoalHandle<target_pos_t>;

  public:
    TestControllerClient() : rclcpp::Node("test_client")
    {
        m_action_client =
            rclcpp_action::create_client<cmr_msgs::action::TestTargetPosition>(
                this, "test_target");
        m_target_listener = rclcpp::create_subscription<geometry_msgs::msg::Point>(
            *this, "/test/set_target", 10,
            [this](std::shared_ptr<geometry_msgs::msg::Point> msg) {
                return handle_target(msg);
            });
        RCLCPP_INFO(get_logger(),
                    "Controller listening for points on /test/set_target");
    }

  private:
    /**
     * @brief Callback for receiving a new target point
     *
     *
     * @param msg
     */
    void handle_target(const std::shared_ptr<geometry_msgs::msg::Point>& msg)
    {
        RCLCPP_INFO(get_logger(), "Controller got target {%f, %f, %f}", msg->x,
                    msg->y, msg->z);
        if (!m_action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_INFO(get_logger(),
                        "Cannot connect to action server, ignoring request");
        } else {
            auto send_goal_options = rclcpp_action::Client<
                cmr_msgs::action::TestTargetPosition>::SendGoalOptions();
            send_goal_options.feedback_callback = [this](auto handle,
                                                         auto feedback) {
                return on_goal_feedback(handle, feedback);
            };
            send_goal_options.result_callback = [this](auto result) {
                return on_goal_result(result);
            };
            send_goal_options.goal_response_callback = [this](auto future) {
                RCLCPP_INFO(get_logger(), "Was goal accepted?: %d",
                            static_cast<bool>(future.get()));
            };
            auto goal = cmr_msgs::action::TestTargetPosition_Goal{};
            goal.x = static_cast<float>(msg->x);
            goal.y = static_cast<float>(msg->y);
            goal.z = 0;
            m_action_client->async_send_goal(goal, send_goal_options);
        }
    }

    void on_goal_feedback(
        std::shared_ptr<target_pos_handle_t>,
        const std::shared_ptr<const target_pos_t::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "We are %f away from target", feedback->distance);
    }

    void on_goal_result(const target_pos_handle_t::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (result.result->success) {
                    RCLCPP_INFO(get_logger(),
                                "We have reached the target successfully");
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to reach target");
                }
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(get_logger(), "Goal canceled");
                break;
            default:
                RCLCPP_INFO(get_logger(), "Goal unknown or aborted");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    printf("The test_controller node.\n");
    rclcpp::spin(std::make_shared<TestControllerClient>());

    rclcpp::shutdown();
    return 0;
}
