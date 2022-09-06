#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "cmr_msgs/action/test_target_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "rclcpp_components/register_node_macro.hpp"

// Note that the test "rover" can only move along the x-direction

class PositionControllerNode : public rclcpp::Node
{
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> m_odom_listener;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> m_cmd_vel_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_frame_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_frame_listener;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    using target_pos_t = cmr_msgs::action::TestTargetPosition;
    using target_pos_goal_t = target_pos_t::Goal;
    using target_pos_handle_t = rclcpp_action::ServerGoalHandle<target_pos_t>;

    std::list<std::shared_ptr<target_pos_handle_t>> m_active_goals;

    std::shared_ptr<rclcpp_action::Server<target_pos_t>> m_target_server;
    std::shared_ptr<rclcpp::WallTimer<std::function<void()>>> m_goal_timer;
    std::optional<std::chrono::time_point<std::chrono::system_clock>>
        m_goal_start_time = {};

    auto handle_goal(const rclcpp_action::GoalUUID&,
                     std::shared_ptr<const target_pos_goal_t>)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    auto handle_cancel(const std::shared_ptr<target_pos_handle_t> goal_handle)
    {
        const auto it = std::find_if(m_active_goals.begin(), m_active_goals.end(),
                                     [&goal_handle](const auto& handle) {
                                         return handle->get_goal_id() ==
                                                goal_handle->get_goal_id();
                                     });
        if (it != m_active_goals.end()) {
            RCLCPP_INFO(get_logger(), "Removing active goal due to cancel");
            m_active_goals.erase(it);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

  public:
    PositionControllerNode() : rclcpp::Node("test_position_controller")
    {
        using namespace std::placeholders;  // NOLINT
        m_odom_listener = this->create_subscription<nav_msgs::msg::Odometry>(
            "/test/odometry", 10,
            // NOLINTNEXTLINE(performance-*)
            [this](std::shared_ptr<nav_msgs::msg::Odometry> msg) {
                return handle_odom_msg(msg);
            });
        m_frame_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        m_target_server = rclcpp_action::create_server<target_pos_t>(
            this, "test_target",
            // NOLINTNEXTLINE(modernize-avoid-bind)
            std::bind(&PositionControllerNode::handle_goal, this, _1, _2),
            // NOLINTNEXTLINE(modernize-avoid-bind)
            std::bind(&PositionControllerNode::handle_cancel, this, _1),
            // NOLINTNEXTLINE(modernize-avoid-bind)
            std::bind(&PositionControllerNode::handle_accepted, this, _1));
        m_goal_timer = create_wall_timer<int64_t, std::milli, std::function<void()>>(
            std::chrono::milliseconds(500),
            [this]() { return move_towards_goal(); });
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_frame_listener =
            std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_cmd_vel_pub =
            create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel", 10);
    }

  private:
    void handle_odom_msg(const std::shared_ptr<nav_msgs::msg::Odometry>& msg)
    {
        // If rover gets stuck, but keeps spinning wheels, odom thinks
        // it is still moving
        const auto now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped frame;
        frame.header.stamp = now;
        frame.header.frame_id = "world";
        frame.child_frame_id = "base";
        frame.transform.rotation = msg->pose.pose.orientation;
        frame.transform.translation.x = msg->pose.pose.position.x;
        frame.transform.translation.y = msg->pose.pose.position.y;
        frame.transform.translation.z = msg->pose.pose.position.z;

        m_frame_broadcaster->sendTransform(frame);
    }

    void handle_accepted(std::shared_ptr<target_pos_handle_t> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a
        // new thread
        RCLCPP_INFO(get_logger(), "Accepted goal");
        m_active_goals.emplace_back(std::move(goal_handle));
    }

    /*std::optional<tf2::Quaternion> turn_towards_goal(
        std::chrono::milliseconds goal_duration, const target_pos_goal_t& target_pos,
        const geometry_msgs::msg::TransformStamped& world_pos)
    {
        const tf2::Vector3 target{target_pos.x, target_pos.y, 0};
        const tf2::Vector3 current{world_pos.transform.translation.x,
                                   world_pos.transform.translation.y, 0};
        const auto angle_diff = std::abs(tf2::tf2Angle(target, current));
        if ((angle_diff > 0 && angle_diff < 0.5) ||
            (angle_diff > 5.9 && angle_diff < 6.6)) {
            const auto target_rot = tf2::shortestArcQuat(target, current);
            const auto cur_quat = tf2::Quaternion(
                world_pos.transform.rotation.x, world_pos.transform.rotation.y,
                world_pos.transform.rotation.z, world_pos.transform.rotation.w);
            return tf2::slerp(cur_quat, target_rot,
                              static_cast<double>(goal_duration.count()) / 1300.0);
        }
        return {};
    }*/

    /// Returns true if we reached the target
    bool update_cmd_vel_and_check_reach_target(const tf2::Vector3& target,
                                               const tf2::Vector3& current)
    {
        if (tf2::tf2Distance(target, current) > 0.1) {
            geometry_msgs::msg::Twist update_msg;
            const auto linear = (target - current).normalize() * 0.2;
            update_msg.linear.x = linear.x();
            update_msg.linear.y = linear.y();
            update_msg.linear.z = linear.z();
            m_cmd_vel_pub->publish(update_msg);
            RCLCPP_INFO(get_logger(), "Moving towards goal!");
            return false;
        } else {
            RCLCPP_INFO(get_logger(), "Move complete!");
            m_active_goals.pop_front();
            m_goal_start_time = {};
            return true;
        }
    }

    void move_towards_goal()
    {
        if (!m_active_goals.empty()) {
            auto handle = m_active_goals.front();
            if (!m_goal_start_time) {
                m_goal_start_time = std::chrono::system_clock::now();
            }
            const auto target_pos = handle->get_goal();
            geometry_msgs::msg::TransformStamped world_pos;
            try {
                world_pos = m_tf_buffer->lookupTransform("world", "base",
                                                         tf2::TimePointZero);
            } catch (tf2::TransformException&) {
                RCLCPP_WARN(get_logger(), "Could not transform frame");
                return;
            }

            const tf2::Vector3 target{target_pos->x, target_pos->y, 0};
            const tf2::Vector3 current{world_pos.transform.translation.x,
                                       world_pos.transform.translation.y, 0};
            if (update_cmd_vel_and_check_reach_target(target, current)) {
                auto result = std::make_shared<target_pos_t::Result>();
                result->success = true;
                handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Sending back success response");
            } else {
                auto feedback = std::make_shared<target_pos_t::Feedback>();
                feedback->distance =
                    static_cast<float>(tf2::tf2Distance(target, current));
                RCLCPP_INFO(get_logger(), "Sending feedback");
                handle->publish_feedback(feedback);
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    printf("The test_tf2 node.\n");
    rclcpp::spin(std::make_shared<PositionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
