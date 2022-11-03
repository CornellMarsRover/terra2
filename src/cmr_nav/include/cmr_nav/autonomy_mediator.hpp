#pragma once

// #include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

namespace cmr
{

using DependencyMap = std::unordered_map<std::string, std::string>;
using MoveBaseClient = BtActionNode<nav2_msgs::action::FollowPath>;
// Nav 1 used action client, gave postion to navigate to, need to find out how nav 2
// works
using MoveBaseCallback_t =
    std::function<void(const actionlib::SimpleClientGoalState&,
                       const move_base_msgs::MoveBaseResultConstPtr&)>;

enum class Signal { None, Teleoperation, Autonomous, Arrival };

class AutonomyMediator
{
    rclcpp::Node m_led_publisher
        // Publish to topic for Led
        rclcpp::PublisherBase m_feedback_pub;
    rclcpp::PublisherBase m_pos_pub;

    std::unique_ptr<MoveBaseClient> m_move_client;

  public:
    AutonomyMediator(rclcpp::Node& n, const DependencyMap& deps);

    /**
     * Signals status by turning on the correct LED color
     * Basic guaruntee
     */
    void signal(Signal sig);

    /**
     * Publishes a success frame on the feedback topic
     * to send status back to the plan client
     */
    inline void notify_success() { return send_autonomy_feedback(true, {}); }

    /**
     * Sends a failure status and error message back to the planner
     * but publishing a message on the feedback topic
     */
    inline void notify_fail(const std::string& msg)
    {
        return send_autonomy_feedback(false, std::ref(msg));
    }

    /// Publishes the target position
    inline void log_target_pos(geometry_msgs::msg::PoseStamped pos)
    {
        m_pos_pub.publish(pos);
    }

    /// Sends a goal to the move base client and registers the
    /// callback function
    void send_move_goal(Actions::Goal<move_base_msgs::MoveBaseAction> goal,
                        MoveBaseCallback_t callback);

    /// Gets the current state of the move base client
    inline auto get_move_state() { return m_move_client->getState(); }

    /**
     * Attempts to connect to the move base client
     * If connection fails, operator will be notified via feedback topic.
     * Basic gauruntee. Failure notification may error after connection already
     * failed
     * @param timeoutSec, the max amount of seconds to wait to connect
     * @return true on success, false on failure
     */
    bool connect_to_move_base(rclcpp::Duration timeoutSec = rclcpp::Duration(5, 0));

    /// Stops all move base goals
    inline void halt_move_base() { m_move_client->cancelAllGoals(); }

  private:
    void send_autonomy_feedback(
        bool success, std::optional<std::reference_wrapper<const std::string>> msg);
};

}  // namespace cmr