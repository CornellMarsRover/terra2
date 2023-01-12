#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_bt_navigator
{
/**
 * @class ARTagNavigator
 * @brief A navigator for navigating to a specified pose
 */
class ARTagNavigator
    : public nav2_core::BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose>
{
  public:
    using ActionT = nav2_msgs::action::NavigateToPose;

    /**
     * @brief A constructor for ARTagNavigator
     */
    ARTagNavigator() : BehaviorTreeNavigator() {}

    /**
     * @brief A configure state transition to configure navigator's state
     * @param node Weakptr to the lifecycle node
     * @param odom_smoother Object to get current smoothed robot's speed
     */
    bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node,
                   std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

    /**
     * @brief A cleanup state transition to remove memory allocated
     */
    bool cleanup() override;

    /**
     * @brief A subscription and callback to handle the topic-based goal
     * published from rviz
     * @param pose Pose received via atopic
     */
    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string getName() { return std::string("ar_tag_navigate"); }

    /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string getDefaultBTFilepath(
        rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  protected:
    /**
     * @brief A callback to be called when a new goal is received by the BT
     * action server Can be used to check if goal is valid and put values on
     * the blackboard which depend on the received goal
     * @param goal Action template's goal message
     * @return bool if goal was received successfully to be processed
     */
    bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

    /**
     * @brief A callback that defines execution that happens on one iteration
     * through the BT Can be used to publish action feedback
     */
    void onLoop() override;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

    /**
     * @brief A callback that is called when a the action is completed, can
     * fill in action result message or indicate that this action is done.
     * @param result Action template result message to populate
     * @param final_bt_status Resulting status of the behavior tree execution
     * that may be referenced while populating the result.
     */
    void goalCompleted(typename ActionT::Result::SharedPtr result,
                       const nav2_behavior_tree::BtStatus final_bt_status) override;

    /**
     * @brief Goal pose initialization on the blackboard
     * @param goal Action template's goal message to process
     */
    void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

    rclcpp::Time start_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<ActionT>::SharedPtr self_client_;

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;

    // Odometry smoother object
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif