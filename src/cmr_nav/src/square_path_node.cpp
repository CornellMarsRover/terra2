#include "cmr_nav/square_path_node.hpp"

#include <math.h>

#include <rclcpp/rclcpp.hpp>

namespace cmr
{
const std::string SquarePathAction::input = "has_arrived";
const std::string SquarePathAction::initial_goal_input = "initial_position";
const std::string SquarePathAction::output = "goal";

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
SquarePathAction::SquarePathAction(const std::string& xml_tag_name,
                                   const std::string&,
                                   const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf),
      m_origin_init(false),
      m_pose_id(0),
      m_unit_distance(2)
{
    m_previous_pose = geometry_msgs::msg::PoseStamped();
}

geometry_msgs::msg::PoseStamped SquarePathAction::generate_next_pose() const
{
    auto new_pose_stamp = geometry_msgs::msg::PoseStamped();
    if (m_pose_id == 0) {
        new_pose_stamp.pose.position.x = m_unit_distance;
    } else if (m_pose_id == 1) {
        new_pose_stamp.pose.position.y = m_unit_distance;
    } else if (m_pose_id == 2) {
        new_pose_stamp.pose.position.x = -m_unit_distance;
    } else if (m_pose_id == 3) {
        new_pose_stamp.pose.position.y = -m_unit_distance;
    }

    new_pose_stamp.header.frame_id = "map";

    new_pose_stamp.pose.position.x += m_origin.pose.position.x;
    new_pose_stamp.pose.position.y += m_origin.pose.position.y;
    return new_pose_stamp;
}

void SquarePathAction::on_tick()
{
    bool has_arrived = false;
    getInput<bool>(input, has_arrived);
    if (!m_origin_init) {
        getInput<geometry_msgs::msg::PoseStamped>(initial_goal_input, m_origin);
        m_origin_init = true;
        std::string init_out = std::to_string(m_origin.pose.position.x) + ", " +
                               std::to_string(m_origin.pose.position.y);
        RCLCPP_INFO(rclcpp::get_logger("Square Path Logger"), "Initial Pose: %s",
                    init_out.c_str());
    }

    if (has_arrived) {
        m_previous_pose = generate_next_pose();
        m_pose_id++;
    }
    setOutput(output, m_previous_pose);

    auto goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>(output);

    std::string goal_out = std::to_string(goal.pose.position.x) + ", " +
                           std::to_string(goal.pose.position.y) +
                           " Frame: " + goal.header.frame_id;

    std::string prev_out = std::to_string(m_previous_pose.pose.position.x) + ", " +
                           std::to_string(m_previous_pose.pose.position.y);

    RCLCPP_INFO(rclcpp::get_logger("Square Path Logger"), "Goal Pose: %s",
                goal_out.c_str());
    RCLCPP_INFO(rclcpp::get_logger("Square Path Logger"), "Prev Pose: %s",
                prev_out.c_str());
}
}  // namespace cmr

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<cmr::SquarePathAction>(name, "squarePath", config);
    };

    factory.registerBuilder<cmr::SquarePathAction>("SquarePath", builder);
}