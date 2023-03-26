#include "cmr_nav/pose_generator_node.hpp"

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#define PI 3.1415926535
namespace cmr
{
const std::string PoseGeneratorAction::input = "has_arrived";
const std::string PoseGeneratorAction::initial_goal_input = "initial_position";
const std::string PoseGeneratorAction::output = "goal";

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
PoseGeneratorAction::PoseGeneratorAction(const std::string& xml_tag_name,
                                         const std::string&,
                                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf),
      m_origin_init(false),
      m_pose_id(0),
      m_unit_distance(2)
{
    m_previous_pose = geometry_msgs::msg::PoseStamped();
}

geometry_msgs::msg::PoseStamped PoseGeneratorAction::generate_next_pose() const
{
    int circle = (m_pose_id / 4) + 1;
    int location = m_pose_id % 4;
    auto new_pose_stamp = geometry_msgs::msg::PoseStamped();
    new_pose_stamp.pose.position.x = m_unit_distance * circle * ((location + 1) % 2);
    new_pose_stamp.pose.position.y = m_unit_distance * circle * (location % 2);
    if (location > 1) {
        new_pose_stamp.pose.position.x = -new_pose_stamp.pose.position.x;
        new_pose_stamp.pose.position.y = -new_pose_stamp.pose.position.y;
    }
    double angle = 0;
    if (location == 0 && circle > 1) {
        angle = atan2(circle - 1, circle);
    } else if (location != 0) {
        angle = (45 + 90 * location) * (PI / 180);
    }
    double c = cos(angle * 0.5);
    double s = sin(angle * 0.5);

    new_pose_stamp.pose.orientation.w = c;
    new_pose_stamp.pose.orientation.z = s;
    new_pose_stamp.header.frame_id = "map";

    new_pose_stamp.pose.position.x += m_origin.pose.position.x;
    new_pose_stamp.pose.position.y += m_origin.pose.position.y;
    return new_pose_stamp;
}

void PoseGeneratorAction::on_tick()
{
    bool has_arrived = false;
    getInput<bool>(input, has_arrived);
    if (!m_origin_init) {
        getInput<geometry_msgs::msg::PoseStamped>(initial_goal_input, m_origin);
        m_origin_init = true;
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

    RCLCPP_INFO(rclcpp::get_logger("Pose Generator Logger"), "Goal Pose: %s",
                goal_out.c_str());
    RCLCPP_INFO(rclcpp::get_logger("Pose Generator Logger"), "Prev Pose: %s",
                prev_out.c_str());
}
}  // namespace cmr

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<cmr::PoseGeneratorAction>(name, "poseGenerator",
                                                          config);
    };

    factory.registerBuilder<cmr::PoseGeneratorAction>("PoseGenerator", builder);
}