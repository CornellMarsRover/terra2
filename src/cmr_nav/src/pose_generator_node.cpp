#include "cmr_nav/pose_generator_node.hpp"

#include <math.h>

#define PI 3.1415926535
namespace cmr
{
const std::string PoseGeneratorAction::input = "has_arrived";
const std::string PoseGeneratorAction::output = "next_pose";

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
PoseGeneratorAction::PoseGeneratorAction(const std::string& xml_tag_name,
                                         const std::string&,
                                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf), m_pose_id(0)
{
    m_previous_pose = geometry_msgs::msg::Pose();
}

geometry_msgs::msg::Pose PoseGeneratorAction::generate_next_pose()
{
    int circle = (m_pose_id / 4) + 1;
    int location = m_pose_id % 4;
    auto pose = geometry_msgs::msg::Pose();
    pose.position.x = m_unit_distance * circle * ((location + 1) % 2);
    pose.position.y = m_unit_distance * circle * (location % 2);
    if (location > 1) {
        pose.position.x = -pose.position.x;
        pose.position.y = -pose.position.y;
    }
    double angle = 0;
    if (location == 0 && circle > 1) {
        angle = atan2(circle - 1, circle);
    } else if (location != 0) {
        angle = (45 + 90 * location) * (PI / 180);
    }
    double c = cos(angle * 0.5);
    double s = sin(angle * 0.5);

    pose.orientation.w = c;
    pose.orientation.z = s;
    return pose;
}

void PoseGeneratorAction::on_tick()
{
    bool has_arrived = false;
    getInput<bool>(input, has_arrived);
    if (has_arrived) {
        m_previous_pose = generate_next_pose();
    }
    setOutput(output, m_previous_pose);
}
}  // namespace cmr