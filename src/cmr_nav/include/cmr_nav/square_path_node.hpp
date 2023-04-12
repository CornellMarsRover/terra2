#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr
{
class SquarePathAction : public BT::SyncActionNode
{
  public:
    /**
     * @brief A constructor for cmr::PoseGeneratorAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    SquarePathAction(const std::string& xml_tag_name, const std::string& action_name,
                     const BT::NodeConfiguration& conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     */
    void on_tick();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    // NOLINTNEXTLINE(readability-identifier-naming)
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>(input, false,
                                    "Whether or not rover is at outputted pose"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>(initial_goal_input),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>(
                    output, "The next pose to travel to")};
    }

    BT::NodeStatus tick() override
    {
        if (m_pose_id == 4) {
            return BT::NodeStatus::SUCCESS;
        }
        on_tick();
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Uses the previous pose to determine the next pose to go to
     *
     * @return geometry_msgs::msg::Pose The next pose to go to
     */
    geometry_msgs::msg::PoseStamped generate_next_pose() const;

  private:
    geometry_msgs::msg::PoseStamped m_previous_pose;
    geometry_msgs::msg::PoseStamped m_origin;
    bool m_origin_init;
    int m_pose_id;
    int m_unit_distance;
    static std::string const input;
    static std::string const initial_goal_input;
    static std::string const output;
};
}  // namespace cmr