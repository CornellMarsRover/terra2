#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
namespace cmr
{

/**
 * `InverseKinematics`
 *
 * The inverse kinematics node plans and executes arm poses. A base station
 * node sends pose messages (position and orientation) to the node, which then
 * either plans and executes the pose or returns an error if the pose is unreachable.
 * It subscribes to the "/arm_pose_topic" topic for poses and requires
 * MoveIt 2 to be launched to function.
 */
class InverseKinematics : public cmr::fabric::FabricNode
{
  private:
    fabric::LifecycleSubscription<geometry_msgs::msg::PoseStamped>::ptr_t m_arm_pose_sub;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
        m_move_group_interface;
    std::shared_ptr<rclcpp::Node> m_node;
    std::unique_ptr<fabric::GenericLifecycle> m_wall_timer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  public:
    /**
     * Constructs a `InverseKinematics`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit InverseKinematics(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    void update_arm_position(const geometry_msgs::msg::PoseStamped msg);
};

}  // namespace cmr