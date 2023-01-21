#include "cmr_arm/inverse_kinematics.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/external/tomlcpp.hxx"

namespace cmr
{

InverseKinematics::InverseKinematics(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
}

bool InverseKinematics::update_arm_position(const geometry_msgs::msg::Pose msg)
{
    m_move_group_interface->setPoseTarget(msg);
    // Create a plan to that target pose
    const auto [success, plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan msg2;
        auto const ok = static_cast<bool>(m_move_group_interface->plan(msg2));
        return std::make_pair(ok, msg2);
    }();
    // Execute the plan
    if (success) {
        m_move_group_interface->execute(plan);
    } else {
        CMR_LOG(ERROR, "Planning failed!");
    }
    return true;
}

bool InverseKinematics::configure(const std::shared_ptr<toml::Table>& table)
{
    CMR_LOG(INFO, "Configuring!");
    const auto node_settings = table->getTable("node");
    const auto [ok_arm_group, arm_planning_group_name] =
        node_settings->getString("arm_planning_group");
    if (!ok_arm_group) {
        return false;
    }
    const auto& arm_pgroup = arm_planning_group_name;
    m_arm_pose_sub =
        std::make_unique<fabric::LifecycleSubscription<geometry_msgs::msg::Pose>>(
            create_lifecycle_subscription<geometry_msgs::msg::Pose>(
                "arm_pose_topic", 10,
                std::bind(&InverseKinematics::update_arm_position, this,
                          std::placeholders::_1)));

    m_node = std::make_shared<rclcpp::Node>(
        "movegroupinterface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    using namespace std::chrono_literals;
    m_wall_timer = create_lifecycle_timer(
        10ms, std::function([this]() { rclcpp::spin_some(m_node); }));

    using moveit::planning_interface::MoveGroupInterface;
    m_move_group_interface =
        std::make_unique<MoveGroupInterface>(m_node, arm_pgroup);

    return true;
}

bool InverseKinematics::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    m_arm_pose_sub->activate();
    m_wall_timer->activate();
    return true;
}

bool InverseKinematics::deactivate()
{
    // undo the effects of activate here
    m_arm_pose_sub->deactivate();
    m_wall_timer->deactivate();
    return true;
}

bool InverseKinematics::cleanup()
{
    // undo the effects of configure here
    return true;
}

}  // namespace cmr
