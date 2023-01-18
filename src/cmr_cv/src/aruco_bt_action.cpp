#include "cmr_cv/aruco_bt_action.hpp"

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

BT::NodeStatus ArucoAction::tick()
{
    rclcpp::spin_some(m_ros_node);

    const auto now = m_ros_node->now();

    const auto diftime = now - m_latest;

    if (diftime < rclcpp::Duration(5, 0)) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
ArucoAction::ArucoAction(const std::string& name, const std::string&,
                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf), m_latest({})
{
    // NOLINTNEXTLINE
    using namespace std::placeholders;
    m_ros_node = std::make_shared<rclcpp::Node>("aruco_listener_node");
    m_ros_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&ArucoAction::topic_callback, this, _1));
}

void ArucoAction::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    const auto time_now = msg->header.stamp;
    if (time_now.sec > m_latest.sec ||
        (time_now.nanosec > m_latest.nanosec && time_now.sec == m_latest.sec)) {
        m_latest = time_now;
    }
}

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<ArucoAction>(name, "aruco", config);
    };

    factory.registerBuilder<ArucoAction>("Aruco", builder);
}