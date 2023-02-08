#include "cmr_cv/aruco_bt_action.hpp"

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

// Template specialization to convert a string to Position2D.
namespace BT
{
template <>
inline geometry_msgs::msg::Vector3 convertFromString<geometry_msgs::msg::Vector3>(
    BT::StringView str)
{
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    } else {
        geometry_msgs::msg::Vector3 output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);

        return output;
    }
}
}  // end namespace BT

// Tick method that spins the node and returns success if there are any AR tags
// in the vector (meaning one was detected) and failure if there wasn't one detected;
// posts the latest position to the blackboard that is the average of the similarly
// located tags in the vector
BT::NodeStatus ArucoAction::tick()
{
    rclcpp::spin_some(m_ros_node);

    setOutput("ARTag", m_latest_position_average);

    if (!m_node_vector.empty()) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

ArucoAction::ArucoAction(const std::string& name, const std::string&,
                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf), m_latest({}), m_latest_position({})
{
    // NOLINTNEXTLINE
    using namespace std::placeholders;
    m_ros_node = std::make_shared<rclcpp::Node>("aruco_listener_node");
    m_ros_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&ArucoAction::topic_callback, this, _1));

    // get position of first pose in vector, add it to a separate vector
    m_latest_position.x = m_node_vector[0].position.x;
    m_latest_position.y = m_node_vector[0].position.y;
    m_latest_position.z = m_node_vector[0].position.z;
    m_position_to_post.push_back(m_node_vector[0]);

    // loop through the rest of the poses and if the coordinates of a pose are
    // similar to the coordinates of the first pose add it to the separate vector
    for (unsigned int i = 1; i < m_node_vector.size(); i++) {
        if (m_node_vector[i].position.x - m_latest_position.x < 1 &&
            m_node_vector[i].position.y - m_latest_position.y < 1 &&
            m_node_vector[i].position.z - m_latest_position.z < 1) {
            m_position_to_post.push_back(m_node_vector[i]);
        }
    }

    // average the coordinates of the poses in the m_position_to_post vector to 
    // get the coordinate that the rover should circle around
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    for (auto & i : m_position_to_post) {
        sum_x += i.position.x;
        sum_y += i.position.y;
        sum_z += i.position.z;
    }
    auto size = static_cast<double>(m_position_to_post.size());
    double x_position = sum_x / size;
    double y_position = sum_y / size;
    double z_position = sum_z / size;
    m_latest_position_average.x=x_position;
    m_latest_position_average.y=y_position;
    m_latest_position_average.z=z_position;
}

// The callback function calculates the time that is stamped on the detected
// AR tag and sees if it is sooner than the previous time calculated. It also
// loops through the Aruco poses to add each pose to the vector of AR tags.
void ArucoAction::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    const auto time_now = msg->header.stamp;
    if (time_now.sec > m_latest.sec ||
        (time_now.nanosec > m_latest.nanosec && time_now.sec == m_latest.sec)) {
        m_latest = time_now;
    }
    for (const auto& pose : msg->poses) {
        m_node_vector.push_back(pose);
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

static BT::PortsList provided_ports()
{
    return {BT::OutputPort<geometry_msgs::msg::Vector3>("ARTag")};
}