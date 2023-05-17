#include "cmr_cv/aruco_bt_action.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

const std::string ArucoAction::output = "ARTag";
/** Template specialization to convert a string to Position2D. */
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

BT::NodeStatus ArucoAction::tick()
{
    rclcpp::spin_some(m_ros_node);

    // If the vector of nodes is not empty, meaning that there was at least 1
    // aruco pose detected, then the average position of the AR tags is
    // posted to the ARTag output port and a SUCCESS is returned; if the vector
    // is empty and no AR tags were detected then FAILURE is returned
    if (!m_node_vector.empty()) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

ArucoAction::ArucoAction(const std::string& name, const std::string&,
                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf)
{
    // NOLINTNEXTLINE
    using namespace std::placeholders;
    m_ros_node = std::make_shared<rclcpp::Node>("aruco_listener_node");
    m_sub = m_ros_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&ArucoAction::topic_callback, this, _1));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_ros_node->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void ArucoAction::transformhelper(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    try {
        auto t = m_tf_buffer->lookupTransform("map", msg->header.frame_id,
                                              tf2::TimePointZero);

        geometry_msgs::msg::Vector3Stamped in{};

        in.vector.x = m_latest_position_average.x;
        in.vector.y = m_latest_position_average.y;
        in.vector.z = m_latest_position_average.z;

        geometry_msgs::msg::Vector3Stamped out;

        tf2::doTransform(in, out, t);

        setOutput(output, out.vector);

    } catch (const tf2::TransformException& ex) {
        RCLCPP_INFO(m_ros_node->get_logger(), "Could not transform %s to %s: %s",
                    "world", msg->header.frame_id.c_str(), ex.what());
        return;
    }
}

void ArucoAction::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_node_vector = {};
    for (const auto& pose : msg->poses) {
        m_node_vector.push_back(pose);
    }
    geometry_msgs::msg::Vector3 latest_position;
    std::vector<geometry_msgs::msg::Pose> poses_to_average;
    // get position of first pose in vector, add it to a separate vector
    latest_position.x = m_node_vector[0].position.x;
    latest_position.y = m_node_vector[0].position.y;
    latest_position.z = m_node_vector[0].position.z;
    poses_to_average.push_back(m_node_vector[0]);
    // loop through the rest of the poses and if the coordinates of a pose are
    // similar to the coordinates of the first pose add it to the separate vector
    for (unsigned int i = 1; i < m_node_vector.size(); i++) {
        if (abs(m_node_vector[i].position.x - latest_position.x) < 1 &&
            abs(m_node_vector[i].position.y - latest_position.y) < 1 &&
            abs(m_node_vector[i].position.z - latest_position.z) < 1) {
            poses_to_average.push_back(m_node_vector[i]);
        }
    }
    // average the coordinates of the poses in the m_position_to_post vector to
    // get the coordinate that the rover should circle around
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    for (auto& i : poses_to_average) {
        sum_x += i.position.x;
        sum_y += i.position.y;
        sum_z += i.position.z;
    }
    auto size = static_cast<double>(poses_to_average.size());
    double x_position = sum_x / size;
    double y_position = sum_y / size;
    double z_position = sum_z / size;
    m_latest_position_average.x = x_position;
    m_latest_position_average.y = y_position;
    m_latest_position_average.z = z_position;

    transformhelper(msg);
}

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<ArucoAction>(name, "aruco", config);
    };

    factory.registerBuilder<ArucoAction>("Aruco", builder);
}
