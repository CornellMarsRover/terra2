#include "cmr_cv/aruco_bt_action.hpp"

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
<<<<<<< HEAD
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

// Template specialization to converts a string to Position2D.
=======
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

/** Template specialization to convert a string to Position2D. */
>>>>>>> 3a05e0e6871bd37e617feb49dd167de6abf1f4ef
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

<<<<<<< HEAD
ArucoAction::ArucoAction(const std::string& name, const std::string&,
                         const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf), m_latest({}), m_latest_position({})
{
    // NOLINTNEXTLINE
    using namespace std::placeholders;
    m_ros_node = std::make_shared<rclcpp::Node>("aruco_listener_node", "cmr");
    m_sub = m_ros_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aruco_poses", 10, std::bind(&ArucoAction::topic_callback, this, _1));
    RCLCPP_INFO(m_ros_node->get_logger(), "Aruco: Listener node created!");

    m_latest_position.x = 1;
    m_latest_position.y = 2;
    m_latest_position.z = 3;
}

=======
>>>>>>> 3a05e0e6871bd37e617feb49dd167de6abf1f4ef
BT::NodeStatus ArucoAction::tick()
{
    rclcpp::spin_some(m_ros_node);

<<<<<<< HEAD
    const auto now = m_ros_node->now();

    const auto diftime = now - m_latest;

    setOutput("ARTag", m_latest_position);

    if (diftime < rclcpp::Duration(5, 0)) {
        RCLCPP_INFO(rclcpp::get_logger("Aruco Logger"), "Aruco: Success!");
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("Aruco Logger"), "Aruco: Failure!");
        return BT::NodeStatus::FAILURE;
    }
    // return BT::NodeStatus::SUCCESS;
=======
    // If the vector of nodes is not empty, meaning that there was at least 1
    // aruco pose detected, then the average position of the AR tags is
    // posted to the ARTag output port and a SUCCESS is returned; if the vector
    // is empty and no AR tags were detected then FAILURE is returned
    if (!m_node_vector.empty()) {
        setOutput("ARTag", m_latest_position_average);
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
    m_ros_node->create_subscription<geometry_msgs::msg::PoseArray>(
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

        setOutput("ARTag", out.vector);

    } catch (const tf2::TransformException& ex) {
        RCLCPP_INFO(m_ros_node->get_logger(), "Could not transform %s to %s: %s",
                    "world", msg->header.frame_id.c_str(), ex.what());
        return;
    }
>>>>>>> 3a05e0e6871bd37e617feb49dd167de6abf1f4ef
}

void ArucoAction::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
<<<<<<< HEAD
    RCLCPP_INFO(rclcpp::get_logger("Aruco Logger"), "Aruco: Call back called");

    const auto time_now = msg->header.stamp;
    if (time_now.sec > m_latest.sec ||
        (time_now.nanosec > m_latest.nanosec && time_now.sec == m_latest.sec)) {
        m_latest = time_now;
        RCLCPP_INFO(rclcpp::get_logger("Aruco Logger"),
                    "Aruco: Lastest Time updated!");

        // msg->poses[0];
        // m_latest_position =
    }
=======
    for (const auto& pose : msg->poses) {
        m_node_vector.push_back(pose);
    }
    geometry_msgs::msg::Vector3 latest_position;
    std::vector<geometry_msgs::msg::Pose> poses_to_average;
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
>>>>>>> 3a05e0e6871bd37e617feb49dd167de6abf1f4ef
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
<<<<<<< HEAD
}
=======
}
>>>>>>> 3a05e0e6871bd37e617feb49dd167de6abf1f4ef
