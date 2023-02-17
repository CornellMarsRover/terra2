#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/node.hpp>

/**
 * The ArucoAction class subscribes to aruco_poses topic. The constructor, methods,
 * and variables listed work together to post the average position of the detected
 * aruco tags that are within 1 meter of each other in order for the rover to
 * circle the post at the location.
 */
class ArucoAction : public BT::SyncActionNode
{
  public:
    ArucoAction(const std::string& name, const std::string& action_name,
                const BT::NodeConfiguration& conf);

    /**
     * Tick method that spins the node and returns success if there are any AR tags
     * in the vector (meaning one was detected) and failure if there wasn't one
     * detected; posts the latest position to the blackboard that is the average of
     * the similarly located tags in the vector
     * */
    BT::NodeStatus tick() override;

    static BT::PortsList provided_ports();

  private:
    std::shared_ptr<rclcpp::Node> m_ros_node;

    /**
     * The callback function calculates the time that is stamped on the detected
     * AR tag and sees if it is sooner than the previous time calculated. It also
     * loops through the Aruco poses to add each pose to the vector of AR tags.
     */

    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    std::vector<geometry_msgs::msg::Pose> m_node_vector;

    geometry_msgs::msg::Vector3 m_latest_position_average;
};