#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/node.hpp>


/**
 *The class ArucoAction contains several methods, functions, and member 
 *variables that assist in the aruco tag detection. They work together to detect
 *multiple AR tags in the field, calculate the average coordinate position, and 
 * post that position in a transformed reference frame to the blackboard for the 
 * rover to follow.
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
     */

    BT::NodeStatus tick() override;

    static BT::PortsList provided_ports();

  private:
    std::shared_ptr<rclcpp::Node> m_ros_node;

    /**
     * The callback function loops through the Aruco poses to add each pose to 
     * the vector of AR tags. It then averages the coordinates of the closest 
     * tags to get the position that the rover should circle around. It then
     * calls the transform helper function to transform the calculated coordinates
     * into the correct reference frame.
     */

    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * The transformhelper function completes the transform of the initial position
     * coordinates into the necessary reference frame of "map". It uses the msg
     * argument to do the transform. The function is then called in the
     * topic_callback function to be ran there and to then post the final position in
     * the correct reference frame to the blackboard.
     */

    void transformhelper(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    std::vector<geometry_msgs::msg::Pose> m_node_vector;

    geometry_msgs::msg::Vector3 m_latest_position_average;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
};
