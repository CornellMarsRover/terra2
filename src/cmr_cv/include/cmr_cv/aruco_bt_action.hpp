#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/node.hpp>

class ArucoAction : public BT::SyncActionNode
{
  public:
    ArucoAction(const std::string& name, const std::string& action_name,
                const BT::NodeConfiguration& conf);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<rclcpp::Node> m_ros_node;

    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    builtin_interfaces::msg::Time m_latest;
};