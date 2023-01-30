#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/node.hpp>

class ArucoAction : public BT::SyncActionNode
{
  public:
    ArucoAction(const std::string& name, const std::string& action_name,
                const BT::NodeConfiguration& conf);

    BT::NodeStatus tick() override;

    static BT::PortsList provided_ports();

  private:
    std::shared_ptr<rclcpp::Node> m_ros_node;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_sub;

    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    builtin_interfaces::msg::Time m_latest;

    geometry_msgs::msg::Vector3 m_latest_position;
};
