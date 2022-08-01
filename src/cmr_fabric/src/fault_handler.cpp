#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"

class FaultHandler : public rclcpp::Node
{

public:
  FaultHandler()
  : rclcpp::Node("fault_handler", "fabric")
  {
    initialize();
  }

private:
  std::shared_ptr<rclcpp::Service<cmr_msgs::srv::RecoverFault>> recover_fault_service;
  std::vector<std::string> nodes_to_restart;

  void initialize()
  {
    auto recover_fault_callback =
      [this](const std::shared_ptr<cmr_msgs::srv::RecoverFault::Request> request,
        std::shared_ptr<cmr_msgs::srv::RecoverFault::Response>) {
        RCLCPP_INFO(get_logger(), "Recovering fault for node %s...\n", request->node_name.c_str());
        // TODO trigger restart
        nodes_to_restart.push_back(request->node_name);
      };
    recover_fault_service = this->create_service<cmr_msgs::srv::RecoverFault>(
      get_effective_namespace() + "/recover_fault",
      recover_fault_callback);
    RCLCPP_INFO(get_logger(), "fault handler initialized");
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FaultHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
