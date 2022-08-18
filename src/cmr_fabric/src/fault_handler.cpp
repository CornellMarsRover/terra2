#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "cmr_utils/services.hpp"
#include "cmr_msgs/srv/activate_node.hpp"
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
  std::shared_ptr<rclcpp::TimerBase> timer;

  std::unordered_set<std::string> nodes_to_restart;
  // we use a mutex to make sure we don't add any nodes while the timer is executing its callback.
  // otherwise we might end up losing some nodes in the process
  // TODO not sure that this mutex is needed, because we are using a single-threaded executor and
  // so the timer and service callbacks might just be executed in series. but maybe it's worth
  // leaving here to be agnostic to the kind of executor we're dealing with?
  std::mutex nodes_to_restart_mutex;

  void initialize()
  {
    auto recover_fault_callback =
      [this](const std::shared_ptr<cmr_msgs::srv::RecoverFault::Request> request,
        std::shared_ptr<cmr_msgs::srv::RecoverFault::Response>) {
        std::lock_guard<std::mutex> guard(nodes_to_restart_mutex);
        nodes_to_restart.emplace(request->node_name);
      };
    recover_fault_service = this->create_service<cmr_msgs::srv::RecoverFault>(
      get_effective_namespace() + "/recover_fault",
      recover_fault_callback);

    auto timer_callback = [this]() {
        std::lock_guard<std::mutex> guard(nodes_to_restart_mutex);

        for (auto node : nodes_to_restart) {
          RCLCPP_INFO(get_logger(), "Recovering fault for node %s...\n", node.c_str());
          auto request = std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
          request->node_name = node;
          cmr::sendRequest<cmr_msgs::srv::ActivateNode>("/fabric/activate_node", request);
        }

        nodes_to_restart.clear();
      };
    timer = this->create_wall_timer(1s, timer_callback);

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
