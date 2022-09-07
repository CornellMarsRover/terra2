#include <cstdio>

#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/services.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief Sends the activate node request (handled by the lifecycle manager) to all
 * specified nodes
 *
 * This function yields to ROS
 *
 * @param node The node who is sending the request
 * @param nodes_to_remove list of node names to send the request to
 */
void send_activate_requests(
    const rclcpp::Node& node,
    std::vector<std::reference_wrapper<const std::string>>&& nodes_to_remove)
{
    for (const auto node_name : nodes_to_remove) {
        RCLCPP_INFO(node.get_logger(), "Recovering fault for node %s...\n",
                    node_name.get().c_str());
        const auto request =
            std::make_shared<cmr_msgs::srv::ActivateNode::Request>();
        request->node_name = node_name;
        cmr::send_request<cmr_msgs::srv::ActivateNode>(
            node.get_effective_namespace() + "/activate", request);
    }
}

class FaultHandler : public rclcpp::Node
{
  private:
    std::unordered_map<std::string, int> m_nodes_to_restart;
    // we use a mutex to make sure we don't add any nodes while the timer is
    // executing its callback. otherwise we might end up losing some nodes in the
    // process
    std::mutex m_nodes_to_restart_mutex;

    // Destruction occurs in reverse order of construction. Therefore, m_timer should
    // be declared last so that all members used in its callback get destroyed after
    // it does. This prevents a situation where the callback is called while the node
    // is being destroyed, and some of the members have already been destroyed
    std::shared_ptr<rclcpp::Service<cmr_msgs::srv::RecoverFault>>
        m_recover_fault_service;
    std::shared_ptr<rclcpp::TimerBase> m_timer;

    /**
     * @brief This is the callback called by the timer
     * This keeps track of the restart delay for all nodes waiting to be restarted,
     * by being called at each 1s clock tick and decrementing all the nodes' restart
     * delay
     */
    void timer_callback()
    {
        std::unique_lock guard(m_nodes_to_restart_mutex);

        // Reference type is safe here since it will not outlive m_nodes_to_restart
        // keep nodes_to_remove local to this function
        std::vector<std::reference_wrapper<const std::string>> nodes_to_remove;

        for (auto& [node_name, delay] : m_nodes_to_restart) {
            if (delay-- <= 0) {
                nodes_to_remove.emplace_back(node_name);
            }
        }

        for (const auto& node_name : nodes_to_remove) {
            m_nodes_to_restart.erase(m_nodes_to_restart.find(node_name));
        }
        guard.unlock();
        send_activate_requests(*this, std::move(nodes_to_remove));
    }

  public:
    FaultHandler() : rclcpp::Node("fault_handler")
    {
        const auto recover_fault_callback =
            [this](
                const std::shared_ptr<cmr_msgs::srv::RecoverFault::Request>& request,
                const std::shared_ptr<cmr_msgs::srv::RecoverFault::Response>&) {
                std::lock_guard<std::mutex> guard(m_nodes_to_restart_mutex);
                m_nodes_to_restart.emplace(request->node_name,
                                           request->restart_delay);
            };
        m_recover_fault_service = this->create_service<cmr_msgs::srv::RecoverFault>(
            get_effective_namespace() + "/recover_fault", recover_fault_callback);

        const auto timer_cb = [this]() { return timer_callback(); };
        m_timer = this->create_wall_timer(1s, timer_cb);

        RCLCPP_INFO(get_logger(), "fault handler initialized");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<FaultHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
