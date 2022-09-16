#pragma once
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{

class FaultHandler : public rclcpp::Node
{
    using time_pt_t = std::chrono::time_point<std::chrono::system_clock>;

  private:
    std::unordered_map<std::string, time_pt_t> m_nodes_to_restart;

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
     * This checks all nodes waiting to be restarted and if the current time is past
     * the restart time, it sends the restart request
     */
    void timer_callback();

  public:
    explicit FaultHandler(const std::string& node_name = "fault_handler",
                          const std::string& node_namespace = "fabric");
};
}  // namespace cmr::fabric