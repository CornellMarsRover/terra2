#pragma once
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/clock.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/services.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr::fabric
{

/**
 * @brief The FaultHandler class provides a service interface to allow nodes to
 * restart themselves if an error occurs.
 *
 * The service is created on `/<namespace>/recover_fault` and takes messages of type
 * `cmr_msgs::srv::RecoverFault`.
 *
 * The fault handler will activate nodes through the lifecycle manager.
 */
class FaultHandler : public rclcpp::Node
{
    using time_pt_t = std::chrono::time_point<std::chrono::system_clock>;

  private:
    /** Nodes to restart, and the time at which we can restart them */
    std::unordered_map<std::string, time_pt_t> m_nodes_to_restart;

    std::mutex m_nodes_to_restart_mutex;

    std::shared_ptr<cmr::Clock<std::chrono::system_clock>> m_base_clock;
    std::shared_ptr<cmr::Clock<std::chrono::system_clock>> m_check_clock;

    // Destruction occurs in reverse order of construction. Therefore, m_timer
    // should be declared last so that all members used in its callback get
    // destroyed after it does. This prevents a situation where the callback is
    // called while the node is being destroyed, and some of the members have
    // already been destroyed
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
                          const std::string& node_namespace = "fabric")
        : FaultHandler(std::make_shared<cmr::RealClock<std::chrono::system_clock>>(),
                       std::make_shared<cmr::RealClock<std::chrono::system_clock>>(),
                       node_name, node_namespace)
    {
    }

    /**
     * @brief Construct a new Fault Handler object, specifying the clocks to use
     *
     * @param base_clock The base clock is used to compute the restart times. So
     * when the fault handler should restart a node in 2s. It will schedule the
     * restart at 1m_base_clock->now() + 2s`.
     * @param check_clock The check clock is used to compare the current time with
     * the restart time. So a node will restart if its restart_time is `>
     * check_clock->now()`.
     * @param node_name
     * @param node_namespace
     */
    FaultHandler(std::shared_ptr<cmr::Clock<std::chrono::system_clock>> base_clock,
                 std::shared_ptr<cmr::Clock<std::chrono::system_clock>> check_clock,
                 const std::string& node_name = "fault_handler",
                 const std::string& node_namespace = "fabric");
};
}  // namespace cmr::fabric