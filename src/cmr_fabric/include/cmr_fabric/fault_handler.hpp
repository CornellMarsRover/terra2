#pragma once
#include "cmr_msgs/srv/activate_node.hpp"
#include "cmr_msgs/srv/recover_fault.hpp"
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

#ifdef BUILD_TESTS
    time_pt_t m_base_time, m_check_time;
    bool m_mocked_base = false, m_mocked_check = false;
    time_pt_t m_seen_check_time;
    std::mutex m_mock_mutex;
    std::mutex m_seen_check_mutex;
    std::condition_variable m_mock_cv;
#endif

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

    time_pt_t base_time_now();
    time_pt_t check_time_now();

#ifdef BUILD_TESTS
    void test_signal_seen_check_time(time_pt_t time);
#endif

  public:
    explicit FaultHandler(const std::string& node_name = "fault_handler",
                          const std::string& node_namespace = "fabric");

#ifdef BUILD_TESTS
    /**
     * @brief Sets the base and check time.
     *
     * The base time is the time at which restart delays will be computed from.
     * So a restart delay of `2` means that we can restart at time `time + 2s`.
     *
     * The check time is time at which restart delays will be checked against.
     * So a node will restart if its restart_time is `> time`
     *
     * Meant to be called in a separate thread from the fault handler
     *
     * @param time
     */
    void test_mock_base_check_time(time_pt_t time);

    /**
     * @brief Sets the time at which restart delays will be checked against and waits
     * until the fault handler sees this time. So a node will restart if its
     * restart_time is `> time`
     *
     * Meant to be called in a separate thread from the fault handler
     *
     * @param time
     */
    void test_mock_check_time_and_wait(time_pt_t time);
#endif
};
}  // namespace cmr::fabric