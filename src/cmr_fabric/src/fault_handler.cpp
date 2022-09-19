#include "cmr_fabric/fault_handler.hpp"

#include "cmr_fabric/lifecycle_helpers.hpp"

namespace cmr::fabric
{
using namespace std::chrono_literals;

/**
 * @brief Sends the activate node request (handled by the lifecycle manager) to all
 * specified nodes
 *
 * This function yields to ROS
 *
 * @param node The node who is sending the request
 * @param nodes_to_remove list of node names to send the request to
 * @return a vector of node names which could not be activated
 */
static auto send_activate_requests(const rclcpp::Node& node,
                                   std::vector<std::string>&& nodes_to_remove)
{
    std::vector<std::string> failed_nodes;
    for (const auto& node_name : nodes_to_remove) {
        RCLCPP_INFO(node.get_logger(), "Recovering fault for node %s...\n",
                    node_name.c_str());
        if (!activate_node(node_name)) {
            failed_nodes.emplace_back(node_name);
            RCLCPP_WARN(node.get_logger(),
                        "Failed to activate node %s. Will try again\n",
                        node_name.c_str());
        }
    }
    return failed_nodes;
}

void FaultHandler::timer_callback()
{
    std::vector<std::string> nodes_to_remove;
    std::unique_lock lock(m_nodes_to_restart_mutex);
    const auto time = m_check_clock->now();
    for (const auto& [node_name, restart_time] : m_nodes_to_restart) {
        if (time > restart_time) {
            nodes_to_remove.emplace_back(node_name);
            CMR_LOG(DEBUG, "Restarting node %s at %zd", node_name.c_str(),
                    restart_time.time_since_epoch().count());
        }
    }

    for (const auto& node_name : nodes_to_remove) {
        m_nodes_to_restart.erase(m_nodes_to_restart.find(node_name));
    }
    lock.unlock();
    const auto failed_nodes =
        send_activate_requests(*this, std::move(nodes_to_remove));
    if (!failed_nodes.empty()) {
        lock.lock();
        for (const auto& node_name : failed_nodes) {
            m_nodes_to_restart[node_name] = m_base_clock->now() + 1s;
        }
    }
    m_check_clock->register_time(time);
}

FaultHandler::FaultHandler(const std::string& node_name,
                           const std::string& node_namespace)
    : rclcpp::Node(node_name, node_namespace)
{
    const auto recover_fault_callback =
        [this](const std::shared_ptr<cmr_msgs::srv::RecoverFault::Request>& request,
               const std::shared_ptr<cmr_msgs::srv::RecoverFault::Response>&) {
            CMR_LOG(INFO, "Got schedule request");
            std::lock_guard lk(m_nodes_to_restart_mutex);
            m_nodes_to_restart.emplace(
                request->node_name,
                m_base_clock->now() + std::chrono::seconds(request->restart_delay));
            CMR_LOG(
                INFO, "Scheduled restart for %s at %zd", request->node_name.c_str(),
                m_nodes_to_restart[request->node_name].time_since_epoch().count());
        };
    m_recover_fault_service = this->create_service<cmr_msgs::srv::RecoverFault>(
        get_effective_namespace() + "/recover_fault", recover_fault_callback);

    const auto timer_cb = [this]() { return timer_callback(); };
    m_timer = this->create_wall_timer(1s, timer_cb);

    RCLCPP_INFO(get_logger(), "fault handler initialized");
}
}  // namespace cmr::fabric
