#include "cmr_astrotech/site_analyze.hpp"

namespace cmr
{

SiteAnalyze::SiteAnalyze(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    m_node = rclcpp::Node::make_shared("site_analyze_server");
    m_service = m_node->create_service<cmr_msgs::srv::SiteAnalyze>(
        "site_analyze", &SiteAnalyze::handle_request);
    m_motor_state_publisher =
        m_node->create_publisher<cmr_msgs::msg::MotorWriteBatch>(
            "/motor", rclcpp::SystemDefaultsQoS());
}
// NOLINTNEXTLINE
void SiteAnalyze::handle_request(
    const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request> request,
    std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response)
{
    response->success = true;
    switch (request->site_num) {
        case 1:
            for (int i = 0; i < 4; i++) {
                fill({1});
                analyze();
            }
            break;
        case 2:
            fill({1, 2});
            analyze();
            for (int i = 0; i < 3; i++) {
                fill({2});
                analyze();
            }
            break;
        case 3:
            break;
        case 4:
            fill({2, 3});
            analyze();
            for (int i = 0; i < 3; i++) {
                fill({3});
                analyze();
            }
            fill({3, 4});
            analyze();

            for (int i = 0; i < 4; i++) {
                fill({4});
                analyze();
            }

            for (int i = 0; i < 3; i++) {
                analyze();
            }
            break;
        case 5:
            for (int i = 0; i < 4; i++) {
                scoop({i});
                gearshift();
            }
            break;
        default:
            response->success = false;
            break;
    }
}

void SiteAnalyze::analyze() { fill({5}); }

void SiteAnalyze::fill(std::vector<int> sites)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.size = static_cast<int32_t>(sites.size());

    for (int i : sites) {
        msg.motor_ids.push_back(i + 217);
        msg.control_modes.push_back(2);
        msg.values.push_back(20);
    }
    m_motor_state_publisher->publish(msg);
}

void SiteAnalyze::gearshift(int site)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {0xDF};
}

void SiteAnalyze::scoop(int site)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {0xD6};
    msg.control_modes={2};
    msg.values={100};
}

bool SiteAnalyze::configure(const std::shared_ptr<toml::Table>&)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    return true;
}

bool SiteAnalyze::activate()
{
    // do any last-minute things before activation here
    // it should be quick

    return true;
}

bool SiteAnalyze::deactivate()
{
    // undo the effects of activate here

    return true;
}

bool SiteAnalyze::cleanup()
{
    // undo the effects of configure here

    return true;
}

}  // namespace cmr
