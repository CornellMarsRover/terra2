#include "cmr_astrotech/site_analyze.hpp"

namespace cmr
{
constexpr int pre_scoop_angle = 90;
constexpr int post_scoop_angle = 90;
constexpr int dump_angle = 90;
constexpr int neutral_angle = 90;

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
                astrotech(i);
                gearshift(i);
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

void SiteAnalyze::gearshift(int /*site*/)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {0xDF};
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

void SiteAnalyze::publishmsg(int angle)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {0xDF};
    msg.control_modes = {1};
    msg.values = {angle};
    m_motor_state_publisher->publish(msg);
}

void SiteAnalyze::astrotech(int site)
{
    // const auto cm = 0xD4;
    // const auto csr = 0xDF;
    // const auto csl = 0xDF;
    const static std::vector<int> g_sites = {1, 2, 3, 4};
    // for (auto site : g_sites) {
    if (site == 1 || site == 2) {
        fill({site});
        publishmsg(pre_scoop_angle);
        // turn CM CCW until limit switch is triggered
        publishmsg(post_scoop_angle);
        // turn CM CW until limit switch is triggered
        publishmsg(dump_angle);
        publishmsg(neutral_angle);
    }
    if (site == 3 || site == 4) {
        fill({site});
        publishmsg(pre_scoop_angle);
        // turn CM CCW until limit switch is triggered
        publishmsg(post_scoop_angle);
        // turn CM CW until limit switch is triggered
        publishmsg(dump_angle);
        publishmsg(neutral_angle);
    }
}
// };

}  // namespace cmr
