#include "cmr_astrotech/site_analyze.hpp"

namespace cmr
{
// TODO(unknown): CHANGE TO CORRECT ANGLE VALUES
// Angle and motor constants for analysis and collection
constexpr int pre_scoop_angle = 90;
constexpr int post_scoop_angle = 90;
constexpr int dump_angle = 90;
constexpr int neutral_angle = 90;
constexpr int turn_angle_pos = 100;
constexpr int turn_angle_neg = -100;
constexpr int collection_servo_motor_right = {0xE5};
constexpr int collection_servo_motor_left = {0xE6};
constexpr int analysis_motor = {0xDB};
constexpr int lead_screw_motor = {0xD2};
using namespace std::chrono_literals;
constexpr auto action_delay = 3s;

SiteAnalyze::SiteAnalyze(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
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
        case 3:
            fill({1, 2});
            analyze();
            for (int i = 0; i < 3; i++) {
                fill({2});
                analyze();
            }
            break;
        default:
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
    }
}

void SiteAnalyze::collection_handle_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = true;
    collection();
}

void SiteAnalyze::collection()
{
    for (int i = 0; i < 4; i++) {
        scoop(i);
        gearshift();
    }
}

void SiteAnalyze::publishmsg(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    int id, int mode, int angle)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {id};
    msg.control_modes = {mode};
    msg.values = {angle};
    m_motor_state_publisher->publish(msg);
    std::this_thread::sleep_for(action_delay);
}

void SiteAnalyze::analyze() { publishmsg(analysis_motor, 2, turn_angle_pos); }

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
    std::this_thread::sleep_for(action_delay);
}

void SiteAnalyze::gearshift()
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    publishmsg(collection_servo_motor_right, 2, turn_angle_pos);
}

bool SiteAnalyze::configure(const std::shared_ptr<toml::Table>&)
{
    using namespace std::placeholders;
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here

    // Ex. const auto node_settings = table->getTable("node");
    m_service = create_lifecycle_service<cmr_msgs::srv::SiteAnalyze>(
        "site_analyze", std::bind(&SiteAnalyze::handle_request, this, _1, _2));
    m_motor_state_publisher =
        create_lifecycle_publisher<cmr_msgs::msg::MotorWriteBatch>(
            "/motor", rclcpp::SystemDefaultsQoS());
    m_collection_service = create_lifecycle_service<std_srvs::srv::Trigger>(
        "site_analyze",
        std::bind(&SiteAnalyze::collection_handle_request, this, _1, _2));
    return true;
}

bool SiteAnalyze::activate()
{
    // do any last-minute things before activation here
    // it should be quick
    m_service->activate();
    m_motor_state_publisher->on_activate();
    m_collection_service->activate();

    return true;
}

bool SiteAnalyze::deactivate()
{
    // undo the effects of activate here
    m_service->deactivate();
    m_motor_state_publisher->on_deactivate();
    m_collection_service->deactivate();

    return true;
}

bool SiteAnalyze::cleanup()
{
    // undo the effects of configure here

    return true;
}

void SiteAnalyze::scoop(int site)
{
    // TODO(unknown): CHANGE TO CORRECT ANGLE VALUES AND ID OF RIGHT AND LEFT
    // COLLECTION SERVO MOTORS

    const static std::vector<int> g_sites = {1, 2, 3, 4};
    if (site == 1 || site == 2) {
        fill({site});
        publishmsg(collection_servo_motor_right, 1, pre_scoop_angle);
        publishmsg(lead_screw_motor, 2, turn_angle_neg);
        publishmsg(collection_servo_motor_right, 1, post_scoop_angle);
        publishmsg(lead_screw_motor, 2, turn_angle_pos);
        publishmsg(collection_servo_motor_right, 1, dump_angle);
        publishmsg(collection_servo_motor_right, 1, neutral_angle);
    }
    if (site == 3 || site == 4) {
        fill({site});
        publishmsg(collection_servo_motor_left, 1, pre_scoop_angle);
        publishmsg(lead_screw_motor, 2, turn_angle_neg);
        publishmsg(collection_servo_motor_left, 1, post_scoop_angle);
        publishmsg(lead_screw_motor, 2, turn_angle_pos);
        publishmsg(collection_servo_motor_left, 1, dump_angle);
        publishmsg(collection_servo_motor_left, 1, neutral_angle);
    }
}

}  // namespace cmr
