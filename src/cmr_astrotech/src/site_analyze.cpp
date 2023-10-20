#include "cmr_astrotech/site_analyze.hpp"

namespace cmr
{
// TODO(unknown): CHANGE TO CORRECT ANGLE VALUES
// Angle and motor constants for analysis and collection

// The angles will likley be different for each site
// You must test and adjust these angles
constexpr int pre_scoop_angles[4] = {45, 45, 45, 45};
constexpr int post_scoop_angles[4] = {45, 45, 45, 45};
constexpr int dump_angles[4] = {45, 45, 45, 45};
constexpr int neutral_angles[4] = {45, 45, 45, 45};

constexpr int collection_servo_motor_right = {0xE5};
constexpr int collection_servo_motor_left = {0xE6};
constexpr int analysis_motor = {0xDB};
constexpr int lead_screw_motor = {0xD2};

constexpr int pump_motor_1 = {0xD7};
constexpr int pump_motor_2 = {0xD8};
constexpr int pump_motor_3 = {0xD9};
constexpr int pump_motor_4 = {0xDA};

using namespace std::chrono_literals;

SiteAnalyze::SiteAnalyze(const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config)
{
    for (int i = 0; i < 4; i++) {
        analyze();
    }
}
/*
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
*/
void SiteAnalyze::handle_request(
    const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request>,
    std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response)
{
    response->success = true;
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
        // gearshift();
    }
}

void SiteAnalyze::publishmsg(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    int id, int mode, int angle)
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.motor_ids = {static_cast<uint8_t>(id)};
    msg.control_modes = {static_cast<uint8_t>(mode)};
    msg.values = {angle};
    m_motor_state_publisher->publish(msg);
    std::this_thread::sleep_for(3s);
}

// void SiteAnalyze::analyze() { publishmsg(analysis_motor, 2, turn_angle_pos); }

// takes an 8-bit signed integer (effort) and a 16-bit unsigned integer (time)
// concatenates these two in binary
// the output (timEff) can be sent to a brushed motor (with control_mode or func = 4)
// sending such a command will spin the motor at that effort for that amount of time
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
int SiteAnalyze::timed_effort(signed char effort, unsigned short time)
{
    int eff = effort << 16;
    int tim_eff = eff + time;
    return tim_eff << 8;
}

void SiteAnalyze::analyze()
{
    // spin analysis motor until the cuvette falls into the spectrometer
    publishmsg(analysis_motor, 3, 100);

    //(maybe) spin analysis motor slightly more
    // publishmsg(analysis_motor, 3, analysis_effort_1|time);

    // fill the next cuvette and wait
    // fill(request->site_num);

    // spin analysis motor, raising cuvette out of spectrometer
    publishmsg(analysis_motor, 3, -80);
}

/*
void SiteAnalyze::fill(std::vector<int> sites)
{
    std::chrono::milliseconds action_delay = 0ms;

    cmr_msgs::msg::MotorWriteBatch msg{};
    msg.size = static_cast<int32_t>(sites.size());

    for (int i : sites) {
        if (!m_action_delay_bool[static_cast<unsigned int>(i)]) {
            action_delay = 9005ms;
        } else {
            action_delay = 2005ms;
        }

        msg.motor_ids.push_back(i + 217);
        msg.control_modes.push_back(2);
        msg.values.push_back(20);
        m_action_delay_bool[static_cast<unsigned int>(i)];
    }
    m_motor_state_publisher->publish(msg);
    std::this_thread::sleep_for(action_delay);
}
*/
void SiteAnalyze::fill(int site)
{
    // PUMP TIME NEEDS TO BE TUNED
    // MAY NEED TO USE DIFFERENT VALUE FOR EACH SITE
    unsigned short delay = 2005;

    // start filling next cuvette
    if (site == 1) {
        publishmsg(pump_motor_1, 2, timed_effort(100, delay));
    } else if (site == 2) {
        publishmsg(pump_motor_2, 2, timed_effort(100, delay));
    } else if (site == 3) {
        publishmsg(pump_motor_3, 2, timed_effort(100, delay));
    } else if (site == 4) {
        publishmsg(pump_motor_4, 2, timed_effort(100, delay));
    }
    // sleep while cuvette fills and spectrometer analyzes samples
}
/*
void SiteAnalyze::gearshift()
{
    cmr_msgs::msg::MotorWriteBatch msg{};
    publishmsg(collection_servo_motor_right, 2, turn_angle_pos);
}
*/
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

// NOLINTNEXTLINE
void SiteAnalyze::scoop(int site)
{
    // TODO(unknown): CHANGE TO CORRECT ANGLE VALUES AND ID OF RIGHT AND LEFT
    // COLLECTION SERVO MOTORS

    const static std::vector<int> g_sites = {1, 2, 3, 4};
    int n = site - 1;

    if (site == 1 || site == 2) {
        // lead screw at intermediate pos (reed), scoop at neutral_angle
        // move scoop to pre_scoop_angle... lower lead screw to bottom...
        publishmsg(collection_servo_motor_right, 1, pre_scoop_angles[n]);
        publishmsg(lead_screw_motor, 4, timed_effort(-100, 3000));
        std::this_thread::sleep_for(3s);

        // lead screw at bottom, scoop at pre_scoop_angle
        // scoop dirt...
        publishmsg(collection_servo_motor_right, 1, post_scoop_angles[n]);

        // lead screw at bottom, scoop at post_scoop_angle
        // raise lead screw...
        publishmsg(lead_screw_motor, 3, 100);
        std::this_thread::sleep_for(3s);

        // lead screw at intermediate pos (at reed), scoop at post_scoop_angle
        // move scoop slightly... raise lead screw...
        publishmsg(collection_servo_motor_right, 1, neutral_angles[n]);
        std::this_thread::sleep_for(200ms);
        publishmsg(lead_screw_motor, 3, 100);

        // lead screw at dump pos (at limit), scoop at neutral_angle
        // dump scoop...
        publishmsg(collection_servo_motor_right, 1, dump_angles[n]);
        std::this_thread::sleep_for(200ms);

        // scoop at dump_angle
        // move scoop slightly back... move lead screw down
        publishmsg(collection_servo_motor_right, 1, neutral_angles[n]);
        publishmsg(lead_screw_motor, 3, -100);
        std::this_thread::sleep_for(500ms);
    }

    if (site == 3 || site == 4) {
        // lead screw at intermediate pos (reed), scoop at neutral_angle
        // move scoop to pre_scoop_angle... lower lead screw to bottom...
        publishmsg(collection_servo_motor_left, 1, pre_scoop_angles[n + 1]);
        publishmsg(lead_screw_motor, 4, timed_effort(-100, 3000));
        std::this_thread::sleep_for(3s);

        // lead screw at bottom, scoop at pre_scoop_angle
        // scoop dirt...
        publishmsg(collection_servo_motor_left, 1, post_scoop_angles[n]);

        // lead screw at bottom, scoop at post_scoop_angle
        // raise lead screw...
        publishmsg(lead_screw_motor, 3, 100);
        std::this_thread::sleep_for(3s);

        // lead screw at intermediate pos (at reed), scoop at post_scoop_angle
        // move scoop slightly... raise lead screw...
        publishmsg(collection_servo_motor_left, 1, neutral_angles[n]);
        std::this_thread::sleep_for(200ms);
        publishmsg(lead_screw_motor, 3, 100);

        // lead screw at dump pos (at limit), scoop at neutral_angle
        // dump scoop...
        publishmsg(collection_servo_motor_left, 1, dump_angles[n]);
        std::this_thread::sleep_for(200ms);

        // scoop at dump_angle
        // move scoop slightly back... move lead screw down
        publishmsg(collection_servo_motor_left, 1, neutral_angles[n]);
        publishmsg(lead_screw_motor, 3, -100);
        std::this_thread::sleep_for(500ms);
    }

}  // namespace cmr
}  // namespace cmr