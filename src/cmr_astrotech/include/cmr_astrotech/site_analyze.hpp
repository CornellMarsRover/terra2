#pragma once

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/motor_write_batch.hpp"
#include "cmr_msgs/srv/site_analyze.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cmr
{

/**
 * `SiteAnalyze`
 *
 * The SiteAnalyze class controls the collection and analyze tasks for astrotech.
 * It completes several tasks including analyzing, filling sites, gearshifting,
 * and scooping. The class listens for messages of MotorWriteBatch and uses
 * these messages to change aspects of the different motors being used, such as
 * the motor id, mode, and angles.
 *

 */
class SiteAnalyze : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `SiteAnalyze`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit SiteAnalyze(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::MotorWriteBatch>::SharedPtr
        m_motor_state_publisher;
    fabric::LifecycleService<cmr_msgs::srv::SiteAnalyze>::ptr_t m_service;
    fabric::LifecycleService<std_srvs::srv::Trigger>::ptr_t m_collection_service;

    /**
     * Runs the astrotech analysis system; filling and analyzing the sites
     * depending on the sites being called. The function takes in a request and
     * a response as parameters.
     *
     */
    void handle_request(
        const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request> request,
        std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response);

    /**
     * Runs the astrotech collection system; doing the gearshift and scoop
     * functions at each site. The function takes in a request and
     * a response as parameters.
     */

    void collection_handle_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * Fills each site number after running for a specified number of seconds.
     * Takes in the site numbers as a parameter.
     */

    void fill(std::vector<int> sites);

    /**
     * Runs the right collection servo motor the speficied number of degrees
     * counterclockwise
     */
    void gearshift();

    /**
     * Runs the analysis motor clockwise for the specified number of
     * seconds, then takes data with the spectrometer, then runs the analysis
     * motor clockwise again.
     */

    void analyze();

    void fill(int site);

    /**
     * Calls scoop at each site and then runs the gearshift function.
     */
    void collection();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    /**
     * For each site, the analysis chamber is rotated, and then either the left
     * or right collection servo motors are turned clockwise/counter clockwise
     * and new angles are set based on when the scoop occurred.
     *
     * @param site
     */

    void scoop(int site);

    /**
     * Sets the motor id, mode, and angle for a given motor, and the motor write
     * message is published with this specific motor identification.
     *
     * @param id
     * @param mode
     * @param angle
     */

    void publishmsg(int id, int mode, int angle);

    int timed_effort(signed char effort, unsigned short time);

    const std::vector<bool> m_action_delay_bool = {false, false, false, false};
};

}  // namespace cmr