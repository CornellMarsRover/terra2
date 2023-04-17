#pragma once
#include <std_msgs/msg/detail/bool__struct.hpp>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/motor_write_batch.hpp"
#include "cmr_msgs/srv/site_analyze.hpp"

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
    fabric::LifecycleService<std_msgs::msg::Bool>::ptr_t m_collection_service;

    void handle_request(
        const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request> request,
        std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response);

    void collection_handle_request(
        const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request> request,
        std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response);

    void fill(std::vector<int> sites);

    void gearshift(int site);

    void analyze();
    void collection();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    void scoop(int site);
    void publishmsg(int id, int mode, int angle);
};

}  // namespace cmr