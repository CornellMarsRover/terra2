#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/motor_write_batch.hpp"
#include "cmr_msgs/srv/site_analyze.hpp"

namespace cmr
{

/**
 * `SiteAnalyze`
 *
 * TODO
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
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<cmr_msgs::msg::MotorWriteBatch>::SharedPtr
        m_motor_state_publisher;
    rclcpp::Service<cmr_msgs::srv::SiteAnalyze>::SharedPtr m_service;

    void handle_request(
        const std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Request> request,
        std::shared_ptr<cmr_msgs::srv::SiteAnalyze::Response> response);

    void fill(std::vector<int> sites);

    void gearshift(int site);

    void analyze();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    void scoop(int site);
    void publishmsg(
        cmr_msgs::msg::MotorWriteBatch_<std::allocator<void>>::_motor_ids_type id,
        cmr_msgs::msg::MotorWriteBatch_<std::allocator<void>>::_control_modes_type
            mode,
        int angle);
};

}  // namespace cmr