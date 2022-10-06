#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include <wall_timer.h>

namespace cmr
{

/**
 * `Joystick`
 *
 * TODO
 */
class Joystick : public cmr::fabric::FabricNode
{
  //Initiates basic ROS2 Wall Timer for callback function with joystick output
  ros::WallTimer = walltimer_;

  public:
    /**
     * Constructs a `Joystick`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit Joystick(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

}  // namespace cmr