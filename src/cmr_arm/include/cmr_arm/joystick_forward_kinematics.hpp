#pragma once
#include "cmr_fabric/fabric_node.hpp"

namespace cmr {

/**
 * `JoystickForwardKinematics`
 *
 * TODO
 */
class JoystickForwardKinematics : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `JoystickForwardKinematics`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty optional
     * to start the node from a launch file or via ROS
     */
    explicit JoystickForwardKinematics(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

} // namespace cmr