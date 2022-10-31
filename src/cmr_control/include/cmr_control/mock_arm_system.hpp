#include "cmr_control/base_arm_system.hpp"
#include "rclcpp/macros.hpp"

namespace cmr_control
{

/**
 * @brief MockArmSystemHardware is a mock hardware interface for a robot arm.
 * This hardware interface converts effort commands to velocity and position.
 */
class MockArmSystemHardware : public cmr_control::BaseArmSystemHardware
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MockArmSystemHardware);

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;
};

}  // namespace cmr_control
