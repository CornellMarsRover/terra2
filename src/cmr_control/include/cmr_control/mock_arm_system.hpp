#include "cmr_control/base_arm_system.hpp"
#include "rclcpp/macros.hpp"

namespace cmr_control
{

/**
 * @brief MockArmSystemHardware is a hardware interface for the arm that mocks
 * effort and position control to each of the arm's joints.
 *
 * No actual hardware commands are sent, but the arm's position and velocity are
 * updated based on the commanded effort and position times a constant factor
 * that's defined in the implementation.
 */
class MockArmSystemHardware : public cmr_control::BaseArmSystemHardware
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MockArmSystemHardware)

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;
};

}  // namespace cmr_control
