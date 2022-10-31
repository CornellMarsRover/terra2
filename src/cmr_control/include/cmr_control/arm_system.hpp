#include "cmr_control/base_arm_system.hpp"
#include "rclcpp/macros.hpp"

namespace cmr_control
{

/**
 * @brief ArmSystemHardware is a hardware interface for the arm that provides
 * effort and position control to each of the arm's joints. It also reads the
 * current position and velocity of each arm joint from the encoders.
 */
class ArmSystemHardware : public cmr_control::BaseArmSystemHardware
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;
};

}  // namespace cmr_control
