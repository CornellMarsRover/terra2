#include "cmr_control/base_arm_system.hpp"
#include "rclcpp/macros.hpp"

namespace cmr_control
{

/**
 * @brief ArmSystemHardware is a hardware interface for the arm, meant to be
 * used when using forward kinematics.
 *
 */
class ArmSystemHardware : public cmr_control::BaseArmSystemHardware
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware);

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;
};

}  // namespace cmr_control
