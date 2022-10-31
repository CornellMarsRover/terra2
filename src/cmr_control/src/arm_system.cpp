#include "cmr_control/arm_system.hpp"

namespace cmr_control
{

hardware_interface::return_type ArmSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // TODO(fad35): hook this up once firmware message schema is decided
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // TODO(fad35): hook this up once firmware message schema is decided
    return hardware_interface::return_type::OK;
}

}  // namespace cmr_control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::ArmSystemHardware,
                       hardware_interface::SystemInterface)
