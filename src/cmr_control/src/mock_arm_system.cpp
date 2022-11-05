#include "cmr_control/mock_arm_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// these are arbitrarily chosen values that are used to simulate the arm's
// motion when using forward kinematics.
const double velocity_factor = 10;
const double position_factor = 0.01;

namespace cmr_control
{

hardware_interface::return_type MockArmSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockArmSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] == hardware_interface::HW_IF_POSITION) {
            // update velocity
            m_hw_velocities[i] =
                (m_hw_position_commands[i] - m_hw_positions[i]) / period.seconds();

            // update position
            m_hw_positions[i] = m_hw_position_commands[i];
        } else if (m_hw_control_modes[i] == hardware_interface::HW_IF_EFFORT) {
            const auto& effort = m_hw_effort_commands[i];
            const auto computed_vel = effort * velocity_factor;

            // update velocity
            m_hw_velocities[i] = computed_vel;

            if (abs(computed_vel) > std::numeric_limits<double>::epsilon()) {
                // update position
                m_hw_positions[i] += computed_vel * position_factor;
            }
        }
    }
    return hardware_interface::return_type::OK;
}

}  // namespace cmr_control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::MockArmSystemHardware,
                       hardware_interface::SystemInterface)
