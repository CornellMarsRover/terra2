#include "cmr_control/drives_system.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cmr_control
{

hardware_interface::CallbackReturn DrivesSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // Call parent's on_init to initialize fields
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize members to a default value for each joint.
    // We use NaN to indicate that data has not yet been set.
    m_hw_commands.resize(info_.joints.size(),
                         std::numeric_limits<double>::quiet_NaN());
    m_hw_velocities.resize(info_.joints.size(),
                           std::numeric_limits<double>::quiet_NaN());
    m_hw_positions.resize(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());

    return validate_interfaces() ? hardware_interface::CallbackReturn::SUCCESS
                                 : hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
DrivesSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &m_hw_positions[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &m_hw_velocities[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DrivesSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &m_hw_commands[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn DrivesSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // set initial values
    std::fill(m_hw_commands.begin(), m_hw_commands.end(), 0.0);
    std::fill(m_hw_velocities.begin(), m_hw_velocities.end(), 0.0);
    std::fill(m_hw_positions.begin(), m_hw_positions.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrivesSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DrivesSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // TODO(fad35): hook this up once firmware message schema is decided
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DrivesSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // TODO(fad35): hook this up once firmware message schema is decided
    return hardware_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-function-size)
bool DrivesSystemHardware::validate_interfaces() const
{
    // Enforce requirements on the system's state and command interfaces.
    // We expect each joint to have one velocity command interface and one
    // velocity state interface.
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
            return false;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but velocity interface was expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return false;
        }

        if (joint.state_interfaces.size() != 2) {
            CMR_LOG(FATAL, "Joint '%s' has %zu state interfaces found. 2 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
            return false;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %s state interface, but position interface was "
                    "expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str());
            return false;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %s state interface, but velocity interface was "
                    "expected.",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str());
            return false;
        }
    }
    return true;
}

}  // namespace cmr_control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::DrivesSystemHardware,
                       hardware_interface::SystemInterface)
