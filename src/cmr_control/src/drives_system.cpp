#include "cmr_control/drives_system.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cmr::control
{

// NOLINTNEXTLINE
hardware_interface::CallbackReturn DrivesSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // Call parent's on_init to initialize fields
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Enforce requirements on the system's state and command interfaces.
    // We expect each joint to have one velocity command interface and one
    // velocity state interface.
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but velocity interface was expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1) {
            CMR_LOG(FATAL, "Joint '%s' has %zu state interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %s state interface, but velocity interface was "
                    "expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DrivesSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &m_hw_states[i]));
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
    std::fill(m_hw_states.begin(), m_hw_states.end(), 0.0);
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

}  // namespace cmr::control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr::control::DrivesSystemHardware,
                       hardware_interface::SystemInterface)
