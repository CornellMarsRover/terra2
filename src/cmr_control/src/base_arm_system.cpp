#include "cmr_control/base_arm_system.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cmr_control
{

hardware_interface::CallbackReturn BaseArmSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // Call parent's on_init to initialize members like info_.
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize members to a default value for each joint.
    // We use NaN to indicate that data has not yet been set.
    m_hw_position_commands.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());
    m_hw_effort_commands.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
    m_hw_velocities.resize(info_.joints.size(),
                           std::numeric_limits<double>::quiet_NaN());
    m_hw_positions.resize(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());

    // Effort control is set as a sensible default, and will be changed if needed by
    // the controller.
    m_hw_control_modes.resize(info_.joints.size(), hardware_interface::HW_IF_EFFORT);

    return validate_interfaces() ? hardware_interface::CallbackReturn::SUCCESS
                                 : hardware_interface::CallbackReturn::ERROR;
};

std::vector<hardware_interface::StateInterface>
BaseArmSystemHardware::export_state_interfaces()
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
};

std::vector<hardware_interface::CommandInterface>
BaseArmSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &m_hw_position_commands[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &m_hw_effort_commands[i]));
    }

    return command_interfaces;
};

hardware_interface::CallbackReturn BaseArmSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // set initial values
    std::fill(m_hw_position_commands.begin(), m_hw_position_commands.end(), 0.0);
    std::fill(m_hw_effort_commands.begin(), m_hw_effort_commands.end(), 0.0);
    std::fill(m_hw_velocities.begin(), m_hw_velocities.end(), 0.0);
    std::fill(m_hw_positions.begin(), m_hw_positions.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn BaseArmSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::return_type BaseArmSystemHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Prepare for new command modes
    std::vector<std::string> new_modes;
    for (auto i = 0u; i < start_interfaces.size(); i++) {
        const auto& interface = start_interfaces[i];
        auto joint = info_.joints[i];
        if (interface == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
            new_modes.emplace_back(hardware_interface::HW_IF_POSITION);
        } else if (interface ==
                   joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
            new_modes.emplace_back(hardware_interface::HW_IF_EFFORT);
        } else {
            CMR_LOG(ERROR, "Unsupported interface '%s' requested.",
                    interface.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

    if (!validate_switch(new_modes)) {
        CMR_LOG(
            ERROR,
            "All joints must receive the same new command mode at the same time.");
        return hardware_interface::return_type::ERROR;
    }

    halt_all_motion();

    // Set new command modes
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] != "undefined") {
            CMR_LOG(ERROR, "Joint '%s' is in use by another interface.",
                    info_.joints[i].name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        m_hw_control_modes[i] = new_modes[i];
    }

    return hardware_interface::return_type::OK;
}

// NOLINTNEXTLINE(readability-function-size)
bool BaseArmSystemHardware::validate_interfaces()
{
    // Enforce requirements on the system's state and command interfaces.
    // We expect each joint to have one velocity command interface and one
    // velocity state interface.
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 2) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %zu command interfaces found. 2 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
            return false;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but position interface was expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return false;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but effort interface was expected.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str());
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
};

bool BaseArmSystemHardware::validate_switch(
    const std::vector<std::string>& new_modes)
{
    return new_modes.size() == info_.joints.size() &&
           std::adjacent_find(new_modes.begin(), new_modes.end(),
                              std::not_equal_to<>()) == new_modes.end();
}

void BaseArmSystemHardware::halt_all_motion()
{
    // Stop motion on all joints in the current mode
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] == hardware_interface::HW_IF_POSITION) {
            m_hw_position_commands[i] = m_hw_positions[i];
        } else if (m_hw_control_modes[i] == hardware_interface::HW_IF_EFFORT) {
            m_hw_effort_commands[i] = 0.0;
        }
        m_hw_control_modes[i] = "undefined";
    }
}

}  // namespace cmr_control