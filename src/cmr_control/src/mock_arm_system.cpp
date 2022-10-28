#include "cmr_control/mock_arm_system.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// these are arbitrarily chosen values that are used to simulate the arm's
// motion when using forward kinematics.
const float velocity_factor = 10;
const float position_factor = 0.01;

namespace cmr_control
{

// NOLINTNEXTLINE
hardware_interface::CallbackReturn MockArmSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // Call parent's on_init to initialize fields
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    m_hw_position_commands.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());
    m_hw_effort_commands.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());
    m_hw_velocities.resize(info_.joints.size(),
                           std::numeric_limits<double>::quiet_NaN());
    m_hw_positions.resize(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());
    m_hw_control_modes.resize(info_.joints.size(), hardware_interface::HW_IF_EFFORT);

    // Enforce requirements on the system's state and command interfaces.
    // We expect each joint to have one velocity command interface and one
    // velocity state interface.
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 2) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %zu command interfaces found. 2 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but position interface was expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but effort interface was expected.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            CMR_LOG(FATAL, "Joint '%s' has %zu state interfaces found. 2 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %s state interface, but position interface was "
                    "expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(FATAL,
                    "Joint '%s' has %s state interface, but velocity interface was "
                    "expected.",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MockArmSystemHardware::export_state_interfaces()
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
MockArmSystemHardware::export_command_interfaces()
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
}

// NOLINTNEXTLINE(readability-function-size)
hardware_interface::return_type MockArmSystemHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Prepare for new command modes
    std::vector<std::string> new_modes;
    for (auto i = 0u; i < start_interfaces.size(); i++) {
        auto interface = start_interfaces[i];
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

    // Ensure all joints are given the same mode at the same time
    if (new_modes.size() != info_.joints.size()) {
        CMR_LOG(ERROR, "All joints must receive new command mode at the same time.");
        return hardware_interface::return_type::ERROR;
    }
    if (std::adjacent_find(new_modes.begin(), new_modes.end(),
                           std::not_equal_to<>()) != new_modes.end()) {
        CMR_LOG(ERROR, "All joints must be given the same command mode.");
        return hardware_interface::return_type::ERROR;
    }

    // Stop motion on all joints in the current mode
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] == hardware_interface::HW_IF_POSITION) {
            m_hw_position_commands[i] = m_hw_positions[i];
        } else if (m_hw_control_modes[i] == hardware_interface::HW_IF_EFFORT) {
            m_hw_effort_commands[i] = 0.0;
        }
        m_hw_control_modes[i] = "undefined";
    }

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

hardware_interface::CallbackReturn MockArmSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // set initial values
    std::fill(m_hw_position_commands.begin(), m_hw_position_commands.end(), 0.0);
    std::fill(m_hw_effort_commands.begin(), m_hw_effort_commands.end(), 0.0);
    std::fill(m_hw_velocities.begin(), m_hw_velocities.end(), 0.0);
    std::fill(m_hw_positions.begin(), m_hw_positions.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockArmSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

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
            auto effort = m_hw_effort_commands[i];
            if (effort > 0) {
                auto computed_vel = effort * velocity_factor;

                // update velocity
                m_hw_velocities[i] = computed_vel;

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
