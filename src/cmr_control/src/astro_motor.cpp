#include "cmr_control/astro_motor.hpp"

#include <chrono>
#include <cmath>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <limits>
#include <memory>
#include <vector>

#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr_control
{

AstroMotor::AstroMotor()
    : m_position(0.0), m_position_cmd(0.0), m_velocity(0.0), m_velocity_cmd(0.0)
{
}

hardware_interface::CallbackReturn AstroMotor::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::ActuatorInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info.joints.size() != 1) {
        CMR_LOG(ERROR, "AstroMotor can only have one joint");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> AstroMotor::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // This actuator controls only 1 joint so we hardcode joint index 0
    state_interfaces.emplace_back(info_.joints[0].name,
                                  hardware_interface::HW_IF_POSITION, &m_position);
    state_interfaces.emplace_back(info_.joints[0].name,
                                  hardware_interface::HW_IF_VELOCITY, &m_velocity);
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AstroMotor::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // This actuator controls only 1 joint so we hardcode joint index 0
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &m_position_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &m_velocity_cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn AstroMotor::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AstroMotor::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AstroMotor::read(const rclcpp::Time& /*time*/,
                                                 const rclcpp::Duration& /*period*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AstroMotor::write(const rclcpp::Time& /*time*/,
                                                  const rclcpp::Duration& period)
{
    // TODO(apl88): write the velocity and position state variables based on command
    // mode in m_command_mode
    if (m_command_mode == hardware_interface::HW_IF_VELOCITY) {
        m_position += m_velocity_cmd * period.seconds();
        m_velocity = m_velocity_cmd;
    } else if (m_command_mode == hardware_interface::HW_IF_POSITION) {
        m_position = m_position_cmd;
        m_velocity = (m_position_cmd - m_position) / period.seconds();
    }

    return hardware_interface::return_type::OK;
}

inline bool is_relevant_interface(const std::vector<std::string>& interfaces,
                                  const std::string& joint_name)
{
    return std::find_if(interfaces.begin(), interfaces.end(), [&](const auto& val) {
               const auto slash_pos = val.find("/");
               return slash_pos != std::string::npos &&
                      val.substr(0, slash_pos) == joint_name;
           }) != interfaces.end();
}

hardware_interface::return_type AstroMotor::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /* stop_interfaces */)
{
    // TODO(apl88): actual implementation

    if (!is_relevant_interface(start_interfaces, info_.joints[0].name)) {
        // ignore interface switches for other hardware interfaces
        return hardware_interface::return_type::OK;
    }

    if (start_interfaces.size() != 1) {
        CMR_LOG(ERROR, "AstroMotorHardware should only control one joint");
        return hardware_interface::return_type::ERROR;
    }

    if (start_interfaces[0].find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
        m_command_mode = hardware_interface::HW_IF_POSITION;
    } else if (start_interfaces[0].find(hardware_interface::HW_IF_VELOCITY) !=
               std::string::npos) {
        m_command_mode = hardware_interface::HW_IF_VELOCITY;
    } else {
        CMR_LOG(ERROR,
                "AstroMotorHardware can only accept position or velocity "
                "commands");
        return hardware_interface::return_type::ERROR;
    }

    // Halt all motion
    m_position_cmd = m_position;
    m_velocity = 0;
    m_velocity_cmd = 0;

    return hardware_interface::return_type::OK;
}

}  // namespace cmr_control

#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::AstroMotor,
                       hardware_interface::ActuatorInterface)