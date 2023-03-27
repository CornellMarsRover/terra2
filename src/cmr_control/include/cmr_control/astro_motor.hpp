#pragma once
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace cmr_control
{

/**
 * @brief `AstroMotor` is a hardware interface for controlling a single motor
 * with position or velocity commands.
 *
 * It expects that it will only be passed a single joint within the
 * `HardwareInfo` in the call to `on_init()`. This means, from a user perspective
 * that the AstroMotor should only have a single motor's state interface, command
 * interface, and joint under its control as defined in the URDFs and YAML controller
 * config files.
 *
 * A good controller for this is either
 * `position_controllers/JointGroupPositionController` or
 * `velocity_controllers/JointGroupVelocityController`.
 */

class AstroMotor : public hardware_interface::ActuatorInterface
{
  public:
    AstroMotor();

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

  private:
    /** The current command mode */
    std::string m_command_mode;
    /** Position state interface value */
    double m_position;
    /** Position command interface value */
    double m_position_cmd;
    /** Velocity state interface value */
    double m_velocity;
    /** Velocity command interface value */
    double m_velocity_cmd;
};

}  // namespace cmr_control