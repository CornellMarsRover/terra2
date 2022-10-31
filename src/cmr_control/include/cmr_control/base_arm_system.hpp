#pragma once

#include "hardware_interface/system_interface.hpp"
namespace cmr_control
{

/**
 * @brief A base class for arm system hardware.
 *
 * Arm hardware is expected to have both position and effort command interfaces and
 * both position and velocity state interfaces. This class provides default
 * implementations of required methods for a SystemInterface, namely in initializing
 * and validating the command and state interfaces as well as implementing
 * `prepare_command_mode_switch` to set the `m_hw_control_modes` vector
 * appropriately.
 *
 * Subclasses should implement the `read` and `write` methods to read and write
 * data to and from hardware, including mocking up the data when appropriate.
 * Subclasses may access protected members of this class to get the current values
 * on each command and state interface, as well as the control mode for each joint.
 */
class BaseArmSystemHardware : public hardware_interface::SystemInterface
{
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

  protected:
    // current control modes of each joint
    std::vector<std::string> m_hw_control_modes;
    // outgoing positions of each joint
    std::vector<double> m_hw_position_commands;
    // outgoing efforts of each joint
    std::vector<double> m_hw_effort_commands;
    // incoming measured velocities of each joint
    std::vector<double> m_hw_velocities;
    // incoming measured velocities of each joint
    std::vector<double> m_hw_positions;

  private:
    /**
     * @brief Returns true if the interfaces we're configured with make sense for
     * this hardware.
     *
     * Checks that we have the expected number and types of command and
     * state interfaces declared in the ROS2 Control portion of this hardware's URDF
     * file.
     */
    bool validate_interfaces();

    /**
     * @brief Returns true if all control modes are changed at the same time and
     * are all given the same control mode.
     *
     * @param new_modes The new control modes to set.
     */
    bool validate_switch(const std::vector<std::string>& new_modes);

    /**
     * @brief Halts all motion in the current control mode and sets the current
     * control mode of all joints to "undefined".
     *
     * This is called when the hardware interface is told to switch control modes
     * between position and effort control. If any of the joints are not in the
     * "undefined" mode, this indicates that another hardware resource is using
     * this particular joint and therefore it is not safe to switch control mode
     * at this time.
     */
    void halt_all_motion();
};

}  // namespace cmr_control
