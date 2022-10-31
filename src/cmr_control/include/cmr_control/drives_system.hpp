#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace cmr_control
{

/**
 * @brief Hardware interface for the drives system of the rover, which consists
 * of a number of wheel motors and their encoders.
 *
 * This hardware interface supports commanding the velocity of each wheel, and
 * reading the current velocity and position of each wheel reported from the
 * wheel encoders. This hardware interface is mainly used by the
 * diff_drive_controller to provide velocity control of the rover.
 */
class DrivesSystemHardware : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DrivesSystemHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

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

  private:
    // outgoing velocities of each joint
    std::vector<double> m_hw_commands;
    // incoming measured velocities of each joint
    std::vector<double> m_hw_velocities;
    // incoming measured positions of each joint
    std::vector<double> m_hw_positions;

    /**
     * @brief Returns true if the interfaces we're configured with make sense for
     * this hardware.
     *
     * Checks that we have the expected number and types of command and
     * state interfaces declared in the ROS2 Control portion of this hardware's URDF
     * file.
     */
    bool validate_interfaces() const;
};

}  // namespace cmr_control
