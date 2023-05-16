#pragma once

#include "cmr_msgs/msg/motor_write_batch.hpp"
#include "cmr_msgs/msg/sensor_read_batch.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr_control
{

/**
 * @brief ArmSystemHardware is a hardware interface for the arm that provides
 * velocity and position control to each of the arm's joints. It also reads the
 * current position of each arm joint from the encoders.
 */
class ArmSystemHardware : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

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

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

  private:
    // current control modes of each joint
    std::vector<std::string> m_hw_control_modes;
    // outgoing positions of each joint
    std::vector<double> m_hw_position_commands;
    // outgoing velocity of each joint
    std::vector<double> m_hw_velocity_commands;
    // incoming measured velocities of each joint
    std::vector<double> m_hw_velocities;
    // incoming measured velocities of each joint
    std::vector<double> m_hw_positions;
    // buffered position readings from encoders.
    std::shared_ptr<rclcpp::Node> m_comm_node;
    std::vector<int> m_hw_buffer;
    std::shared_ptr<rclcpp::Publisher<cmr_msgs::msg::MotorWriteBatch>>
        m_motor_write_pub;
    std::shared_ptr<rclcpp::Subscription<cmr_msgs::msg::SensorReadBatch>>
        m_motor_read_sub;

    void sensor_callback(const cmr_msgs::msg::SensorReadBatch& msg);

    /**
     * @brief Returns true if the interfaces we're configured with make sense for
     * this hardware.
     *
     * Checks that we have the expected number and types of command and
     * state interfaces declared in the ROS2 Control portion of this hardware's URDF
     * file.
     */
    bool validate_interfaces() const;

    /**
     * @brief Returns true if all control modes are changed at the same time and
     * are all given the same control mode.
     *
     * @param new_modes The new control modes to set.
     */
    bool validate_switch(const std::vector<std::string>& new_modes) const;

    /**
     * @brief Halts all motion in the current control mode and sets the current
     * control mode of all joints to "undefined".
     *
     * This is called when the hardware interface is told to switch control modes
     * between position and velocity control. If any of the joints are not in the
     * "undefined" mode, this indicates that another hardware resource is using
     * this particular joint and therefore it is not safe to switch control mode
     * at this time.
     */
    void halt_all_motion();

    /**
     * @brief Set all joints to velocity mode.
     *
     * Warning: this function does not provide any confirmation for whether the
     * mode was set; it is possible that the joint misses this message.
     */
    void set_velocity_mode() const;

    /**
     * @brief Set all joints to position mode.
     *
     * Warning: this function does not provide any confirmation for whether the
     * mode was set; it is possible that the joint misses this message.
     */
    void set_position_mode() const;
};

}  // namespace cmr_control
