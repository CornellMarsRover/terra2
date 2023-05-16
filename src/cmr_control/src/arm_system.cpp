#include "cmr_control/arm_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "cmr_control/external/boards.h"
#include "std_msgs/msg/string.hpp"

const std::array<uint8_t, 6> motor_ids = {BLDC_AR1, BLDC_AR2, BLDC_AR3,
                                          BLDC_AR6, BLDC_AR5, BLDC_AR6};

namespace cmr_control
{

hardware_interface::CallbackReturn ArmSystemHardware::on_init(
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
    m_hw_velocity_commands.resize(info_.joints.size(),
                                  std::numeric_limits<double>::quiet_NaN());
    m_hw_velocities.resize(info_.joints.size(),
                           std::numeric_limits<double>::quiet_NaN());
    m_hw_positions.resize(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());
    m_hw_buffer.resize(info.joints.size(), std::numeric_limits<int>::quiet_NaN());

    // Velocity control is set as a sensible default, and will be changed if needed
    // by the controller.
    m_hw_control_modes.resize(info_.joints.size(),
                              hardware_interface::HW_IF_VELOCITY);

    m_comm_node = rclcpp::Node::make_shared("arm_system_communicator");
    m_motor_write_pub =
        m_comm_node->create_publisher<cmr_msgs::msg::MotorWriteBatch>("/ccb/motors",
                                                                      10);
    m_motor_read_sub =
        m_comm_node->create_subscription<cmr_msgs::msg::SensorReadBatch>(
            "/ccb/sensors", 10,
            std::bind(&ArmSystemHardware::sensor_callback, this,
                      std::placeholders::_1));

    return validate_interfaces() ? hardware_interface::CallbackReturn::SUCCESS
                                 : hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
ArmSystemHardware::export_state_interfaces()
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
ArmSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &m_hw_position_commands[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &m_hw_velocity_commands[i]));
    }

    return command_interfaces;
};

hardware_interface::CallbackReturn ArmSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // set initial values
    std::fill(m_hw_position_commands.begin(), m_hw_position_commands.end(), 0.0);
    std::fill(m_hw_velocity_commands.begin(), m_hw_velocity_commands.end(), 0.0);
    std::fill(m_hw_velocities.begin(), m_hw_velocities.end(), 0.0);
    std::fill(m_hw_positions.begin(), m_hw_positions.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
};

// TODO(unknown): consider revisiting the below function after testing to simplify
// the logic and avoid the extra loop iterations. Right now this directly mimics the
// ROS2 Control demo code, but it might be performing more work than necessary to
// ensure all the joints are in the same control mode.

hardware_interface::return_type ArmSystemHardware::prepare_command_mode_switch(
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
                   joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
            new_modes.emplace_back(hardware_interface::HW_IF_VELOCITY);
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

hardware_interface::return_type ArmSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    rclcpp::spin_some(m_comm_node);

    // std::stringstream result;
    // std::copy(m_hw_buffer.begin(), m_hw_buffer.end(),
    //           std::ostream_iterator<int>(result, " "));
    // CMR_LOG(INFO, "Got buffer %s", result.str().c_str());

    for (size_t i = 0; i < m_hw_buffer.size(); i++) {
        // The following block is for testing the joystick with RVIZ.
        // auto vel = m_hw_velocity_commands[i];
        // auto pos = vel * period.seconds();
        // m_hw_positions[i] += pos;
        // m_hw_velocities[i] = vel;

        auto prev_pos = m_hw_positions[i];
        m_hw_positions[i] = m_hw_buffer[i];
        // CMR_LOG(INFO, "set position of %f at joint %zu", m_hw_positions[i], i);

        if (period.seconds() > std::numeric_limits<double>::epsilon()) {
            auto vel = (m_hw_positions[i] - prev_pos) / period.seconds();
            m_hw_velocities[i] = vel;
            // CMR_LOG(INFO, "set velocity of %f at joint %zu",
            // m_hw_velocities[i],
            // i);
        } else {
            CMR_LOG(
                WARN,
                "got period value of 0; velocity data is momentarily unreliable");
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::stringstream result;
    std::copy(m_hw_velocity_commands.begin(), m_hw_velocity_commands.end(),
              std::ostream_iterator<int>(result, " "));
    CMR_LOG(INFO, "Got write commands %s", result.str().c_str());

    cmr_msgs::msg::MotorWriteBatch msg;
    msg.motor_ids = {BLDC_AR1, BLDC_AR2, BLDC_AR3, BLDC_AR4,
                     BLDC_AR5, BLDC_AR6, BDC_ENDO};
    msg.control_modes.resize(msg.motor_ids.size());
    msg.values.resize(msg.motor_ids.size());
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] == hardware_interface::HW_IF_POSITION) {
            msg.control_modes[i] = 0;
            msg.values[i] = static_cast<int>(round(m_hw_position_commands[i]));
        } else if (m_hw_control_modes[i] == hardware_interface::HW_IF_VELOCITY) {
            msg.control_modes[i] = 0;
            msg.values[i] = static_cast<int>(round(m_hw_velocity_commands[i]));
        }
    }

    m_motor_write_pub->publish(msg);
    return hardware_interface::return_type::OK;
}

void ArmSystemHardware::sensor_callback(const cmr_msgs::msg::SensorReadBatch& msg)
{
    // the ROS2 Control framework will decide when it's time to read from
    // this system's sensors. this callback stores incoming drives encoder
    // data in a buffer, and the read function will then set these as the
    // state values once invoked by the framework.
    size_t start_index =
        6;  // 6th position of sensor read array (in cmr_micro_ros.c)
    size_t end_index = 12;
    for (size_t i = start_index;
         i < std::min(end_index, static_cast<size_t>(msg.size)); i++) {
        int expected_sensor_id = static_cast<int>(BLDC_AR1 + i - start_index);
        if (msg.sensor_ids[i] != expected_sensor_id) {
            CMR_LOG(ERROR,
                    "got sensor ID %d in position %zu; expected %d. Skipping...",
                    msg.sensor_ids[i], i, expected_sensor_id);
            continue;
        }

        // the CCB expects integer values here, but the CommandInterface only
        // allows doubles. to rectify, we simply round.
        m_hw_buffer[i - start_index] = static_cast<int>(round(msg.values[i]));
        // CMR_LOG(INFO, "storing %d at index %zu", m_hw_buffer[i], i);
    }
}

// NOLINTNEXTLINE(readability-function-size)
bool ArmSystemHardware::validate_interfaces() const
{
    // Enforce requirements on the system's state and command interfaces.
    // We expect each joint to have one position command interface, one
    // velocity command interface, one position state interface, and one velocity
    // state interface.
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

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            CMR_LOG(
                FATAL,
                "Joint '%s' has %s interface, but velocity interface was expected.",
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

bool ArmSystemHardware::validate_switch(
    const std::vector<std::string>& new_modes) const
{
    return new_modes.size() == info_.joints.size() &&
           std::adjacent_find(new_modes.begin(), new_modes.end(),
                              std::not_equal_to<>()) == new_modes.end();
}

void ArmSystemHardware::halt_all_motion()
{
    // Stop motion on all joints in the current mode
    for (auto i = 0u; i < info_.joints.size(); i++) {
        if (m_hw_control_modes[i] == hardware_interface::HW_IF_POSITION) {
            m_hw_position_commands[i] = m_hw_positions[i];
        } else if (m_hw_control_modes[i] == hardware_interface::HW_IF_VELOCITY) {
            m_hw_velocity_commands[i] = 0.0;
        }
        m_hw_control_modes[i] = "undefined";
    }
}

void ArmSystemHardware::set_velocity_mode() const
{
    cmr_msgs::msg::MotorWriteBatch msg;
    for (const uint8_t id : motor_ids) {
        msg.motor_ids.push_back(id);
        // control mode of 3 indicates we want to set an ODrive setting
        msg.control_modes.push_back(3);
        // value 2 indicates we want to use velocity mode
        msg.values.push_back(2);
    }

    msg.size = static_cast<uint8_t>(msg.motor_ids.size());
    m_motor_write_pub->publish(msg);
}

void ArmSystemHardware::set_position_mode() const
{
    cmr_msgs::msg::MotorWriteBatch msg;
    for (const uint8_t id : motor_ids) {
        msg.motor_ids.push_back(id);
        // control mode of 3 indicates we want to set an ODrive setting
        msg.control_modes.push_back(3);
        // value 3 indicates we want to use position mode
        msg.values.push_back(3);
    }

    msg.size = static_cast<uint8_t>(msg.motor_ids.size());
    m_motor_write_pub->publish(msg);
}

}  // namespace cmr_control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::ArmSystemHardware,
                       hardware_interface::SystemInterface)
