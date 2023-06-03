#include "cmr_control/drives_system.hpp"

#include "cmr_control/external/boards.h"
#include "cmr_utils/cmr_debug.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

const std::array<uint8_t, 6> motor_ids = {BLDC_D1, BLDC_D2, BLDC_D3,
                                          BLDC_D4, BLDC_D5, BLDC_D6};

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
    m_hw_buffer.resize(info_.joints.size() * 2,
                       std::numeric_limits<int>::quiet_NaN());

    m_astrotech_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/astrotech", buf_size, [this](const std_msgs::msg::Int32& msg) {
            update_astrotech(msg.data);
        });

    // Since hardware interfaces are not nodes, we need to create a node to do
    // our communication with the CCB for us.
    m_comm_node = rclcpp::Node::make_shared("drives_system_communicator");

    // SensorDataQoS tells ROS that we don't care about resending the message
    // if it was not acknowledged. This will prevent the system from getting
    // severely backed up by motor messages.
    m_motor_write_pub =
        m_comm_node->create_publisher<cmr_msgs::msg::MotorWriteBatch>(
            "/ccb/motors", rclcpp::SensorDataQoS());

    m_motor_read_sub =
        m_comm_node->create_subscription<cmr_msgs::msg::SensorReadBatch>(
            "/ccb/sensors", rclcpp::SensorDataQoS(),
            std::bind(&DrivesSystemHardware::sensor_callback, this,
                      std::placeholders::_1));

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

void DrivesSystemHardware::update_astrotech(int start)
{
    CMR_LOG(DEBUG, "JoystickDirectControl::update_astrotech");

    cmr_msgs::msg::MotorWriteBatch msg;
    msg.motor_ids = {0xc0 | 0x10 | 0x3};
    msg.control_modes = {0x1};
    msg.values = {start};
    msg.size = 1;
    m_motor_write_pub->publish(msg);
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

    // set velocity mode for all motors
    set_velocity_mode();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrivesSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DrivesSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    rclcpp::spin_some(m_comm_node);
    for (size_t i = 0; i < m_hw_buffer.size(); i++) {
        m_hw_velocities[i] = m_hw_buffer[i];

        // if period is not close to zero, accumulate position with instantaneous vel
        if (period.seconds() > std::numeric_limits<double>::epsilon()) {
            m_hw_positions[i] += m_hw_buffer[i] * period.seconds();
        } else {
            CMR_LOG(
                WARN,
                "got period value of 0; velocity data is momentarily unreliable");
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DrivesSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    cmr_msgs::msg::MotorWriteBatch msg;

    auto n = m_hw_commands.size();
    msg.size = static_cast<uint8_t>(n);

    for (size_t i = 0; i < n; i++) {
        msg.motor_ids.push_back(motor_ids.at(i));
        // control mode of 0 indicates we're going to write a velocity value
        msg.control_modes.push_back(0);
        // the CCB expects integer values here, but the CommandInterface only
        // allows doubles. to rectify, we simply round.
        if (motor_ids.at(i) <= 0x41 || motor_ids.at(i) == 0x45) {
            msg.values.push_back(static_cast<int>(round(m_hw_commands[i])));
        } else {
            msg.values.push_back(static_cast<int>(round(-1 * m_hw_commands[i])));
        }
    }

    m_motor_write_pub->publish(msg);

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

void DrivesSystemHardware::sensor_callback(const cmr_msgs::msg::SensorReadBatch& msg)
{
    // the ROS2 Control framework will decide when it's time to read from
    // this system's sensors. this callback stores incoming drives encoder
    // data in a buffer, and the read function will then set these as the
    // state values once invoked by the framework.
    auto n = m_hw_velocities.size();
    for (size_t i = 0; i < std::min(n, static_cast<size_t>(msg.size)); i++) {
        int expected_sensor_id = static_cast<int>(BLDC_D1 + i);
        if (msg.sensor_ids[i] != expected_sensor_id) {
            CMR_LOG(ERROR,
                    "got sensor ID %d in position %zu; expected %d. Skipping...",
                    msg.sensor_ids[i], i, expected_sensor_id);
            continue;
        }

        // the CCB gives us integer values here, but the CommandInterface only
        // allows doubles. to rectify, we simply round.
        m_hw_buffer[i] = static_cast<int>(round(msg.values[i]));
    }
}

void DrivesSystemHardware::set_velocity_mode() const
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

}  // namespace cmr_control

// This exposes this hardware interface so that it can be discovered by the
// ROS2 Control hardware resource manager
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::DrivesSystemHardware,
                       hardware_interface::SystemInterface)