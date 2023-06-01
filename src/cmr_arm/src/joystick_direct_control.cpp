#include "cmr_arm/joystick_direct_control.hpp"

#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/external/tomlcpp.hxx"
using cmr_msgs::msg::JoystickReading;
namespace cmr
{

constexpr auto base_rotate_idx = 0;
constexpr auto shoulder_idx = 1;
constexpr auto elbow_idx = 2;
constexpr auto second_rotate_idx = 3;
constexpr auto third_tilt_idx = 4;
constexpr auto third_rotate_idx = 5;
constexpr auto end_effector_idx = 6;

JoystickDirectControl::JoystickDirectControl(
    const std::optional<cmr::fabric::FabricNodeConfig>& config)
    : cmr::fabric::FabricNode::FabricNode(config),
      m_sensitivity(0),
      m_sens_scaler(0),
      m_is_activated(false),
      m_active_segment(ArmSegment::Base)
{
}

inline auto scale_magnitude(double mag, double factor)
{
    if (mag < 0) {
        return -pow(-1 * mag, factor);
    } else {
        return pow(mag, factor);
    }
}

/**
 * @brief Get the arm control msg object based on the current joystick reading,
 * arm segment, and sensitivity
 *
 * @param msg the joystick reading message
 * @param sensitivity the sensitivity of the joystick
 * @param current_segment the arm segment being controlled
 * @return std_msgs::msg::Float64MultiArray the message to send to the hardware
 */
// NOLINTNEXTLINE
static auto get_arm_control_msg(const JoystickReading& msg, double sensitivity,
                                ArmSegment current_segment)
{
    auto message = std_msgs::msg::Float64MultiArray();
    message.data.resize(7, 0.0);

    auto mag = scale_magnitude(msg.magnitude, 2);

    // logic to send effort values to certain motors based on control value
    if (msg.control_id == JoystickReading::MAIN_JOYSTICK_ID) {
        switch (current_segment) {
            case ArmSegment::Base:
                if (msg.axis_id == JoystickReading::Y_AXIS_ID) {
                    message.data[shoulder_idx] = mag;
                } else if (msg.axis_id == JoystickReading::Z_AXIS_ID) {
                    message.data[base_rotate_idx] = mag * sensitivity;
                }
                break;
            case ArmSegment::Elbow:
                if (msg.axis_id == JoystickReading::Y_AXIS_ID) {
                    message.data[elbow_idx] = mag;
                } else if (msg.axis_id == JoystickReading::Z_AXIS_ID) {
                    message.data[second_rotate_idx] = mag * sensitivity;
                }
                break;
            case ArmSegment::Wrist:
                if (msg.axis_id == JoystickReading::Y_AXIS_ID) {
                    message.data[third_tilt_idx] = mag;
                } else if (msg.axis_id == JoystickReading::Z_AXIS_ID) {
                    message.data[third_rotate_idx] = mag * sensitivity;
                }
                break;
            default:
                CMR_ASSERT_MSG(false, "Invalid arm segment %u",
                               static_cast<uint8_t>(current_segment));
                break;
        }
    } else if (msg.control_id == JoystickReading::SECOND_JOYSTICK_ID) {
        if (msg.axis_id == JoystickReading::Y_AXIS_ID) {
            message.data[end_effector_idx] = mag;
        }
    }
    return message;
}

void JoystickDirectControl::update_arm_position(const JoystickReading& msg)
{
    if (!m_is_activated) {
        return;
    }
    CMR_LOG(DEBUG, "JoystickDirectControl::update_arm_position");
    if (msg.control_id == JoystickReading::SENS_SLIDER_ID) {
        m_sensitivity = msg.magnitude * m_sens_scaler;
    } else if (msg.control_id == JoystickReading::THIRD_BUTTON_ID &&
               msg.magnitude == 1) {
        m_active_segment =
            static_cast<ArmSegment>((static_cast<int>(m_active_segment) + 1) % 3);
        CMR_LOG(INFO, "Active segment: %d", static_cast<uint8_t>(m_active_segment));
    } else {
        const auto message =
            get_arm_control_msg(msg, m_sensitivity, m_active_segment);

        m_arm_control_pub->publish(message);
        CMR_LOG(INFO, "JoystickDirectControl::update_arm_position: published");
    }
}

void JoystickDirectControl::update_end_effector_velocity(int vel)
{
    if (!m_is_activated) {
        return;
    }
    CMR_LOG(DEBUG, "JoystickDirectControl::update_end_effector_velocity");

    cmr_msgs::msg::MotorWriteBatch msg;
    msg.motor_ids = {0xA4};
    msg.control_modes = {0x2};
    msg.values = {vel};
    msg.size = 1;
    m_motor_write_pub->publish(msg);
}

void JoystickDirectControl::update_hex_driver_velocity(int vel)
{
    if (!m_is_activated) {
        return;
    }
    CMR_LOG(DEBUG, "JoystickDirectControl::update_end_effector_velocity");

    cmr_msgs::msg::MotorWriteBatch msg;
    // TODO(dz268): update motor ID to reflect correct ID for hex driver servo
    msg.motor_ids = {0x00};
    msg.control_modes = {0x2};
    msg.values = {vel};
    msg.size = 1;
    m_motor_write_pub->publish(msg);
}

void JoystickDirectControl::update_extendo_velocity(int vel)
{
    if (!m_is_activated) {
        return;
    }
    CMR_LOG(DEBUG, "JoystickDirectControl::update_extendo_velocity");

    cmr_msgs::msg::MotorWriteBatch msg;
    msg.motor_ids = {0x90};
    msg.control_modes = {0x2};
    msg.values = {vel};
    msg.size = 1;
    m_motor_write_pub->publish(msg);
}

bool JoystickDirectControl::configure(const std::shared_ptr<toml::Table>& table)
{
    // read node config; setup subscriptions, clients, services, etc.; and
    // most of the node setup logic here
    const auto node_settings = table->getTable("node");

    const auto [ok_buffer_size, buffer_size] = node_settings->getInt("buffer_size");
    const auto [ok_sensi_scaler, sensi_scaler] =
        node_settings->getDouble("sensitivity_scaler");
    const auto [ok_start_sens, start_sens] =
        node_settings->getDouble("starting_sensitivity");
    if (!ok_buffer_size || !ok_sensi_scaler || !ok_start_sens) {
        return false;
    }
    m_sens_scaler = sensi_scaler;
    const auto buf_size = static_cast<uint64_t>(buffer_size);
    m_joystick_sub = this->create_subscription<cmr_msgs::msg::JoystickReading>(
        "js_input", buf_size, [this](const JoystickReading& joy_msg) {
            return update_arm_position(joy_msg);
        });
    m_end_effector_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/ee_input", buf_size, [this](const std_msgs::msg::Int32& msg) {
            update_end_effector_velocity(msg.data);
        });
    m_hex_driver_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/hex_input", buf_size, [this](const std_msgs::msg::Int32& msg) {
            update_hex_driver_velocity(msg.data);
        });
    m_extendo_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/extendo_input", buf_size, [this](const std_msgs::msg::Int32& msg) {
            update_extendo_velocity(msg.data);
        });
    m_arm_control_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", buf_size);
    m_sensitivity = start_sens;
    return true;
}

bool JoystickDirectControl::activate()
{
    m_arm_control_pub->on_activate();
    m_motor_write_pub->on_activate();
    m_is_activated = true;
    return true;
}

bool JoystickDirectControl::deactivate()
{
    m_arm_control_pub->on_deactivate();
    m_motor_write_pub->on_deactivate();
    m_is_activated = false;
    return true;
}

bool JoystickDirectControl::cleanup()
{
    m_joystick_sub.reset();
    m_arm_control_pub.reset();
    return true;
}

}  // namespace cmr
