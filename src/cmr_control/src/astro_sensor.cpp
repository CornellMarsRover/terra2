#include "cmr_control/astro_sensor.hpp"

#include <chrono>
#include <cmath>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmr_control
{

hardware_interface::CallbackReturn AstroSensorHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SensorInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // how to get params: info_.hardware_parameters["name_of_param"]

    m_sensor_states.resize(info_.sensors[0].state_interfaces.size(),
                           std::numeric_limits<double>::quiet_NaN());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AstroSensorHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++) {
        state_interfaces.emplace_back(info_.sensors[0].name,
                                      info_.sensors[0].state_interfaces[i].name,
                                      &m_sensor_states[i]);
    }

    return state_interfaces;
}

hardware_interface::CallbackReturn AstroSensorHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AstroSensor"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn AstroSensorHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("AstroSensor"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// dummy data to get different readings based on the interface name
inline double get_reading(const std::string& sensor_interface_name)
{
    const auto sensor_type = [&sensor_interface_name]() {
        if (const auto index = sensor_interface_name.find('/');
            index != std::string::npos) {
            return sensor_interface_name.substr(
                index + 1, sensor_interface_name.size() - index - 1);
        } else {
            return sensor_interface_name;
        }
    }();
    if (sensor_type == "c02") {
        return 10.0;
    } else if (sensor_type == "nitrogen") {
        return 20.0;
    } else if (sensor_type == "potassium") {
        return 30.0;
    } else {
        return 100.0;
    }
}

hardware_interface::return_type AstroSensorHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // dummy data to see if read works
    for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++) {
        m_sensor_states[i] = get_reading(info_.sensors[0].state_interfaces[i].name);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace cmr_control

#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::AstroSensorHardware,
                       hardware_interface::SensorInterface)
