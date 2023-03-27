#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace cmr_control
{
/**
 * @brief AstroSensorHardware is a hardware interface for the astro sensor that
 * provides readings of a sensor's floting point readings.
 *
 * Can provide different ways of getting readings depending on the state interface
 * name. The hardware returns a floating point value for each reading it
 * is configured to get in the controller YAML config file.
 *
 * Meant to be used with the `cmr_control::AstroSensorBroadcaster`.
 */
class AstroSensorHardware : public hardware_interface::SensorInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AstroSensorHardware);

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

  private:
    /**
     * The sensor readings.
     *
     * This will be the same length as `info_.sensors.size()` and the `ith` sensor
     * defined in the controller config YAML file will be the `ith` sensor in
     * `info_.sensors` and the `ith` value in this vector.
     */
    std::vector<double> m_sensor_states;
};

}  // namespace cmr_control
