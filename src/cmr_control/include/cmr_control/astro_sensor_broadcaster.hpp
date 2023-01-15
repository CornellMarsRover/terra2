#pragma once
#include <memory>
#include <string>
#include <vector>

#include "cmr_control/astro_sensor_component.hpp"
#include "cmr_msgs/msg/float64_array_stamped.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace cmr_control
{

/**
 * @brief The AstroSensorBroadcaster is a broadcaster which publishes AstroTech
 * sensor data (or really any floating point sensor data) along with
 * a timestamp to a ROS topic.
 *
 * It can be configured to publish multiple readings of different types.
 * The broadcaster publishes messages of type cmr_msgs::msg::Float64ArrayStamped
 * which contains a timestamp and a vector of floating point values.
 *
 */
class AstroSensorBroadcaster : public controller_interface::ControllerInterface
{
  public:
    AstroSensorBroadcaster();

    controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;

    controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief [Pimpl idiom](https://en.cppreference.com/w/cpp/language/pimpl)
     * used to hide parameter lib from clients of `cmr_control` lib.
     *
     * While it is not strictly necessary, this would allow linking the parameter
     * lib to `cmr_control` privately, and without exporting it
     */
    struct ParamsPimpl;

  private:
    std::unique_ptr<ParamsPimpl> m_params_impl;
    std::unique_ptr<semantic_components::AstroSensor> m_sensor;

    using StatePublisher =
        realtime_tools::RealtimePublisher<cmr_msgs::msg::Float64ArrayStamped>;
    rclcpp::Publisher<cmr_msgs::msg::Float64ArrayStamped>::SharedPtr
        m_sensor_state_publisher;
    std::unique_ptr<StatePublisher> m_realtime_publisher;
};

}  // namespace cmr_control
