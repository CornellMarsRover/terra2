#include "cmr_control/astro_sensor_broadcaster.hpp"

#include "astro_sensor_broadcaster_params.hpp"

namespace cmr_control
{

struct AstroSensorBroadcaster::ParamsPimpl {
    astro_sensor_broadcaster::Params params;
    std::shared_ptr<astro_sensor_broadcaster::ParamListener> param_listener;
};

AstroSensorBroadcaster::AstroSensorBroadcaster()
    : controller_interface::ControllerInterface(),
      m_params_impl(std::make_unique<ParamsPimpl>())
{
}

controller_interface::CallbackReturn AstroSensorBroadcaster::on_init()
{
    try {
        m_params_impl->param_listener =
            std::make_shared<astro_sensor_broadcaster::ParamListener>(get_node());
        m_params_impl->params = m_params_impl->param_listener->get_params();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

inline void set_publisher_frame_id(
    const std::string& frame_id, std::unique_ptr<realtime_tools::RealtimePublisher<
                                     cmr_msgs::msg::Float64ArrayStamped>>& publisher)
{
    publisher->lock();
    publisher->msg_.header.frame_id = frame_id;
    publisher->unlock();
}

controller_interface::CallbackReturn AstroSensorBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    m_params_impl->params = m_params_impl->param_listener->get_params();
    const bool interface_names_defined =
        !m_params_impl->params.interface_names.empty();

    if (m_params_impl->params.sensor_name.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "'sensor_name' parameter has to be "
                     "specified.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (interface_names_defined) {
        m_sensor = std::make_unique<semantic_components::AstroSensor>(
            m_params_impl->params.sensor_name,
            m_params_impl->params.interface_names);
    } else {
        m_sensor = std::make_unique<semantic_components::AstroSensor>(
            m_params_impl->params.sensor_name);
    }

    try {
        // register astro sensor data publisher
        m_sensor_state_publisher =
            get_node()->create_publisher<cmr_msgs::msg::Float64ArrayStamped>(
                "~/sensor", rclcpp::SystemDefaultsQoS());
        m_realtime_publisher =
            std::make_unique<StatePublisher>(m_sensor_state_publisher);
    } catch (const std::exception& e) {
        fprintf(stderr,
                "Exception thrown during publisher creation at configure stage with "
                "message : %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    set_publisher_frame_id(m_params_impl->params.frame_id, m_realtime_publisher);
    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AstroSensorBroadcaster::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
AstroSensorBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = m_sensor->get_state_interface_names();
    return state_interfaces_config;
}

controller_interface::CallbackReturn AstroSensorBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return m_sensor->assign_loaned_state_interfaces(state_interfaces_)
               ? controller_interface::CallbackReturn::SUCCESS
               : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn AstroSensorBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    m_sensor->release_interfaces();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AstroSensorBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (m_realtime_publisher && m_realtime_publisher->trylock()) {
        m_realtime_publisher->msg_.header.stamp = time;
        if (!m_sensor->get_values_as_message(m_realtime_publisher->msg_)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get sensor values");
            m_realtime_publisher->unlock();
            return controller_interface::return_type::ERROR;
        } else {
            m_realtime_publisher->unlockAndPublish();
        }
    }

    return controller_interface::return_type::OK;
}

}  // namespace cmr_control

#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cmr_control::AstroSensorBroadcaster,
                       controller_interface::ControllerInterface)