#pragma once
#include <cmr_msgs/msg/float64_array_stamped.hpp>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components
{

/**
 * @brief The AstroSensor class is a semantic component which represents an
 * sensor returning one or multiple floating point values.
 *
 * It is used by the AstroSensorBroadcaster to get the sensor readings and store them
 * in a ROS message to be published.
 */
class AstroSensor
    : public SemanticComponentInterface<cmr_msgs::msg::Float64ArrayStamped>
{
  private:
    /** Gets the latest readings from the state interface and stores them into
     * `m_readings`
     */
    void update_readings()
    {
        for (auto i = 0u; i < m_readings.size(); ++i) {
            m_readings[i] = state_interfaces_[i].get().get_value();
        }
    }

  public:
    /** Creates a new AstroSensor with a single reading interface
     * @param name The name of the sensor, and the "namespace" of the interface name
     */
    explicit AstroSensor(const std::string& name)
        : SemanticComponentInterface(name, 1), m_readings(1, 0.0)
    {
        interface_names_.emplace_back(name_ + "/" + "reading");
    }

    /** Creates a new AstroSensor with multiple reading interfaces
     * @param name The name of the sensor, and the namespace of the reading
     * interfaces
     * @param names The relative names of the reading interfaces. Requires `names`
     * is not empty
     */
    explicit AstroSensor(const std::string& name,
                         const std::vector<std::string>& names)
        : SemanticComponentInterface(name, names.size()),
          m_readings(names.size(), 0.0)
    {
        for (const auto& interface_name : names) {
            interface_names_.emplace_back(name_ + "/" + interface_name);
        }
    }

    virtual ~AstroSensor() = default;

    /**
     * @brief Updates a `cmr_msgs::msg::Float64ArrayStamped` message with the
     * current sensor readings.
     *
     * Adheres to the static interface of
     * `SemanticComponentInterface::get_values_as_message`
     *
     * @param message message to update
     * @return true on success
     */
    [[nodiscard]] bool get_values_as_message(
        cmr_msgs::msg::Float64ArrayStamped& message)
    {
        // This function is not very idiomatic, but I kept the signature the
        // same to be consistent with the SemanticInterface static
        // interface. This function shadows the base class function, and
        // I'm going to be honest I'm not sure why it is done this way.
        update_readings();

        // update the message values
        message.data = m_readings;

        // The current implementation always returns true, but we shouldn't
        // write code that assumes this, hence the `[[nodiscard]]`
        return true;
    }

  private:
    /**
     * The sensor readings.
     *
     * The `ith` reading in this vector corresponds to the `ith` state interface.
     */
    std::vector<double> m_readings;
};

}  // namespace semantic_components