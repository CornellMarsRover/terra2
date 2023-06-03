#pragma once

#include <linux/joystick.h>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
#include "cmr_utils/thread_wrapper.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

namespace cmr
{

/**
 * `Joystick`
 *  TOML File:
 *      restart_attempts - Amount of restart attempts
 *                         used for fault handeling.
 *      restart_delay - The delay used for restarts in
 *                      fault handling
 *      device - Where the device is located in the cmr files
 *      type - Which controller is being used (armcontroller for arm;
 *             drivescontroller for drives)
 *      arm_max_speed - The maximum speed used for the arm
 *      max_speed - The maximum driving speed for the rover
 *
 *  Use:
 *      Each joystick for drives and arm has its own joystick TOML
 *      file. This device, connected by usb, is read through js_events.
 *      These events are then translated to readable output evered 5 ms
 *      through the use of a wall_timer and sent to the appropriate topics
 *      for the arm and drives nodes.
 *
 * Publishers and Subscribers:
 *      Input is taken through USB devices so there are no subscribers
 *
 *      Two publishers are used. One for the drives input (m_drives_pub)
 *      and the other for the arm input (m_joystick_pub).
 *
 *      m_joystick_pub - subscribes to topic "js_input"
 *      m_drives_pub - subscribe to topic "/drives_controller/cmd_vel"
 *
 *
 */
class Joystick : public cmr::fabric::FabricNode
{
  public:
    /**
     * Constructs a `Joystick`, optionally passing in config parameters for
     * testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit Joystick(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

    /**
     * Current state of an axis.
     */
    struct AxisState {
        int16_t x, y;
    };

  private:
    /**
     * @brief the device name listed in the toml file
     */
    std::string m_device_name;

    /**
     * @brief the type of the device (arm joystick or drives joystick)
     */
    std::string m_type_name;

    /**
     * @brief the max speed for drives
     * auto-set to 2.5 if no max speed is found in the toml file
     */
    double m_max_speed = 2.5;

    /**
     * @brief the max speed for the arm
     * auto-set to 10 if no max speed is found in the toml file
     */
    double m_max_arm_speed = 10;

    /**
     * @brief Changes to 1 if the joystick is succesfully read
     * else stays at 0
     */
    int m_js;

    /**
     * @brief Used to repeat the arm callback in order to continuously send
     * messages to the arm node.
     */
    std::unique_ptr<fabric::GenericLifecycleTimer> m_buffer_timer;

    /**
     * @brief Used to repeat the drives callback in order to continuously send
     * messages to the arm node.
     */
    std::unique_ptr<fabric::GenericLifecycleTimer> m_drives_publish_timer;

    /**
     * @brief Checks if a message has been received in order to continue to the
     * publish part knowing a message exists
     */
    bool m_got_first_message = false;

    /**
     * @brief contains the state of a particular axis on a joystick. The state
     * includes the position of a given x or y (or z).
     */
    std::array<AxisState, 3> m_axis_state{};

    /**
     * @brief The publisher for the joystick read function
     */
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::JoystickReading>>
        m_joystick_pub;

    /**
     * @brief The publisher for the end effector read function
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_end_effector_pub;

    /**
     * @brief The publisher for the hex driver read function
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_hex_driver_pub;

    /**
     * @brief The publisher for the extendo cam read function
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_extendo_pub;

    /**
     * @brief The publisher for the pantilt camera read function, pan value
     * for camera 1
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_pan_cam1_pub;

    /**
     * @brief The publisher for the pantilt camera read function, tilt value
     * for camera 1
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_tilt_cam1_pub;

    /**
     * @brief The publisher for the pantilt camera read function, pan value
     * for camera 2
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_pan_cam2_pub;

    /**
     * @brief The publisher for the pantilt camera read function, tilt value
     * for camera 2
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_tilt_cam2_pub;

    /**
     * @brief The publisher for the astrotech start function for scooping
     */
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>>
        m_astrotech_pub;

    /**
     * @brief The publisher for the drives read function
     */
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>>
        m_drives_pub;

    /**
     * @brief contains the last message that was sent out by the drives callback
     * function
     */
    geometry_msgs::msg::TwistStamped m_last_drives_twist;

    /**
     * @brief contains the last camera pan message that was sent out by the drives
     * callback function
     */
    std_msgs::msg::Int32 m_last_pan;

    /**
     * @brief contains the last camera tilt message that was sent out by the drives
     * callback function
     */
    std_msgs::msg::Int32 m_last_tilt;

    /**
     * @brief Used to read values from the joystick and send inputs in the form of
     * messages. Will send the x, y, z axis and magnitudes for every control stick on
     * the main arm joystick.
     *
     * @param axis_state an array of axis pulled from the js_events
     * @return publishes the arm movement message to the m_joystick_pub
     */
    void arm_callback(std::array<AxisState, 3>& axis_state) const;

    /**
     * @brief Used to read values from the drives controller and send inputs in the
     * form of messages. Will store the messages in the form of a TwistStamped to be
     * published by another callback timer
     *
     * Also used for pan and tilting camera.
     *
     * @param axis_state an array of axis pulled from the js_events
     */
    void drives_callback(std::array<AxisState, 3>& axis_state);

    /**
     * @brief Used to continuously display messages through the use of a seperate
     * callback timer than the one used for the drives_callback.
     *
     * @return publishes the stored TwistStamped drives message to the m_drives_pub
     */
    void drives_publish_callback() const;

    /**
     * @brief Used to assign values and publish a message whenever a button is
     * pressed or released. This is used for arm callback.
     *
     * @param event the js_event by which the values are pulled from
     * @param message the message that is copied from arm_callback and will
     * eventually be published
     */
    void js_event_button_arm_publ(std::optional<js_event> event,
                                  cmr_msgs::msg::JoystickReading message) const;

    /**
     * @brief initilizes publishers and member variables to be used
     *
     * @param table the table used to pull the node_settings
     * @return true when all member variables are pulled properly
     * @return false whell a member variable is not pulled properly
     */
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;
};

}  // namespace cmr