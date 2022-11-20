#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/arm_joint_effort.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace cmr
{
/**
 * @brief An ArmSegment is a segment of the arm that can be controlled by mutiple
 * joints for multiple degrees of freedom
 *
 */
enum class ArmSegment : uint8_t { Base = 0, Elbow, Wrist };

/**
 * @brief The JoystickDirectControl node is the node called to activate arm control
 * via direct joystick control (joystick hardware directly controls joint motor
 * efforts). A subscriber is set up to listen for messages, and a callback function
 * sends messages via one of many publishers to individual joint motors.
 *
 * The subscriber topic listens for messages of type JoystickReading.msg on topic
 * "/js_input", which contains an int control_id, int axis_id, and float64 magnitude.
 * The control and id numbers affect which motor is being controlled, and the
 * magnitude affects how much.
 *
 * The publisher topics are set individually to target specific motors. They're named
 * according to the names in the arm URDF. The message type these send out consist of
 * only a float64 effort: the magnitude of effort for the motor to exert.
 */
class JoystickDirectControl : public cmr::fabric::FabricNode
{
  private:
    /** listens for messages from the joystick hardware */
    std::shared_ptr<rclcpp::Subscription<cmr_msgs::msg::JoystickReading>>
        m_joystick_sub;
    /** publishes messages to the ros2 control hardware interface */
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
        m_arm_control_pub;
    double m_sensitivity;
    double m_sens_scaler;
    bool m_is_activated;
    /** The current arm segment being controlled */
    ArmSegment m_active_segment;
    /** Timer to toggle motors off */
    std::shared_ptr<rclcpp::WallTimer<std::function<void()>>> m_timer;
    /** The last time we wrote to the hadware interface */
    rclcpp::Time m_last_msg_time;
    /** If we wrote to the hardware interface after the last time we toggled controls
     * off */
    bool m_dirty;

  public:
    /**
     * Constructs a `JoystickDirectControl`, optionally passing in config parameters
     * for testing.
     *
     * @param config the configuration struct for starting the node or an empty
     * optional to start the node from a launch file or via ROS
     */
    explicit JoystickDirectControl(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

  private:
    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    /**
     * @brief update_arm_position is the callback function for when the subscriber
     * receives a message from the joystick hardware. It handles logic for where to
     * publish what message to (i.e. a certain control, axis, and magnitude number
     * corresponds to a certain effort amount to a certain motor's axis.)
     */
    void update_arm_position(const cmr_msgs::msg::JoystickReading& msg);

    /**
     * @brief Callback called periodically to turn off the motors if the joystick
     * hasn't been used in a while.
     */
    void toggle_off_callback();
};

}  // namespace cmr