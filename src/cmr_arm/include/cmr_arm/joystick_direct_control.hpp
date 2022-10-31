#pragma once
#include "cmr_fabric/fabric_node.hpp"
#include "cmr_msgs/msg/arm_joint_effort.hpp"
#include "cmr_msgs/msg/joystick_reading.hpp"
namespace cmr
{
/**
 * @brief The JoystickDirectControl node is the node called to activate arm control
 * via direct joystick control (joystick hardware directly controls joint motor
 * efforts). A subscriber is set up to listen for messages, and a callback function
 * sends messages via one of many publishers to individual joint motors.
 */
class JoystickDirectControl : public cmr::fabric::FabricNode
{
  private:
    /**
     * Publishers and subcribers declared here. One subscriber listens for messages
     * from the joystick hardware, and one publisher for each individual joint motor.
     */
    std::shared_ptr<rclcpp::Subscription<cmr_msgs::msg::JoystickReading>>
        m_joystick_sub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_base_rotate_effort_pub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_shoulder_effort_pub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_elbow_effort_pub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_second_rotate_effort_pub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_third_tilt_effort_pub;
    std::shared_ptr<
        rclcpp_lifecycle::LifecyclePublisher<cmr_msgs::msg::ArmJointEffort>>
        m_third_rotate_effort_pub;
    double m_sensitivity;
    double m_sens_scaler;
    bool m_is_activated;

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
};

}  // namespace cmr