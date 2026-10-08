/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/arm_controller/cmd_vel";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "end_effector";
const std::string BASE_FRAME_ID = "base_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis {
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    LEFT_TRIGGER = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5,
    D_PAD_X = 6,
    D_PAD_Y = 7
};
enum Button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    CHANGE_VIEW = 6,
    MENU = 7,
    HOME = 8,
    LEFT_STICK_CLICK = 9,
    RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER, 1.0}, {RIGHT_TRIGGER, 1.0}};
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the
// above enums and the follow 2 functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or
 * JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a
 * JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
    const auto axis = [&axes](std::size_t index) -> double {
        return index < axes.size() ? axes[index] : 0.0;
    };
    const auto button = [&buttons](std::size_t index) -> int {
        return index < buttons.size() ? buttons[index] : 0;
    };

    // The direct USB DualSense reports six axes:
    // left X/Y, right X/Y, L2, R2. Its triggers rest at 0.0. The existing
    // competition/UDP controller message uses the eight-axis Xbox layout and
    // its triggers rest at 1.0. Handle both without indexing past the message.
    const bool dualsense_layout = axes.size() == 6;

    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (!dualsense_layout &&
        (button(B) || button(Y) || axis(D_PAD_X) || axis(D_PAD_Y))) {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("base_joint");
        joint->velocities.push_back(axis(D_PAD_X));
        joint->joint_names.push_back("shoulder_joint");
        joint->velocities.push_back(axis(D_PAD_Y));

        // Map square/circle to gearbox six roll. X/triangle are reserved for
        // end-effector servos and should not produce IK commands.
        joint->joint_names.push_back("wrist_rotate_two");
        joint->velocities.push_back(button(Y) - button(B));
        return false;
    }

    // Arm teleop mapping:
    //   - right stick up/down    -> vertical translation
    //   - right stick left/right -> right/left plane translation
    //   - right trigger / R1     -> forward/back translation
    //   - left stick up/down     -> pitch (up = pitch down, down = pitch up)
    //   - left stick left/right  -> yaw
    //   - square/circle          -> direct gearbox six roll joint jog above
    const std::size_t right_stick_x = dualsense_layout ? 2 : RIGHT_STICK_X;
    const std::size_t right_stick_y = dualsense_layout ? 3 : RIGHT_STICK_Y;
    twist->twist.linear.z = axis(right_stick_y);
    twist->twist.linear.x = axis(right_stick_x);

    const double lin_forward_right = dualsense_layout
                                         ? -axis(RIGHT_TRIGGER)
                                         : 0.5 * (axis(RIGHT_TRIGGER) -
                                                  AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    const double lin_backward_r1 = button(RIGHT_BUMPER);
    twist->twist.linear.y = lin_forward_right + lin_backward_r1;

    twist->twist.angular.x = -axis(LEFT_STICK_Y);
    twist->twist.angular.z = axis(LEFT_STICK_X);

    twist->twist.angular.y = 0.0;

    return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command
 * frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>&)
{
    frame_name = BASE_FRAME_ID;
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
  public:
    JoyToServoPub(const rclcpp::NodeOptions& options)
        : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
    {
        // Setup pub/sub
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                return joyCB(msg);
            });

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
        // Create a service client to start the ServoNode
        servo_start_client_ =
            this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        servo_start_client_->wait_for_service(std::chrono::seconds(1));
        servo_start_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());

    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
    {
        // Create the messages we might publish
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // This call updates the frame for twist commands
        updateCmdFrame(frame_to_publish_, msg->buttons);

        // Convert the joystick message to Twist or JointJog and publish
        if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg)) {
            // publish the TwistStamped
            twist_msg->header.frame_id = frame_to_publish_;
            twist_msg->header.stamp = this->now();
            twist_pub_->publish(std::move(twist_msg));
        } else {
            // publish the JointJog
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = BASE_FRAME_ID;
            joint_pub_->publish(std::move(joint_msg));
        }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    std::string frame_to_publish_;
};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
