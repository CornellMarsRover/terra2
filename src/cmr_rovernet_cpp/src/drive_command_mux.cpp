// C++ port of cmr_rovernet/command_mux.py (executable `drive_command_mux`).
//
// Selects one drive command source and republishes it on /cmd_vel at 20 Hz:
//   /cmd_vel/teleop, /cmd_vel/autonomy (cmr_msgs/DriveCommand)  inputs
//   /cmd_vel/estop  (std_msgs/Bool)    latch / release the emergency stop
//   /cmd_vel/source (std_msgs/String)  select "teleop" or "autonomy"
//   /cmd_vel        (cmr_msgs/DriveCommand)  selected command, or all zeros
//                   when estopped / the selected source is stale or silent.
#include <memory>
#include <optional>
#include <string>

#include <cmr_msgs/msg/drive_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmr_rovernet_cpp/command_mux_core.hpp"
#include "cmr_rovernet_cpp/time_util.hpp"

namespace cmr_rovernet_cpp {

class DriveCommandMux : public rclcpp::Node
{
public:
  DriveCommandMux()
  : rclcpp::Node("drive_command_mux"),
    // Both throw std::invalid_argument on a bad value, like the Python
    // ValueError that aborted the node's constructor.
    mux_(declare_parameter<double>("command_timeout_s", 0.5))
  {
    // NOTE(port): Python declared active_source before command_timeout_s;
    // declaration order is not observable.
    mux_.select(declare_parameter<std::string>("active_source", "teleop"));

    output_ = create_publisher<cmr_msgs::msg::DriveCommand>("/cmd_vel", 10);
    teleop_sub_ = create_subscription<cmr_msgs::msg::DriveCommand>(
      "/cmd_vel/teleop", 10,
      [this](cmr_msgs::msg::DriveCommand::ConstSharedPtr msg) {receive("teleop", *msg);});
    autonomy_sub_ = create_subscription<cmr_msgs::msg::DriveCommand>(
      "/cmd_vel/autonomy", 10,
      [this](cmr_msgs::msg::DriveCommand::ConstSharedPtr msg) {receive("autonomy", *msg);});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/cmd_vel/estop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {set_estop(*msg);});
    source_sub_ = create_subscription<std_msgs::msg::String>(
      "/cmd_vel/source", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {set_source(*msg);});
    // rclpy create_timer() runs on the node clock (ROS time), so does this.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.05), [this]() {publish_selected();});
  }

private:
  void receive(const std::string & source, const cmr_msgs::msg::DriveCommand & msg)
  {
    if (!mux_.receive(source, {msg.vx, msg.vy, msg.omega, msg.speed_rps}, mono_now())) {
      RCLCPP_ERROR(get_logger(), "Rejected non-finite %s command", source.c_str());
    }
  }

  void set_estop(const std_msgs::msg::Bool & msg)
  {
    if (msg.data && !estop_) {
      mux_.emergency_stop();
    } else if (!msg.data && estop_) {
      mux_.reset();
    }
    estop_ = msg.data;
  }

  void set_source(const std_msgs::msg::String & msg)
  {
    try {
      mux_.select(msg.data);
    } catch (const std::invalid_argument & exc) {
      RCLCPP_ERROR(get_logger(), "%s", exc.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Drive source selected: %s", msg.data.c_str());
  }

  void publish_selected()
  {
    const auto selected = mux_.output(mono_now());
    const std::string status = estop_ ? "estop" : (selected ? mux_.source() : "timeout");
    if (!last_status_ || status != *last_status_) {
      RCLCPP_INFO(get_logger(), "Drive mux status: %s", status.c_str());
      last_status_ = status;
    }
    cmr_msgs::msg::DriveCommand out;  // all zeros
    if (selected) {
      out.vx = (*selected)[0];
      out.vy = (*selected)[1];
      out.omega = (*selected)[2];
      out.speed_rps = (*selected)[3];
    }
    output_->publish(out);
  }

  CommandMux mux_;
  bool estop_ = false;
  std::optional<std::string> last_status_;

  rclcpp::Publisher<cmr_msgs::msg::DriveCommand>::SharedPtr output_;
  rclcpp::Subscription<cmr_msgs::msg::DriveCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<cmr_msgs::msg::DriveCommand>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cmr_rovernet_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<cmr_rovernet_cpp::DriveCommandMux>());
  } catch (const std::exception & exc) {
    // Python: an uncaught exception (bad command_timeout_s / active_source,
    // bad parameter type) terminated the process with a traceback.
    RCLCPP_FATAL(rclcpp::get_logger("drive_command_mux"), "%s", exc.what());
    rc = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
