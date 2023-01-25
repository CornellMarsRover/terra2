#include "cmr_nav/autonomy_mediator.hpp"

#include <cstdio>

using namespace cmr;

AutonomyMediator::AutonomyMediator(rclcpp::Node& n, const DependencyMap& deps)
    : led_client(std::make_unique<SetLedClient>(n, deps.at("hal") + "/set_led")),
      feedback_pub(
          n.create_publisher<cmr_hw_msgs::PlanFeedback>("autonomy/feedback", 1)),
      pos_pub(n.create_publisher<geometry_msgs::PoseStamped>("log/target_pose", 1,
                                                             true)),
      move_client(
          std::make_unique<MoveBaseClient>(n, deps.at("nav") + "/move_base", false))
{
}

void cmr::AutonomyMediator::signal(Signal sig)
{
    Actions::Result<cmr_hal::SetLedAction> result;
    const auto goal = genLedGoal(signal2Color(sig));
    const auto timeout = rclcpp::Duration(2);
    const auto success = led_client->call(goal, result, timeout, timeout);

    if (!success) {
        logger(LogLevel::ERROR)
            << "Could not set LED!\nLED Status: " << result.status;
    }
}