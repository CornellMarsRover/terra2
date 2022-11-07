#include "cmr_nav/gps_waypoint_follower.hpp"

#include <string>
#include <vector>

namespace cmr
{
GPSWayPointFollowerClient::GPSWayPointFollowerClient()
    : Node("GPSWaypointFollowerClient"), m_goal_done(false)
{
    m_gps_waypoint_follower_action_client = rclcpp_action::create_client<ClientT>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(),
        "follow_gps_waypoints");
    this->m_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GPSWayPointFollowerClient::start_waypoint_following, this));
    // number of poses that robot will go throug, specified in yaml file
    this->declare_parameter("waypoints", std::vector<std::string>({"0"}));
    m_subscription = this->create_subscription<GPSWaypoints>(
        "gps_waypoints", 10,
        std::bind(&GPSWayPointFollowerClient::gps_waypoints_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Loaded %i GPS waypoints from YAML, gonna pass them to "
                "FollowGPSWaypoints...",
                static_cast<int>(m_gps_poses.size()));
    RCLCPP_INFO(this->get_logger(),
                "Created an Instance of GPSWayPointFollowerClient");
}

GPSWayPointFollowerClient::~GPSWayPointFollowerClient()
{
    RCLCPP_INFO(this->get_logger(),
                "Destroyed an Instance of GPSWayPointFollowerClient");
}

std::vector<geometry_msgs::msg::PoseStamped> convert_waypoints_to_pose(
    GPSWayPointFollowerClient::GPSWaypoints gps_poses)
{
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (auto it = gps_poses.begin(); it < gps_poses.end(); it++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = it->header;
        pose.pose.position.x = it->latitude;
        pose.pose.position.y = it->longitude;
        pose.pose.position.z = it->altitude;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        poses.push_back(pose);
        // pose.pose.orientation
    }
    return poses;
}

// NOLINTNEXTLINE(readability-function-size)
void GPSWayPointFollowerClient::start_waypoint_following()
{
    using namespace std::placeholders;
    this->m_timer->cancel();
    this->m_goal_done = false;

    if (!this->m_gps_waypoint_follower_action_client) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    auto is_action_server_ready =
        m_gps_waypoint_follower_action_client->wait_for_action_server(
            std::chrono::seconds(1));
    if (!is_action_server_ready) {
        RCLCPP_ERROR(
            this->get_logger(),
            "FollowGPSWaypoints action server is not available."
            " Make sure an instance of GPSWaypointFollower is up and running");
        this->m_goal_done = true;
        return;
    }
    m_gps_waypoint_follower_goal = ClientT::Goal();
    // Send the goal poses
    m_gps_waypoint_follower_goal.poses = convert_waypoints_to_pose(m_gps_poses);

    RCLCPP_INFO(this->get_logger(), "Sending a path of %zu gps_poses:",
                m_gps_waypoint_follower_goal.poses.size());
    for (auto pose_stamped : m_gps_waypoint_follower_goal.poses) {
        auto position = pose_stamped.pose.position;
        RCLCPP_DEBUG(this->get_logger(), "\t(%lf, %lf)", position.x, position.y);
    }

    auto goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();

    goal_options.goal_response_callback =
        std::bind(&GPSWayPointFollowerClient::goal_response_callback, this, _1);

    goal_options.feedback_callback =
        std::bind(&GPSWayPointFollowerClient::feedback_callback, this, _1, _2);

    goal_options.result_callback =
        std::bind(&GPSWayPointFollowerClient::result_callback, this, _1);

    auto future_goal_handle = m_gps_waypoint_follower_action_client->async_send_goal(
        m_gps_waypoint_follower_goal, goal_options);
}

void GPSWayPointFollowerClient::gps_waypoints_callback(
    std::shared_ptr<GPSWaypoints> msg)
{
    m_gps_poses = *msg;
}

void GPSWayPointFollowerClient::goal_response_callback(
    GPSWaypointFollowerGoalHandle::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
    }
}

void GPSWayPointFollowerClient::feedback_callback(
    GPSWaypointFollowerGoalHandle::SharedPtr,
    const std::shared_ptr<const ClientT::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Current waypoint: %i",
                feedback->current_waypoint);
}

void GPSWayPointFollowerClient::result_callback(
    const GPSWaypointFollowerGoalHandle::WrappedResult& result)
{
    this->m_goal_done = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->missed_waypoints) {
        RCLCPP_INFO(this->get_logger(), "Missed Waypoint %i", number);
    }
}

bool GPSWayPointFollowerClient::is_goal_done() const { return this->m_goal_done; }

}  // namespace cmr