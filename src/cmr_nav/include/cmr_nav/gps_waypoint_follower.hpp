#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "cmr_fabric/fabric_node.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace cmr
{

enum class ActionStatus { UNKNOWN, PROCESSING, FAILED, SUCCEEDED };

/**
 * @brief A ros node that drives robot through gievn way points from YAML file
 *
 */
class GPSWayPointFollowerClient : public cmr::fabric::FabricNode
{
  public:
    using GPSWaypoints = std::vector<sensor_msgs::msg::NavSatFix>;

    using ClientT = nav2_msgs::action::FollowWaypoints;
    // shorten the Goal handler Client type
    using GPSWaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<ClientT>;

    /**
     * @brief Construct a new WayPoint Folllower object
     *
     */
    GPSWayPointFollowerClient(
        const std::optional<cmr::fabric::FabricNodeConfig>& config = std::nullopt);

    /**
     * @brief Destroy the Way Point Folllower object
     *
     */
    ~GPSWayPointFollowerClient();

    bool configure(const std::shared_ptr<toml::Table>& table) override;

    bool activate() override;

    bool deactivate() override;

    bool cleanup() override;

    // /**
    //  * @brief send robot through each of the pose in poses vector
    //  *
    //  * @param poses
    //  */
    // void start_waypoint_following();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool is_goal_done() const;

    /**
     * @brief given a parameter name on the yaml file, loads this parameter as
     * sensor_msgs::msgs:NavSatFix Note that this parameter needs to be an
     * array of doubles
     *
     * @return sensor_msgs::msgs:NavSatFix
     */
    void gps_waypoints_callback(std::shared_ptr<GPSWaypoints> msg);

    void goal_response_callback(
        GPSWaypointFollowerGoalHandle::SharedPtr goal_handle);

    void feedback_callback(GPSWaypointFollowerGoalHandle::SharedPtr,
                           const std::shared_ptr<const ClientT::Feedback> feedback);

    void result_callback(const GPSWaypointFollowerGoalHandle::WrappedResult& result);

  private:
    bool m_goal_done;
    bool m_active;
    rclcpp::TimerBase::SharedPtr m_timer;
    // client to connect waypoint follower service(FollowWaypoints)
    rclcpp_action::Client<ClientT>::SharedPtr m_gps_waypoint_follower_action_client;

    // goal handler to query state of goal
    ClientT::Goal m_gps_waypoint_follower_goal;

    GPSWaypointFollowerGoalHandle::SharedPtr m_gps_waypoint_follower_goalhandle;

    GPSWaypoints m_gps_poses;
    std::shared_ptr<rclcpp::Node> m_gps_node;
    rclcpp::Subscription<GPSWaypoints>::SharedPtr m_subscription;
};

}  // namespace cmr