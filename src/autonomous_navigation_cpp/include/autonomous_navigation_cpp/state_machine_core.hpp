// Port of autonomous_navigation/state_machine_core.py (namespace mission) and
// the ROS-free mission bookkeeping of state_machine.py (StateMachineCore).
#pragma once

#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "autonomous_navigation_cpp/planner_core.hpp"   // Pt, py_dist

namespace autonomous_navigation_cpp
{
namespace mission
{

struct TargetDecision
{
  Pt target;
  bool reached{false};
  bool timed_out{false};
  bool pop_coarse{false};
  bool pop_search{false};
  bool announce_object{false};
};

// Choose a coarse waypoint while the global goal is still far away.
TargetDecision far_target(
  const Pt & position, const Pt & goal, const Pt & current_target,
  const std::deque<Pt> & coarse_waypoints, bool pursuing_object, double coarse_threshold = 2.5);

// Choose a coordinate goal and report when it has been reached.
TargetDecision coordinate_target(const Pt & position, const Pt & goal, double threshold = 2.0);

// Choose the next object-search waypoint or report search exhaustion.
TargetDecision search_target(
  const Pt & position, const Pt & current_target, const std::deque<Pt> & search_waypoints,
  double threshold = 2.0);

// Continue toward a detected object until its standoff is reached.
TargetDecision object_target(const Pt & position, const Pt & target, double threshold = 1.5);

// Generate the legacy expanding search pattern around a target.
// NOTE(port): the dx/dy formula (cos - sin, sin + sin) is ported verbatim
// from state_machine_core.py; it is not a circle/spiral.
std::vector<Pt> search_waypoints(
  const Pt & center, double angle_step, double radius_step, double maximum_radius);

// Convert latitude/longitude pairs (degrees) to local north/west meters.
Pt north_west_meters(
  double origin_lat, double origin_lon, double target_lat, double target_lon,
  double earth_radius = 6378137.0);

// Select the active mission target from the complete navigation state.
TargetDecision select_target(
  const Pt & position, const Pt & goal, const Pt & current_target,
  const std::deque<Pt> & coarse_waypoints, const std::string & current_object,
  bool object_found, const std::deque<Pt> & search_points, double far_threshold = 10.0,
  double coordinate_threshold = 2.0);

}  // namespace mission

// One entry of the waypoints YAML.
struct MissionWaypoint
{
  double latitude{0.0};
  double longitude{0.0};
  std::optional<std::string> type;
  std::optional<double> threshold;
};

class StateMachineCore
{
public:
  enum class Level { Info, Error };
  struct Hooks
  {
    std::function<void(Level, const std::string &)> log;
    std::function<void(const std::string &)> publish_target_name;   // /autonomy/target_object/name
    std::function<void()> blink_led;                                // /autonomy/led "blink"
  };

  // `waypoints` must not be empty (the node refuses to start otherwise).
  StateMachineCore(bool real, std::vector<MissionWaypoint> waypoints, Hooks hooks);

  // pose_callback
  void set_pose(double north, double west, double yaw);
  // object_callback
  void on_object(double x, double y, double z);

  // control_loop(): returns false when all waypoints are reached (the node
  // then publishes 'mission_complete' on /autonomy/stop); otherwise runs
  // check_targets() and leaves the target to publish in target_position().
  bool control_loop();

  const Pt & target_position() const {return target_position_;}
  int current_waypoint_index() const {return current_waypoint_index_;}
  bool object_found() const {return object_found_;}
  const std::optional<std::string> & current_object() const {return current_object_;}
  const std::deque<Pt> & coarse_waypoints() const {return coarse_waypoints_;}
  const std::deque<Pt> & search_waypoints() const {return search_waypoints_;}
  const Pt & next_coordinate() const {return next_coordinate_;}

private:
  void update_search_params();
  void check_targets();
  void info(const std::string & s) const {if (hooks_.log) {hooks_.log(Level::Info, s);}}

  bool real_;
  std::vector<MissionWaypoint> waypoints_;
  Hooks hooks_;
  double initial_lat_, initial_lon_;
  std::map<int, std::vector<Pt>> coarse_paths_;
  std::deque<Pt> coarse_waypoints_;
  int current_waypoint_index_{1};
  double north_{0.0}, west_{0.0}, yaw_{0.0};
  // index -> (object, threshold, search radius)
  std::map<int, std::tuple<std::string, double, double>> targets_;
  std::optional<std::string> current_object_;
  std::optional<double> threshold_;
  std::optional<double> search_radius_;
  bool object_found_{false};
  Pt target_position_{0.0, 0.0};
  Pt next_coordinate_{0.0, 0.0};
  double ang_step_{30.0};
  double r_step_{0.1};
  std::deque<Pt> search_waypoints_;
};

}  // namespace autonomous_navigation_cpp
