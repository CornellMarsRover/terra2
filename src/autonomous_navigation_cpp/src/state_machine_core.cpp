#include "autonomous_navigation_cpp/state_machine_core.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "autonomous_navigation_cpp/py_compat.hpp"
#include "autonomous_navigation_cpp/target_contract.hpp"

namespace autonomous_navigation_cpp
{
namespace mission
{

TargetDecision far_target(
  const Pt & position, const Pt & goal, const Pt & current_target,
  const std::deque<Pt> & coarse_waypoints, bool pursuing_object, double coarse_threshold)
{
  TargetDecision d;
  if (pursuing_object) {
    d.target = current_target;
    return d;
  }
  d.pop_coarse = !coarse_waypoints.empty() &&
    py_dist(position, coarse_waypoints.front()) < coarse_threshold;
  const std::size_t first = d.pop_coarse ? 1 : 0;
  d.target = coarse_waypoints.size() > first ? coarse_waypoints[first] : goal;
  return d;
}

TargetDecision coordinate_target(const Pt & position, const Pt & goal, double threshold)
{
  TargetDecision d;
  d.target = goal;
  d.reached = py_dist(position, goal) < threshold;
  return d;
}

TargetDecision search_target(
  const Pt & position, const Pt & current_target, const std::deque<Pt> & search_waypoints,
  double threshold)
{
  TargetDecision d;
  if (search_waypoints.empty()) {
    d.target = current_target;
    d.timed_out = true;
    return d;
  }
  const Pt & next_target = search_waypoints.front();
  if (py_dist(position, next_target) >= threshold) {
    d.target = next_target;
    d.announce_object = true;
    return d;
  }
  d.pop_search = true;
  if (search_waypoints.size() == 1) {
    d.target = current_target;
    d.timed_out = true;
    return d;
  }
  d.target = search_waypoints[1];
  return d;
}

TargetDecision object_target(const Pt & position, const Pt & target, double threshold)
{
  TargetDecision d;
  d.target = target;
  d.reached = py_dist(position, target) < threshold;
  return d;
}

std::vector<Pt> search_waypoints(
  const Pt & center, double angle_step, double radius_step, double maximum_radius)
{
  if (radius_step <= 0.0 || !std::isfinite(radius_step)) {
    throw std::invalid_argument("radius_step must be finite and positive");
  }
  if (maximum_radius < 0.0 || !std::isfinite(maximum_radius)) {
    throw std::invalid_argument("maximum_radius must be finite and nonnegative");
  }
  std::vector<Pt> points;
  double radius = radius_step;
  double angle = 0.0;
  while (radius < maximum_radius) {
    if (angle > 180) {
      angle = -(360 - angle);
    }
    const double radians = py_radians(angle);
    const double dx = radius * (std::cos(radians) - std::sin(radians));
    const double dy = radius * (std::sin(radians) + std::sin(radians));
    points.push_back(Pt{center.x + dx, center.y + dy});
    angle += angle < 0 ? -angle_step : angle_step;
    radius += radius_step;
  }
  return points;
}

Pt north_west_meters(
  double origin_lat, double origin_lon, double target_lat, double target_lon,
  double earth_radius)
{
  const double latitude_1 = py_radians(origin_lat);
  const double latitude_2 = py_radians(target_lat);
  const double longitude_1 = py_radians(origin_lon);
  const double longitude_2 = py_radians(target_lon);
  const double north = (latitude_2 - latitude_1) * earth_radius;
  const double mean_latitude = 0.5 * (latitude_1 + latitude_2);
  const double west = -(longitude_2 - longitude_1) * earth_radius * std::cos(mean_latitude);
  return Pt{north, west};
}

TargetDecision select_target(
  const Pt & position, const Pt & goal, const Pt & current_target,
  const std::deque<Pt> & coarse_waypoints, const std::string & current_object,
  bool object_found, const std::deque<Pt> & search_points, double far_threshold,
  double coordinate_threshold)
{
  if (py_dist(position, goal) > far_threshold) {
    return far_target(
      position, goal, current_target, coarse_waypoints,
      current_object != "coordinate" && object_found);
  }
  if (current_object == "coordinate") {
    return coordinate_target(position, goal, coordinate_threshold);
  }
  if (!object_found) {
    return search_target(position, current_target, search_points);
  }
  return object_target(position, current_target);
}

}  // namespace mission

// ---------------------------------------------------------------------------
// StateMachineCore (state_machine.py)
// ---------------------------------------------------------------------------
StateMachineCore::StateMachineCore(
  bool real, std::vector<MissionWaypoint> waypoints, Hooks hooks)
: real_(real), waypoints_(std::move(waypoints)), hooks_(std::move(hooks))
{
  if (waypoints_.empty()) {
    throw std::invalid_argument("No waypoints loaded.");
  }
  // reference start coordinate
  initial_lat_ = waypoints_[0].latitude;
  initial_lon_ = waypoints_[0].longitude;
  info("Loaded " + std::to_string(waypoints_.size()) + " waypoints.");

  // Preplanned coarse paths (used on real hardware only)
  coarse_paths_[1] = {{0.06498901918530464, -0.0850945757701993},
    {3.86498901899904, 13.71490542421816}, {13.86498901899904, 23.71490542421816}};
  coarse_paths_[2] = {{20.664989018812776, 39.314905424194876},
    {30.664989018812776, 49.314905424194876}};
  coarse_paths_[3] = {{52.064989019185305, 58.71490542421816},
    {62.064989019185305, 68.71490542421816}, {72.0649890191853, 78.71490542421816},
    {82.0649890191853, 88.71490542421816}};
  coarse_paths_[4] = {{94.46498901862651, 104.71490542421816},
    {84.46498901862651, 114.71490542421816}, {74.46498901862651, 124.71490542421816},
    {64.46498901862651, 134.71490542421816}};
  coarse_paths_[5] = {{61.26498901844025, 144.31490542419488},
    {65.0649890191853, 158.11490542418323}};
  coarse_paths_[6] = {{69.86498901899904, 171.51490542420652},
    {56.26498901844025, 175.31490542419488}, {42.664989018812776, 179.11490542418323},
    {29.064989019185305, 182.9149054242298}, {15.464989018626511, 186.71490542421816}};
  coarse_paths_[7] = {{1.8649890189990401, 188.71490542421816},
    {-11.735010981559753, 185.11490542418323}, {-25.335010981187224, 181.51490542420652},
    {-38.935010980814695, 177.9149054242298}, {-52.53501098137349, 181.71490542421816},
    {-66.13501098100096, 178.11490542418323}};

  // Object-search configuration: index -> (object, threshold, search radius)
  targets_ = {
    {1, {"coordinate", 2.0, 10.0}},
    {2, {"coordinate", 2.0, 10.0}},
    {3, {"ar1", 2.0, 10.0}},
    {4, {"ar2", 2.0, 10.0}},
    {5, {"ar3", 2.0, 15.0}},
    {6, {"coordinate", 2.0, 10.0}},
    {7, {"coordinate", 2.0, 10.0}},
  };
  for (std::size_t index = 1; index < waypoints_.size(); ++index) {
    const MissionWaypoint & waypoint = waypoints_[index];
    if (waypoint.type && *waypoint.type == "coordinate") {
      targets_[static_cast<int>(index)] =
      {"coordinate", waypoint.threshold ? *waypoint.threshold : 2.0, 10.0};
    }
  }

  // Kick off with the first target
  info("Initializing first target & coarse segment");
  update_search_params();
}

void StateMachineCore::update_search_params()
{
  const auto target = targets_.find(current_waypoint_index_);
  if (current_waypoint_index_ >= static_cast<int>(waypoints_.size()) ||
    target == targets_.end())
  {
    // NOTE(port): a waypoint index with no targets entry (index > 7 and not
    // type 'coordinate' in the YAML) silently keeps the previous segment's
    // object/goal state, as in state_machine.py:update_search_params.
    return;
  }
  const std::string & obj = std::get<0>(target->second);
  const double thr = std::get<1>(target->second);
  const double sr = std::get<2>(target->second);
  current_object_ = obj;
  threshold_ = thr;
  search_radius_ = sr;
  object_found_ = false;
  search_waypoints_.clear();

  // load precomputed coarse path for this segment (if real)
  if (real_) {
    const auto coarse = coarse_paths_.find(current_waypoint_index_);
    coarse_waypoints_.clear();
    if (coarse != coarse_paths_.end()) {
      coarse_waypoints_.assign(coarse->second.begin(), coarse->second.end());
    }
    info("Loaded " + std::to_string(coarse_waypoints_.size()) +
      " coarse waypoints for segment #" + std::to_string(current_waypoint_index_));
  }

  // global-frame target position (north/west) of the GPS waypoint itself
  const MissionWaypoint & wp = waypoints_[static_cast<std::size_t>(current_waypoint_index_)];
  const Pt nw = mission::north_west_meters(
    initial_lat_, initial_lon_, wp.latitude, wp.longitude);
  next_coordinate_ = nw;
  if (obj != "coordinate") {
    const std::vector<Pt> pts = mission::search_waypoints(nw, ang_step_, r_step_, sr);
    search_waypoints_.assign(pts.begin(), pts.end());
    std::string s = "search waypoints: deque([";
    for (std::size_t i = 0; i < pts.size(); ++i) {
      s += (i ? ", " : "") + fmt_pt(pts[i]);
    }
    info(s + "])");
  }
  char buf[128];
  std::snprintf(buf, sizeof(buf), "[%.1f, %.1f]", nw.x, nw.y);
  info("Advancing to WP#" + std::to_string(current_waypoint_index_) + ": " + obj + " @ " +
    buf + " (thr=" + py_repr(thr) + ")");
}

bool StateMachineCore::control_loop()
{
  if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
    return false;
  }
  check_targets();
  return true;
}

void StateMachineCore::check_targets()
{
  const double dx2 = next_coordinate_.x - north_;
  const double dy2 = next_coordinate_.y - west_;
  const double d = std::sqrt((dx2 * dx2) + (dy2 * dy2));
  std::string waypoint_dist = "None";
  if (!coarse_waypoints_.empty()) {
    const double dx1 = coarse_waypoints_.front().x - north_;
    const double dy1 = coarse_waypoints_.front().y - west_;
    waypoint_dist = py_repr(std::sqrt((dx1 * dx1) + (dy1 * dy1)));
  }
  info("coordinate distance: " + py_repr(d) + "   waypoint distance: " + waypoint_dist +
    "   threshold: " + (threshold_ ? py_repr(*threshold_) : std::string("None")));

  // NOTE(port): current_object/threshold are always set by the time this
  // runs (index 1 always has a targets entry); the fallbacks are unreachable.
  const mission::TargetDecision decision = mission::select_target(
    Pt{north_, west_}, next_coordinate_, target_position_, coarse_waypoints_,
    current_object_ ? *current_object_ : std::string(), object_found_, search_waypoints_,
    10.0, threshold_ ? *threshold_ : 2.0);
  if (decision.pop_coarse) {
    coarse_waypoints_.pop_front();
  }
  if (decision.pop_search) {
    search_waypoints_.pop_front();
  }
  if (decision.announce_object && hooks_.publish_target_name) {
    hooks_.publish_target_name(target_contract::encode_target_request(
        current_waypoint_index_, current_object_ ? *current_object_ : std::string()));
  }
  target_position_ = decision.target;

  if (decision.reached || decision.timed_out) {
    if (decision.timed_out) {
      info("TARGET TIMEOUT");
    }
    if (decision.reached) {
      info("TARGET REACHED");
    }
    if (hooks_.blink_led) {
      hooks_.blink_led();
    }
    current_waypoint_index_ += 1;
    update_search_params();
  }
}

void StateMachineCore::set_pose(double north, double west, double yaw)
{
  north_ = north;
  west_ = west;
  yaw_ = yaw;
}

void StateMachineCore::on_object(double x, double y, double z)
{
  if (current_object_ && *current_object_ == "coordinate") {
    return;
  }
  // NOTE(port): Python's int(msg.linear.z) raises on NaN/inf and kills the
  // node; such detections are ignored here.
  if (!std::isfinite(z)) {
    return;
  }
  // int() truncates toward zero.
  if (std::trunc(z) != static_cast<double>(current_waypoint_index_)) {
    return;
  }
  object_found_ = true;
  target_position_ = Pt{x, y};
  info("Found " + (current_object_ ? *current_object_ : std::string("None")) + " @ [" +
    py_repr(x) + ", " + py_repr(y) + "]");
}

}  // namespace autonomous_navigation_cpp
