// ROS-free state + algorithms of autonomous_navigation/local_planner.py
// (LocalPlannerNode), built on planner_core.
#pragma once

#include <array>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autonomous_navigation_cpp/planner_core.hpp"

namespace autonomous_navigation_cpp
{

class LocalPlannerCore
{
public:
  enum class Level { Info, Warn, Error };
  using LogFn = std::function<void (Level, const std::string &)>;

  explicit LocalPlannerCore(bool real, LogFn log = nullptr);

  // Callbacks
  void on_costmap(const std::vector<float> & data);        // new_obstacle_callback
  void on_target(const std::vector<float> & data);         // next_target_callback
  // update_pose. Returns false (state untouched) for a non-finite pose.
  bool on_pose(double x, double y, double yaw);

  // Timers
  void validate_path();
  // publish_waypoint(): the next_waypoint payload [x, y, 1.0, len(path)];
  // `plan` receives the flattened /autonomy/path/plan payload.
  std::array<float, 4> make_waypoint(std::vector<float> * plan = nullptr);

  // Algorithms
  double get_neighbor_costs(double rx, double ry, double weight = 1, int n = 1) const;
  double point_risk(const Pt & point, int gap = 4) const;
  std::pair<double, double> compute_segment_cost(const Pt & start, const Pt & end, int gap = 4) const;
  void compute_path(int gap = 4);
  void smooth_path();

  // Cost map access (mutations invalidate the per-cell risk cache).
  const CostLookup & costs() const {return costs_;}
  void set_cost(double x, double y, double cost);
  void clear_costs();

  // State (public for the node wrapper and tests)
  Pt robot_position{0.0, 0.0};
  double yaw{0.0};
  Pt next_target{0.0, 0.0};          // never None in the Python node
  std::optional<double> target_yaw;
  std::deque<Pt> current_path;
  std::optional<Pt> next_waypoint;
  planner::SegmentCounts invalidated_segments;
  bool path_hold{false};
  bool use_stanley{false};

  // Parameters
  double max_cell_threshold{20.0};
  double distance_weight{0.1};
  double cost_weight{1.0};
  double threshold{4};
  double cell_size{0.25};
  double waypoint_threshold{0.3};

private:
  // costs.get(cell, 0.0) + neighbor_cost(costs, cell, cell_size, gap) for an
  // already snapped cell; memoized per gap until the cost map changes.
  double cell_risk(const Pt & cell, int gap) const;
  void log(Level l, const std::string & s) const {if (log_) {log_(l, s);}}

  CostLookup costs_;
  mutable std::array<CostLookup, 2> risk_cache_;   // [0]: gap 4, [1]: gap 3
  LogFn log_;
};

}  // namespace autonomous_navigation_cpp
