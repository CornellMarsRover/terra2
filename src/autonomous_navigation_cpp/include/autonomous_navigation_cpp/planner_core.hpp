// Port of autonomous_navigation/planner_core.py: ROS-independent
// local-planning decisions. Python ValueError -> std::invalid_argument.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{

struct Pt
{
  double x{0.0};
  double y{0.0};
  bool operator==(const Pt & o) const {return x == o.x && y == o.y;}
  bool operator!=(const Pt & o) const {return !(*this == o);}
  // Python tuple ordering.
  bool operator<(const Pt & o) const {return x < o.x || (x == o.x && y < o.y);}
};

inline double py_dist(const Pt & a, const Pt & b) {return py_hypot(a.x - b.x, a.y - b.y);}

// repr((x, y))
std::string fmt_pt(const Pt & p);

// Python dict {(x, y): value} keyed on exact float values (0.0 == -0.0;
// NaN keys can never be looked up again, so they are not stored).
class CostLookup
{
public:
  void clear() {map_.clear();}
  void set(double x, double y, double value);
  // dict.get((x, y), 0.0)
  double get(double x, double y) const;
  // nullptr when absent
  const double * find(double x, double y) const;
  std::size_t size() const {return map_.size();}
  bool empty() const {return map_.empty();}

private:
  struct Key
  {
    uint64_t x, y;
    bool operator==(const Key & o) const {return x == o.x && y == o.y;}
  };
  struct Hash
  {
    std::size_t operator()(const Key & k) const
    {
      uint64_t h = k.x * 0x9E3779B97F4A7C15ULL;
      h ^= (h >> 32);
      h += k.y * 0xC2B2AE3D27D4EB4FULL;
      h ^= (h >> 29);
      return static_cast<std::size_t>(h);
    }
  };
  static bool make_key(double x, double y, Key & k);
  std::unordered_map<Key, double, Hash> map_;
};

namespace planner
{

struct PlanarTarget
{
  Pt target;
  std::optional<double> yaw;
};
// Parse a finite x/y target and optional yaw from a ROS array payload.
PlanarTarget parse_planar_target(const std::vector<float> & data);

using Segment = std::pair<int, int>;
using SegmentCounts = std::map<Segment, int>;
// Track consecutive blocked observations and report confirmed blockage.
std::pair<SegmentCounts, bool> record_segment_observation(
  const SegmentCounts & counts, Segment segment, bool blocked, int confirmations = 3);

struct Bounds
{
  double min_x, max_x, min_y, max_y;
};
// Return the nearest in-bounds grid point that is not blocked.
Pt nearest_clear_goal(
  const Pt & goal, const Bounds & bounds, const std::function<bool(const Pt &)> & is_blocked,
  double step);

struct ParsedCostmap
{
  CostLookup costs;
  std::set<std::pair<double, double>> obstacles;
};
// Return cell costs and currently occupied cells from xyz-style triples.
ParsedCostmap parse_costmap(
  const std::vector<float> & data, double threshold, bool collect_obstacles = true);

struct Advance
{
  std::vector<Pt> path;
  Pt next_waypoint;
  std::optional<Pt> reached;
};
// Advance one reached waypoint and return path, next waypoint, and reached.
Advance advance_path(
  const Pt & position, const Pt & target, const std::deque<Pt> & path, const Pt & waypoint,
  double threshold);

// Return whether any three consecutive points form two short segments.
bool path_is_dense(const std::deque<Pt> & path, double distance_threshold = 1.0);

// Calculate a point's perpendicular distance from an infinite line.
double perpendicular_distance(const Pt & point, const Pt & start, const Pt & end);

// Simplify a path with the Ramer-Douglas-Peucker algorithm.
std::vector<Pt> simplify_path(const std::vector<Pt> & points, double epsilon);

// Return inverse-distance-weighted costs around a grid cell.
double neighbor_cost(const CostLookup & costs, const Pt & cell, double cell_size, int radius);

// Treat above-threshold segments as impassable, not merely expensive.
inline bool segment_traversable(double maximum_cost, double threshold)
{
  return std::isfinite(maximum_cost) && maximum_cost <= threshold;
}

// Allow safe motion or a strict risk-reducing escape from inflation.
inline bool transition_traversable(
  double maximum_cost, double start_risk, double end_risk, double threshold)
{
  return segment_traversable(maximum_cost, threshold) ||
         (std::isfinite(start_risk) && std::isfinite(end_risk) && start_risk > threshold &&
         end_risk < start_risk);
}

// round(value / cell_size) * cell_size for both coordinates.
inline Pt snap_cell(const Pt & p, double cell_size)
{
  return Pt{py_round(p.x / cell_size) * cell_size, py_round(p.y / cell_size) * cell_size};
}

// Shared sampling loop of segment_cost(); risk(cell) must return
// costs.get(cell, 0.0) + neighbor_cost(costs, cell, cell_size, radius).
template<class Risk>
std::pair<double, double> segment_cost_with(
  const Pt & start, const Pt & end, double cell_size, Risk && risk)
{
  const double distance = py_dist(start, end);
  if (distance < 1e-6) {
    return {0.0, 0.0};
  }
  const double ratio_count = py_round(distance / (cell_size * 2));
  if (!(ratio_count < 1e7)) {
    // NOTE(port): Python raises (round(nan/inf)) or effectively hangs on an
    // absurdly long segment; report it as impassable instead.
    const double inf = std::numeric_limits<double>::infinity();
    return {inf, inf};
  }
  const long samples = ratio_count > 1.0 ? static_cast<long>(ratio_count) : 1;
  double maximum = 0.0, total = 0.0;
  for (long index = 0; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(samples);
    const Pt point{start.x + ratio * (end.x - start.x), start.y + ratio * (end.y - start.y)};
    const double value = risk(snap_cell(point, cell_size));
    // max(values): first element, then replaced only by a strictly greater
    // one; sum(values): plain left-to-right float addition (CPython 3.10).
    if (index == 0 || value > maximum) {
      maximum = value;
    }
    total = (index == 0) ? value : total + value;
  }
  return {maximum, total};
}

// Return maximum and accumulated sampled cost along a segment.
std::pair<double, double> segment_cost(
  const CostLookup & costs, const Pt & start, const Pt & end, double cell_size = 0.25,
  int neighbor_radius = 4);

// Return the next smaller inflation radius without violating the minimum.
std::optional<int> next_safety_gap(int current, int minimum);

}  // namespace planner
}  // namespace autonomous_navigation_cpp
