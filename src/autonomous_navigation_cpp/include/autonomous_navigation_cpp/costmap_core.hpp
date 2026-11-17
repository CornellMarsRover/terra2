// ROS-free core of the costmap node: port of autonomous_navigation/costmap_core.py
// (project_point / observed_cost / decay_costs) plus the grid bookkeeping of
// autonomous_navigation/costmap.py (CostmapCore).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{

struct XYZ
{
  float x;
  float y;
  float z;
};

// ---------------------------------------------------------------------------
// costmap_core.py
// ---------------------------------------------------------------------------
struct GridObservation
{
  double cell_x;
  double cell_y;
  double height;
};

// costmap_core.py:project_point. `rotation` is row-major [[r00, r01], [r10, r11]].
// NOTE(port): Python raises (ValueError / OverflowError from round()) on a
// non-finite projected coordinate, which kills the node; here such points
// yield no observation.
inline std::optional<GridObservation> project_point(
  double p0, double p1, double p2, double pose_n, double pose_w,
  const double rotation[4], bool real, double camera_height, double min_depth,
  double max_depth, double cell_size, double max_bearing_deg = 55.0)
{
  double forward, left, height;
  if (real) {
    forward = p0;
    left = p1;
    height = camera_height + p2;
    const double distance = py_hypot(forward, left);
    const double bearing = py_degrees(std::atan2(left, forward));
    if (distance > max_depth || distance < min_depth || std::fabs(bearing) > max_bearing_deg) {
      return std::nullopt;
    }
  } else {
    forward = p2;
    left = p0;
    height = camera_height - p1;
    if (forward > max_depth || forward < min_depth) {
      return std::nullopt;
    }
  }
  const double rotated_forward = rotation[0] * forward + rotation[1] * left;
  const double rotated_left = rotation[2] * forward + rotation[3] * left;
  const double global_forward = rotated_forward + pose_n;
  const double global_left = (real ? rotated_left : -rotated_left) + pose_w;
  const double qx = py_round(global_forward / cell_size);
  const double qy = py_round(global_left / cell_size);
  if (!std::isfinite(qx) || !std::isfinite(qy)) {
    return std::nullopt;
  }
  return GridObservation{qx * cell_size, qy * cell_size, height};
}

// costmap_core.py:observed_cost. nullopt == traversable (Python None).
inline std::optional<int> observed_cost(
  int current, double height, double ground_threshold, double obstacle_threshold,
  double clearance_height, int maximum)
{
  if ((ground_threshold < height && height < obstacle_threshold) || height > clearance_height) {
    return std::nullopt;
  }
  const int increment = height > 0.5 ? 7 : 2;
  const int next = current + increment;
  return maximum < next ? maximum : next;
}

using GridMap = std::unordered_map<int64_t, int>;

// In-place form of costmap_core.py:decay_costs. Cells for which eligible(key)
// is false keep their cost; every cell whose resulting cost is <= 0 is dropped
// (including ineligible ones, as in Python).
template<class Eligible>
void decay_costs_inplace(GridMap & costs, int amount, Eligible && eligible)
{
  if (amount < 0) {
    throw std::invalid_argument("decay amount cannot be negative");
  }
  for (auto it = costs.begin(); it != costs.end(); ) {
    if (eligible(it->first)) {
      const int v = it->second - amount;
      it->second = v > 0 ? v : 0;
    }
    if (it->second > 0) {
      ++it;
    } else {
      it = costs.erase(it);
    }
  }
}

// costmap_core.py:decay_costs (returns a decayed copy; eligible == nullptr
// means every cell).
inline GridMap decay_costs(
  const GridMap & costs, int amount, const std::unordered_set<int64_t> * eligible = nullptr)
{
  if (amount < 0) {
    throw std::invalid_argument("decay amount cannot be negative");
  }
  GridMap out = costs;
  decay_costs_inplace(out, amount, [eligible](int64_t key) {
      return eligible == nullptr || eligible->count(key) != 0;
    });
  return out;
}

// Minimal stand-in for shapely Polygon(...).contains(Point(x, y)):
// strict interior test (points on the boundary are NOT contained), even-odd
// rule (which is what GEOS' ray-crossing counter does for invalid rings too).
class Polygon2D
{
public:
  Polygon2D() = default;
  Polygon2D(std::vector<double> xs, std::vector<double> ys);
  bool contains(double px, double py) const;
  std::size_t size() const {return xs_.size();}

private:
  std::vector<double> xs_, ys_;
  double minx_{0}, maxx_{0}, miny_{0}, maxy_{0};
};

class CostmapCore
{
public:
  explicit CostmapCore(bool real);

  // costmap.py:object_callback
  void set_object_position(double x, double y);
  // costmap.py:pointcloud_callback min_depth switch
  void set_last_movement_point_turn(bool point_turn) {min_depth_ = point_turn ? 0.3 : 1.2;}

  // costmap.py:pointcloud_callback / point_cloud_point_to_grid for a whole cloud.
  // (north, west) is the pose, (c, s) = (cos, sin) of the rotation angle used to
  // build R = [[c, -s], [s, c]] (yaw in real mode, -yaw in sim). Produces
  // exactly what calling project_point()/observed_cost() per point would.
  void process_cloud(const XYZ * pts, std::size_t n, double north, double west, double c,
    double s);

  // costmap.py:ground_plane_callback (vertices already rotated + translated
  // into the global frame). Returns false (and changes nothing) if the polygon
  // would have made shapely raise (1 or 2 vertices).
  bool set_ground_plane(std::vector<double> xs, std::vector<double> ys);

  void decay_ground();
  void decay_cost();

  // costmap.py:publish_obstacles payload: [x, y, cost, ...] for cost > 0.
  void collect(std::vector<float> & out) const;

  bool in_ground_plane(int32_t ix, int32_t iy);

  // Direct access for tests.
  const GridMap & grid() const {return grid_;}
  int cost_at(double x, double y) const;
  void set_cost(int32_t ix, int32_t iy, int c) {grid_[cell_key_public(ix, iy)] = c;}
  static int64_t cell_key_public(int32_t ix, int32_t iy);

  bool real() const {return real_;}

private:
  enum : uint8_t
  {
    kTouched = 1, kObst2 = 2, kObst7 = 4, kGroundKnown = 8, kGroundIn = 16
  };
  struct PtCell
  {
    int32_t ix;
    int32_t iy;
    uint8_t cls;   // 0 = free space, 2 = +2, 7 = +7
  };

  template<class StateAt>
  void run_cells(StateAt && state_at);

  bool real_;
  int max_cost_{100};
  double obstacle_threshold_;
  double ground_threshold_;
  double camera_height_{1.0};
  double clearance_height_{2.0};
  double max_depth_{6.0};
  double min_depth_{1.2};
  double cell_size_{0.25};
  double k_{4.0};   // 1 / cell_size_ (exact)
  double max_bearing_deg_{55.0};

  GridMap grid_;
  bool has_ground_{false};
  Polygon2D ground_;
  std::unordered_map<int64_t, bool> ground_cache_;

  // Reused per-cloud buffers (no per-point allocation).
  std::vector<PtCell> cells_;
  std::vector<uint8_t> window_;
  std::unordered_map<int64_t, uint8_t> sparse_window_;
};

}  // namespace autonomous_navigation_cpp
