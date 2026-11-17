#include "autonomous_navigation_cpp/planner_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace autonomous_navigation_cpp
{

std::string fmt_pt(const Pt & p) {return "(" + py_repr(p.x) + ", " + py_repr(p.y) + ")";}

// ---------------------------------------------------------------------------
// CostLookup
// ---------------------------------------------------------------------------
bool CostLookup::make_key(double x, double y, Key & k)
{
  if (std::isnan(x) || std::isnan(y)) {
    return false;
  }
  x += 0.0;  // -0.0 -> +0.0 (equal dict keys in Python)
  y += 0.0;
  std::memcpy(&k.x, &x, sizeof(double));
  std::memcpy(&k.y, &y, sizeof(double));
  return true;
}

void CostLookup::set(double x, double y, double value)
{
  Key k;
  if (make_key(x, y, k)) {
    map_[k] = value;
  }
}

const double * CostLookup::find(double x, double y) const
{
  Key k;
  if (!make_key(x, y, k)) {
    return nullptr;
  }
  const auto it = map_.find(k);
  return it == map_.end() ? nullptr : &it->second;
}

double CostLookup::get(double x, double y) const
{
  const double * v = find(x, y);
  return v ? *v : 0.0;
}

namespace planner
{

PlanarTarget parse_planar_target(const std::vector<float> & data)
{
  if (data.size() < 2) {
    throw std::invalid_argument("target must contain x and y");
  }
  const std::size_t n = std::min<std::size_t>(data.size(), 3);
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isfinite(data[i])) {
      throw std::invalid_argument("target values must be finite");
    }
  }
  PlanarTarget out;
  out.target = Pt{static_cast<double>(data[0]), static_cast<double>(data[1])};
  if (n == 3) {
    out.yaw = static_cast<double>(data[2]);
  }
  return out;
}

std::pair<SegmentCounts, bool> record_segment_observation(
  const SegmentCounts & counts, Segment segment, bool blocked, int confirmations)
{
  if (confirmations < 1) {
    throw std::invalid_argument("confirmations must be positive");
  }
  SegmentCounts updated = counts;
  if (!blocked) {
    updated.erase(segment);
    return {updated, false};
  }
  const int count = ++updated[segment];
  return {updated, count >= confirmations};
}

Pt nearest_clear_goal(
  const Pt & goal, const Bounds & b, const std::function<bool(const Pt &)> & is_blocked,
  double step)
{
  if (step <= 0.0) {
    throw std::invalid_argument("step must be positive");
  }
  const Pt origin{
    py_max(py_min(goal.x, b.max_x), b.min_x),
    py_max(py_min(goal.y, b.max_y), b.min_y)};
  const double span = py_max(b.max_x - b.min_x, b.max_y - b.min_y);
  const double max_radius_f = std::ceil(span / step);
  // NOTE(port): Python raises on a NaN/inf radius and would loop for a very
  // long time on a huge one; fall back to the clamped goal instead.
  if (!(max_radius_f < 1e6)) {
    return origin;
  }
  const long max_radius = static_cast<long>(max_radius_f);
  struct Candidate
  {
    double distance;
    Pt point;
  };
  std::vector<Candidate> ring;
  for (long radius = 0; radius <= max_radius; ++radius) {
    ring.clear();
    for (long dx = -radius; dx <= radius; ++dx) {
      for (long dy = -radius; dy <= radius; ++dy) {
        if (std::max(std::labs(dx), std::labs(dy)) != radius) {
          continue;
        }
        const Pt p{origin.x + static_cast<double>(dx) * step,
          origin.y + static_cast<double>(dy) * step};
        ring.push_back({py_dist(origin, p), p});
      }
    }
    // sorted(key=(dist, point)); stable like Python's sort.
    std::stable_sort(ring.begin(), ring.end(), [](const Candidate & l, const Candidate & r) {
        return l.distance < r.distance || (l.distance == r.distance && l.point < r.point);
      });
    for (const Candidate & c : ring) {
      const bool in_bounds = b.min_x <= c.point.x && c.point.x <= b.max_x &&
        b.min_y <= c.point.y && c.point.y <= b.max_y;
      if (in_bounds && !is_blocked(c.point)) {
        return c.point;
      }
    }
  }
  return origin;
}

ParsedCostmap parse_costmap(const std::vector<float> & data, double threshold, bool collect_obstacles)
{
  if (data.size() % 3) {
    throw std::invalid_argument("costmap data must contain x, y, cost triples");
  }
  ParsedCostmap out;
  for (std::size_t index = 0; index < data.size(); index += 3) {
    const double x = data[index], y = data[index + 1], cost = data[index + 2];
    out.costs.set(x, y, cost);
    if (collect_obstacles && cost > threshold) {
      out.obstacles.emplace(x, y);
    }
  }
  return out;
}

Advance advance_path(
  const Pt & position, const Pt & target, const std::deque<Pt> & path, const Pt & waypoint,
  double threshold)
{
  Advance out;
  out.path.assign(path.begin(), path.end());
  if (py_dist(position, waypoint) >= threshold) {
    out.next_waypoint = waypoint;
    return out;
  }
  out.reached = waypoint;
  if (!out.path.empty() && out.path.front() == waypoint) {
    out.path.erase(out.path.begin());
  }
  out.next_waypoint = out.path.empty() ? target : out.path.front();
  return out;
}

bool path_is_dense(const std::deque<Pt> & path, double distance_threshold)
{
  for (std::size_t index = 0; index + 2 < path.size(); ++index) {
    if (py_dist(path[index], path[index + 1]) < distance_threshold &&
      py_dist(path[index + 1], path[index + 2]) < distance_threshold)
    {
      return true;
    }
  }
  return false;
}

double perpendicular_distance(const Pt & point, const Pt & start, const Pt & end)
{
  if (start == end) {
    return py_dist(point, start);
  }
  const double x0 = point.x, y0 = point.y;
  const double x1 = start.x, y1 = start.y;
  const double x2 = end.x, y2 = end.y;
  const double numerator = std::fabs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  return numerator / py_hypot(y2 - y1, x2 - x1);
}

std::vector<Pt> simplify_path(const std::vector<Pt> & points, double epsilon)
{
  if (points.size() < 3) {
    return points;
  }
  // max(distances) + distances.index(max): first maximal element.
  double max_distance = 0.0;
  std::size_t split = 1;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double d = perpendicular_distance(points[i], points.front(), points.back());
    if (i == 1 || d > max_distance) {
      max_distance = d;
      split = i;
    }
  }
  if (max_distance <= epsilon) {
    return {points.front(), points.back()};
  }
  std::vector<Pt> left = simplify_path(
    std::vector<Pt>(points.begin(), points.begin() + static_cast<long>(split) + 1), epsilon);
  const std::vector<Pt> right = simplify_path(
    std::vector<Pt>(points.begin() + static_cast<long>(split), points.end()), epsilon);
  left.pop_back();
  left.insert(left.end(), right.begin(), right.end());
  return left;
}

double neighbor_cost(const CostLookup & costs, const Pt & cell, double cell_size, int radius)
{
  if (cell_size <= 0.0) {
    throw std::invalid_argument("cell_size must be positive");
  }
  if (radius < 0) {
    throw std::invalid_argument("radius must not be negative");
  }
  double total = 0.0;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double ox = dx * cell_size, oy = dy * cell_size;
      const double distance = py_hypot(ox, oy);
      const double cost = costs.get(cell.x + ox, cell.y + oy);
      // NOTE(port): a cost of exactly 1 is treated as 0 (planner_core.py).
      total += (cost == 1 ? 0.0 : cost) / distance;
    }
  }
  return total;
}

std::pair<double, double> segment_cost(
  const CostLookup & costs, const Pt & start, const Pt & end, double cell_size,
  int neighbor_radius)
{
  return segment_cost_with(start, end, cell_size, [&](const Pt & cell) {
             return costs.get(cell.x, cell.y) +
                    neighbor_cost(costs, cell, cell_size, neighbor_radius);
           });
}

std::optional<int> next_safety_gap(int current, int minimum)
{
  if (minimum < 0) {
    throw std::invalid_argument("minimum safety gap must not be negative");
  }
  if (current <= minimum) {
    return std::nullopt;
  }
  const int candidate = current - 1;
  if (candidate >= minimum) {
    return candidate;
  }
  return std::nullopt;
}

}  // namespace planner
}  // namespace autonomous_navigation_cpp
