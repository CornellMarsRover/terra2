#include "autonomous_navigation_cpp/local_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{

using planner::segment_traversable;
using planner::transition_traversable;

LocalPlannerCore::LocalPlannerCore(bool real, LogFn log)
: log_(std::move(log))
{
  max_cell_threshold = 20.0;
  if (!real) {
    max_cell_threshold = 20.0;
  }
}

void LocalPlannerCore::set_cost(double x, double y, double cost)
{
  costs_.set(x, y, cost);
  risk_cache_[0].clear();
  risk_cache_[1].clear();
}

void LocalPlannerCore::clear_costs()
{
  costs_.clear();
  risk_cache_[0].clear();
  risk_cache_[1].clear();
}

void LocalPlannerCore::on_costmap(const std::vector<float> & data)
{
  try {
    // NOTE(port): the `obstacles` set (cost > threshold) is never read in
    // local_planner.py, so it is not built here.
    costs_ = std::move(planner::parse_costmap(data, threshold, false).costs);
    risk_cache_[0].clear();
    risk_cache_[1].clear();
  } catch (const std::invalid_argument & error) {
    log(Level::Error, std::string("Ignoring malformed costmap: ") + error.what());
  }
}

void LocalPlannerCore::on_target(const std::vector<float> & data)
{
  Pt new_target;
  try {
    const planner::PlanarTarget parsed = planner::parse_planar_target(data);
    new_target = parsed.target;
    target_yaw = parsed.yaw;
  } catch (const std::invalid_argument & error) {
    log(Level::Error, std::string("Ignoring malformed target: ") + error.what());
    return;
  }
  if (next_target != new_target) {
    next_target = new_target;
    current_path.clear();
    current_path.push_back(next_target);
    next_waypoint = next_target;
    log(Level::Info, "New next target: " + fmt_pt(next_target));
  }
}

bool LocalPlannerCore::on_pose(double x, double y, double yaw_in)
{
  // NOTE(port): Python has no check here; a NaN pose counts as "waypoint
  // reached" in advance_path and then crashes the node in validate_path
  // (round(nan)). Non-finite poses are dropped instead.
  if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw_in))) {
    return false;
  }
  robot_position = {x, y};
  yaw = yaw_in;
  if (next_waypoint) {
    planner::Advance adv = planner::advance_path(
      robot_position, next_target, current_path, *next_waypoint, waypoint_threshold);
    if (adv.reached) {
      invalidated_segments.clear();
      current_path.assign(adv.path.begin(), adv.path.end());
      next_waypoint = adv.next_waypoint;
      // NOTE(port): Python also appends to self.previous_points, a list that
      // is never read and grows without bound; not kept.
    }
  }
  return true;
}

void LocalPlannerCore::validate_path()
{
  if (current_path.empty()) {
    return;
  }
  bool current_segment_blocked = false;
  std::vector<Pt> path_points;
  path_points.reserve(current_path.size() + 1);
  path_points.push_back(robot_position);
  path_points.insert(path_points.end(), current_path.begin(), current_path.end());

  for (std::size_t i = 0; i + 1 < path_points.size(); ++i) {
    const Pt start_pt = path_points[i];
    const Pt end_pt = path_points[i + 1];
    const double max_cell = compute_segment_cost(start_pt, end_pt, 4).first;
    const bool blocked = !transition_traversable(
      max_cell, point_risk(start_pt), point_risk(end_pt), max_cell_threshold);
    const planner::Segment segment{static_cast<int>(i), static_cast<int>(i) + 1};
    if (blocked) {
      current_segment_blocked = current_segment_blocked || i == 0;
      bool confirmed;
      std::tie(invalidated_segments, confirmed) =
        planner::record_segment_observation(invalidated_segments, segment, true);
      if (confirmed) {
        log(Level::Info, "Segment from " + fmt_pt(start_pt) + " to " + fmt_pt(end_pt) +
          " above threshold with cost " + py_repr(max_cell) + ". Re-planning from " +
          fmt_pt(start_pt) + " to final target.");
        // Keep the valid path portion up to 'i'
        std::deque<Pt> valid_portion(path_points.begin() + 1,
          path_points.begin() + static_cast<long>(i) + 1);
        // Temporarily override robot_position so compute_path() starts from start_pt
        const Pt old_robot_position = robot_position;
        robot_position = start_pt;
        // NOTE(port): if compute_path() fails, current_path still holds the
        // old full path, so the valid portion ends up duplicated in front of
        // it (local_planner.py:validate_path). Preserved.
        compute_path();
        smooth_path();
        robot_position = old_robot_position;
        valid_portion.insert(valid_portion.end(), current_path.begin(), current_path.end());
        current_path = std::move(valid_portion);
        if (current_path.empty() || current_path.back() != next_target) {
          current_path.push_back(next_target);
        }
        next_waypoint = current_path.front();
        break;  // only re-plan once per validation cycle
      }
    } else {
      invalidated_segments =
        planner::record_segment_observation(invalidated_segments, segment, false).first;
    }
  }
  path_hold = current_segment_blocked;
}

std::array<float, 4> LocalPlannerCore::make_waypoint(std::vector<float> * plan)
{
  if (!next_waypoint) {
    next_waypoint = next_target;
  }
  if (current_path.empty() || current_path.back() != next_target) {
    current_path.push_back(next_target);
  }
  use_stanley = planner::path_is_dense(current_path);
  const double use_stanley_flag = 1.0;  // Python hard-codes 1.0
  // While the first segment is blocked, command the robot's own position.
  const Pt commanded = path_hold ? robot_position : *next_waypoint;
  if (plan) {
    plan->clear();
    plan->reserve(current_path.size() * 2);
    for (const Pt & p : current_path) {
      plan->push_back(static_cast<float>(p.x));
      plan->push_back(static_cast<float>(p.y));
    }
  }
  return std::array<float, 4>{
    static_cast<float>(commanded.x), static_cast<float>(commanded.y),
    static_cast<float>(use_stanley_flag), static_cast<float>(current_path.size())};
}

double LocalPlannerCore::get_neighbor_costs(double rx, double ry, double weight, int n) const
{
  return weight * planner::neighbor_cost(costs_, Pt{rx, ry}, cell_size, n);
}

double LocalPlannerCore::cell_risk(const Pt & cell, int gap) const
{
  CostLookup * cache = gap == 4 ? &risk_cache_[0] : (gap == 3 ? &risk_cache_[1] : nullptr);
  if (cache) {
    if (const double * hit = cache->find(cell.x, cell.y)) {
      return *hit;
    }
  }
  const double value = costs_.get(cell.x, cell.y) + get_neighbor_costs(cell.x, cell.y, 1, gap);
  if (cache) {
    cache->set(cell.x, cell.y, value);
  }
  return value;
}

double LocalPlannerCore::point_risk(const Pt & point, int gap) const
{
  return cell_risk(planner::snap_cell(point, cell_size), gap);
}

std::pair<double, double> LocalPlannerCore::compute_segment_cost(
  const Pt & start, const Pt & end, int gap) const
{
  return planner::segment_cost_with(start, end, cell_size, [&](const Pt & cell) {
             return cell_risk(cell, gap);
           });
}

void LocalPlannerCore::compute_path(int gap)
{
  // Define local search region
  const Pt start = robot_position;
  const double half_width = 8.0;
  const double step_size = 0.5;
  const double min_x = start.x - half_width - 1.0;
  const double max_x = start.x + half_width + 1.0;
  const double min_y = start.y - half_width - 1.0;
  const double max_y = start.y + half_width + 1.0;

  // len(np.arange(lo, hi, step)) == ceil((hi - lo) / step)
  auto arange_len = [](double lo, double hi, double step) -> long {
      const double l = std::ceil((hi - lo) / step);
      return (l > 0 && l < 1e9) ? static_cast<long>(l) : 0;
    };
  const long nx = arange_len(min_x, max_x + step_size, step_size);
  const long ny = arange_len(min_y, max_y + step_size, step_size);

  const Pt goal_clamped = planner::nearest_clear_goal(
    next_target, planner::Bounds{min_x, max_x, min_y, max_y},
    // NOTE(port): candidates are not snapped to the 0.25 m grid, so these
    // lookups only hit when the (clamped) goal lies on it. Preserved.
    [this](const Pt & point) {
      return get_neighbor_costs(point.x, point.y, 1, 3) > max_cell_threshold * 2;
    },
    step_size);

  // int(round(v)); -1 for anything that cannot be a valid index.
  auto to_index = [](double v) -> long {
      const double r = py_round(v);
      return (r >= 0 && r < 1e9) ? static_cast<long>(r) : -1;
    };
  auto get_coords = [&](long i, long j) {
      return Pt{min_x + static_cast<double>(i) * step_size,
               min_y + static_cast<double>(j) * step_size};
    };

  const long si = to_index((start.x - min_x) / step_size);
  const long sj = to_index((start.y - min_y) / step_size);
  const long gi = to_index((goal_clamped.x - min_x) / step_size);
  const long gj = to_index((goal_clamped.y - min_y) / step_size);
  if (!(0 <= si && si < nx && 0 <= sj && sj < ny)) {
    log(Level::Warn, "Start out of local region. Cannot plan.");
    return;
  }
  if (!(0 <= gi && gi < nx && 0 <= gj && gj < ny)) {
    log(Level::Warn, "Goal out of local region. Cannot plan.");
    return;
  }

  const std::size_t N = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
  auto flat = [ny](long i, long j) {return static_cast<std::size_t>(i * ny + j);};
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(N, inf);
  std::vector<long> came_from(N, -1);

  // heapq of (f, (i, j)): ties on f are broken by (i, j) lexicographically.
  using Entry = std::tuple<double, long, long>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_set;
  open_set.emplace(0.0, si, sj);
  g_score[flat(si, sj)] = 0.0;

  static const int deltas[8][2] = {
    {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

  const Pt goal_coords = get_coords(gi, gj);
  bool found_path = false;
  while (!open_set.empty()) {
    const long ci = std::get<1>(open_set.top());
    const long cj = std::get<2>(open_set.top());
    open_set.pop();
    if (ci == gi && cj == gj) {
      found_path = true;
      break;
    }
    // As in Python, stale heap entries are not skipped: the latest g is used.
    const double cur_g = g_score[flat(ci, cj)];
    const Pt cur_coords = get_coords(ci, cj);
    for (const auto & dn : deltas) {
      const long ni = ci + dn[0];
      const long nj = cj + dn[1];
      if (!(0 <= ni && ni < nx && 0 <= nj && nj < ny)) {
        continue;
      }
      const Pt nbr_coords = get_coords(ni, nj);
      const double seg_cost = compute_segment_cost(cur_coords, nbr_coords, gap).first;
      if (!transition_traversable(
          seg_cost, point_risk(cur_coords, gap), point_risk(nbr_coords, gap),
          max_cell_threshold))
      {
        continue;
      }
      const double step_distance = py_dist(cur_coords, nbr_coords);
      const double travel_cost = (distance_weight * step_distance) + (cost_weight * seg_cost);
      const double tentative_g = cur_g + travel_cost;
      const std::size_t nf = flat(ni, nj);
      if (tentative_g < g_score[nf]) {
        came_from[nf] = static_cast<long>(flat(ci, cj));
        g_score[nf] = tentative_g;
        const double f_score = tentative_g + py_dist(nbr_coords, goal_coords) * distance_weight;
        open_set.emplace(f_score, ni, nj);
      }
    }
  }

  if (!found_path) {
    const std::optional<int> reduced_gap = planner::next_safety_gap(gap, 3);
    if (reduced_gap) {
      log(Level::Info, "No path found. Trying reduced gap for cost sampling.");
      compute_path(*reduced_gap);
    } else {
      log(Level::Error, "A* found no route at the minimum safe clearance.");
    }
    return;
  }

  // Reconstruct path
  std::vector<std::size_t> path_indices;
  std::size_t cur = flat(gi, gj);
  while (came_from[cur] >= 0) {
    path_indices.push_back(cur);
    cur = static_cast<std::size_t>(came_from[cur]);
  }
  path_indices.push_back(flat(si, sj));

  current_path.clear();
  if (path_indices.size() < 2) {
    return;
  }
  for (auto it = path_indices.rbegin(); it != path_indices.rend(); ++it) {
    const long i = static_cast<long>(*it) / ny;
    const long j = static_cast<long>(*it) % ny;
    current_path.push_back(get_coords(i, j));
  }
  // pop off the first: it is the current robot position
  current_path.pop_front();
  if (!current_path.empty()) {
    next_waypoint = current_path.front();
  }
}

void LocalPlannerCore::smooth_path()
{
  if (current_path.size() <= 2) {
    return;
  }
  const double epsilon = 0.3;
  const std::vector<Pt> smoothed = planner::simplify_path(
    std::vector<Pt>(current_path.begin(), current_path.end()), epsilon);
  // Only adopt the simplified path if every segment of it is traversable.
  Pt prev = robot_position;
  for (const Pt & p : smoothed) {
    if (!segment_traversable(compute_segment_cost(prev, p, 4).first, max_cell_threshold)) {
      log(Level::Info, "Keeping dense path: smoothed path is blocked");
      return;
    }
    prev = p;
  }
  current_path.assign(smoothed.begin(), smoothed.end());
  if (!current_path.empty()) {
    next_waypoint = current_path.front();
  }
}

}  // namespace autonomous_navigation_cpp
