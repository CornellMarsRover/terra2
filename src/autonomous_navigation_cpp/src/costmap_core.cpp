#include "autonomous_navigation_cpp/costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{

// ---------------------------------------------------------------------------
// Polygon2D
// ---------------------------------------------------------------------------
Polygon2D::Polygon2D(std::vector<double> xs, std::vector<double> ys)
: xs_(std::move(xs)), ys_(std::move(ys))
{
  if (xs_.empty()) {
    return;
  }
  minx_ = *std::min_element(xs_.begin(), xs_.end());
  maxx_ = *std::max_element(xs_.begin(), xs_.end());
  miny_ = *std::min_element(ys_.begin(), ys_.end());
  maxy_ = *std::max_element(ys_.begin(), ys_.end());
}

bool Polygon2D::contains(double px, double py) const
{
  const std::size_t n = xs_.size();
  if (n < 3) {
    return false;  // empty / degenerate polygon has no interior
  }
  // A point on (or outside) the bounding box edge cannot be in the interior.
  if (!(px > minx_ && px < maxx_ && py > miny_ && py < maxy_)) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = xs_[i], yi = ys_[i], xj = xs_[j], yj = ys_[j];
    // Boundary => not contained (shapely.contains semantics).
    const double cross = (xj - xi) * (py - yi) - (yj - yi) * (px - xi);
    if (cross == 0.0 && px >= std::min(xi, xj) && px <= std::max(xi, xj) &&
      py >= std::min(yi, yj) && py <= std::max(yi, yj))
    {
      return false;
    }
    if ((yi > py) != (yj > py)) {
      const double x_int = (xj - xi) * (py - yi) / (yj - yi) + xi;
      if (px < x_int) {
        inside = !inside;
      }
    }
  }
  return inside;
}

// ---------------------------------------------------------------------------
// CostmapCore
// ---------------------------------------------------------------------------
CostmapCore::CostmapCore(bool real)
: real_(real)
{
  if (real_) {
    obstacle_threshold_ = 0.25;
    ground_threshold_ = -0.3;
  } else {
    obstacle_threshold_ = 0.15;
    ground_threshold_ = -0.2;
  }
  camera_height_ = 1.0;
  cells_.reserve(1 << 20);
}

int64_t CostmapCore::cell_key_public(int32_t ix, int32_t iy) {return cell_key(ix, iy);}

int CostmapCore::cost_at(double x, double y) const
{
  auto it = grid_.find(cell_key(static_cast<int32_t>(py_round(x * k_)),
    static_cast<int32_t>(py_round(y * k_))));
  return it == grid_.end() ? 0 : it->second;
}

void CostmapCore::set_object_position(double x, double y)
{
  // N = 1; range(-N, N) -> {-1, 0}.  NOTE(port): asymmetric 2x2 block, as in
  // costmap.py:object_callback. game_object_position is never read in Python,
  // so it is not stored here.
  const int N = 1;
  const double xs = py_round(x * k_);
  const double ys = py_round(y * k_);
  if (!index_in_range(xs) || !index_in_range(ys)) {
    return;
  }
  for (int dx = -N; dx < N; ++dx) {
    for (int dy = -N; dy < N; ++dy) {
      // x_snap + dx*0.25 == (round(x*4) + dx) / 4 exactly.
      grid_[cell_key(static_cast<int32_t>(xs) + dx, static_cast<int32_t>(ys) + dy)] = 0;
    }
  }
}

bool CostmapCore::in_ground_plane(int32_t ix, int32_t iy)
{
  if (!has_ground_) {
    return false;  // costmap.py:in_ground_plane - no ground plane yet
  }
  const int64_t key = cell_key(ix, iy);
  auto it = ground_cache_.find(key);
  if (it != ground_cache_.end()) {
    return it->second;
  }
  const bool b = ground_.contains(ix * cell_size_, iy * cell_size_);
  ground_cache_.emplace(key, b);
  return b;
}

bool CostmapCore::set_ground_plane(std::vector<double> xs, std::vector<double> ys)
{
  if (xs.size() == 1 || xs.size() == 2) {
    // NOTE(port): shapely raises ValueError for a 1/2-vertex ring, which kills
    // the Python node. We keep the previous ground plane instead.
    return false;
  }
  ground_ = Polygon2D(std::move(xs), std::move(ys));
  has_ground_ = true;
  ground_cache_.clear();
  decay_ground();
  return true;
}

void CostmapCore::decay_ground()
{
  // eligible = cells inside the ground plane; decay_costs(grid, 4, eligible)
  decay_costs_inplace(grid_, 4, [this](int64_t key) {
      return in_ground_plane(key_x(key), key_y(key));
    });
}

void CostmapCore::decay_cost()
{
  decay_costs_inplace(grid_, 1, [](int64_t) {return true;});
}

void CostmapCore::collect(std::vector<float> & out) const
{
  // NOTE(port): Python publishes cells in dict insertion order; here the order
  // is the hash map's. Consumers (planner_core.parse_costmap) are order-free.
  out.clear();
  out.reserve(grid_.size() * 3);
  for (const auto & kv : grid_) {
    if (kv.second > 0) {
      out.push_back(static_cast<float>(key_x(kv.first) * cell_size_));
      out.push_back(static_cast<float>(key_y(kv.first) * cell_size_));
      out.push_back(static_cast<float>(kv.second));
    }
  }
}

template<class StateAt>
void CostmapCore::run_cells(StateAt && state_at)
{
  // Sequential pass in point order: the first obstacle point that lands in a
  // cell decides its increment (+2 or +7), exactly like the Python loop with
  // its curr_obstacles set.
  for (const PtCell & pc : cells_) {
    uint8_t & s = state_at(pc.ix, pc.iy);
    if (s & (kObst2 | kObst7)) {
      continue;  // already in curr_obstacles
    }
    if (real_) {
      if (!(s & kGroundKnown)) {
        s |= kGroundKnown;
        if (in_ground_plane(pc.ix, pc.iy)) {
          s |= kGroundIn;
        }
      }
      if (s & kGroundIn) {
        continue;
      }
    }
    s |= kTouched;
    if (pc.cls == 2) {
      s |= kObst2;
    } else if (pc.cls == 7) {
      s |= kObst7;
    }
  }
}

void CostmapCore::process_cloud(
  const XYZ * pts, std::size_t n, double north, double west, double c, double s)
{
  cells_.clear();
  int32_t minx = std::numeric_limits<int32_t>::max(), maxx = std::numeric_limits<int32_t>::min();
  int32_t miny = minx, maxy = maxx;

  // Real-mode view test (costmap_core.project_point):
  //   hypot(f, l) > max_depth or < min_depth or |degrees(atan2(l, f))| > 55
  // hypot/atan2 per point are the hot spot, so points are classified with
  // cheap squared-distance / slope comparisons, and only the ones within a
  // 1e-9 relative band of a threshold (where rounding could matter) take the
  // exact Python-identical path.
  constexpr double kBand = 1e-9;
  const double max2_lo = max_depth_ * max_depth_ * (1.0 - kBand);
  const double max2_hi = max_depth_ * max_depth_ * (1.0 + kBand);
  const double min2_lo = min_depth_ * min_depth_ * (1.0 - kBand);
  const double min2_hi = min_depth_ * min_depth_ * (1.0 + kBand);
  const double tan_b = std::tan(py_radians(max_bearing_deg_));
  const double tan_lo = tan_b * (1.0 - kBand);
  const double tan_hi = tan_b * (1.0 + kBand);

  for (std::size_t i = 0; i < n; ++i) {
    const XYZ & pt = pts[i];
    double height, x, y;
    if (real_) {
      // ZED: X forward, Y left, Z up.
      x = pt.x;
      y = pt.y;
      height = camera_height_ + static_cast<double>(pt.z);
      const double sq = x * x + y * y;
      if (sq > max2_hi || sq < min2_lo) {
        continue;
      }
      if (!(sq < max2_lo && sq > min2_hi)) {
        const double distance = py_hypot(x, y);
        if (distance > max_depth_ || distance < min_depth_) {
          continue;
        }
      }
      // min_depth > 0, so x <= 0 here means |bearing| >= 90 deg.
      const double ay = std::fabs(y);
      if (x <= 0.0 || ay > x * tan_hi) {
        continue;
      }
      if (!(ay < x * tan_lo)) {
        if (std::fabs(py_degrees(std::atan2(y, x))) > max_bearing_deg_) {
          continue;
        }
      }
    } else {
      // Gazebo: Y points down, Z forward.
      height = camera_height_ - static_cast<double>(pt.y);
      x = pt.z;
      y = pt.x;
      if (x > max_depth_ || x < min_depth_) {
        continue;
      }
    }
    // R.dot([x, y]) with R = [[c, -s], [s, c]]
    const double r0 = c * x + (-s) * y;
    const double r1 = s * x + c * y;
    const double x_rot = r0 + north;
    const double y_rot = real_ ? (r1 + west) : ((-1.0 * r1) + west);

    const double qx = py_round(x_rot * k_);
    const double qy = py_round(y_rot * k_);
    if (!index_in_range(qx) || !index_in_range(qy)) {
      continue;  // NOTE(port): non-finite / absurd coordinates dropped
    }
    PtCell pc;
    pc.ix = static_cast<int32_t>(qx);
    pc.iy = static_cast<int32_t>(qy);
    if ((ground_threshold_ < height && height < obstacle_threshold_) ||
      height > clearance_height_)
    {
      pc.cls = 0;
    } else {
      pc.cls = (height > 0.5) ? 7 : 2;
    }
    cells_.push_back(pc);
    minx = std::min(minx, pc.ix);
    maxx = std::max(maxx, pc.ix);
    miny = std::min(miny, pc.iy);
    maxy = std::max(maxy, pc.iy);
  }
  if (cells_.empty()) {
    return;
  }

  const int64_t w = static_cast<int64_t>(maxx) - minx + 1;
  const int64_t h = static_cast<int64_t>(maxy) - miny + 1;
  auto apply = [this](int32_t ix, int32_t iy, uint8_t st) {
      if (!(st & kTouched)) {
        return;
      }
      int & cost = grid_.try_emplace(cell_key(ix, iy), 0).first->second;
      if (st & kObst2) {
        cost = std::min(max_cost_, cost + 2);
      } else if (st & kObst7) {
        cost = std::min(max_cost_, cost + 7);
      }
    };

  if (w * h <= (1 << 22)) {
    // Dense window (normal case: real mode is bounded by max_depth).
    window_.assign(static_cast<std::size_t>(w * h), 0);
    run_cells([&](int32_t ix, int32_t iy) -> uint8_t & {
        return window_[static_cast<std::size_t>((iy - miny) * w + (ix - minx))];
      });
    for (int64_t j = 0; j < h; ++j) {
      for (int64_t i = 0; i < w; ++i) {
        const uint8_t st = window_[static_cast<std::size_t>(j * w + i)];
        if (st & kTouched) {
          apply(static_cast<int32_t>(minx + i), static_cast<int32_t>(miny + j), st);
        }
      }
    }
  } else {
    sparse_window_.clear();
    run_cells([&](int32_t ix, int32_t iy) -> uint8_t & {
        return sparse_window_[cell_key(ix, iy)];
      });
    for (const auto & kv : sparse_window_) {
      apply(key_x(kv.first), key_y(kv.first), kv.second);
    }
  }
}

}  // namespace autonomous_navigation_cpp
