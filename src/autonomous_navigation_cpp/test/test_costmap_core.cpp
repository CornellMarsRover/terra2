// Port of src/autonomous_navigation/test/test_costmap_core.py, plus golden
// comparisons of CostmapCore against the Python CostmapNode.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autonomous_navigation_cpp/costmap_core.hpp"
#include "autonomous_navigation_cpp/py_compat.hpp"
#include "golden_data.hpp"

using autonomous_navigation_cpp::cell_key;
using autonomous_navigation_cpp::CostmapCore;
using autonomous_navigation_cpp::decay_costs;
using autonomous_navigation_cpp::GridMap;
using autonomous_navigation_cpp::observed_cost;
using autonomous_navigation_cpp::Polygon2D;
using autonomous_navigation_cpp::project_point;
using autonomous_navigation_cpp::py_radians;
using autonomous_navigation_cpp::XYZ;

namespace
{
const double kIdentity[4] = {1.0, 0.0, 0.0, 1.0};

// Same LCG + float32 arithmetic as golden.py:lcg_points.
std::vector<XYZ> lcg_points(uint64_t seed, int n, const std::array<std::array<double, 2>, 3> & r)
{
  std::vector<XYZ> pts;
  uint64_t s = seed;
  for (int i = 0; i < n; ++i) {
    float v[3];
    for (int k = 0; k < 3; ++k) {
      s = s * 6364136223846793005ULL + 1442695040888963407ULL;
      const uint64_t q = (s >> 33) % 10001;
      const float step = static_cast<float>((r[k][1] - r[k][0]) / 10000.0);
      v[k] = static_cast<float>(r[k][0]) + static_cast<float>(q) * step;
    }
    pts.push_back({v[0], v[1], v[2]});
  }
  return pts;
}

std::vector<XYZ> real_cloud()
{
  auto pts = lcg_points(12345, 3000, {{{-1.0, 7.0}, {-5.0, 5.0}, {-1.6, 1.4}}});
  for (std::size_t i = 0; i + 2 < golden::kEdge.size(); i += 3) {
    pts.push_back({static_cast<float>(golden::kEdge[i]), static_cast<float>(golden::kEdge[i + 1]),
        static_cast<float>(golden::kEdge[i + 2])});
  }
  return pts;
}

using CellSet = std::set<std::array<int, 3>>;

CellSet grid_set(const GridMap & grid)
{
  CellSet out;
  for (const auto & kv : grid) {
    out.insert({autonomous_navigation_cpp::key_x(kv.first),
        autonomous_navigation_cpp::key_y(kv.first), kv.second});
  }
  return out;
}

CellSet to_set(const std::vector<std::array<int, 3>> & v) {return {v.begin(), v.end()};}
}  // namespace

// ---------------------------------------------------------------------------
// test_costmap_core.py
// ---------------------------------------------------------------------------
TEST(CostmapCorePy, RealPointProjectsIntoRotatedGlobalGrid)
{
  const double rotation[4] = {0.0, -1.0, 1.0, 0.0};
  const auto result = project_point(2.0, 0.5, -0.8, 10, 20, rotation, true, 1, 1.2, 6, 0.25);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->cell_x, 9.5);
  EXPECT_EQ(result->cell_y, 22.0);
  EXPECT_NEAR(result->height, 0.2, 1e-9);
}

TEST(CostmapCorePy, RealPointRejectsInvalidDepthOrBearing)
{
  const double points[4][3] = {{0.5, 0.0, 0.0}, {7.0, 0.0, 0.0}, {2.0, 3.0, 0.0}, {-2.0, 0.0, 0.0}};
  for (const auto & p : points) {
    EXPECT_FALSE(project_point(p[0], p[1], p[2], 0, 0, kIdentity, true, 1, 1.2, 6, 0.25));
  }
}

TEST(CostmapCorePy, SimPointUsesCameraAxesAndFlipsGlobalLeft)
{
  const auto result = project_point(0.6, 0.8, 2.0, 1, 3, kIdentity, false, 1, 1.2, 6, 0.25);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->cell_x, 3.0);
  EXPECT_EQ(result->cell_y, 2.5);
  EXPECT_NEAR(result->height, 0.2, 1e-9);
}

TEST(CostmapCorePy, SimPointRejectsInvalidDepth)
{
  for (double depth : {0.5, 7.0}) {
    EXPECT_FALSE(project_point(0.0, 0.0, depth, 0, 0, kIdentity, false, 1, 1.2, 6, 0.25));
  }
}

TEST(CostmapCorePy, ProjectionSnapsToRequestedCellSize)
{
  const double angle = py_radians(0);
  const double rotation[4] = {std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle)};
  const auto result = project_point(1.26, 0, 2.24, 0, 0, rotation, false, 1, 1, 6, 0.5);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->cell_x, 2.0);
  EXPECT_EQ(result->cell_y, -1.5);
}

TEST(CostmapCorePy, ObservedCostMarksGroundAndOverheadAsTraversable)
{
  EXPECT_FALSE(observed_cost(4, 0.0, -0.2, 0.15, 2, 100).has_value());
  EXPECT_FALSE(observed_cost(4, 3.0, -0.2, 0.15, 2, 100).has_value());
}

TEST(CostmapCorePy, ObservedCostAccumulatesLowAndTallObstaclesWithCap)
{
  EXPECT_EQ(observed_cost(4, 0.3, -0.2, 0.15, 2, 100), 6);
  EXPECT_EQ(observed_cost(97, 0.8, -0.2, 0.15, 2, 100), 100);
}

TEST(CostmapCorePy, DecayCostsRemovesZeroesAndPreservesIneligibleCells)
{
  const GridMap costs = {{cell_key(0, 0), 1}, {cell_key(1, 0), 4}, {cell_key(2, 0), 5}};
  const std::unordered_set<int64_t> eligible = {cell_key(0, 0), cell_key(1, 0)};
  const GridMap expected = {{cell_key(1, 0), 2}, {cell_key(2, 0), 5}};
  EXPECT_EQ(decay_costs(costs, 2, &eligible), expected);
  EXPECT_EQ(costs.size(), 3u);   // input untouched
  EXPECT_EQ(costs.at(cell_key(0, 0)), 1);
}

TEST(CostmapCorePy, DecayCostsDefaultsToAllCells)
{
  const GridMap expected = {{cell_key(0, 0), 1}};
  EXPECT_EQ(decay_costs({{cell_key(0, 0), 2}}, 1), expected);
}

TEST(CostmapCorePy, DecayCostsRejectsNegativeAmount)
{
  EXPECT_THROW(decay_costs({}, -1), std::invalid_argument);
}

TEST(CostmapCorePy, RealPointAcceptsWideZedEdgeBearing)
{
  const double left = 2.0 * std::tan(py_radians(50.0));
  EXPECT_TRUE(project_point(2.0, left, -0.8, 0, 0, kIdentity, true, 1, 1.2, 6, 0.25).has_value());
}

// ---------------------------------------------------------------------------
// CostmapCore vs. the Python CostmapNode (golden.py)
// ---------------------------------------------------------------------------
TEST(Polygon, ContainsIsStrictInterior)
{
  Polygon2D sq({0, 2, 2, 0}, {0, 0, 2, 2});
  EXPECT_TRUE(sq.contains(1, 1));
  EXPECT_FALSE(sq.contains(0, 1));     // on edge
  EXPECT_FALSE(sq.contains(2, 2));     // vertex
  EXPECT_FALSE(sq.contains(3, 1));
  Polygon2D concave({0, 4, 4, 2, 0}, {0, 0, 4, 1, 4});
  EXPECT_TRUE(concave.contains(1, 0.5));
  EXPECT_FALSE(concave.contains(2, 3));  // in the notch
  EXPECT_FALSE(Polygon2D().contains(0, 0));
}

TEST(Costmap, RealModeMatchesPython)
{
  CostmapCore c(true);
  const double north = 1.3, west = -0.7, yaw = 0.6;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const auto pts = real_cloud();
  // No ground plane yet: nothing is "in the ground plane", every point counts.
  c.process_cloud(pts.data(), pts.size(), north, west, cs, sn);
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kReal));
  c.decay_cost();
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kRealDecay));
  c.set_object_position(2.6, -0.45);
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kObj));

  // Second cloud while point turning (min_depth 0.3).
  const auto turn = lcg_points(777, 3000, {{{-1.0, 7.0}, {-5.0, 5.0}, {-1.6, 1.4}}});
  c.set_last_movement_point_turn(true);
  c.process_cloud(turn.data(), turn.size(), north, west, cs, sn);
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kTurn));

  // Ground plane (as the node builds it: R . [x, y] + pose) decays the cells
  // inside it by 4 and masks later points that land inside it.
  std::vector<double> xs, ys;
  for (std::size_t i = 0; i < golden::kGroundX.size(); ++i) {
    const double x = golden::kGroundX[i], y = golden::kGroundY[i];
    xs.push_back((cs * x + (-sn) * y) + north);
    ys.push_back((sn * x + cs * y) + west);
  }
  ASSERT_TRUE(c.set_ground_plane(xs, ys));
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kGround));
  c.set_last_movement_point_turn(false);
  c.process_cloud(pts.data(), pts.size(), north, west, cs, sn);
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kGroundCloud));

  std::vector<float> out;
  c.collect(out);
  CellSet published;
  for (std::size_t i = 0; i + 2 < out.size(); i += 3) {
    published.insert({static_cast<int>(std::lround(out[i] * 4)),
        static_cast<int>(std::lround(out[i + 1] * 4)), static_cast<int>(out[i + 2])});
  }
  EXPECT_EQ(published, to_set(golden::kGroundPublished));
  EXPECT_EQ(out.size(), golden::kGroundPublished.size() * 3);
}

TEST(Costmap, SimModeMatchesPython)
{
  CostmapCore c(false);
  const auto pts = lcg_points(999, 3000, {{{-6.0, 6.0}, {-1.5, 1.5}, {0.0, 7.0}}});
  const double yaw = -1.1;
  c.process_cloud(pts.data(), pts.size(), -2.2, 3.1, std::cos(-1.0 * yaw), std::sin(-1.0 * yaw));
  EXPECT_EQ(grid_set(c.grid()), to_set(golden::kSim));
}

// process_cloud() classifies points with cheap comparisons and only takes the
// exact hypot/atan2 path near a threshold. It must agree with the per-point
// reference functions everywhere, especially right at the thresholds.
TEST(Costmap, FastPathAgreesWithReferenceAtThresholds)
{
  std::vector<XYZ> pts = lcg_points(4242, 20000, {{{-7.0, 7.0}, {-7.0, 7.0}, {-1.6, 1.4}}});
  const double tan55 = std::tan(py_radians(55.0));
  for (double depth : {0.3, 1.2, 6.0}) {
    for (int a = -60; a <= 60; a += 3) {
      const double ang = py_radians(a);
      float x = static_cast<float>(depth * std::cos(ang));
      float y = static_cast<float>(depth * std::sin(ang));
      // walk a few float32 ulps either side of the range circle
      for (int k = -6; k <= 6; ++k) {
        float xx = x;
        for (int s = 0; s < std::abs(k); ++s) {
          xx = std::nextafterf(xx, k < 0 ? -100.0f : 100.0f);
        }
        pts.push_back({xx, y, 0.9f});
      }
    }
  }
  for (float x : {0.5f, 1.0f, 2.0f, 3.3f, 4.123f}) {
    for (double sign : {1.0, -1.0}) {
      float y = static_cast<float>(sign * x * tan55);
      for (int k = -6; k <= 6; ++k) {
        float yy = y;
        for (int s = 0; s < std::abs(k); ++s) {
          yy = std::nextafterf(yy, k < 0 ? -100.0f : 100.0f);
        }
        pts.push_back({x, yy, 0.2f});
      }
    }
  }
  pts.push_back({0.0f, 0.0f, 0.0f});
  pts.push_back({0.0f, 2.0f, 0.0f});
  pts.push_back({-0.0f, -2.0f, 0.0f});

  for (bool point_turn : {false, true}) {
    const double north = 0.37, west = -1.21, yaw = 0.9;
    const double rot[4] = {std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw)};
    const double min_depth = point_turn ? 0.3 : 1.2;

    // Reference: the Python per-point loop on project_point/observed_cost.
    std::map<std::pair<double, double>, int> grid;
    std::set<std::pair<double, double>> curr_obstacles;
    for (const XYZ & p : pts) {
      const auto obs = project_point(p.x, p.y, p.z, north, west, rot, true, 1.0, min_depth, 6.0, 0.25);
      if (!obs) {
        continue;
      }
      const std::pair<double, double> cell{obs->cell_x, obs->cell_y};
      if (curr_obstacles.count(cell)) {
        continue;
      }
      int & cost = grid[cell];
      const auto next = observed_cost(cost, obs->height, -0.3, 0.25, 2.0, 100);
      if (!next) {
        continue;
      }
      cost = *next;
      curr_obstacles.insert(cell);
    }
    CellSet expected;
    for (const auto & kv : grid) {
      expected.insert({static_cast<int>(std::lround(kv.first.first * 4)),
          static_cast<int>(std::lround(kv.first.second * 4)), kv.second});
    }

    CostmapCore c(true);
    c.set_last_movement_point_turn(point_turn);
    c.process_cloud(pts.data(), pts.size(), north, west, rot[0], rot[2]);
    EXPECT_EQ(grid_set(c.grid()), expected) << "point_turn=" << point_turn;
    EXPECT_GT(expected.size(), 100u);
  }
}

TEST(Costmap, GroundPlaneDecayAndRepeatedHits)
{
  CostmapCore c(true);
  // Same obstacle cell hit by many points in one cloud: only one increment.
  std::vector<XYZ> pts(50, XYZ{2.0f, 0.0f, 0.0f});   // height 1.0 > 0.5 => +7
  c.process_cloud(pts.data(), pts.size(), 0, 0, 1, 0);
  EXPECT_EQ(c.cost_at(2.0, 0.0), 7);
  for (int i = 0; i < 20; ++i) {
    c.process_cloud(pts.data(), pts.size(), 0, 0, 1, 0);
  }
  EXPECT_EQ(c.cost_at(2.0, 0.0), 100);  // capped at max_cost
  // Ground plane covering the cell decays it by 4 and caches membership.
  ASSERT_TRUE(c.set_ground_plane({0, 4, 4, 0}, {-2, -2, 2, 2}));
  EXPECT_EQ(c.cost_at(2.0, 0.0), 96);
  // Points now inside the ground plane are ignored.
  c.process_cloud(pts.data(), pts.size(), 0, 0, 1, 0);
  EXPECT_EQ(c.cost_at(2.0, 0.0), 96);
  std::vector<float> out;
  c.collect(out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_FLOAT_EQ(out[0], 2.0f);
  EXPECT_FLOAT_EQ(out[2], 96.0f);
  // 1/2-vertex polygons are rejected (Python would crash).
  EXPECT_FALSE(c.set_ground_plane({0, 1}, {0, 1}));
}
