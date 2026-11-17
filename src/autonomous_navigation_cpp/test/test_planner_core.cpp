// Port of src/autonomous_navigation/test/test_planner_core.py, plus golden
// comparisons of LocalPlannerCore against the Python LocalPlannerNode.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autonomous_navigation_cpp/local_planner_core.hpp"
#include "autonomous_navigation_cpp/planner_core.hpp"
#include "autonomous_navigation_cpp/py_compat.hpp"
#include "golden_data.hpp"
#include "trace_util.hpp"

using autonomous_navigation_cpp::CostLookup;
using autonomous_navigation_cpp::LocalPlannerCore;
using autonomous_navigation_cpp::Pt;
using autonomous_navigation_cpp::py_degrees;
using autonomous_navigation_cpp::py_hypot;
using autonomous_navigation_cpp::py_radians;
using autonomous_navigation_cpp::py_repr;
namespace planner = autonomous_navigation_cpp::planner;

namespace
{
using Path = std::vector<Pt>;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

CostLookup lookup(std::initializer_list<std::pair<Pt, double>> cells)
{
  CostLookup c;
  for (const auto & kv : cells) {
    c.set(kv.first.x, kv.first.y, kv.second);
  }
  return c;
}

void expect_path(const std::deque<Pt> & got, const std::vector<double> & want)
{
  ASSERT_EQ(got.size() * 2, want.size());
  for (std::size_t i = 0; i < got.size(); ++i) {
    EXPECT_EQ(got[i].x, want[2 * i]) << "i=" << i;
    EXPECT_EQ(got[i].y, want[2 * i + 1]) << "i=" << i;
  }
}
}  // namespace

// ---------------------------------------------------------------------------
// Python numeric semantics the planner depends on
// ---------------------------------------------------------------------------
TEST(PyCompat, HypotMatchesCPythonBitForBit)
{
  for (std::size_t i = 0; i + 3 < golden::kHypot.size(); i += 4) {
    const double a = golden::kHypot[i], b = golden::kHypot[i + 1];
    EXPECT_EQ(py_hypot(a, b), golden::kHypot[i + 2]) << a << ", " << b;
    EXPECT_EQ(autonomous_navigation_cpp::py_dist({a, 1.0}, {0.0, 1.0 + b}), golden::kHypot[i + 3]);
  }
  EXPECT_EQ(py_hypot(0.0, -0.0), 0.0);
  EXPECT_EQ(py_hypot(kInf, kNan), kInf);
  EXPECT_TRUE(std::isnan(py_hypot(1.0, kNan)));
  EXPECT_EQ(py_hypot(3e-320, 4e-320), 5e-320);
}

TEST(PyCompat, DegreesRadiansMatchCPython)
{
  for (std::size_t i = 0; i + 2 < golden::kAngles.size(); i += 3) {
    EXPECT_EQ(py_radians(golden::kAngles[i]), golden::kAngles[i + 1]);
    EXPECT_EQ(py_degrees(golden::kAngles[i] / 100.0), golden::kAngles[i + 2]);
  }
}

TEST(PyCompat, ReprMatchesCPython)
{
  ASSERT_EQ(golden::kReprValues.size(), golden::kReprStrings.size());
  for (std::size_t i = 0; i < golden::kReprValues.size(); ++i) {
    EXPECT_EQ(py_repr(golden::kReprValues[i]), golden::kReprStrings[i]);
  }
  EXPECT_EQ(py_repr(kNan), "nan");
  EXPECT_EQ(py_repr(-kInf), "-inf");
}

// ---------------------------------------------------------------------------
// test_planner_core.py
// ---------------------------------------------------------------------------
TEST(PlannerCorePy, SegmentObservationsRequireConsecutiveConfirmations)
{
  const planner::SegmentCounts original;
  auto [counts, confirmed] = planner::record_segment_observation(original, {0, 1}, true);
  EXPECT_TRUE(original.empty());
  EXPECT_EQ(counts, (planner::SegmentCounts{{{0, 1}, 1}}));
  EXPECT_FALSE(confirmed);

  std::tie(counts, confirmed) = planner::record_segment_observation(counts, {0, 1}, true, 2);
  EXPECT_TRUE(confirmed);
  std::tie(counts, confirmed) = planner::record_segment_observation(counts, {0, 1}, false, 2);
  EXPECT_TRUE(counts.empty());
  EXPECT_FALSE(confirmed);
}

TEST(PlannerCorePy, SegmentObservationsRejectInvalidConfirmationCount)
{
  EXPECT_THROW(planner::record_segment_observation({}, {0, 1}, true, 0), std::invalid_argument);
}

TEST(PlannerCorePy, NearestClearGoalClampsAndAcceptsClearGoal)
{
  const Pt r = planner::nearest_clear_goal({4, -1}, {0, 2, 0, 2}, [](const Pt &) {return false;}, 1);
  EXPECT_EQ(r, (Pt{2, 0}));
}

TEST(PlannerCorePy, NearestClearGoalUsesDeterministicNeighbor)
{
  const Pt r = planner::nearest_clear_goal(
    {1, 1}, {0, 2, 0, 2}, [](const Pt & p) {return p == Pt{1, 1};}, 1);
  EXPECT_EQ(r, (Pt{0, 1}));
}

TEST(PlannerCorePy, NearestClearGoalIsBoundedWhenEveryCellIsBlocked)
{
  const Pt r = planner::nearest_clear_goal({1, 1}, {0, 2, 0, 2}, [](const Pt &) {return true;}, 1);
  EXPECT_EQ(r, (Pt{1, 1}));
}

TEST(PlannerCorePy, NearestClearGoalRejectsNonpositiveStep)
{
  EXPECT_THROW(
    planner::nearest_clear_goal({0, 0}, {0, 1, 0, 1}, [](const Pt &) {return false;}, 0),
    std::invalid_argument);
}

TEST(PlannerCorePy, ParseCostmapSplitsCostsAndObstacles)
{
  const auto parsed = planner::parse_costmap({1, 2, 3, 4, 5, 8}, 4);
  EXPECT_EQ(parsed.costs.size(), 2u);
  EXPECT_EQ(parsed.costs.get(1.0, 2.0), 3.0);
  EXPECT_EQ(parsed.costs.get(4.0, 5.0), 8.0);
  EXPECT_EQ(parsed.obstacles, (std::set<std::pair<double, double>>{{4.0, 5.0}}));
}

TEST(PlannerCorePy, ParseCostmapRejectsIncompleteTriple)
{
  EXPECT_THROW(planner::parse_costmap({1, 2}, 4), std::invalid_argument);
}

TEST(PlannerCorePy, EmptyCostmapClearsAllPlannerState)
{
  const auto parsed = planner::parse_costmap({}, 4);
  EXPECT_TRUE(parsed.costs.empty());
  EXPECT_TRUE(parsed.obstacles.empty());
}

TEST(PlannerCorePy, ParsePlanarTargetAcceptsOptionalYaw)
{
  auto t = planner::parse_planar_target({1, 2});
  EXPECT_EQ(t.target, (Pt{1.0, 2.0}));
  EXPECT_FALSE(t.yaw.has_value());
  t = planner::parse_planar_target({1, 2, 0.5});
  EXPECT_EQ(t.target, (Pt{1.0, 2.0}));
  ASSERT_TRUE(t.yaw.has_value());
  EXPECT_EQ(*t.yaw, 0.5);
}

TEST(PlannerCorePy, ParsePlanarTargetRejectsMalformedPayloads)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<std::vector<float>> cases = {{}, {1}, {nan, 2}, {1, inf}};
  for (const auto & data : cases) {
    EXPECT_THROW(planner::parse_planar_target(data), std::invalid_argument);
  }
}

TEST(PlannerCorePy, AdvancePathWaitsOutsideThreshold)
{
  const auto r = planner::advance_path({0, 0}, {4, 0}, {{2, 0}, {4, 0}}, {2, 0}, 0.3);
  EXPECT_EQ(r.path, (Path{{2, 0}, {4, 0}}));
  EXPECT_EQ(r.next_waypoint, (Pt{2, 0}));
  EXPECT_FALSE(r.reached.has_value());
}

TEST(PlannerCorePy, AdvancePathSelectsFollowingWaypoint)
{
  const auto r = planner::advance_path({1.9, 0}, {4, 0}, {{2, 0}, {4, 0}}, {2, 0}, 0.3);
  EXPECT_EQ(r.path, (Path{{4, 0}}));
  EXPECT_EQ(r.next_waypoint, (Pt{4, 0}));
  ASSERT_TRUE(r.reached.has_value());
  EXPECT_EQ(*r.reached, (Pt{2, 0}));
}

TEST(PlannerCorePy, AdvancePathFallsBackToTarget)
{
  const auto r = planner::advance_path({2, 0}, {4, 0}, {}, {2, 0}, 0.3);
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.next_waypoint, (Pt{4, 0}));
  EXPECT_EQ(*r.reached, (Pt{2, 0}));
}

TEST(PlannerCorePy, PathDensityRequiresTwoShortSegments)
{
  EXPECT_TRUE(planner::path_is_dense({{0, 0}, {0.5, 0}, {1.0, 0}}));
  EXPECT_FALSE(planner::path_is_dense({{0, 0}, {2, 0}, {2.5, 0}}));
  EXPECT_FALSE(planner::path_is_dense({{0, 0}, {0.5, 0}}));
}

TEST(PlannerCorePy, PerpendicularDistanceHandlesLineAndPoint)
{
  EXPECT_NEAR(planner::perpendicular_distance({1, 1}, {0, 0}, {2, 0}), 1.0, 1e-12);
  EXPECT_NEAR(planner::perpendicular_distance({1, 1}, {0, 0}, {0, 0}), std::sqrt(2.0), 1e-12);
}

TEST(PlannerCorePy, SimplifyPathHandlesShortStraightAndBentPaths)
{
  EXPECT_EQ(planner::simplify_path({{0, 0}, {1, 0}}, 0.1), (Path{{0, 0}, {1, 0}}));
  EXPECT_EQ(planner::simplify_path({{0, 0}, {1, 0}, {2, 0}}, 0.1), (Path{{0, 0}, {2, 0}}));
  EXPECT_EQ(
    planner::simplify_path({{0, 0}, {1, 1}, {2, 0}}, 0.1), (Path{{0, 0}, {1, 1}, {2, 0}}));
}

TEST(PlannerCorePy, NeighborCostWeightsCellsAndIgnoresUnknownMarker)
{
  const CostLookup costs = lookup({{{-0.25, -0.25}, 2}, {{-0.25, 0}, 1}});
  EXPECT_NEAR(planner::neighbor_cost(costs, {0, 0}, 0.25, 1), 2 / std::sqrt(0.125), 1e-12);
  EXPECT_EQ(planner::neighbor_cost(CostLookup(), {0, 0}, 0.25, 0), 0);
}

TEST(PlannerCorePy, SegmentCostHandlesStationaryAndSampledSegments)
{
  EXPECT_EQ(planner::segment_cost(CostLookup(), {0, 0}, {0, 0}), std::make_pair(0.0, 0.0));
  const auto [maximum, total] =
    planner::segment_cost(lookup({{{0.5, 0.0}, 4.0}}), {0, 0}, {1, 0}, 0.25, 0);
  EXPECT_EQ(maximum, 4.0);
  EXPECT_EQ(total, 4.0);
}

TEST(PlannerCorePy, NeighborCostIsDirectionallySymmetric)
{
  const int offsets[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
  for (const auto & o : offsets) {
    const Pt cell{o[0] * 0.25, o[1] * 0.25};
    const double expected = 2.0 / std::sqrt(cell.x * cell.x + cell.y * cell.y);
    EXPECT_NEAR(planner::neighbor_cost(lookup({{cell, 2.0}}), {0, 0}, 0.25, 1), expected, 1e-12);
  }
}

TEST(PlannerCorePy, NeighborCostRejectsInvalidGridParameters)
{
  const double cases[4][2] = {{0, 1}, {-1, 1}, {1, -1}, {-0.1, -1}};
  for (const auto & c : cases) {
    EXPECT_THROW(
      planner::neighbor_cost(CostLookup(), {0, 0}, c[0], static_cast<int>(c[1])),
      std::invalid_argument);
  }
}

TEST(PlannerCorePy, SegmentTraversabilityEnforcesThresholdAndFiniteCost)
{
  EXPECT_TRUE(planner::segment_traversable(20, 20));
  EXPECT_FALSE(planner::segment_traversable(20.01, 20));
  EXPECT_FALSE(planner::segment_traversable(kNan, 20));
}

TEST(PlannerCorePy, TransitionTraversabilityAllowsOnlySafeOrDecreasingRisk)
{
  EXPECT_TRUE(planner::transition_traversable(10, 30, 40, 20));
  EXPECT_TRUE(planner::transition_traversable(30, 40, 30, 20));
  EXPECT_FALSE(planner::transition_traversable(30, 40, 40, 20));
  EXPECT_FALSE(planner::transition_traversable(30, 30, 40, 20));
}

TEST(PlannerCorePy, SafetyGapNeverDropsBelowMinimum)
{
  EXPECT_EQ(planner::next_safety_gap(4, 3), 3);
  EXPECT_FALSE(planner::next_safety_gap(3, 3).has_value());
}

TEST(PlannerCorePy, SafetyGapRejectsNegativeMinimum)
{
  EXPECT_THROW(planner::next_safety_gap(4, -1), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// LocalPlannerCore vs. the Python LocalPlannerNode (golden.py)
// ---------------------------------------------------------------------------
namespace
{
LocalPlannerCore make_toy()
{
  LocalPlannerCore p(true);
  for (int j = -12; j <= 12; ++j) {
    p.set_cost(1.5, j * 0.25, 50.0);
  }
  p.set_cost(2.0, 2.0, 1.0);
  p.set_cost(0.5, 0.5, 7.0);
  p.robot_position = {0.1, -0.2};
  p.next_target = {3.0, 1.0};
  return p;
}
}  // namespace

TEST(LocalPlanner, NeighborAndSegmentCostsMatchPython)
{
  auto p = make_toy();
  EXPECT_EQ(p.get_neighbor_costs(1.25, 0.0, 1, 3), golden::kNeighbor[0]);
  EXPECT_EQ(p.get_neighbor_costs(1.0, 0.5, 1, 2), golden::kNeighbor[1]);
  EXPECT_EQ(p.point_risk({1.1, 0.3}), golden::kNeighbor[2]);
  EXPECT_EQ(p.point_risk({1.1, 0.3}, 3), golden::kNeighbor[3]);
  // memoized second lookups return the same values
  EXPECT_EQ(p.point_risk({1.1, 0.3}), golden::kNeighbor[2]);
  EXPECT_EQ(p.point_risk({1.1, 0.3}, 3), golden::kNeighbor[3]);
  const auto s1 = p.compute_segment_cost({0.1, -0.2}, {3.0, 1.0}, 2);
  EXPECT_EQ(s1.first, golden::kSegment[0]);
  EXPECT_EQ(s1.second, golden::kSegment[1]);
  const auto s2 = p.compute_segment_cost({0.0, 0.0}, {0.5, 0.5}, 3);
  EXPECT_EQ(s2.first, golden::kSegment[2]);
  EXPECT_EQ(s2.second, golden::kSegment[3]);
  const auto s3 = p.compute_segment_cost({0.1, -0.2}, {0.9, 2.6});
  EXPECT_EQ(s3.first, golden::kSegment[4]);
  EXPECT_EQ(s3.second, golden::kSegment[5]);
  // The free function (no memo) agrees with the memoized member.
  const auto s4 = planner::segment_cost(p.costs(), {0.1, -0.2}, {0.9, 2.6});
  EXPECT_EQ(s4, s3);
}

TEST(LocalPlanner, AStarAroundWallMatchesPython)
{
  auto p = make_toy();
  p.compute_path();
  expect_path(p.current_path, golden::kPath);
  p.smooth_path();
  expect_path(p.current_path, golden::kSmooth);
  ASSERT_TRUE(p.next_waypoint.has_value());
  EXPECT_EQ(p.next_waypoint->x, golden::kSmooth[0]);
  EXPECT_EQ(p.next_waypoint->y, golden::kSmooth[1]);
}

TEST(LocalPlanner, BlockedGoalMovesToNearestClearCellLikePython)
{
  auto p = make_toy();
  for (int i = -3; i <= 3; ++i) {
    for (int j = -3; j <= 3; ++j) {
      p.set_cost(3.0 + i * 0.25, 1.0 + j * 0.25, 100.0);
    }
  }
  const Pt goal = planner::nearest_clear_goal(
    {3.0, 1.0}, {-8.9, 9.1, -9.2, 8.8},
    [&](const Pt & q) {return p.get_neighbor_costs(q.x, q.y, 1, 3) > 40.0;}, 0.5);
  EXPECT_EQ(goal.x, golden::kClearGoal[0]);
  EXPECT_EQ(goal.y, golden::kClearGoal[1]);
  p.compute_path();
  expect_path(p.current_path, golden::kPathBlockedGoal);
}

TEST(LocalPlanner, NonFinitePoseIsIgnored)
{
  LocalPlannerCore p(true);
  p.on_target({4.0f, 5.0f});
  EXPECT_FALSE(p.on_pose(kNan, 0.0, 0.0));
  EXPECT_FALSE(p.on_pose(0.0, kInf, 0.0));
  EXPECT_EQ(p.robot_position, (Pt{0.0, 0.0}));
  EXPECT_EQ(p.current_path.size(), 1u);
  p.validate_path();   // must not blow up
}

// Full closed-loop session: targets, costmaps (incl. malformed ones), poses,
// validate_path() and publish_waypoint() exactly as the Python node saw them.
TEST(LocalPlanner, SessionTraceMatchesPython)
{
  std::vector<std::string> sink;
  LocalPlannerCore core(
    true, [&](LocalPlannerCore::Level level, const std::string & s) {
      const char * name = level == LocalPlannerCore::Level::Info ? "info" :
      (level == LocalPlannerCore::Level::Warn ? "warn" : "error");
      sink.push_back(std::string("log ") + name + " " + s);
    });
  trace::replay(
    golden::kPlannerTrace, {"log info Planner Node initialized"},   // logged by the node
    [&](const std::vector<std::string> & t, std::vector<std::string> & out) {
      if (t[0] == "pose") {
        EXPECT_TRUE(core.on_pose(trace::num(t[1]), trace::num(t[2]), trace::num(t[3])));
      } else if (t[0] == "costmap") {
        core.on_costmap(trace::floats(t, 1));
      } else if (t[0] == "target") {
        core.on_target(trace::floats(t, 1));
      } else if (t[0] == "validate") {
        core.validate_path();
        sink.push_back(
          std::string("state hold=") + (core.path_hold ? "1" : "0") + " next=" +
          (core.next_waypoint ? autonomous_navigation_cpp::fmt_pt(*core.next_waypoint) : "None") +
          " n=" + std::to_string(core.current_path.size()));
      } else if (t[0] == "publish") {
        std::vector<float> plan;
        const auto wp = core.make_waypoint(&plan);
        sink.push_back(trace::pub("/autonomy/path/next_waypoint", wp));
        sink.push_back(trace::pub("/autonomy/path/plan", plan));
      } else {
        FAIL() << "unknown op " << t[0];
      }
      out = sink;
      sink.clear();
    });
}
