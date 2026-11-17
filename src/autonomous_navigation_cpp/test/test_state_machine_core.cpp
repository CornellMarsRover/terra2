// Port of src/autonomous_navigation/test/test_state_machine_core.py, plus a
// replay of missions recorded from the Python StateMachineNode.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomous_navigation_cpp/state_machine_core.hpp"
#include "golden_data.hpp"
#include "trace_util.hpp"

using autonomous_navigation_cpp::MissionWaypoint;
using autonomous_navigation_cpp::Pt;
using autonomous_navigation_cpp::StateMachineCore;
namespace mission = autonomous_navigation_cpp::mission;

namespace
{
using Pts = std::deque<Pt>;
const double kInf = std::numeric_limits<double>::infinity();
}

TEST(StateMachineCorePy, FarTargetKeepsDetectedObjectTarget)
{
  const auto d = mission::far_target({0, 0}, {20, 0}, {3, 4}, Pts{{5, 0}}, true);
  EXPECT_EQ(d.target, (Pt{3, 4}));
  EXPECT_FALSE(d.pop_coarse);
}

TEST(StateMachineCorePy, FarTargetUsesCurrentCoarseWaypoint)
{
  const auto d = mission::far_target({0, 0}, {20, 0}, {0, 0}, Pts{{5, 0}}, false);
  EXPECT_EQ(d.target, (Pt{5, 0}));
  EXPECT_FALSE(d.pop_coarse);
}

TEST(StateMachineCorePy, FarTargetAdvancesReachedCoarseWaypoint)
{
  auto d = mission::far_target({0, 0}, {20, 0}, {0, 0}, Pts{{1, 0}, {5, 0}}, false);
  EXPECT_EQ(d.target, (Pt{5, 0}));
  EXPECT_TRUE(d.pop_coarse);
  d = mission::far_target({0, 0}, {20, 0}, {0, 0}, Pts{{1, 0}}, false);
  EXPECT_EQ(d.target, (Pt{20, 0}));
  EXPECT_TRUE(d.pop_coarse);
}

TEST(StateMachineCorePy, CoordinateTargetReportsThresholdCrossing)
{
  EXPECT_TRUE(mission::coordinate_target({0, 0}, {1, 0}).reached);
  EXPECT_FALSE(mission::coordinate_target({0, 0}, {3, 0}).reached);
}

TEST(StateMachineCorePy, SearchTargetReportsEmptySearchAsTimeout)
{
  const auto d = mission::search_target({0, 0}, {4, 4}, Pts{});
  EXPECT_EQ(d.target, (Pt{4, 4}));
  EXPECT_TRUE(d.timed_out);
}

TEST(StateMachineCorePy, SearchTargetAnnouncesActiveSearch)
{
  const auto d = mission::search_target({0, 0}, {4, 4}, Pts{{3, 0}});
  EXPECT_EQ(d.target, (Pt{3, 0}));
  EXPECT_TRUE(d.announce_object);
}

TEST(StateMachineCorePy, SearchTargetTimesOutAfterFinalReachedPoint)
{
  const auto d = mission::search_target({1, 0}, {4, 4}, Pts{{1, 0}});
  EXPECT_EQ(d.target, (Pt{4, 4}));
  EXPECT_TRUE(d.timed_out);
  EXPECT_TRUE(d.pop_search);
}

TEST(StateMachineCorePy, SearchTargetAdvancesToFollowingPoint)
{
  const auto d = mission::search_target({1, 0}, {4, 4}, Pts{{1, 0}, {2, 0}});
  EXPECT_EQ(d.target, (Pt{2, 0}));
  EXPECT_TRUE(d.pop_search);
  EXPECT_FALSE(d.announce_object);
}

TEST(StateMachineCorePy, ObjectTargetReportsStandoffCrossing)
{
  EXPECT_TRUE(mission::object_target({0, 0}, {1, 0}).reached);
  EXPECT_FALSE(mission::object_target({0, 0}, {2, 0}).reached);
}

TEST(StateMachineCorePy, SearchWaypointsExpandAndWrapAngles)
{
  const auto points = mission::search_waypoints({10, 20}, 200, 1, 4);
  ASSERT_EQ(points.size(), 3u);
  EXPECT_EQ(points[0], (Pt{11, 20}));
  EXPECT_NE(points[1], points[2]);
}

TEST(StateMachineCorePy, SearchWaypointsRejectsUnsafeRadiusParameters)
{
  const double cases[4][2] = {{0, 1}, {-1, 1}, {kInf, 1}, {1, kInf}};
  for (const auto & c : cases) {
    EXPECT_THROW(mission::search_waypoints({0, 0}, 10, c[0], c[1]), std::invalid_argument);
  }
}

TEST(StateMachineCorePy, NorthWestConversionPreservesExpectedSigns)
{
  const Pt nw = mission::north_west_meters(42.0, -76.0, 42.00001, -76.00001);
  EXPECT_GT(nw.x, 0);
  EXPECT_GT(nw.y, 0);
  EXPECT_EQ(mission::north_west_meters(42, -76, 42, -76), (Pt{0, 0}));
}

TEST(StateMachineCorePy, SelectTargetRoutesEachMissionMode)
{
  struct Case {Pt goal; const char * object; bool found; Pts search; Pt expected;};
  const Case cases[] = {
    {{20, 0}, "coordinate", false, {}, {4, 0}},
    {{1, 0}, "coordinate", false, {}, {1, 0}},
    {{1, 0}, "ar1", false, {{3, 0}}, {3, 0}},
    {{1, 0}, "ar1", true, {}, {2, 0}}};
  for (const Case & c : cases) {
    const auto d = mission::select_target(
      {0, 0}, c.goal, {2, 0}, Pts{{4, 0}}, c.object, c.found, c.search);
    EXPECT_EQ(d.target, c.expected) << c.object;
  }
}

TEST(StateMachineCorePy, SelectTargetRespectsCoordinateThreshold)
{
  EXPECT_FALSE(
    mission::select_target({1.5, 0}, {0, 0}, {0, 0}, {}, "coordinate", false, {}, 10.0, 1.0)
    .reached);
  EXPECT_TRUE(
    mission::select_target({1.5, 0}, {0, 0}, {0, 0}, {}, "coordinate", false, {}, 10.0, 2.0)
    .reached);
}

// ---------------------------------------------------------------------------
// Golden values / traces from the Python implementation
// ---------------------------------------------------------------------------
TEST(StateMachineGolden, SearchPatternAndNorthWestMatchPython)
{
  const auto points = mission::search_waypoints({10.0, 20.0}, 30.0, 0.1, 10.0);
  ASSERT_EQ(points.size() * 2, golden::kSearch.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(points[i].x, golden::kSearch[2 * i]) << i;
    EXPECT_EQ(points[i].y, golden::kSearch[2 * i + 1]) << i;
  }
  const Pt nw = mission::north_west_meters(38.41910142, -110.782723, 38.41928851, -110.7831721);
  EXPECT_EQ(nw.x, golden::kNorthWest[0]);
  EXPECT_EQ(nw.y, golden::kNorthWest[1]);
}

namespace
{
void replay_mission(
  bool real, const std::vector<double> & latlon, const std::vector<const char *> & golden,
  bool skip_distance_logs)
{
  std::vector<MissionWaypoint> waypoints;
  for (std::size_t i = 0; i + 1 < latlon.size(); i += 2) {
    MissionWaypoint w;
    w.latitude = latlon[i];
    w.longitude = latlon[i + 1];
    waypoints.push_back(w);
  }
  std::vector<std::string> sink;
  StateMachineCore::Hooks hooks;
  hooks.log = [&](StateMachineCore::Level level, const std::string & s) {
      if (skip_distance_logs && s.rfind("coordinate distance", 0) == 0) {
        return;
      }
      sink.push_back(
        std::string(level == StateMachineCore::Level::Error ? "log error " : "log info ") + s);
    };
  hooks.publish_target_name = [&](const std::string & name) {
      sink.push_back("pub /autonomy/target_object/name " + name);
    };
  hooks.blink_led = [&]() {sink.push_back("pub /autonomy/led blink");};
  StateMachineCore core(real, waypoints, hooks);
  const std::vector<std::string> initial = sink;
  sink.clear();
  bool announced_done = false;
  trace::replay(
    golden, initial,
    [&](const std::vector<std::string> & t, std::vector<std::string> & out) {
      if (t[0] == "pose") {
        core.set_pose(trace::num(t[1]), trace::num(t[2]), trace::num(t[3]));
      } else if (t[0] == "object") {
        core.on_object(trace::num(t[1]), trace::num(t[2]), trace::num(t[3]));
      } else if (t[0] == "tick") {
        if (!core.control_loop()) {
          if (!announced_done) {
            sink.push_back("log info All waypoints reached.");   // logged once by the node
            announced_done = true;
          }
          sink.push_back("pub /autonomy/stop mission_complete");
        } else {
          const std::vector<double> target = {
            static_cast<double>(static_cast<float>(core.target_position().x)),
            static_cast<double>(static_cast<float>(core.target_position().y))};
          sink.push_back(trace::pub("/autonomy/target/local", target));
          sink.push_back(trace::pub("/autonomy/target/global", target));
        }
      } else {
        FAIL() << "unknown op " << t[0];
      }
      out = sink;
      sink.clear();
    });
  EXPECT_TRUE(announced_done);
}
}  // namespace

TEST(StateMachineGolden, SimMissionMatchesPython)
{
  replay_mission(false, golden::kMissionWaypointsSim, golden::kMissionTraceSim, false);
}

TEST(StateMachineGolden, RealMissionMatchesPython)
{
  replay_mission(true, golden::kMissionWaypointsReal, golden::kMissionTraceReal, true);
}

TEST(StateMachine, CoordinateTypeWaypointsOverrideTargets)
{
  // waypoints[3] is 'ar1' by default; a YAML `type: coordinate` entry turns it
  // into a coordinate goal with its own threshold.
  std::vector<MissionWaypoint> waypoints(5);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    waypoints[i].latitude = 38.0 + 1e-5 * static_cast<double>(i);
    waypoints[i].longitude = -110.0;
  }
  waypoints[3].type = "coordinate";
  waypoints[3].threshold = 0.5;
  std::vector<std::string> logs;
  StateMachineCore::Hooks hooks;
  hooks.log = [&](StateMachineCore::Level, const std::string & s) {logs.push_back(s);};
  StateMachineCore core(false, waypoints, hooks);
  for (int i = 0; i < 50 && core.current_waypoint_index() < 3; ++i) {
    core.set_pose(core.next_coordinate().x, core.next_coordinate().y, 0.0);
    ASSERT_TRUE(core.control_loop());
  }
  ASSERT_EQ(core.current_waypoint_index(), 3);
  EXPECT_EQ(*core.current_object(), "coordinate");
  EXPECT_NE(logs.back().find("(thr=0.5)"), std::string::npos) << logs.back();
  EXPECT_THROW(StateMachineCore(false, {}, hooks), std::invalid_argument);
}
