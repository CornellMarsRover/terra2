// Port of src/autonomous_navigation/test/test_drive_command.py, plus a replay
// of traces recorded from the Python ControllerNode.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomous_navigation_cpp/controller_core.hpp"
#include "autonomous_navigation_cpp/drive_command.hpp"
#include "golden_data.hpp"
#include "trace_util.hpp"

using autonomous_navigation_cpp::ControllerCore;
using autonomous_navigation_cpp::py_radians;
namespace dc = autonomous_navigation_cpp::drive_command;

namespace
{
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
}

TEST(DriveCommand, ForwardCommandUsesNormalizedChassisAxes)
{
  const auto command = dc::forward_heading_command(0.6, 20.0);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.6);
  EXPECT_EQ(command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, py_radians(20.0));
}

TEST(DriveCommand, WaypointReachedIncludesToleranceBoundary)
{
  EXPECT_TRUE(dc::waypoint_reached(0.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(dc::waypoint_reached(0.3, 0.0, 0.0, 0.0));
}

TEST(DriveCommand, WaypointReachedRejectsOutsideAndInvalidTolerance)
{
  EXPECT_FALSE(dc::waypoint_reached(0.31, 0.0, 0.0, 0.0));
  EXPECT_THROW(dc::waypoint_reached(0.0, 0.0, 0.0, 0.0, -0.1), std::invalid_argument);
}

TEST(DriveCommand, ForwardCommandClampsSpeedAndHeading)
{
  const auto command = dc::forward_heading_command(4.0, 90.0);
  EXPECT_EQ(command.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(command.angular_z, py_radians(35.0));
}

TEST(DriveCommand, PointTurnPreservesRosYawSign)
{
  for (double angular_z : {0.4, -0.4, 0.0}) {
    EXPECT_EQ(dc::point_turn_command(angular_z).angular_z, angular_z);
  }
}

TEST(DriveCommand, PointTurnClampsToNormalizedRange)
{
  EXPECT_EQ(dc::point_turn_command(3.0).angular_z, 1.0);
  EXPECT_EQ(dc::point_turn_command(-3.0).angular_z, -1.0);
}

TEST(DriveCommand, NonpositiveHeadingBoundIsRejected)
{
  EXPECT_THROW(dc::forward_heading_command(0.5, 10.0, 0.0), std::invalid_argument);
}

TEST(DriveCommand, ZeroCommandIsAllZero)
{
  dc::ChassisCommand zero;
  zero.linear_x = 0.0; zero.linear_y = 0.0; zero.angular_z = 0.0;
  EXPECT_TRUE(dc::ChassisCommand() == zero);
}

TEST(DriveCommand, RejectsNonfiniteValues)
{
  for (double value : {kNan, kInf, -kInf}) {
    EXPECT_THROW(dc::point_turn_command(value), std::invalid_argument);
    EXPECT_THROW(dc::forward_heading_command(value, 0.0), std::invalid_argument);
  }
}

TEST(DriveCommand, InputFreshnessMatrix)
{
  struct Case {double now; std::optional<double> pose, waypoint; double timeout; bool expected;};
  const Case cases[] = {
    {10, 10.0, 10.0, 1, true}, {10, 9.0, 9.0, 1, true},
    {10, 8.99, 10.0, 1, false}, {10, 10.0, 8.99, 1, false},
    {10, std::nullopt, 10.0, 1, false}, {10, 10.0, std::nullopt, 1, false},
    {10, 10.01, 10.0, 1, false}, {10, 10.0, 10.01, 1, false},
    {10, kNan, 10.0, 1, false}, {10, 10.0, kInf, 1, false}};
  int i = 0;
  for (const Case & c : cases) {
    EXPECT_EQ(dc::inputs_fresh(c.now, c.pose, c.waypoint, c.timeout), c.expected) << i++;
  }
}

TEST(DriveCommand, InputFreshnessRejectsInvalidTimeout)
{
  for (double timeout : {0.0, -1.0, kInf, kNan}) {
    EXPECT_THROW(dc::inputs_fresh(1, 1.0, 1.0, timeout), std::invalid_argument);
  }
}

// ---------------------------------------------------------------------------
// ControllerCore vs. the Python ControllerNode
// ---------------------------------------------------------------------------
namespace
{
void replay_controller(bool real, const std::vector<const char *> & golden)
{
  ControllerCore core(real, 0.3, 1.0, 100LL * 1000000000LL + 123456789LL);
  const double drive_rps = 4.0;
  trace::replay(
    golden, {},
    [&](const std::vector<std::string> & t, std::vector<std::string> & out) {
      if (t[0] == "pose") {
        if (!core.update_pose(trace::num(t[1]), trace::num(t[2]), trace::num(t[3]),
        std::stoll(t[4])))
        {
          out.push_back("log error Ignoring non-finite autonomy pose");
        }
      } else if (t[0] == "wp") {
        if (!core.update_waypoint(trace::floats(t, 2), std::stoll(t[1]))) {
          out.push_back("log error Ignoring malformed autonomy waypoint");
        }
      } else if (t[0] == "stop") {
        core.set_stop();
      } else if (t[0] == "tick") {
        const ControllerCore::Output o = core.step(std::stoll(t[1]));
        out.push_back(trace::pub("/cmd_vel/autonomy",
        std::vector<double>{o.drive.linear_x, o.drive.linear_y, o.drive.angular_z, drive_rps}));
        if (o.movement) {
          out.push_back("pub /autonomy/move/move_type " + *o.movement);
        }
      } else {
        FAIL() << "unknown op " << t[0];
      }
    });
}
}  // namespace

TEST(Controller, RealTraceMatchesPython) {replay_controller(true, golden::kControllerTraceReal);}
TEST(Controller, SimTraceMatchesPython) {replay_controller(false, golden::kControllerTraceSim);}

TEST(Controller, RejectsUnusableParameters)
{
  EXPECT_THROW(ControllerCore(true, 0.3, 0.0, 0), std::invalid_argument);
  EXPECT_THROW(ControllerCore(true, 0.3, kNan, 0), std::invalid_argument);
  EXPECT_THROW(ControllerCore(true, -0.1, 1.0, 0), std::invalid_argument);
}

TEST(Controller, StampMatchesRclpyToMsg)
{
  const auto s = ControllerCore::stamp_from_ns(1234567890123LL);
  EXPECT_EQ(s.sec, 1234);
  EXPECT_EQ(s.nanosec, 567890123u);
  EXPECT_DOUBLE_EQ(ControllerCore::compute_time_delta({5, 100000000u}, {3, 900000000u}), 1.2);
}
