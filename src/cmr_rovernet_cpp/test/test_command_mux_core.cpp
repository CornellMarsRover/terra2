// gtest port of cmr_rovernet/test/test_command_mux_core.py (same cases, same
// order), plus a few extra checks of the Python semantics the node relies on.
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "cmr_rovernet_cpp/command_mux_core.hpp"

using cmr_rovernet_cpp::CommandMux;
using Values = CommandMux::Values;

namespace {
const std::optional<Values> kNone = std::nullopt;
}

TEST(CommandMuxCore, SelectedZeroDoesNotFallThrough)
{
  CommandMux mux;
  mux.receive("teleop", {0, 0, 0}, 1);
  mux.receive("autonomy", {1, 0, 0}, 1);
  EXPECT_EQ(mux.output(1.1), std::optional<Values>(Values{0, 0, 0}));
  EXPECT_EQ(mux.output(1.5), kNone);
}

TEST(CommandMuxCore, SwitchRequiresFreshSelectedInput)
{
  CommandMux mux;
  mux.receive("autonomy", {1}, 1);
  mux.select("autonomy");
  EXPECT_EQ(mux.output(1.1), kNone);
  mux.receive("autonomy", {2}, 1.2);
  EXPECT_EQ(mux.output(1.3), std::optional<Values>(Values{2}));
}

TEST(CommandMuxCore, EstopDiscardsCommandsUntilResetAndFreshInput)
{
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  mux.emergency_stop();
  mux.receive("teleop", {2}, 1.1);
  EXPECT_EQ(mux.output(1.2), kNone);
  mux.reset();
  EXPECT_EQ(mux.output(1.2), kNone);
  mux.receive("teleop", {0}, 1.3);
  EXPECT_EQ(mux.output(1.4), std::optional<Values>(Values{0}));
}

class CommandMuxBadValue : public ::testing::TestWithParam<double> {};

TEST_P(CommandMuxBadValue, InvalidInputClearsPreviousMotion)
{
  const double bad = GetParam();
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  EXPECT_FALSE(mux.receive("teleop", {bad}, 1.1));
  EXPECT_EQ(mux.output(1.2), kNone);
  EXPECT_THROW(CommandMux{bad}, std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(
  NonFinite, CommandMuxBadValue,
  ::testing::Values(
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()));

// ---- Extra checks (not in the Python test file; behaviour verified against
// ---- command_mux_core.py by test/compare_swerve_with_python.py's mux trace).

TEST(CommandMuxCoreExtra, TimeoutMustBePositive)
{
  EXPECT_THROW(CommandMux{0.0}, std::invalid_argument);
  EXPECT_THROW(CommandMux{-0.5}, std::invalid_argument);
  EXPECT_NO_THROW(CommandMux{1e-9});
}

TEST(CommandMuxCoreExtra, TimeoutWindowIsHalfOpenAndRejectsTimeGoingBackwards)
{
  CommandMux mux(0.5);
  mux.receive("teleop", {1, 2, 3, 4}, 10.0);
  EXPECT_TRUE(mux.output(10.0).has_value());    // age 0 is fresh
  EXPECT_TRUE(mux.output(10.25).has_value());
  EXPECT_FALSE(mux.output(10.5).has_value());   // age == timeout is stale
  EXPECT_FALSE(mux.output(9.75).has_value());   // negative age is stale
  EXPECT_FALSE(mux.output(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(mux.output(std::numeric_limits<double>::infinity()).has_value());
}

TEST(CommandMuxCoreExtra, UnknownSourcesThrow)
{
  CommandMux mux;
  EXPECT_THROW(mux.select("tank"), std::invalid_argument);
  EXPECT_THROW(mux.select(""), std::invalid_argument);
  EXPECT_THROW(mux.receive("tank", {1}, 1), std::invalid_argument);
  EXPECT_EQ(mux.source(), "teleop");
}

TEST(CommandMuxCoreExtra, FailedSelectKeepsSourceAndCommands)
{
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  EXPECT_THROW(mux.select("tank"), std::invalid_argument);
  EXPECT_EQ(mux.output(1.1), std::optional<Values>(Values{1}));
}

TEST(CommandMuxCoreExtra, ReselectingSameSourceDiscardsItsCommand)
{
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  mux.select("teleop");
  EXPECT_EQ(mux.output(1.1), kNone);
}

TEST(CommandMuxCoreExtra, NonFiniteTimeErasesAndIsRejected)
{
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  EXPECT_FALSE(mux.receive("teleop", {1}, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(mux.output(1.1), kNone);
}

TEST(CommandMuxCoreExtra, NonFiniteOnOtherSourceLeavesSelectedCommand)
{
  CommandMux mux;
  mux.receive("teleop", {1}, 1);
  EXPECT_FALSE(mux.receive("autonomy", {std::numeric_limits<double>::infinity()}, 1.1));
  EXPECT_EQ(mux.output(1.2), std::optional<Values>(Values{1}));
}

TEST(CommandMuxCoreExtra, EstopAcceptsButDoesNotStore)
{
  CommandMux mux;
  mux.emergency_stop();
  EXPECT_TRUE(mux.estop());
  EXPECT_TRUE(mux.receive("teleop", {1}, 1));   // accepted (True) ...
  mux.reset();
  EXPECT_FALSE(mux.estop());
  EXPECT_EQ(mux.output(1.1), kNone);            // ... but never stored
}

TEST(CommandMuxCoreExtra, SelectDuringEstopKeepsEstop)
{
  CommandMux mux;
  mux.emergency_stop();
  mux.select("autonomy");
  EXPECT_TRUE(mux.estop());
  mux.receive("autonomy", {1}, 1);
  EXPECT_EQ(mux.output(1.1), kNone);
}
