// gtest port of cmr_rovernet/test/test_drive_modes.py (same cases).
#include <array>
#include <stdexcept>

#include <gtest/gtest.h>

#include "cmr_rovernet_cpp/drive_modes.hpp"

using cmr_rovernet_cpp::DriveMode;
using cmr_rovernet_cpp::drive_mode_for_dpad;
using cmr_rovernet_cpp::drive_mode_value;
using cmr_rovernet_cpp::parse_drive_mode;
using cmr_rovernet_cpp::shape_motion_for_mode;

using Motion = std::array<double, 3>;

TEST(DriveModes, CardinalDpadSelectsEachDriveMode)
{
  EXPECT_EQ(drive_mode_for_dpad(0), DriveMode::TRANSLATION_ROTATION);
  EXPECT_EQ(drive_mode_for_dpad(2), DriveMode::ACKERMANN);
  EXPECT_EQ(drive_mode_for_dpad(4), DriveMode::POINT_TURN);
  EXPECT_EQ(drive_mode_for_dpad(6), DriveMode::STEADY_HEADING);
}

TEST(DriveModes, NeutralDiagonalAndUnknownDpadDoNotChangeMode)
{
  for (int dpad : {1, 3, 5, 7, 8, 99}) {
    EXPECT_FALSE(drive_mode_for_dpad(dpad).has_value()) << "dpad=" << dpad;
  }
  EXPECT_FALSE(drive_mode_for_dpad(-1).has_value());
}

TEST(DriveModes, TranslationRotationPreservesAllAxes)
{
  EXPECT_EQ(shape_motion_for_mode("translation_rotation", 0.7, -0.3, 0.4), (Motion{0.7, -0.3, 0.4}));
}

TEST(DriveModes, SteadyHeadingRemovesRotationOnly)
{
  EXPECT_EQ(
    shape_motion_for_mode(DriveMode::STEADY_HEADING, 0.7, -0.3, 0.4), (Motion{0.7, -0.3, 0.0}));
}

TEST(DriveModes, PointTurnRemovesTranslationOnly)
{
  EXPECT_EQ(
    shape_motion_for_mode(DriveMode::POINT_TURN, 0.7, -0.3, 0.4), (Motion{0.0, 0.0, 0.4}));
}

TEST(DriveModes, AckermannRemovesStrafeAndPreventsStationarySpin)
{
  EXPECT_EQ(
    shape_motion_for_mode(DriveMode::ACKERMANN, 0.5, -0.3, 0.8), (Motion{0.5, 0.0, 0.4}));
  EXPECT_EQ(
    shape_motion_for_mode(DriveMode::ACKERMANN, 0.0, 0.0, 0.8), (Motion{0.0, 0.0, 0.0}));
}

TEST(DriveModes, InvalidModeIsRejected)
{
  EXPECT_THROW(parse_drive_mode("tank"), std::invalid_argument);
  EXPECT_THROW(shape_motion_for_mode("tank", 0.0, 0.0, 0.0), std::invalid_argument);
}

// ---- Extra: parse_drive_mode is `DriveMode(str(value).strip().lower())`.
TEST(DriveModesExtra, ParseStripsAndLowercases)
{
  EXPECT_EQ(parse_drive_mode("  Ackermann\n"), DriveMode::ACKERMANN);
  EXPECT_EQ(parse_drive_mode("POINT_TURN"), DriveMode::POINT_TURN);
  EXPECT_EQ(parse_drive_mode("steady_heading"), DriveMode::STEADY_HEADING);
  EXPECT_EQ(parse_drive_mode("translation_rotation"), DriveMode::TRANSLATION_ROTATION);
  EXPECT_THROW(parse_drive_mode(""), std::invalid_argument);
  EXPECT_THROW(parse_drive_mode("point turn"), std::invalid_argument);
  EXPECT_THROW(parse_drive_mode("TRANSLATION_ROTATION_"), std::invalid_argument);
}

TEST(DriveModesExtra, ValuesRoundTrip)
{
  for (DriveMode m : {DriveMode::TRANSLATION_ROTATION, DriveMode::ACKERMANN,
      DriveMode::STEADY_HEADING, DriveMode::POINT_TURN})
  {
    EXPECT_EQ(parse_drive_mode(drive_mode_value(m)), m);
  }
}

TEST(DriveModesExtra, AckermannReverseKeepsYawSign)
{
  // omega * abs(vx): reversing does not flip the commanded yaw sign.
  EXPECT_EQ(
    shape_motion_for_mode(DriveMode::ACKERMANN, -0.5, 0.3, 0.8), (Motion{-0.5, 0.0, 0.4}));
}
