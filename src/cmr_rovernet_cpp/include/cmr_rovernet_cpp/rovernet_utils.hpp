// C++ port of the shared, hardware-independent parts of
// cmr_rovernet/rovernet_utils.py: subteam / motor IDs, controller button
// constants, the 40-byte CCB command encoder and the 20-byte CCB reply decoder.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace cmr_rovernet_cpp {

// VALID SUBTEAMS
constexpr uint8_t DRIVES = 0x01;
constexpr uint8_t ARM = 0x02;
constexpr uint8_t ASTROTECH = 0x03;
constexpr uint8_t RECEIVE_DATA = 0x04;
constexpr uint8_t BRAKE = 0x08;

// VALID MOTORS
constexpr uint8_t FRONT_LEFT = 0x01;
constexpr uint8_t BACK_RIGHT = 0x04;
constexpr uint8_t FRONT_RIGHT = 0x03;
constexpr uint8_t BACK_LEFT = 0x02;

// VALID ARM MOTORS
constexpr uint8_t ARM_BASE = 0x09;
constexpr uint8_t ARM_SHOULDER = 0x0A;
constexpr uint8_t ARM_ELBOW = 0x0B;
constexpr uint8_t WRIST_ROTATE_1 = 0x0C;
constexpr uint8_t WRIST_ROTATE_2 = 0x0E;
constexpr uint8_t WRIST_TILT = 0x0D;

// TRIGGERS - INDEX 0 OF BUTTON_ARRAY (legacy packed format)
constexpr int64_t L1 = 1;
constexpr int64_t R1 = 256;
constexpr int64_t L2_MIN = 65536;
constexpr int64_t L2 = 16711680;
constexpr int64_t R2_MIN = 26711680;
constexpr int64_t R2 = -16777216;

// BUTTONS - INDEX 1 OF BUTTON_ARRAY (legacy packed format)
constexpr int64_t SQUARE = 1;
constexpr int64_t X = 256;
constexpr int64_t CIRCLE = 65536;
constexpr int64_t TRIANGLE = 16777216;

// SWERVE MOTORS
constexpr uint8_t FRONT_LEFT_SWERVE = 0x05;
constexpr uint8_t BACK_RIGHT_SWERVE = 0x08;
constexpr uint8_t FRONT_RIGHT_SWERVE = 0x07;
constexpr uint8_t BACK_LEFT_SWERVE = 0x06;

constexpr std::size_t kCommandSize = 40;
using CommandBytes = std::array<uint8_t, kCommandSize>;

/// Port of rovernet_utils.byte_command_converter.  std::nullopt fields are
/// encoded as 0xFF bytes exactly like the Python `None` handling.  Floats are
/// packed little-endian float32 (Python `struct.pack('f')` on aarch64/x86).
/// Throws std::invalid_argument for an invalid subteam (Python ValueError)
/// and std::overflow_error for finite values outside float32 range
/// (struct.pack OverflowError).
CommandBytes byte_command_converter(
  uint8_t subteam, uint8_t motor, std::optional<double> position,
  std::optional<double> drives_velocity, std::optional<double> max_torque,
  std::optional<double> max_vel, std::optional<double> max_accel,
  std::optional<double> ff_torque);

constexpr std::size_t kReplySize = 20;

/// struct.unpack('<BBBffffB', data) of a CCB reply.
struct CcbReply
{
  uint8_t subteam = 0;
  uint8_t motor_id = 0;
  uint8_t mode = 0;
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float current = 0.0f;
  uint8_t fault = 0;
};

/// Returns std::nullopt if @p size != 20 (Python struct.error).
std::optional<CcbReply> parse_ccb_reply(const uint8_t * data, std::size_t size);

/// rovernet_utils.scale_value
inline double scale_value(double value, double old_min, double old_max, double new_min,
  double new_max)
{
  return (new_max - new_min) * (value - old_min) / (old_max - old_min) + new_min;
}

}  // namespace cmr_rovernet_cpp
