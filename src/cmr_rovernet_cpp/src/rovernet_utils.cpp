#include "cmr_rovernet_cpp/rovernet_utils.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace cmr_rovernet_cpp {
namespace {

// Little-endian float32 encoder (matches struct.pack('f') on the Jetson).
void put_f32(uint8_t * out, double value)
{
  // struct.pack('f', v) raises OverflowError for finite values outside the
  // float32 range (inf / nan pass through unchanged).
  if (std::isfinite(value)) {
    const float rounded = static_cast<float>(value);
    if (std::isinf(rounded)) {
      throw std::overflow_error("float too large to pack with f format");
    }
  }
  const float f = static_cast<float>(value);
  uint32_t bits = 0;
  std::memcpy(&bits, &f, sizeof(bits));
  out[0] = static_cast<uint8_t>(bits & 0xFF);
  out[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
}

float get_f32(const uint8_t * in)
{
  const uint32_t bits = static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
    (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

std::size_t put_opt_f32(uint8_t * out, const std::optional<double> & value)
{
  if (value) {
    put_f32(out, *value);
  } else {
    std::memset(out, 0xFF, 4);
  }
  return 4;
}

}  // namespace

CommandBytes byte_command_converter(
  uint8_t subteam, uint8_t motor, std::optional<double> position,
  std::optional<double> drives_velocity, std::optional<double> max_torque,
  std::optional<double> max_vel, std::optional<double> max_accel,
  std::optional<double> ff_torque)
{
  if (subteam < DRIVES || subteam > RECEIVE_DATA) {
    throw std::invalid_argument("Invalid Subteam in command");
  }

  CommandBytes out{};  // zero padded to 40 bytes (ljust(40, b'\x00'))
  std::size_t i = 0;
  out[i++] = subteam;
  out[i++] = motor;
  i += put_opt_f32(&out[i], position);
  i += put_opt_f32(&out[i], drives_velocity);
  out[i++] = drives_velocity ? static_cast<uint8_t>(*drives_velocity < 0) : 0xFF;
  i += put_opt_f32(&out[i], max_torque);
  i += put_opt_f32(&out[i], max_vel);
  i += put_opt_f32(&out[i], max_accel);
  i += put_opt_f32(&out[i], ff_torque);
  return out;
}

std::optional<CcbReply> parse_ccb_reply(const uint8_t * data, std::size_t size)
{
  if (size != kReplySize) {
    return std::nullopt;
  }
  CcbReply r;
  r.subteam = data[0];
  r.motor_id = data[1];
  r.mode = data[2];
  r.position = get_f32(&data[3]);
  r.velocity = get_f32(&data[7]);
  r.torque = get_f32(&data[11]);
  r.current = get_f32(&data[15]);
  r.fault = data[19];
  return r;
}

}  // namespace cmr_rovernet_cpp
