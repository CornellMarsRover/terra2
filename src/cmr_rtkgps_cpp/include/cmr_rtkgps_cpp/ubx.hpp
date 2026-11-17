// Minimal UBX encode/decode: just the subset of pyubx2 that cmr_rtkgps/rover.py uses
// (CFG-VALDEL / CFG-VALSET construction and NAV position message decoding).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cmr_rtkgps_cpp
{

constexpr uint8_t UBX_SYNC1 = 0xB5;
constexpr uint8_t UBX_SYNC2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_CLASS_CFG = 0x06;
constexpr uint8_t UBX_CLASS_NAV2 = 0x29;
constexpr uint8_t UBX_ID_CFG_VALSET = 0x8A;
constexpr uint8_t UBX_ID_CFG_VALDEL = 0x8C;
constexpr uint8_t UBX_ID_NAV_POSLLH = 0x02;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
constexpr uint8_t UBX_ID_NAV_HPPOSLLH = 0x14;
constexpr uint8_t UBX_ID_NAV_PVAT = 0x17;

// Configuration database item (key ID + storage size in bytes), from
// pyubx2 ubxtypes_configdb.py.
struct CfgKey
{
  const char * name;
  uint32_t id;
  uint8_t size;     // 1 (L/U1/E1), 2 (U2), 4 (U4/I4)
  bool is_signed;
};

// Throws std::invalid_argument for keys not in the (small) local table.
const CfgKey & cfg_key(const std::string & name);

// Full UBX frame (sync chars, class, id, length, payload, Fletcher checksum).
std::vector<uint8_t> ubx_frame(uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload);
void ubx_checksum(const uint8_t * data, size_t len, uint8_t & ck_a, uint8_t & ck_b);

// UBXMessage.config_del(layers, transaction, keys).serialize()
std::vector<uint8_t> config_del(int layers, int transaction, const std::vector<std::string> & keys);

// UBXMessage.config_set(layers, transaction, cfgData).serialize()
std::vector<uint8_t> config_set(
  int layers, int transaction, const std::vector<std::pair<std::string, int64_t>> & cfgData);

// Decoded lat/lon/hAcc of a NAV / NAV2 message, scaled exactly like pyubx2.
struct NavPosition
{
  std::string identity;  // e.g. "NAV-PVT"
  double lat;            // degrees
  double lon;            // degrees
  double hAcc;           // mm
  bool hAcc_is_int;      // pyubx2 returns int for unscaled U4 fields
};

// Returns the identity string ("NAV-PVT", "NAV2-POSLLH", ...) or "UBX-xx-yy".
std::string ubx_identity(uint8_t cls, uint8_t id);

// Decodes NAV-PVT / NAV-POSLLH / NAV-HPPOSLLH / NAV-PVAT (and the NAV2 variants
// of PVT / POSLLH / PVAT).
// Returns nullopt for any other message or a malformed payload.
std::optional<NavPosition> decode_nav_position(
  uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload);

}  // namespace cmr_rtkgps_cpp
