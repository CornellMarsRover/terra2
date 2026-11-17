// Minimal UBX encode/decode (pyubx2 subset used by rover.py).
#include "cmr_rtkgps_cpp/ubx.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "cmr_rtkgps_cpp/py_compat.hpp"

namespace cmr_rtkgps_cpp
{

namespace
{

// Values copied from pyubx2 1.4.1 ubxtypes_configdb.py.
const CfgKey kCfgKeys[] = {
  {"CFG_UART1_BAUDRATE", 0x40520001, 4, false},
  {"CFG_TMODE_MODE", 0x20030001, 1, false},
  {"CFG_TMODE_POS_TYPE", 0x20030002, 1, false},
  {"CFG_TMODE_ECEF_X", 0x40030003, 4, true},
  {"CFG_TMODE_ECEF_Y", 0x40030004, 4, true},
  {"CFG_TMODE_ECEF_Z", 0x40030005, 4, true},
  {"CFG_USBINPROT_RTCM3X", 0x10770004, 1, false},
  {"CFG_USBOUTPROT_UBX", 0x10780001, 1, false},
  {"CFG_USBOUTPROT_NMEA", 0x10780002, 1, false},
  {"CFG_USBOUTPROT_RTCM3X", 0x10780004, 1, false},
  {"CFG_MSGOUT_UBX_NAV_PVT_USB", 0x20910009, 1, false},
  {"CFG_MSGOUT_UBX_NAV_SVIN_USB", 0x2091008B, 1, false},
  {"CFG_MSGOUT_NMEA_NAV2_ID_GGA_USB", 0x20910664, 1, false},
  {"CFG_MSGOUT_NMEA_NAV2_ID_GLL_USB", 0x20910673, 1, false},
  {"CFG_MSGOUT_NMEA_NAV2_ID_GNS_USB", 0x2091065F, 1, false},
  {"CFG_MSGOUT_NMEA_NAV2_ID_GSA_USB", 0x20910669, 1, false},
  {"CFG_RATE_MEAS", 0x30210001, 2, false},
};

void put_le(std::vector<uint8_t> & out, uint64_t v, size_t n)
{
  for (size_t i = 0; i < n; ++i) {
    out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
}

uint32_t get_u4(const std::vector<uint8_t> & p, size_t off)
{
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}

int32_t get_i4(const std::vector<uint8_t> & p, size_t off)
{
  return static_cast<int32_t>(get_u4(p, off));
}

std::vector<uint8_t> cfg_header(int layers, int transaction)
{
  std::vector<uint8_t> payload;
  payload.push_back(transaction == 0 ? 0 : 1);  // version
  payload.push_back(static_cast<uint8_t>(layers));
  payload.push_back(static_cast<uint8_t>(transaction));
  payload.push_back(0);  // reserved
  return payload;
}

// round(val * scale, SCALROUND=12) as pyubx2 does for scaled attributes.
double scaled(double raw, double scale)
{
  return cmr_port::py_round(raw * scale, 12);
}

}  // namespace

const CfgKey & cfg_key(const std::string & name)
{
  for (const auto & k : kCfgKeys) {
    if (name == k.name) {
      return k;
    }
  }
  throw std::invalid_argument("Undefined configuration database key " + name);
}

void ubx_checksum(const uint8_t * data, size_t len, uint8_t & ck_a, uint8_t & ck_b)
{
  uint8_t a = 0, b = 0;
  for (size_t i = 0; i < len; ++i) {
    a = static_cast<uint8_t>(a + data[i]);
    b = static_cast<uint8_t>(b + a);
  }
  ck_a = a;
  ck_b = b;
}

std::vector<uint8_t> ubx_frame(uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> out = {UBX_SYNC1, UBX_SYNC2, cls, id};
  put_le(out, payload.size(), 2);
  out.insert(out.end(), payload.begin(), payload.end());
  uint8_t a, b;
  ubx_checksum(out.data() + 2, out.size() - 2, a, b);
  out.push_back(a);
  out.push_back(b);
  return out;
}

std::vector<uint8_t> config_del(int layers, int transaction, const std::vector<std::string> & keys)
{
  if (keys.size() > 64) {
    throw std::invalid_argument("Number of configuration keys exceeds maximum of 64");
  }
  auto payload = cfg_header(layers, transaction);
  for (const auto & k : keys) {
    put_le(payload, cfg_key(k).id, 4);
  }
  return ubx_frame(UBX_CLASS_CFG, UBX_ID_CFG_VALDEL, payload);
}

std::vector<uint8_t> config_set(
  int layers, int transaction, const std::vector<std::pair<std::string, int64_t>> & cfgData)
{
  if (cfgData.size() > 64) {
    throw std::invalid_argument("Number of configuration tuples exceeds maximum of 64");
  }
  auto payload = cfg_header(layers, transaction);
  for (const auto & [name, val] : cfgData) {
    const auto & k = cfg_key(name);
    put_le(payload, k.id, 4);
    put_le(payload, static_cast<uint64_t>(val), k.size);  // two's complement for I4
  }
  return ubx_frame(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload);
}

std::string ubx_identity(uint8_t cls, uint8_t id)
{
  const char * prefix = cls == UBX_CLASS_NAV ? "NAV" : cls == UBX_CLASS_NAV2 ? "NAV2" : nullptr;
  if (prefix) {
    if (id == UBX_ID_NAV_PVT) {return std::string(prefix) + "-PVT";}
    if (id == UBX_ID_NAV_POSLLH) {return std::string(prefix) + "-POSLLH";}
    if (id == UBX_ID_NAV_PVAT) {return std::string(prefix) + "-PVAT";}
    if (id == UBX_ID_NAV_HPPOSLLH && cls == UBX_CLASS_NAV) {return "NAV-HPPOSLLH";}
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "UBX-%02X-%02X", cls, id);
  return buf;
}

std::optional<NavPosition> decode_nav_position(
  uint8_t cls, uint8_t id, const std::vector<uint8_t> & p)
{
  if (cls != UBX_CLASS_NAV && cls != UBX_CLASS_NAV2) {
    return std::nullopt;
  }
  NavPosition r{};
  r.identity = ubx_identity(cls, id);
  if (id == UBX_ID_NAV_PVT) {
    if (p.size() != 92) {return std::nullopt;}
    r.lon = scaled(get_i4(p, 24), 1e-7);
    r.lat = scaled(get_i4(p, 28), 1e-7);
    r.hAcc = get_u4(p, 40);
    r.hAcc_is_int = true;
    return r;
  }
  if (id == UBX_ID_NAV_POSLLH) {
    if (p.size() != 28) {return std::nullopt;}
    r.lon = scaled(get_i4(p, 4), 1e-7);
    r.lat = scaled(get_i4(p, 8), 1e-7);
    r.hAcc = get_u4(p, 20);
    r.hAcc_is_int = true;
    return r;
  }
  if (id == UBX_ID_NAV_PVAT) {
    // pyubx2 NAV-PVAT: lon I4 @28, lat I4 @32 (1e-7), hAcc U4 @44; 116 bytes.
    if (p.size() != 116) {return std::nullopt;}
    r.lon = scaled(get_i4(p, 28), 1e-7);
    r.lat = scaled(get_i4(p, 32), 1e-7);
    r.hAcc = get_u4(p, 44);
    r.hAcc_is_int = true;
    return r;
  }
  if (id == UBX_ID_NAV_HPPOSLLH && cls == UBX_CLASS_NAV) {
    if (p.size() != 36) {return std::nullopt;}
    const double lon = scaled(get_i4(p, 8), 1e-7);
    const double lat = scaled(get_i4(p, 12), 1e-7);
    const double hp_lon = scaled(static_cast<int8_t>(p[24]), 1e-9);
    const double hp_lat = scaled(static_cast<int8_t>(p[25]), 1e-9);
    r.lon = cmr_port::py_round(lon + hp_lon, 12);
    r.lat = cmr_port::py_round(lat + hp_lat, 12);
    r.hAcc = scaled(get_u4(p, 28), 0.1);
    r.hAcc_is_int = false;
    return r;
  }
  // Other NAV messages have no lat/lon/hAcc: Python's AttributeError is swallowed.
  return std::nullopt;
}

}  // namespace cmr_rtkgps_cpp
