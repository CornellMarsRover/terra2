// Small helpers reproducing Python float semantics used by the original nodes.
#pragma once

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <system_error>

namespace cmr_port
{

// Python's round(x, ndigits): correctly rounded to the nearest decimal with
// ties-to-even on the exact binary value. glibc printf performs exactly this
// rounding, so format and parse back.
inline double py_round(double x, int ndigits)
{
  if (!std::isfinite(x)) {return x;}
  char buf[512];
  std::snprintf(buf, sizeof(buf), "%.*f", ndigits, x);
  return std::strtod(buf, nullptr);
}

// Python's repr(float): shortest round-trip representation.
inline std::string py_float_repr(double x)
{
  if (std::isnan(x)) {return "nan";}
  if (std::isinf(x)) {return x > 0 ? "inf" : "-inf";}
  char buf[64];
  auto res = std::to_chars(buf, buf + sizeof(buf), x);
  std::string s(buf, res.ptr);
  if (s.find_first_of(".en") == std::string::npos) {
    s += ".0";
  }
  return s;
}

}  // namespace cmr_port
