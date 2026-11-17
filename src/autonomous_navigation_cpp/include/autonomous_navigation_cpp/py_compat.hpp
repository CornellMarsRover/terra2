// Small helpers that reproduce Python / numpy numeric semantics exactly.
#pragma once

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

namespace autonomous_navigation_cpp
{

// Python 3 round() on a float is round-half-to-even. std::nearbyint uses the
// current rounding mode, which is FE_TONEAREST (= half-to-even) by default.
inline double py_round(double v) {return std::nearbyint(v);}

// Python built-in min(a, b) / max(a, b) (first argument wins ties / NaNs).
inline double py_min(double a, double b) {return (b < a) ? b : a;}
inline double py_max(double a, double b) {return (b > a) ? b : a;}

// math.degrees(): CPython computes x * (180 / pi); math.radians(): x * (pi / 180).
inline double py_degrees(double rad)
{
  constexpr double kRadToDeg = 180.0 / M_PI;
  return rad * kRadToDeg;
}
inline double py_radians(double deg)
{
  constexpr double kDegToRad = M_PI / 180.0;
  return deg * kDegToRad;
}

// math.hypot(a, b) / math.dist() for two dimensions. CPython (3.10) does not
// call libm hypot(): it uses its own correctly-rounded vector_norm(), and
// glibc's hypot() differs from it in the last bit for ~12% of inputs. The
// planner sorts and tie-breaks on these distances, so port the algorithm.
// Requires -ffp-contract=off (set in CMakeLists.txt).
inline double py_hypot(double a, double b)
{
  const double T27 = 134217729.0;  // 2**27 + 1 (Veltkamp split)
  const double v[2] = {std::fabs(a), std::fabs(b)};
  if (std::isinf(v[0]) || std::isinf(v[1])) {
    return std::numeric_limits<double>::infinity();
  }
  if (std::isnan(v[0]) || std::isnan(v[1])) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double max = v[0] > v[1] ? v[0] : v[1];
  if (max == 0.0) {
    return max;
  }
  int max_e;
  std::frexp(max, &max_e);
  double csum = 1.0, frac1 = 0.0, frac2 = 0.0, x, t, hi, lo, oldcsum;
  if (max_e < -1023) {
    // Subnormal range: divide by max instead of scaling.
    for (double e : v) {
      x = e / max;
      x = x * x;
      oldcsum = csum;
      csum += x;
      frac1 += (oldcsum - csum) + x;
    }
    return max * std::sqrt(csum - 1.0 + frac1);
  }
  const double scale = std::ldexp(1.0, -max_e);
  auto accumulate = [&](double term, double & frac) {
      oldcsum = csum;
      csum += term;
      frac += (oldcsum - csum) + term;
    };
  for (double e : v) {
    x = e * scale;
    t = x * T27;
    hi = t - (t - x);
    lo = x - hi;
    accumulate(hi * hi, frac1);
    accumulate(2.0 * hi * lo, frac2);
    accumulate(lo * lo, frac2);
  }
  const double h = std::sqrt(csum - 1.0 + (frac1 + frac2));
  t = h * T27;
  hi = t - (t - h);
  lo = h - hi;
  accumulate(-hi * hi, frac1);
  accumulate(-2.0 * hi * lo, frac2);
  accumulate(-lo * lo, frac2);
  x = csum - 1.0 + (frac1 + frac2);
  return (h + x / (2.0 * h)) / scale;
}

// repr(float): shortest round-trip digits; fixed notation for
// 1e-4 <= |v| < 1e16, otherwise scientific with a >= 2 digit exponent.
inline std::string py_repr(double v)
{
  if (std::isnan(v)) {
    return "nan";
  }
  if (std::isinf(v)) {
    return v < 0 ? "-inf" : "inf";
  }
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::scientific);
  const std::string sci(buf, res.ptr);          // [-]d[.ddd]e[+-]XX
  const std::size_t epos = sci.find('e');
  const bool neg = sci[0] == '-';
  std::string digits;
  for (std::size_t i = neg ? 1 : 0; i < epos; ++i) {
    if (sci[i] != '.') {
      digits += sci[i];
    }
  }
  const int exp = std::atoi(sci.c_str() + epos + 1);
  std::string out = neg ? "-" : "";
  const int nd = static_cast<int>(digits.size());
  if (exp >= -4 && exp < 16) {
    if (exp < 0) {
      out += "0." + std::string(static_cast<std::size_t>(-exp - 1), '0') + digits;
    } else if (nd <= exp + 1) {
      out += digits + std::string(static_cast<std::size_t>(exp + 1 - nd), '0') + ".0";
    } else {
      out += digits.substr(0, static_cast<std::size_t>(exp + 1)) + "." +
        digits.substr(static_cast<std::size_t>(exp + 1));
    }
  } else {
    out += digits.substr(0, 1);
    if (nd > 1) {
      out += "." + digits.substr(1);
    }
    const int ae = std::abs(exp);
    out += std::string("e") + (exp < 0 ? "-" : "+") + (ae < 10 ? "0" : "") + std::to_string(ae);
  }
  return out;
}

// int(str) for the ASCII subset: optional surrounding whitespace, optional
// sign, decimal digits with single underscores between digits.
inline std::optional<long long> py_int(const std::string & text)
{
  auto is_space = [](unsigned char c) {
      return c == ' ' || (c >= 0x09 && c <= 0x0d) || (c >= 0x1c && c <= 0x1f);
    };
  std::size_t b = 0, e = text.size();
  while (b < e && is_space(static_cast<unsigned char>(text[b]))) {
    ++b;
  }
  while (e > b && is_space(static_cast<unsigned char>(text[e - 1]))) {
    --e;
  }
  bool neg = false;
  if (b < e && (text[b] == '+' || text[b] == '-')) {
    neg = text[b] == '-';
    ++b;
  }
  if (b >= e) {
    return std::nullopt;
  }
  long long value = 0;
  bool prev_digit = false;
  for (std::size_t i = b; i < e; ++i) {
    const char c = text[i];
    if (c == '_') {
      if (!prev_digit || i + 1 == e) {
        return std::nullopt;
      }
      prev_digit = false;
      continue;
    }
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    if (value > (std::numeric_limits<long long>::max() - (c - '0')) / 10) {
      return std::nullopt;  // NOTE(port): Python ints are unbounded
    }
    value = value * 10 + (c - '0');
    prev_digit = true;
  }
  if (!prev_digit) {
    return std::nullopt;
  }
  return neg ? -value : value;
}

// Pack two signed 32-bit quarter-cell indices into one hash key.
inline int64_t cell_key(int32_t ix, int32_t iy)
{
  return static_cast<int64_t>((static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(iy)));
}
inline int32_t key_x(int64_t k) {return static_cast<int32_t>(static_cast<uint64_t>(k) >> 32);}
inline int32_t key_y(int64_t k) {return static_cast<int32_t>(static_cast<uint64_t>(k) & 0xffffffffULL);}

// True if v fits comfortably in an int32 cell index after rounding.
inline bool index_in_range(double v) {return std::fabs(v) < 2.0e9;}

}  // namespace autonomous_navigation_cpp
