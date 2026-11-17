// Port of autonomous_navigation/target_contract.py: contract between mission
// selection and target perception. Python ValueError -> std::invalid_argument.
#pragma once

#include <optional>
#include <stdexcept>
#include <string>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace autonomous_navigation_cpp
{
namespace target_contract
{

// str.strip() for ASCII whitespace.
inline std::string strip(const std::string & s)
{
  auto is_space = [](unsigned char c) {
      return c == ' ' || (c >= 0x09 && c <= 0x0d) || (c >= 0x1c && c <= 0x1f);
    };
  std::size_t b = 0, e = s.size();
  while (b < e && is_space(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  while (e > b && is_space(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

// Encode the mission step separately from the requested visual target.
inline std::string encode_target_request(long long mission_index, const std::string & name)
{
  const std::string target_name = strip(name);
  if (mission_index < 1 || target_name.empty() || target_name.find(':') != std::string::npos) {
    throw std::invalid_argument("invalid target request");
  }
  return std::to_string(mission_index) + ":" + target_name;
}

struct TargetRequest
{
  long long mission_index;
  std::string target_name;
  std::optional<long long> marker_id;   // ArUco marker ID for "ar<N>" targets
  bool operator==(const TargetRequest & o) const
  {
    return mission_index == o.mission_index && target_name == o.target_name &&
           marker_id == o.marker_id;
  }
};

// Return mission index, target name, and optional ArUco marker ID.
inline TargetRequest decode_target_request(const std::string & value)
{
  const std::size_t colon = value.find(':');
  if (colon == std::string::npos) {
    throw std::invalid_argument("target request must be '<mission>:<name>'");
  }
  const std::optional<long long> mission_index = py_int(value.substr(0, colon));
  if (!mission_index) {
    throw std::invalid_argument("target request must be '<mission>:<name>'");
  }
  TargetRequest out;
  out.mission_index = *mission_index;
  out.target_name = value.substr(colon + 1);
  if (out.mission_index < 1 || out.target_name.empty() ||
    out.target_name.find(':') != std::string::npos)
  {
    throw std::invalid_argument("invalid target request");
  }
  if (out.target_name.rfind("ar", 0) == 0) {
    const std::optional<long long> marker = py_int(out.target_name.substr(2));
    if (!marker) {
      throw std::invalid_argument("ArUco target must end in a numeric ID");
    }
    if (*marker < 0) {
      throw std::invalid_argument("ArUco marker ID must not be negative");
    }
    out.marker_id = *marker;
  }
  return out;
}

}  // namespace target_contract
}  // namespace autonomous_navigation_cpp
