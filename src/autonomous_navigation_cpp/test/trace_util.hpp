// Helpers for replaying golden traces recorded from the Python nodes.
// A trace is a list of lines: "> op args..." is an input applied to the node
// logic; the lines up to the next "> " are everything the Python node
// published / logged in response, formatted exactly as these helpers do.
#pragma once

#include <gtest/gtest.h>

#include <cstdlib>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "autonomous_navigation_cpp/py_compat.hpp"

namespace trace
{

inline std::vector<std::string> split(const std::string & s)
{
  std::istringstream in(s);
  std::vector<std::string> out;
  for (std::string tok; in >> tok; ) {
    out.push_back(tok);
  }
  return out;
}

inline double num(const std::string & s) {return std::strtod(s.c_str(), nullptr);}

inline std::vector<float> floats(const std::vector<std::string> & tok, std::size_t first)
{
  std::vector<float> out;
  for (std::size_t i = first; i < tok.size(); ++i) {
    out.push_back(static_cast<float>(num(tok[i])));
  }
  return out;
}

// "pub <topic> v0 v1 ..." with Python float repr of each value.
template<class Range>
std::string pub(const std::string & topic, const Range & values)
{
  std::string s = "pub " + topic + " ";
  bool first = true;
  for (double v : values) {
    s += (first ? "" : " ") + autonomous_navigation_cpp::py_repr(v);
    first = false;
  }
  return s;
}

// `initial` = lines produced while constructing; `apply(tokens, out)` runs one
// "> op" line and appends everything it produced to `out`.
inline void replay(
  const std::vector<const char *> & golden, const std::vector<std::string> & initial,
  const std::function<void(const std::vector<std::string> &, std::vector<std::string> &)> & apply)
{
  std::size_t i = 0;
  std::vector<std::string> expected;
  auto read_block = [&]() {
      expected.clear();
      while (i < golden.size() && !(golden[i][0] == '>' && golden[i][1] == ' ')) {
        expected.emplace_back(golden[i++]);
      }
    };
  read_block();
  ASSERT_EQ(initial, expected) << "constructor output";
  std::size_t ops = 0;
  while (i < golden.size()) {
    const std::string op = golden[i++];
    std::vector<std::string> got;
    apply(split(op.substr(2)), got);
    read_block();
    ASSERT_EQ(got, expected) << "after op #" << ops << ": " << op.substr(0, 200);
    ++ops;
  }
  EXPECT_GT(ops, 0u);
}

}  // namespace trace
