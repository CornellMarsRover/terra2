// Comparison helper for test/compare_swerve_with_python.py.  Reads one case
// per line on stdin and prints the C++ result with full precision (%a, exact
// bits) so it can be diffed against the Python originals.
//
//   S <mode> vx vy omega rps o5 o7 o6 o8
//       -> shape_motion_for_mode + calculate_swerve_targets:
//          vx vy omega d1 d2 d3 d4 s5 s7 s6 s8
//   C value limit          -> clamp(value, limit)
//   M <ops...>             -> CommandMux trace (see the Python script)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "cmr_rovernet_cpp/command_mux_core.hpp"
#include "cmr_rovernet_cpp/drive_common.hpp"
#include "cmr_rovernet_cpp/drive_modes.hpp"

using namespace cmr_rovernet_cpp;  // NOLINT

namespace {

// Hex-float tokens (Python float.hex()) parse exactly with strtod.
double num(std::istringstream & in)
{
  std::string tok;
  in >> tok;
  return std::strtod(tok.c_str(), nullptr);
}

void put(double v)
{
  if (std::isnan(v)) {
    std::printf(" nan");
  } else if (std::isinf(v)) {
    std::printf(v > 0 ? " inf" : " -inf");
  } else {
    std::printf(" %a", v);
  }
}

}  // namespace

int main()
{
  std::string line;
  std::optional<CommandMux> mux;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    if (kind == "S") {
      std::string mode;
      in >> mode;
      const double vx = num(in), vy = num(in), omega = num(in), rps = num(in);
      std::map<int, double> offsets;
      for (int id : {5, 7, 6, 8}) {
        offsets[id] = num(in);
      }
      const auto m = shape_motion_for_mode(mode, vx, vy, omega);
      const auto t = drive::calculate_swerve_targets(m[0], m[1], m[2], rps, offsets);
      std::printf("S");
      put(m[0]); put(m[1]); put(m[2]);
      for (const auto & kv : t.drive) {
        put(kv.second);
      }
      for (const auto & kv : t.steer) {
        put(kv.second);
      }
      std::printf("\n");
    } else if (kind == "C") {
      const double value = num(in), limit = num(in);
      std::printf("C");
      put(drive::clamp(value, limit));
      std::printf("\n");
    } else if (kind == "M") {
      // M new <timeout> | select <src> | estop | reset | recv <src> <now> v.. | out <now>
      std::string op;
      in >> op;
      std::printf("M");
      try {
        if (op == "new") {
          CommandMux fresh(num(in));  // may throw; keep the previous mux then
          mux = fresh;
          std::printf(" ok");
        } else if (op == "select") {
          std::string src;
          in >> src;
          mux->select(src);
          std::printf(" ok");
        } else if (op == "estop") {
          mux->emergency_stop();
          std::printf(" ok");
        } else if (op == "reset") {
          mux->reset();
          std::printf(" ok");
        } else if (op == "recv") {
          std::string src;
          in >> src;
          const double now = num(in);
          CommandMux::Values values;
          std::string tok;
          while (in >> tok) {
            values.push_back(std::strtod(tok.c_str(), nullptr));
          }
          std::printf(mux->receive(src, values, now) ? " True" : " False");
        } else if (op == "out") {
          const auto out = mux->output(num(in));
          if (!out) {
            std::printf(" None");
          } else {
            for (double v : *out) {
              put(v);
            }
          }
        }
      } catch (const std::invalid_argument & exc) {
        std::printf(" ValueError %s", exc.what());
      }
      std::printf("\n");
    }
  }
  return 0;
}
