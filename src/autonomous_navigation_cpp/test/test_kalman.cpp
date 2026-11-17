#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "autonomous_navigation_cpp/kalman_core.hpp"
#include "golden_data.hpp"

using autonomous_navigation_cpp::Kalman2;
using autonomous_navigation_cpp::Kalman4;

namespace
{
void near_rel(double a, double b) {EXPECT_NEAR(a, b, 1e-9 * std::max(1.0, std::fabs(b)));}
}

TEST(Kalman4, PredictCorrectMatchesPython)
{
  Kalman4 k;
  EXPECT_FALSE(k.predict(1000000000LL, 0.0, 0.0, 0.1, {0.01, 0.002, 0.003, 0.02}));
  EXPECT_TRUE(k.predict(1100000000LL, 0.05, -0.02, 0.2, {0.012, 0.001, 0.004, 0.03}));
  EXPECT_FALSE(k.correct(42.444, -76.483, 1.5));  // reference fix
  EXPECT_TRUE(k.predict(1250000000LL, 0.11, -0.05, 0.3, {0.011, 0.0, 0.002, 0.025}));
  EXPECT_TRUE(k.correct(42.444001, -76.483002, 0.8));
  for (int i = 0; i < 4; ++i) {
    near_rel(k.x(i), golden::kK4X[i]);
  }
  for (int i = 0; i < 16; ++i) {
    near_rel(k.P(i / 4, i % 4), golden::kK4P[i]);
  }
  EXPECT_DOUBLE_EQ(k.current_yaw, 0.3);
}

// Same call sequence as golden.py; after every call the Python node's
// (initialized, x, P, yaw) snapshot must match.
TEST(Kalman2, ValidatedGatedUpdatesMatchPython)
{
  Kalman2 q;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::array<double, 9> cov{0.5, 0.01, 0, 0.02, 0.6, 0, 0, 0, 1};
  const std::array<double, 9> bad_cov{0.5, inf, 0, 0.02, 0.6, 0, 0, 0, 1};
  auto t = [](long long ns) {
      return static_cast<double>(ns / 1000000000) * 1e9 + static_cast<double>(ns % 1000000000);
    };
  std::size_t call = 0;
  auto check = [&]() {
      const double * g = &golden::kK2Trace[call * 8];
      EXPECT_EQ(q.initialized ? 1.0 : 0.0, g[0]) << "call " << call;
      near_rel(q.x(0), g[1]);
      near_rel(q.x(1), g[2]);
      for (int i = 0; i < 4; ++i) {
        near_rel(q.P(i / 2, i % 2), g[3 + i]);
      }
      EXPECT_EQ(q.current_yaw, g[7]) << "call " << call;
      ++call;
    };
  using P = Kalman2::Predict;
  using C = Kalman2::Correct;
  EXPECT_EQ(q.predict(t(900000000), nan, 0.0, 0.0), P::NonFinite); check();
  EXPECT_EQ(q.predict(t(1000000000), 0.0, 0.0, 0.1), P::Init); check();
  EXPECT_EQ(q.predict(t(1100000000), 0.05, -0.02, 0.2), P::Applied); check();
  EXPECT_EQ(q.predict(t(1150000000), 0.06, -0.02, inf), P::NonFinite); check();
  double speed = 0.0;
  EXPECT_EQ(q.predict(t(1200000000), 5.0, -0.02, 0.2, &speed), P::TooFast); check();
  EXPECT_NEAR(speed, 49.5, 1e-6);
  EXPECT_EQ(q.correct(t(1220000000), -1, 42.444, -76.483, cov), C::Invalid); check();
  EXPECT_EQ(q.correct(t(1230000000), 0, nan, -76.483, cov), C::Invalid); check();
  EXPECT_FALSE(q.initialized);
  EXPECT_EQ(q.correct(t(1250000000), 0, 42.444, -76.483, cov), C::ReferenceSet); check();
  EXPECT_TRUE(q.initialized);
  EXPECT_EQ(q.correct(t(1300000000), 2, 42.444003, -76.483001, cov), C::Updated); check();
  EXPECT_EQ(q.predict(t(1300000000), 5.1, 0.0, 0.3), P::Applied); check();
  EXPECT_EQ(q.correct(t(1400000000), 0, 42.444004, -76.483002, cov), C::Updated); check();
  EXPECT_EQ(q.correct(t(1450000000), 0, 42.444004, -76.483002, bad_cov), C::Invalid); check();
  EXPECT_EQ(q.correct(t(1500000000), 0, 42.5, -76.483002, cov), C::Gated); check();
  EXPECT_EQ(call * 8, golden::kK2Trace.size());
}
