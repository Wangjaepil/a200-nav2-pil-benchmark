#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "a200_dynamic_scan_filter/dynamic_scan_filter_core.hpp"

using a200_dynamic_scan_filter::CircleMask;
using a200_dynamic_scan_filter::MotionGate;
using a200_dynamic_scan_filter::MotionGateParams;
using a200_dynamic_scan_filter::maskScanRanges;

namespace
{

MotionGateParams params()
{
  MotionGateParams p;
  p.enter_speed_mps = 0.20;
  p.exit_speed_mps = 0.10;
  p.enter_hold_sec = 0.20;
  p.exit_hold_sec = 0.60;
  p.enter_min_displacement_m = 0.03;

  p.history_window_sec = 1.50;
  p.history_min_span_sec = 1.00;
  p.history_min_samples = 10U;
  p.motion_bin_count = 5U;

  p.quick_window_sec = 0.80;
  p.quick_min_span_sec = 0.65;
  p.quick_min_samples = 7U;
  p.quick_vote_window = 3U;
  p.quick_votes_required = 2U;
  p.quick_stationary_trend_speed_mps = 0.08;
  p.quick_stationary_activity_speed_mps = 0.11;

  p.stationary_trend_speed_mps = 0.08;
  p.stationary_activity_speed_mps = 0.15;
  p.stationary_confirm_sec = 0.25;

  p.dynamic_trend_speed_mps = 0.14;
  p.dynamic_activity_speed_mps = 0.16;
  return p;
}

void acquireDynamic(MotionGate & gate, const std::uint32_t id, const double speed = 0.50)
{
  for (int i = 0; i <= 15; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    gate.update(id, speed, speed * t, 0.0, t);
  }
  assert(gate.isMasking(id));
}

void testInvalidParameters()
{
  {
    auto p = params();
    p.quick_min_span_sec = p.quick_window_sec + 0.1;
    bool threw = false;
    try {
      MotionGate gate(p);
      (void)gate;
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    assert(threw);
  }

  {
    auto p = params();
    p.quick_votes_required = 4U;
    p.quick_vote_window = 3U;
    bool threw = false;
    try {
      MotionGate gate(p);
      (void)gate;
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    assert(threw);
  }
}

void testFastStopRecognitionWithStaleTrackerVelocity()
{
  MotionGate gate(params());
  acquireDynamic(gate, 1U);

  bool released = false;
  double release_time = -1.0;

  // Physical stop at t=1.5 s, while tracker speed intentionally remains stale.
  for (int i = 16; i <= 35; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double x_noise =
      (i % 5 == 0) ? 0.028 :
      (i % 5 == 1) ? -0.024 :
      (i % 5 == 2) ? 0.018 :
      (i % 5 == 3) ? -0.012 : 0.004;
    const double y_noise =
      (i % 4 == 0) ? -0.020 :
      (i % 4 == 1) ? 0.016 :
      (i % 4 == 2) ? -0.008 : 0.006;

    if (!gate.update(1U, 0.24, 0.75 + x_noise, y_noise, t)) {
      released = true;
      release_time = t;
      break;
    }
  }

  assert(released);
  const double stop_to_release = release_time - 1.5;
  assert(stop_to_release >= 0.5);
  assert(stop_to_release <= 1.2);
  assert(!gate.isMasking(1U));

  const auto quick = gate.quickMotionEstimate(1U);
  assert(quick.valid);
  assert(quick.trend_speed_mps <= 0.08 + 1.0e-9);
  assert(quick.activity_speed_mps <= 0.11 + 1.0e-9);
}

void testBriefHesitationDoesNotRelease()
{
  MotionGate gate(params());
  acquireDynamic(gate, 2U);

  // 0.3 s hesitation should not be enough to hand ownership to static.
  for (int i = 16; i <= 18; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    assert(gate.update(2U, 0.22, 0.75, 0.0, t));
  }

  double x = 0.75;
  for (int i = 19; i <= 35; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    x += 0.025;
    assert(gate.update(2U, 0.25, x, 0.0, t));
  }
  assert(gate.isMasking(2U));
}

void testContinuousSlowMotionStaysDynamic()
{
  MotionGate gate(params());
  acquireDynamic(gate, 3U, 0.40);

  double x = 0.60;
  for (int i = 16; i <= 55; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double jitter = (i % 2 == 0) ? 0.008 : -0.008;
    x += 0.012;  // 0.12 m/s
    assert(gate.update(3U, 0.16, x + jitter, 0.0, t));
  }
  assert(gate.isMasking(3U));
}

void testReversalDoesNotLookStationary()
{
  MotionGate gate(params());
  acquireDynamic(gate, 4U, 0.40);

  for (int i = 16; i <= 60; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double phase = std::fmod(t - 1.6, 0.8);
    const double x = phase <= 0.4 ?
      0.60 + 0.18 * (phase / 0.4) :
      0.78 - 0.18 * ((phase - 0.4) / 0.4);
    assert(gate.update(4U, 0.30, x, 0.0, t));
  }
  assert(gate.isMasking(4U));
}

void testNoImmediateReentryAfterStaticHandoff()
{
  MotionGate gate(params());
  acquireDynamic(gate, 5U);

  double t = 1.6;
  while (gate.isMasking(5U) && t < 4.0) {
    const int index = static_cast<int>(std::round(t * 10.0));
    const double jitter = (index % 2 == 0) ? 0.012 : -0.012;
    gate.update(5U, 0.24, 0.75 + jitter, 0.0, t);
    t += 0.1;
  }
  assert(!gate.isMasking(5U));

  // Stale/high tracker speed alone must not hide the obstacle again.
  for (int i = 0; i < 12; ++i) {
    const double jitter = (i % 2 == 0) ? 0.010 : -0.010;
    assert(!gate.update(5U, 0.25, 0.75 + jitter, 0.0, t));
    t += 0.1;
  }

  // Actual motion resumes; long observed-motion evidence eventually allows
  // dynamic ownership to return.
  double x = 0.75;
  bool reacquired = false;
  for (int i = 0; i < 30; ++i) {
    x += 0.035;
    if (gate.update(5U, 0.35, x, 0.0, t)) {
      reacquired = true;
      break;
    }
    t += 0.1;
  }
  assert(reacquired);
  assert(gate.isMasking(5U));
}

void testLegacyLowSpeedExitStillWorks()
{
  MotionGate gate(params());
  acquireDynamic(gate, 6U, 0.45);

  bool released = false;
  for (int i = 16; i <= 28; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    if (!gate.update(6U, 0.05, 0.675, 0.0, t)) {
      released = true;
      break;
    }
  }
  assert(released);
}

void testTimestampResetFailsSafe()
{
  MotionGate gate(params());
  acquireDynamic(gate, 7U);
  assert(!gate.update(7U, 0.50, 0.0, 0.0, 0.2));
  assert(!gate.isMasking(7U));
}


void testStaticNoiseDoesNotBecomeDynamic()
{
  MotionGate gate(params());

  for (int i = 0; i <= 50; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    const double x = (i % 4 == 0) ? 0.008 : (i % 4 == 1) ? -0.008 :
      (i % 4 == 2) ? 0.005 : -0.005;
    const double y = (i % 3 == 0) ? -0.007 : (i % 3 == 1) ? 0.006 : 0.0;
    assert(!gate.update(8U, 0.25, x, y, t));
  }
  assert(!gate.isMasking(8U));
}

void testPruneFailsSafe()
{
  MotionGate gate(params());
  acquireDynamic(gate, 9U);
  assert(gate.isMasking(9U));
  gate.prune(3.0, 0.50);
  assert(!gate.isMasking(9U));
  assert(!gate.motionEstimate(9U).valid);
  assert(!gate.quickMotionEstimate(9U).valid);
}

void testScanMasking()
{
  std::vector<float> ranges{
    2.0f,
    2.0f,
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(),
    2.0f};

  std::vector<CircleMask> masks{CircleMask{2.0, 0.0, 0.20}};

  const std::size_t masked = maskScanRanges(
    ranges, 0.0, 0.20, 0.05, 12.0, masks);

  assert(masked == 1U);
  assert(std::isnan(ranges[0]));
  assert(std::isfinite(ranges[1]));
  assert(std::isinf(ranges[2]));
  assert(std::isnan(ranges[3]));
  assert(std::isfinite(ranges[4]));
}

}  // namespace

int main()
{
  testInvalidParameters();
  testFastStopRecognitionWithStaleTrackerVelocity();
  testBriefHesitationDoesNotRelease();
  testContinuousSlowMotionStaysDynamic();
  testReversalDoesNotLookStationary();
  testNoImmediateReentryAfterStaticHandoff();
  testLegacyLowSpeedExitStillWorks();
  testTimestampResetFailsSafe();
  testStaticNoiseDoesNotBecomeDynamic();
  testPruneFailsSafe();
  testScanMasking();

  std::cout << "DYNAMIC_SCAN_FILTER_V04_QA_OK\n";
  return 0;
}
