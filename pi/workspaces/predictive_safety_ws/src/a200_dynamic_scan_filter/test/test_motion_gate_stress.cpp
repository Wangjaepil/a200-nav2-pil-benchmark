#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

#include "a200_dynamic_scan_filter/dynamic_scan_filter_core.hpp"

using a200_dynamic_scan_filter::MotionGate;
using a200_dynamic_scan_filter::MotionGateParams;

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

void acquire(MotionGate & gate, const std::uint32_t id)
{
  for (int i = 0; i <= 15; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    gate.update(id, 0.50, 0.50 * t, 0.0, t);
  }
}

}  // namespace

int main()
{
  constexpr int trials = 1000;
  int typical_stop_fast = 0;
  int extreme_stop_eventual = 0;
  int moving_retained = 0;
  int reversal_retained = 0;
  int brief_pause_retained = 0;
  int stop_resume_reacquired = 0;

  // A) Typical stopped-object centroid jitter. Tracker velocity remains stale.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(10000U + static_cast<unsigned int>(trial));
    std::uniform_real_distribution<double> noise(-0.030, 0.030);
    std::uniform_real_distribution<double> stale_speed(0.18, 0.35);
    MotionGate gate(params());
    acquire(gate, 1U);

    bool released = false;
    double release_time = -1.0;
    for (int i = 16; i <= 35; ++i) {
      const double t = 0.1 * static_cast<double>(i);
      if (!gate.update(
          1U, stale_speed(rng), 0.75 + noise(rng), noise(rng), t))
      {
        released = true;
        release_time = t;
        break;
      }
    }

    if (released && release_time - 1.5 <= 1.2 + 1.0e-9) {
      ++typical_stop_fast;
    }
  }

  // B) Extreme +/-4 cm centroid jitter: short-window release may be delayed,
  // but the retained v0.3 long-window path must still hand back eventually.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(20000U + static_cast<unsigned int>(trial));
    std::uniform_real_distribution<double> noise(-0.040, 0.040);
    MotionGate gate(params());
    acquire(gate, 1U);

    bool released = false;
    for (int i = 16; i <= 55; ++i) {
      const double t = 0.1 * static_cast<double>(i);
      if (!gate.update(1U, 0.24, 0.75 + noise(rng), noise(rng), t)) {
        released = true;
        break;
      }
    }
    if (released) {
      ++extreme_stop_eventual;
    }
  }

  // C) Continuous moving obstacle, speed 0.12..0.50 m/s.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(30000U + static_cast<unsigned int>(trial));
    std::uniform_real_distribution<double> speed_dist(0.12, 0.50);
    std::uniform_real_distribution<double> noise(-0.012, 0.012);
    const double speed = speed_dist(rng);
    MotionGate gate(params());
    acquire(gate, 1U);

    bool bad_release = false;
    double x = 0.75;
    for (int i = 16; i <= 60; ++i) {
      const double t = 0.1 * static_cast<double>(i);
      x += speed * 0.1;
      if (!gate.update(
          1U, std::max(0.21, speed), x + noise(rng), noise(rng), t))
      {
        bad_release = true;
        break;
      }
    }
    if (!bad_release && gate.isMasking(1U)) {
      ++moving_retained;
    }
  }

  // D) Reversing / oscillatory obstacle.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(40000U + static_cast<unsigned int>(trial));
    std::uniform_real_distribution<double> noise(-0.010, 0.010);
    std::uniform_real_distribution<double> amplitude_dist(0.12, 0.24);
    const double amplitude = amplitude_dist(rng);
    MotionGate gate(params());
    acquire(gate, 1U);

    bool bad_release = false;
    for (int i = 16; i <= 60; ++i) {
      const double t = 0.1 * static_cast<double>(i);
      const double phase = std::fmod(t - 1.6, 0.8);
      const double nominal_x = phase <= 0.4 ?
        0.75 + amplitude * (phase / 0.4) :
        0.75 + amplitude - amplitude * ((phase - 0.4) / 0.4);

      if (!gate.update(
          1U, 0.30, nominal_x + noise(rng), noise(rng), t))
      {
        bad_release = true;
        break;
      }
    }
    if (!bad_release && gate.isMasking(1U)) {
      ++reversal_retained;
    }
  }

  // E) Brief 0.2..0.4 s hesitation then resume.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(50000U + static_cast<unsigned int>(trial));
    std::uniform_int_distribution<int> pause_steps_dist(2, 4);
    std::uniform_real_distribution<double> noise(-0.008, 0.008);
    const int pause_steps = pause_steps_dist(rng);
    MotionGate gate(params());
    acquire(gate, 1U);

    bool bad_release = false;
    int index = 16;
    for (int j = 0; j < pause_steps; ++j, ++index) {
      const double t = 0.1 * static_cast<double>(index);
      if (!gate.update(1U, 0.22, 0.75 + noise(rng), noise(rng), t)) {
        bad_release = true;
        break;
      }
    }

    double x = 0.75;
    for (; !bad_release && index <= 55; ++index) {
      const double t = 0.1 * static_cast<double>(index);
      x += 0.025;
      if (!gate.update(1U, 0.25, x + noise(rng), noise(rng), t)) {
        bad_release = true;
        break;
      }
    }

    if (!bad_release && gate.isMasking(1U)) {
      ++brief_pause_retained;
    }
  }

  // F) Stop -> static handoff -> stale velocity -> real motion resumes.
  for (int trial = 0; trial < trials; ++trial) {
    std::mt19937 rng(60000U + static_cast<unsigned int>(trial));
    std::uniform_real_distribution<double> stop_noise(-0.020, 0.020);
    std::uniform_real_distribution<double> move_noise(-0.008, 0.008);
    MotionGate gate(params());
    acquire(gate, 1U);

    double t = 1.6;
    while (gate.isMasking(1U) && t < 4.0) {
      gate.update(1U, 0.25, 0.75 + stop_noise(rng), stop_noise(rng), t);
      t += 0.1;
    }
    if (gate.isMasking(1U)) {
      continue;
    }

    // Stale velocity must not immediately reacquire.
    bool chatter = false;
    for (int i = 0; i < 8; ++i) {
      if (gate.update(
          1U, 0.25, 0.75 + stop_noise(rng), stop_noise(rng), t))
      {
        chatter = true;
        break;
      }
      t += 0.1;
    }
    if (chatter) {
      continue;
    }

    double x = 0.75;
    bool reacquired = false;
    for (int i = 0; i < 30; ++i) {
      x += 0.035;
      if (gate.update(1U, 0.35, x + move_noise(rng), move_noise(rng), t)) {
        reacquired = true;
        break;
      }
      t += 0.1;
    }
    if (reacquired) {
      ++stop_resume_reacquired;
    }
  }

  std::cout
    << "typical_stop_fast=" << typical_stop_fast << "/" << trials << '\n'
    << "extreme_stop_eventual=" << extreme_stop_eventual << "/" << trials << '\n'
    << "moving_retained=" << moving_retained << "/" << trials << '\n'
    << "reversal_retained=" << reversal_retained << "/" << trials << '\n'
    << "brief_pause_retained=" << brief_pause_retained << "/" << trials << '\n'
    << "stop_resume_reacquired=" << stop_resume_reacquired << "/" << trials << '\n';

  if (
    typical_stop_fast != trials ||
    extreme_stop_eventual != trials ||
    moving_retained != trials ||
    reversal_retained != trials ||
    brief_pause_retained != trials ||
    stop_resume_reacquired != trials)
  {
    return 1;
  }

  std::cout << "DYNAMIC_SCAN_FILTER_V04_STRESS_OK\n";
  return 0;
}
