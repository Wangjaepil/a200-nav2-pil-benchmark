#ifndef A200_DYNAMIC_SCAN_FILTER__DYNAMIC_SCAN_FILTER_CORE_HPP_
#define A200_DYNAMIC_SCAN_FILTER__DYNAMIC_SCAN_FILTER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace a200_dynamic_scan_filter
{

struct MotionGateParams
{
  // Tracker-velocity hysteresis retained from C1 v0.1.
  double enter_speed_mps{0.20};
  double exit_speed_mps{0.10};
  double enter_hold_sec{0.20};
  double exit_hold_sec{0.60};
  double enter_min_displacement_m{0.03};

  // v0.3 long-window observed-motion estimator.
  double history_window_sec{1.50};
  double history_min_span_sec{1.00};
  std::size_t history_min_samples{10U};
  std::size_t motion_bin_count{5U};

  // v0.4 fast DYNAMIC -> STATIC handoff.
  // This short window is used only while an obstacle is already masked as
  // dynamic. Static ownership is conservative: if recent observed motion
  // collapses, return the obstacle to the global costmap quickly.
  double quick_window_sec{0.80};
  double quick_min_span_sec{0.65};
  std::size_t quick_min_samples{7U};
  std::size_t quick_vote_window{3U};
  std::size_t quick_votes_required{2U};
  double quick_stationary_trend_speed_mps{0.08};
  double quick_stationary_activity_speed_mps{0.11};

  // STATIC evidence: both the long-term trend and the upper-quartile motion activity
  // must be small. This suppresses centroid jitter without mistaking a true
  // reversal / oscillation for a stop.
  double stationary_trend_speed_mps{0.08};
  double stationary_activity_speed_mps{0.15};
  double stationary_confirm_sec{0.25};

  // DYNAMIC evidence has a separate (higher) threshold. The gap between the
  // stationary and dynamic thresholds is deliberate hysteresis.
  double dynamic_trend_speed_mps{0.14};
  double dynamic_activity_speed_mps{0.16};
};

struct MotionEstimate
{
  bool valid{false};
  double trend_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double activity_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double history_span_sec{0.0};
  std::size_t sample_count{0U};
};

class MotionGate
{
public:
  explicit MotionGate(const MotionGateParams & params);

  bool update(
    std::uint32_t id,
    double tracker_speed_mps,
    double x,
    double y,
    double stamp_sec);

  bool isMasking(std::uint32_t id) const;
  MotionEstimate motionEstimate(std::uint32_t id) const;
  MotionEstimate quickMotionEstimate(std::uint32_t id) const;

  void prune(double now_sec, double max_age_sec);

private:
  struct Sample
  {
    double stamp_sec{0.0};
    double x{0.0};
    double y{0.0};
  };

  struct State
  {
    bool initialized{false};
    bool masking{false};

    bool enter_active{false};
    double enter_since_sec{0.0};
    double enter_anchor_x{0.0};
    double enter_anchor_y{0.0};

    bool exit_active{false};
    double exit_since_sec{0.0};

    bool stationary_candidate_active{false};
    double stationary_candidate_since_sec{0.0};

    std::deque<bool> quick_stationary_votes;

    // After an observed stationary handoff, do not immediately re-enter
    // DYNAMIC just because a stale tracker velocity remains high. Require
    // rolling observed-motion evidence before re-entry.
    bool reentry_requires_observed_motion{false};

    double last_seen_sec{0.0};
    std::deque<Sample> history;
    MotionEstimate estimate;
    MotionEstimate quick_estimate;
  };

  void appendHistory(State & state, double x, double y, double stamp_sec);
  MotionEstimate computeMotionEstimate(const State & state) const;
  MotionEstimate computeQuickMotionEstimate(const State & state) const;

  bool hasStationaryEvidence(const MotionEstimate & estimate) const;
  bool hasDynamicEvidence(const MotionEstimate & estimate) const;
  bool hasQuickStationaryEvidence(const MotionEstimate & estimate) const;

  MotionGateParams params_;
  std::unordered_map<std::uint32_t, State> states_;
};

struct CircleMask
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
};

std::size_t maskScanRanges(
  std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  const std::vector<CircleMask> & masks);

}  // namespace a200_dynamic_scan_filter

#endif  // A200_DYNAMIC_SCAN_FILTER__DYNAMIC_SCAN_FILTER_CORE_HPP_
