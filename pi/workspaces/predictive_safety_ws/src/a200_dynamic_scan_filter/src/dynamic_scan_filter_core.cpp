#include "a200_dynamic_scan_filter/dynamic_scan_filter_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace a200_dynamic_scan_filter
{

namespace
{

double upperQuartile(std::vector<double> & values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::sort(values.begin(), values.end());
  const double position = 0.75 * static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));

  if (lower == upper) {
    return values[lower];
  }

  const double alpha = position - static_cast<double>(lower);
  return values[lower] + alpha * (values[upper] - values[lower]);
}

double mean(const std::vector<double> & values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) /
         static_cast<double>(values.size());
}

}  // namespace

MotionGate::MotionGate(const MotionGateParams & params)
: params_(params)
{
  if (
    params_.enter_speed_mps < 0.0 ||
    params_.exit_speed_mps < 0.0 ||
    params_.exit_speed_mps > params_.enter_speed_mps ||
    params_.enter_hold_sec < 0.0 ||
    params_.exit_hold_sec < 0.0 ||
    params_.enter_min_displacement_m < 0.0 ||
    params_.history_window_sec <= 0.0 ||
    params_.history_min_span_sec <= 0.0 ||
    params_.history_min_span_sec > params_.history_window_sec ||
    params_.history_min_samples < 3U ||
    params_.motion_bin_count < 3U ||
    params_.quick_window_sec <= 0.0 ||
    params_.quick_window_sec > params_.history_window_sec ||
    params_.quick_min_span_sec <= 0.0 ||
    params_.quick_min_span_sec > params_.quick_window_sec ||
    params_.quick_min_samples < 3U ||
    params_.quick_vote_window < 2U ||
    params_.quick_votes_required < 1U ||
    params_.quick_votes_required > params_.quick_vote_window ||
    params_.quick_stationary_trend_speed_mps < 0.0 ||
    params_.quick_stationary_activity_speed_mps < 0.0 ||
    params_.stationary_trend_speed_mps < 0.0 ||
    params_.stationary_activity_speed_mps < 0.0 ||
    params_.stationary_confirm_sec < 0.0 ||
    params_.dynamic_trend_speed_mps <= params_.stationary_trend_speed_mps ||
    params_.dynamic_activity_speed_mps <= params_.stationary_activity_speed_mps)
  {
    throw std::invalid_argument("Invalid MotionGate parameters");
  }
}

void MotionGate::appendHistory(
  State & state,
  const double x,
  const double y,
  const double stamp_sec)
{
  state.history.push_back(Sample{stamp_sec, x, y});

  const double oldest_allowed = stamp_sec - params_.history_window_sec;
  while (!state.history.empty() && state.history.front().stamp_sec < oldest_allowed) {
    state.history.pop_front();
  }

  state.estimate = computeMotionEstimate(state);
  state.quick_estimate = computeQuickMotionEstimate(state);
}

MotionEstimate MotionGate::computeMotionEstimate(const State & state) const
{
  MotionEstimate estimate;
  estimate.sample_count = state.history.size();

  if (state.history.size() < params_.history_min_samples) {
    return estimate;
  }

  const double first_stamp = state.history.front().stamp_sec;
  const double last_stamp = state.history.back().stamp_sec;
  const double span = last_stamp - first_stamp;
  estimate.history_span_sec = std::max(0.0, span);

  if (!std::isfinite(span) || span < params_.history_min_span_sec) {
    return estimate;
  }

  struct BinAccumulator
  {
    double sum_t{0.0};
    double sum_x{0.0};
    double sum_y{0.0};
    std::size_t count{0U};
  };

  std::vector<BinAccumulator> bins(params_.motion_bin_count);

  for (const auto & sample : state.history) {
    const double fraction = std::clamp(
      (sample.stamp_sec - first_stamp) / span,
      0.0,
      1.0);

    const std::size_t index = std::min(
      params_.motion_bin_count - 1U,
      static_cast<std::size_t>(
        std::floor(fraction * static_cast<double>(params_.motion_bin_count))));

    auto & bin = bins[index];
    bin.sum_t += sample.stamp_sec;
    bin.sum_x += sample.x;
    bin.sum_y += sample.y;
    ++bin.count;
  }

  std::vector<Sample> centroids;
  centroids.reserve(params_.motion_bin_count);

  for (const auto & bin : bins) {
    if (bin.count == 0U) {
      continue;
    }

    const double divisor = static_cast<double>(bin.count);
    centroids.push_back(Sample{
      bin.sum_t / divisor,
      bin.sum_x / divisor,
      bin.sum_y / divisor});
  }

  if (centroids.size() < 3U) {
    return estimate;
  }

  const double trend_dt = centroids.back().stamp_sec - centroids.front().stamp_sec;
  if (trend_dt <= 1.0e-6) {
    return estimate;
  }

  const double trend_distance = std::hypot(
    centroids.back().x - centroids.front().x,
    centroids.back().y - centroids.front().y);

  std::vector<double> segment_speeds;
  segment_speeds.reserve(centroids.size() - 1U);

  for (std::size_t index = 1U; index < centroids.size(); ++index) {
    const auto & previous = centroids[index - 1U];
    const auto & current = centroids[index];
    const double dt = current.stamp_sec - previous.stamp_sec;
    if (dt <= 1.0e-6) {
      continue;
    }

    segment_speeds.push_back(
      std::hypot(current.x - previous.x, current.y - previous.y) / dt);
  }

  if (segment_speeds.size() < 2U) {
    return estimate;
  }

  const double activity_speed = upperQuartile(segment_speeds);
  const double trend_speed = trend_distance / trend_dt;

  if (!std::isfinite(activity_speed) || !std::isfinite(trend_speed)) {
    return estimate;
  }

  estimate.valid = true;
  estimate.trend_speed_mps = trend_speed;
  estimate.activity_speed_mps = activity_speed;
  return estimate;
}

MotionEstimate MotionGate::computeQuickMotionEstimate(const State & state) const
{
  MotionEstimate estimate;
  if (state.history.empty()) {
    return estimate;
  }

  const double last_stamp = state.history.back().stamp_sec;
  const double oldest_allowed = last_stamp - params_.quick_window_sec;

  std::vector<Sample> raw;
  raw.reserve(state.history.size());
  for (const auto & sample : state.history) {
    if (sample.stamp_sec >= oldest_allowed) {
      raw.push_back(sample);
    }
  }

  estimate.sample_count = raw.size();
  if (raw.size() < params_.quick_min_samples) {
    return estimate;
  }

  const double span = raw.back().stamp_sec - raw.front().stamp_sec;
  estimate.history_span_sec = std::max(0.0, span);
  if (!std::isfinite(span) || span < params_.quick_min_span_sec) {
    return estimate;
  }

  // Three-point moving average suppresses scan-cluster centroid jitter while
  // keeping sub-second motion changes visible.
  std::vector<Sample> smooth;
  smooth.reserve(raw.size());
  for (std::size_t i = 0U; i < raw.size(); ++i) {
    const std::size_t begin = (i == 0U) ? 0U : i - 1U;
    const std::size_t end = std::min(raw.size() - 1U, i + 1U);

    double sum_t = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t count = 0U;
    for (std::size_t j = begin; j <= end; ++j) {
      sum_t += raw[j].stamp_sec;
      sum_x += raw[j].x;
      sum_y += raw[j].y;
      ++count;
    }

    const double divisor = static_cast<double>(count);
    smooth.push_back(Sample{
      sum_t / divisor,
      sum_x / divisor,
      sum_y / divisor});
  }

  const std::size_t edge_count = std::max<std::size_t>(2U, smooth.size() / 3U);
  if (2U * edge_count > smooth.size()) {
    return estimate;
  }

  std::vector<double> first_t, first_x, first_y;
  std::vector<double> last_t, last_x, last_y;
  first_t.reserve(edge_count);
  first_x.reserve(edge_count);
  first_y.reserve(edge_count);
  last_t.reserve(edge_count);
  last_x.reserve(edge_count);
  last_y.reserve(edge_count);

  for (std::size_t i = 0U; i < edge_count; ++i) {
    first_t.push_back(smooth[i].stamp_sec);
    first_x.push_back(smooth[i].x);
    first_y.push_back(smooth[i].y);

    const auto & tail = smooth[smooth.size() - edge_count + i];
    last_t.push_back(tail.stamp_sec);
    last_x.push_back(tail.x);
    last_y.push_back(tail.y);
  }

  const double first_time = mean(first_t);
  const double last_time = mean(last_t);
  const double trend_dt = last_time - first_time;
  if (trend_dt <= 1.0e-6) {
    return estimate;
  }

  const double trend_speed = std::hypot(
    mean(last_x) - mean(first_x),
    mean(last_y) - mean(first_y)) / trend_dt;

  // Three broad temporal bins estimate path activity. Unlike endpoint trend,
  // this stays high for a reversal / oscillation that returns near its start.
  std::vector<Sample> centroids;
  centroids.reserve(3U);
  for (std::size_t bin = 0U; bin < 3U; ++bin) {
    const std::size_t begin = smooth.size() * bin / 3U;
    const std::size_t end = smooth.size() * (bin + 1U) / 3U;
    if (end <= begin) {
      return estimate;
    }

    double sum_t = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
      sum_t += smooth[i].stamp_sec;
      sum_x += smooth[i].x;
      sum_y += smooth[i].y;
    }

    const double divisor = static_cast<double>(end - begin);
    centroids.push_back(Sample{
      sum_t / divisor,
      sum_x / divisor,
      sum_y / divisor});
  }

  std::vector<double> segment_speeds;
  for (std::size_t i = 1U; i < centroids.size(); ++i) {
    const double dt = centroids[i].stamp_sec - centroids[i - 1U].stamp_sec;
    if (dt <= 1.0e-6) {
      return estimate;
    }
    segment_speeds.push_back(
      std::hypot(
        centroids[i].x - centroids[i - 1U].x,
        centroids[i].y - centroids[i - 1U].y) / dt);
  }

  const double activity_speed = mean(segment_speeds);
  if (!std::isfinite(trend_speed) || !std::isfinite(activity_speed)) {
    return estimate;
  }

  estimate.valid = true;
  estimate.trend_speed_mps = trend_speed;
  estimate.activity_speed_mps = activity_speed;
  return estimate;
}

bool MotionGate::hasStationaryEvidence(const MotionEstimate & estimate) const
{
  return
    estimate.valid &&
    estimate.trend_speed_mps <= params_.stationary_trend_speed_mps &&
    estimate.activity_speed_mps <= params_.stationary_activity_speed_mps;
}

bool MotionGate::hasDynamicEvidence(const MotionEstimate & estimate) const
{
  return
    estimate.valid &&
    (
      estimate.trend_speed_mps >= params_.dynamic_trend_speed_mps ||
      estimate.activity_speed_mps >= params_.dynamic_activity_speed_mps);
}

bool MotionGate::hasQuickStationaryEvidence(const MotionEstimate & estimate) const
{
  return
    estimate.valid &&
    estimate.trend_speed_mps <= params_.quick_stationary_trend_speed_mps &&
    estimate.activity_speed_mps <= params_.quick_stationary_activity_speed_mps;
}

bool MotionGate::update(
  const std::uint32_t id,
  const double tracker_speed_mps,
  const double x,
  const double y,
  const double stamp_sec)
{
  auto & state = states_[id];

  if (
    !state.initialized ||
    !std::isfinite(stamp_sec) ||
    (state.initialized && stamp_sec < state.last_seen_sec))
  {
    state = State{};
    state.initialized = true;
  }

  state.last_seen_sec = stamp_sec;

  if (
    !std::isfinite(tracker_speed_mps) ||
    !std::isfinite(x) ||
    !std::isfinite(y) ||
    !std::isfinite(stamp_sec))
  {
    state.enter_active = false;
    state.exit_active = false;
    state.stationary_candidate_active = false;
    state.quick_stationary_votes.clear();
    return state.masking;
  }

  appendHistory(state, x, y, stamp_sec);

  // ----------------------------------------------------------------------
  // Existing DYNAMIC ownership.
  //
  // v0.4 rule: returning a possibly-stopped object to the static/global
  // planner is conservative, so use a short robust observation window. Two
  // of the latest three valid windows must agree. This absorbs one noisy
  // centroid estimate without waiting for the full 1.5 s long-window state.
  // ----------------------------------------------------------------------
  if (state.masking) {
    if (state.quick_estimate.valid) {
      state.quick_stationary_votes.push_back(
        hasQuickStationaryEvidence(state.quick_estimate));
      while (state.quick_stationary_votes.size() > params_.quick_vote_window) {
        state.quick_stationary_votes.pop_front();
      }

      if (state.quick_stationary_votes.size() == params_.quick_vote_window) {
        const std::size_t positive_votes = static_cast<std::size_t>(
          std::count(
            state.quick_stationary_votes.begin(),
            state.quick_stationary_votes.end(),
            true));

        if (positive_votes >= params_.quick_votes_required) {
          state.masking = false;
          state.enter_active = false;
          state.exit_active = false;
          state.stationary_candidate_active = false;
          state.quick_stationary_votes.clear();
          state.reentry_requires_observed_motion = true;

          // Discard pre-handoff motion history. Otherwise an object that has
          // just become STATIC can immediately flip back to DYNAMIC because
          // the long estimator still contains its old moving samples. Dynamic
          // re-entry must be proven from fresh post-handoff observations.
          state.history.clear();
          state.history.push_back(Sample{stamp_sec, x, y});
          return false;
        }
      }
    } else {
      state.quick_stationary_votes.clear();
    }

    // Long-window v0.3 static evidence remains as a second path for cases
    // with unusually noisy short-window tracking.
    const bool observed_stationary = hasStationaryEvidence(state.estimate);
    if (observed_stationary) {
      if (!state.stationary_candidate_active) {
        state.stationary_candidate_active = true;
        state.stationary_candidate_since_sec = stamp_sec;
      }

      if (
        std::max(0.0, stamp_sec - state.stationary_candidate_since_sec) >=
        params_.stationary_confirm_sec)
      {
        state.masking = false;
        state.enter_active = false;
        state.exit_active = false;
        state.stationary_candidate_active = false;
        state.quick_stationary_votes.clear();
        state.reentry_requires_observed_motion = true;
        state.history.clear();
        state.history.push_back(Sample{stamp_sec, x, y});
        return false;
      }
    } else {
      state.stationary_candidate_active = false;
    }

    // Tracker-velocity exit is retained as a final independent fallback.
    if (tracker_speed_mps <= params_.exit_speed_mps) {
      if (!state.exit_active) {
        state.exit_active = true;
        state.exit_since_sec = stamp_sec;
      }

      if (std::max(0.0, stamp_sec - state.exit_since_sec) >= params_.exit_hold_sec) {
        state.masking = false;
        state.enter_active = false;
        state.exit_active = false;
        state.stationary_candidate_active = false;
        state.quick_stationary_votes.clear();
        state.reentry_requires_observed_motion = true;
        state.history.clear();
        state.history.push_back(Sample{stamp_sec, x, y});
        return false;
      }
    } else {
      state.exit_active = false;
    }

    return true;
  }

  // ----------------------------------------------------------------------
  // STATIC ownership: dynamic re-entry remains deliberately slower/stricter.
  // Once an object was handed back to the static costmap, stale tracker speed
  // alone cannot hide it again; long-window observed motion must confirm it.
  // ----------------------------------------------------------------------
  state.exit_active = false;
  state.stationary_candidate_active = false;
  state.quick_stationary_votes.clear();

  if (tracker_speed_mps < params_.enter_speed_mps) {
    state.enter_active = false;
    return false;
  }

  if (!state.enter_active) {
    state.enter_active = true;
    state.enter_since_sec = stamp_sec;
    state.enter_anchor_x = x;
    state.enter_anchor_y = y;
    return false;
  }

  const double enter_elapsed = std::max(0.0, stamp_sec - state.enter_since_sec);
  if (enter_elapsed < params_.enter_hold_sec) {
    return false;
  }

  const double enter_displacement = std::hypot(
    x - state.enter_anchor_x,
    y - state.enter_anchor_y);

  const bool displacement_ok = enter_displacement >= params_.enter_min_displacement_m;
  const bool observed_dynamic = hasDynamicEvidence(state.estimate);

  bool can_enter_dynamic = false;
  if (state.reentry_requires_observed_motion) {
    can_enter_dynamic = displacement_ok && observed_dynamic;
  } else if (state.estimate.valid) {
    can_enter_dynamic = displacement_ok && observed_dynamic;
  } else {
    // Initial acquisition before enough rolling history exists.
    can_enter_dynamic = displacement_ok;
  }

  if (can_enter_dynamic) {
    state.masking = true;
    state.enter_active = false;
    state.reentry_requires_observed_motion = false;
    state.quick_stationary_votes.clear();
    return true;
  }

  state.enter_since_sec = stamp_sec;
  state.enter_anchor_x = x;
  state.enter_anchor_y = y;
  return false;
}

bool MotionGate::isMasking(const std::uint32_t id) const
{
  const auto iterator = states_.find(id);
  return iterator != states_.end() && iterator->second.masking;
}

MotionEstimate MotionGate::motionEstimate(const std::uint32_t id) const
{
  const auto iterator = states_.find(id);
  if (iterator == states_.end()) {
    return MotionEstimate{};
  }
  return iterator->second.estimate;
}

MotionEstimate MotionGate::quickMotionEstimate(const std::uint32_t id) const
{
  const auto iterator = states_.find(id);
  if (iterator == states_.end()) {
    return MotionEstimate{};
  }
  return iterator->second.quick_estimate;
}

void MotionGate::prune(const double now_sec, const double max_age_sec)
{
  if (!std::isfinite(now_sec) || max_age_sec <= 0.0) {
    return;
  }

  for (auto iterator = states_.begin(); iterator != states_.end();) {
    const double age = now_sec - iterator->second.last_seen_sec;
    if (!std::isfinite(age) || age > max_age_sec) {
      iterator = states_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

std::size_t maskScanRanges(
  std::vector<float> & ranges,
  const double angle_min,
  const double angle_increment,
  const double range_min,
  const double range_max,
  const std::vector<CircleMask> & masks)
{
  if (masks.empty() || ranges.empty()) {
    return 0U;
  }

  std::size_t masked_count = 0U;

  for (std::size_t index = 0U; index < ranges.size(); ++index) {
    const double range = static_cast<double>(ranges[index]);

    // Preserve +inf and NaN exactly. This topic is GLOBAL MARKING-only with
    // inf_is_valid=false. Raw /scan remains the independent CLEARING source.
    if (
      !std::isfinite(range) ||
      range < range_min ||
      range > range_max)
    {
      continue;
    }

    const double angle = angle_min + static_cast<double>(index) * angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    bool should_mask = false;
    for (const auto & mask : masks) {
      if (!std::isfinite(mask.x) || !std::isfinite(mask.y) || mask.radius <= 0.0) {
        continue;
      }

      const double dx = x - mask.x;
      const double dy = y - mask.y;
      if (dx * dx + dy * dy <= mask.radius * mask.radius) {
        should_mask = true;
        break;
      }
    }

    if (should_mask) {
      ranges[index] = std::numeric_limits<float>::quiet_NaN();
      ++masked_count;
    }
  }

  return masked_count;
}

}  // namespace a200_dynamic_scan_filter
