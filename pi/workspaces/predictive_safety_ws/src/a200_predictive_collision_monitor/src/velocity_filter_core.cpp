// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include "a200_predictive_collision_monitor/velocity_filter_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace a200_predictive_collision_monitor
{

VelocityFilterController::VelocityFilterController(VelocityFilterConfig config)
: config_(std::move(config))
{
  const std::array<double, 4> values{{
    config_.minimum_stop_hold,
    config_.stop_clear_hold,
    config_.slow_clear_hold,
    config_.scale_release_rate,
  }};
  if (!std::all_of(
      values.begin(), values.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("velocity filter parameters must be finite");
  }
  if (config_.minimum_stop_hold < 0.0 || config_.stop_clear_hold < 0.0 ||
    config_.slow_clear_hold < 0.0 || config_.scale_release_rate <= 0.0)
  {
    throw std::invalid_argument("invalid velocity filter parameter");
  }
}

const VelocityFilterConfig & VelocityFilterController::config() const noexcept
{
  return config_;
}

void VelocityFilterController::reset(const double now_seconds)
{
  if (!std::isfinite(now_seconds)) {
    throw std::invalid_argument("reset time must be finite");
  }
  mode_ = Action::Pass;
  applied_scale_ = 1.0;
  last_update_time_ = now_seconds;
  stop_enter_time_ = now_seconds;
  clear_start_time_ = now_seconds;
  initialized_ = true;
  clear_timer_running_ = false;
}

void VelocityFilterController::enter_stop(const double now_seconds)
{
  if (mode_ != Action::Stop) {
    stop_enter_time_ = now_seconds;
  }
  mode_ = Action::Stop;
  applied_scale_ = 0.0;
  clear_timer_running_ = false;
}

void VelocityFilterController::handle_time_jump(const double now_seconds)
{
  last_update_time_ = now_seconds;
  clear_timer_running_ = false;
  if (mode_ == Action::Stop) {
    stop_enter_time_ = now_seconds;
    applied_scale_ = 0.0;
  }
}

VelocityFilterResult VelocityFilterController::update(
  const Action requested_action,
  const double requested_scale,
  const double now_seconds)
{
  if (!std::isfinite(requested_scale) || !std::isfinite(now_seconds)) {
    throw std::invalid_argument("filter inputs must be finite");
  }
  if (!initialized_) {
    reset(now_seconds);
  }
  if (now_seconds < last_update_time_) {
    handle_time_jump(now_seconds);
  }

  const double elapsed = std::max(0.0, now_seconds - last_update_time_);
  const double safe_requested_scale = std::clamp(requested_scale, 0.0, 1.0);

  if (requested_action == Action::Stop || requested_action == Action::NoData) {
    enter_stop(now_seconds);
  } else if (mode_ == Action::Stop) {
    if (requested_action == Action::Pass) {
      const bool minimum_hold_complete =
        now_seconds - stop_enter_time_ >= config_.minimum_stop_hold;
      if (minimum_hold_complete) {
        if (!clear_timer_running_) {
          clear_start_time_ = now_seconds;
          clear_timer_running_ = true;
        }
        if (now_seconds - clear_start_time_ >= config_.stop_clear_hold) {
          mode_ = Action::Pass;
          clear_timer_running_ = false;
        }
      }
    } else {
      clear_timer_running_ = false;
    }
  } else if (requested_action == Action::Slow) {
    mode_ = Action::Slow;
    clear_timer_running_ = false;
  } else if (mode_ == Action::Slow) {
    if (!clear_timer_running_) {
      clear_start_time_ = now_seconds;
      clear_timer_running_ = true;
    }
    if (now_seconds - clear_start_time_ >= config_.slow_clear_hold) {
      mode_ = Action::Pass;
      clear_timer_running_ = false;
    }
  } else {
    mode_ = Action::Pass;
    clear_timer_running_ = false;
  }

  double target_scale = 1.0;
  if (mode_ == Action::Stop) {
    target_scale = 0.0;
  } else if (mode_ == Action::Slow) {
    target_scale = requested_action == Action::Slow ?
      safe_requested_scale : applied_scale_;
  }

  if (target_scale <= applied_scale_) {
    applied_scale_ = target_scale;
  } else {
    applied_scale_ = std::min(
      target_scale,
      applied_scale_ + config_.scale_release_rate * elapsed);
  }
  applied_scale_ = std::clamp(applied_scale_, 0.0, 1.0);
  last_update_time_ = now_seconds;

  Action applied_action = mode_;
  if (mode_ == Action::Pass && applied_scale_ < 1.0) {
    applied_action = Action::Slow;
  }
  return VelocityFilterResult{
    applied_action,
    applied_scale_,
    mode_ == Action::Stop};
}

}  // namespace a200_predictive_collision_monitor
