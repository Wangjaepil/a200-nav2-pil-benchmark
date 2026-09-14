// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "a200_predictive_collision_monitor/predictive_collision_core.hpp"

namespace a200_predictive_collision_monitor
{

struct VelocityFilterConfig
{
  double minimum_stop_hold{0.50};
  double stop_clear_hold{0.80};
  double slow_clear_hold{0.40};
  double scale_release_rate{1.0};
};

struct VelocityFilterResult
{
  Action applied_action{Action::Pass};
  double applied_scale{1.0};
  bool stop_latched{false};
};

class VelocityFilterController
{
public:
  explicit VelocityFilterController(VelocityFilterConfig config);

  [[nodiscard]] VelocityFilterResult update(
    Action requested_action,
    double requested_scale,
    double now_seconds);

  void reset(double now_seconds = 0.0);

  [[nodiscard]] const VelocityFilterConfig & config() const noexcept;

private:
  void enter_stop(double now_seconds);
  void handle_time_jump(double now_seconds);

  VelocityFilterConfig config_;
  Action mode_{Action::Pass};
  double applied_scale_{1.0};
  double last_update_time_{0.0};
  double stop_enter_time_{0.0};
  double clear_start_time_{0.0};
  bool initialized_{false};
  bool clear_timer_running_{false};
};

}  // namespace a200_predictive_collision_monitor
