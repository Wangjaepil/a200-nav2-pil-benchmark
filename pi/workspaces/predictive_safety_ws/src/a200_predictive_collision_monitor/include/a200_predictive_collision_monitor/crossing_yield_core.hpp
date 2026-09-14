// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "a200_predictive_collision_monitor/predictive_collision_core.hpp"

namespace a200_predictive_collision_monitor
{

enum class YieldEvent : uint8_t
{
  None = 0,
  Entered = 1,
  WaitingMoving = 2,
  WaitingStationary = 3,
  WaitingLost = 4,
  ReleasedPassed = 5,
  ReleasedStatic = 6,
  ReleasedLost = 7,
  Retargeted = 8,
};

struct CrossingYieldConfig
{
  double entry_ttc{3.0};
  double minimum_crossing_angle_rad{0.8726646259971648};  // 50 deg
  double minimum_lateral_speed{0.25};
  double stationary_speed{0.12};
  double stationary_hold{1.5};
  double passed_clear_hold{0.8};
  double target_lost_hold{1.0};
  double static_handoff_cooldown{2.0};
  double footprint_half_width{0.335};
  double base_safety_margin{0.20};
  double minimum_obstacle_radius{0.10};
  double passed_clearance_margin{0.15};
};

struct CrossingYieldDecision
{
  Action requested_action{Action::Pass};
  double requested_scale{1.0};
  bool active{false};
  uint32_t target_id{std::numeric_limits<uint32_t>::max()};
  YieldEvent event{YieldEvent::None};
};

/// Tactical crossing policy layered above the instantaneous TTC evaluator.
///
/// Once a predicted collision with a laterally crossing track is observed,
/// this supervisor latches that track and requests a full yield stop. A
/// momentary TTC improvement caused by DWB steering therefore cannot release
/// the robot into the obstacle's direction of travel. The yield ends only
/// after the target has passed the locked route corridor, has remained
/// stationary long enough to be handed to Nav2 as a static obstacle, or has
/// remained absent from otherwise-valid tracker data for a bounded hold time.
class CrossingYieldSupervisor
{
public:
  explicit CrossingYieldSupervisor(CrossingYieldConfig config);

  [[nodiscard]] CrossingYieldDecision update(
    const RiskResult & risk,
    const Pose2D & robot_pose,
    const std::vector<TrackedObstacle> & obstacles,
    double now_seconds);

  void reset() noexcept;

  [[nodiscard]] const CrossingYieldConfig & config() const noexcept;
  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] uint32_t target_id() const noexcept;

private:
  [[nodiscard]] const TrackedObstacle * find_track(
    const std::vector<TrackedObstacle> & obstacles,
    uint32_t id) const noexcept;

  [[nodiscard]] bool crossing_candidate(
    const RiskResult & risk,
    const Pose2D & robot_pose,
    const std::vector<TrackedObstacle> & obstacles,
    double & lateral_velocity,
    double & release_boundary) const noexcept;

  void begin_yield(
    const Pose2D & robot_pose,
    uint32_t target_id,
    double lateral_velocity,
    double release_boundary,
    double now_seconds) noexcept;

  [[nodiscard]] CrossingYieldDecision raw_decision(
    const RiskResult & risk,
    YieldEvent event = YieldEvent::None,
    uint32_t event_target_id = std::numeric_limits<uint32_t>::max()) const noexcept;

  [[nodiscard]] CrossingYieldDecision stop_decision(YieldEvent event) const noexcept;

  CrossingYieldConfig config_;
  bool active_{false};
  uint32_t target_id_{std::numeric_limits<uint32_t>::max()};
  Pose2D reference_pose_;
  double crossing_direction_{1.0};
  double release_boundary_{0.0};
  bool stationary_timer_running_{false};
  double stationary_start_time_{0.0};
  bool passed_timer_running_{false};
  double passed_start_time_{0.0};
  bool lost_timer_running_{false};
  double lost_start_time_{0.0};
  bool time_initialized_{false};
  double last_update_time_{0.0};
  uint32_t cooldown_target_id_{std::numeric_limits<uint32_t>::max()};
  double cooldown_until_{0.0};
};

}  // namespace a200_predictive_collision_monitor
