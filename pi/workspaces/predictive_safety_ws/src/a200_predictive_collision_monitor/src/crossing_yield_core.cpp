// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include "a200_predictive_collision_monitor/crossing_yield_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace a200_predictive_collision_monitor
{
namespace
{

constexpr uint32_t kNoTrackId = std::numeric_limits<uint32_t>::max();

bool finite_pose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool finite_track(const TrackedObstacle & obstacle)
{
  return std::isfinite(obstacle.position.x) && std::isfinite(obstacle.position.y) &&
         std::isfinite(obstacle.velocity.x) && std::isfinite(obstacle.velocity.y) &&
         std::isfinite(obstacle.radius) && obstacle.radius >= 0.0;
}

}  // namespace

CrossingYieldSupervisor::CrossingYieldSupervisor(CrossingYieldConfig config)
: config_(std::move(config))
{
  const std::array<double, 12> values{{
    config_.entry_ttc,
    config_.minimum_crossing_angle_rad,
    config_.minimum_lateral_speed,
    config_.stationary_speed,
    config_.stationary_hold,
    config_.passed_clear_hold,
    config_.target_lost_hold,
    config_.static_handoff_cooldown,
    config_.footprint_half_width,
    config_.base_safety_margin,
    config_.minimum_obstacle_radius,
    config_.passed_clearance_margin,
  }};
  if (!std::all_of(
      values.begin(), values.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("crossing yield parameters must be finite");
  }
  if (config_.entry_ttc <= 0.0 || config_.minimum_crossing_angle_rad < 0.0 ||
    config_.minimum_crossing_angle_rad > 1.5707963267948966 ||
    config_.minimum_lateral_speed < 0.0 || config_.stationary_speed < 0.0 ||
    config_.stationary_speed >= config_.minimum_lateral_speed ||
    config_.stationary_hold < 0.0 || config_.passed_clear_hold < 0.0 ||
    config_.target_lost_hold < 0.0 || config_.static_handoff_cooldown < 0.0 ||
    config_.footprint_half_width <= 0.0 || config_.base_safety_margin < 0.0 ||
    config_.minimum_obstacle_radius < 0.0 || config_.passed_clearance_margin < 0.0)
  {
    throw std::invalid_argument("invalid crossing yield parameter");
  }
}

const CrossingYieldConfig & CrossingYieldSupervisor::config() const noexcept
{
  return config_;
}

bool CrossingYieldSupervisor::active() const noexcept
{
  return active_;
}

uint32_t CrossingYieldSupervisor::target_id() const noexcept
{
  return target_id_;
}

void CrossingYieldSupervisor::reset() noexcept
{
  active_ = false;
  target_id_ = kNoTrackId;
  stationary_timer_running_ = false;
  passed_timer_running_ = false;
  lost_timer_running_ = false;
  time_initialized_ = false;
  cooldown_target_id_ = kNoTrackId;
  cooldown_until_ = 0.0;
}

const TrackedObstacle * CrossingYieldSupervisor::find_track(
  const std::vector<TrackedObstacle> & obstacles,
  const uint32_t id) const noexcept
{
  const auto iterator = std::find_if(
    obstacles.begin(), obstacles.end(),
    [id](const TrackedObstacle & obstacle) {return obstacle.id == id;});
  if (iterator == obstacles.end() || !finite_track(*iterator)) {
    return nullptr;
  }
  return &(*iterator);
}

bool CrossingYieldSupervisor::crossing_candidate(
  const RiskResult & risk,
  const Pose2D & robot_pose,
  const std::vector<TrackedObstacle> & obstacles,
  double & lateral_velocity,
  double & release_boundary) const noexcept
{
  if (!finite_pose(robot_pose) || risk.track_id == kNoTrackId || risk.ttc < 0.0 ||
    risk.ttc > config_.entry_ttc)
  {
    return false;
  }
  const TrackedObstacle * obstacle = find_track(obstacles, risk.track_id);
  if (obstacle == nullptr) {
    return false;
  }
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  const double longitudinal_velocity =
    cosine * obstacle->velocity.x + sine * obstacle->velocity.y;
  lateral_velocity = -sine * obstacle->velocity.x + cosine * obstacle->velocity.y;
  const double crossing_angle = std::atan2(
    std::abs(lateral_velocity), std::abs(longitudinal_velocity));
  if (std::abs(lateral_velocity) < config_.minimum_lateral_speed ||
    crossing_angle < config_.minimum_crossing_angle_rad)
  {
    return false;
  }
  release_boundary = config_.footprint_half_width +
    std::max(obstacle->radius, config_.minimum_obstacle_radius) +
    config_.base_safety_margin + config_.passed_clearance_margin;
  return true;
}

void CrossingYieldSupervisor::begin_yield(
  const Pose2D & robot_pose,
  const uint32_t target_id,
  const double lateral_velocity,
  const double release_boundary,
  const double now_seconds) noexcept
{
  active_ = true;
  target_id_ = target_id;
  reference_pose_ = robot_pose;
  crossing_direction_ = lateral_velocity >= 0.0 ? 1.0 : -1.0;
  release_boundary_ = release_boundary;
  stationary_timer_running_ = false;
  passed_timer_running_ = false;
  lost_timer_running_ = false;
  stationary_start_time_ = now_seconds;
  passed_start_time_ = now_seconds;
  lost_start_time_ = now_seconds;
}

CrossingYieldDecision CrossingYieldSupervisor::raw_decision(
  const RiskResult & risk,
  const YieldEvent event,
  const uint32_t event_target_id) const noexcept
{
  return CrossingYieldDecision{
    risk.action,
    std::clamp(risk.speed_scale, 0.0, 1.0),
    false,
    event_target_id == kNoTrackId ? risk.track_id : event_target_id,
    event};
}

CrossingYieldDecision CrossingYieldSupervisor::stop_decision(const YieldEvent event) const noexcept
{
  return CrossingYieldDecision{Action::Stop, 0.0, true, target_id_, event};
}

CrossingYieldDecision CrossingYieldSupervisor::update(
  const RiskResult & risk,
  const Pose2D & robot_pose,
  const std::vector<TrackedObstacle> & obstacles,
  const double now_seconds)
{
  if (!std::isfinite(now_seconds)) {
    throw std::invalid_argument("crossing yield update time must be finite");
  }
  if (time_initialized_ && now_seconds < last_update_time_) {
    reset();
  }
  time_initialized_ = true;
  last_update_time_ = now_seconds;

  if (cooldown_target_id_ != kNoTrackId && now_seconds >= cooldown_until_) {
    cooldown_target_id_ = kNoTrackId;
  }

  double candidate_lateral_velocity = 0.0;
  double candidate_release_boundary = 0.0;
  const bool candidate = crossing_candidate(
    risk, robot_pose, obstacles, candidate_lateral_velocity, candidate_release_boundary);

  if (!active_) {
    if (candidate &&
      !(risk.track_id == cooldown_target_id_ && now_seconds < cooldown_until_))
    {
      begin_yield(
        robot_pose, risk.track_id, candidate_lateral_velocity,
        candidate_release_boundary, now_seconds);
      return stop_decision(YieldEvent::Entered);
    }
    return raw_decision(risk);
  }

  const TrackedObstacle * target = find_track(obstacles, target_id_);
  if (target == nullptr) {
    stationary_timer_running_ = false;
    passed_timer_running_ = false;

    if (candidate && risk.track_id != target_id_) {
      begin_yield(
        robot_pose, risk.track_id, candidate_lateral_velocity,
        candidate_release_boundary, now_seconds);
      return stop_decision(YieldEvent::Retargeted);
    }

    if (risk.action == Action::Pass) {
      if (!lost_timer_running_) {
        lost_timer_running_ = true;
        lost_start_time_ = now_seconds;
      }
      if (now_seconds - lost_start_time_ >= config_.target_lost_hold) {
        const uint32_t released_target = target_id_;
        active_ = false;
        target_id_ = kNoTrackId;
        lost_timer_running_ = false;
        return raw_decision(risk, YieldEvent::ReleasedLost, released_target);
      }
    } else {
      lost_timer_running_ = false;
    }
    return stop_decision(YieldEvent::WaitingLost);
  }

  lost_timer_running_ = false;
  const double speed = std::hypot(target->velocity.x, target->velocity.y);
  if (speed <= config_.stationary_speed) {
    if (!stationary_timer_running_) {
      stationary_timer_running_ = true;
      stationary_start_time_ = now_seconds;
    }
    if (now_seconds - stationary_start_time_ >= config_.stationary_hold) {
      const uint32_t released_target = target_id_;
      active_ = false;
      target_id_ = kNoTrackId;
      stationary_timer_running_ = false;
      passed_timer_running_ = false;
      cooldown_target_id_ = released_target;
      cooldown_until_ = now_seconds + config_.static_handoff_cooldown;
      return raw_decision(risk, YieldEvent::ReleasedStatic, released_target);
    }
  } else {
    stationary_timer_running_ = false;
  }

  const double dx = target->position.x - reference_pose_.x;
  const double dy = target->position.y - reference_pose_.y;
  const double lateral_position =
    -std::sin(reference_pose_.yaw) * dx + std::cos(reference_pose_.yaw) * dy;
  const bool passed = crossing_direction_ * lateral_position >= release_boundary_;
  if (passed) {
    if (!passed_timer_running_) {
      passed_timer_running_ = true;
      passed_start_time_ = now_seconds;
    }
    if (now_seconds - passed_start_time_ >= config_.passed_clear_hold) {
      const uint32_t released_target = target_id_;
      active_ = false;
      target_id_ = kNoTrackId;
      stationary_timer_running_ = false;
      passed_timer_running_ = false;
      // The target has already cleared the corridor that was locked when
      // yielding began.  Do not immediately latch the same moving track
      // again merely because Nav2 is still holding an old detour path toward
      // it.  The ordinary TTC result is still returned during this cooldown,
      // so an actually dangerous reversal can still request SLOW or STOP.
      cooldown_target_id_ = released_target;
      cooldown_until_ = now_seconds + config_.static_handoff_cooldown;
      return raw_decision(risk, YieldEvent::ReleasedPassed, released_target);
    }
  } else {
    passed_timer_running_ = false;
  }

  return stop_decision(
    stationary_timer_running_ ? YieldEvent::WaitingStationary : YieldEvent::WaitingMoving);
}

}  // namespace a200_predictive_collision_monitor
