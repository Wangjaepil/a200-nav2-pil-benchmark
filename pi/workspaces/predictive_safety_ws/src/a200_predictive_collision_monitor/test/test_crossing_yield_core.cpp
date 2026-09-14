// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "a200_predictive_collision_monitor/crossing_yield_core.hpp"

namespace a200_predictive_collision_monitor
{
namespace
{

constexpr uint32_t kNoTrackId = std::numeric_limits<uint32_t>::max();

TrackedObstacle obstacle(
  const uint32_t id,
  const double x,
  const double y,
  const double vx,
  const double vy,
  const double radius = 0.35)
{
  TrackedObstacle output;
  output.id = id;
  output.position = Point2D{x, y};
  output.velocity = Point2D{vx, vy};
  output.radius = radius;
  return output;
}

RiskResult risk(
  const uint32_t id,
  const double ttc,
  const Action action = Action::Slow,
  const double scale = 0.5)
{
  RiskResult output;
  output.has_dynamic_tracks = id != kNoTrackId;
  output.track_id = id;
  output.ttc = ttc;
  output.action = action;
  output.speed_scale = scale;
  return output;
}

TEST(CrossingYieldCore, LatchesPerpendicularCrossingAndStopsImmediately)
{
  CrossingYieldSupervisor supervisor(CrossingYieldConfig{});
  const auto decision = supervisor.update(
    risk(7, 2.8, Action::Pass, 1.0), Pose2D{},
    {obstacle(7, 2.0, -1.0, 0.0, 0.7)}, 10.0);

  EXPECT_TRUE(decision.active);
  EXPECT_EQ(decision.requested_action, Action::Stop);
  EXPECT_DOUBLE_EQ(decision.requested_scale, 0.0);
  EXPECT_EQ(decision.target_id, 7u);
  EXPECT_EQ(decision.event, YieldEvent::Entered);
}

TEST(CrossingYieldCore, DoesNotLatchSameDirectionMotion)
{
  CrossingYieldSupervisor supervisor(CrossingYieldConfig{});
  const auto decision = supervisor.update(
    risk(4, 2.0), Pose2D{}, {obstacle(4, 2.0, 0.0, 0.7, 0.0)}, 1.0);

  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.requested_action, Action::Slow);
  EXPECT_DOUBLE_EQ(decision.requested_scale, 0.5);
}

TEST(CrossingYieldCore, RawPathClearDoesNotReleaseMovingTarget)
{
  CrossingYieldSupervisor supervisor(CrossingYieldConfig{});
  const Pose2D pose{};
  const auto moving = obstacle(1, 2.0, -0.5, 0.0, 0.7);
  (void)supervisor.update(risk(1, 2.0), pose, {moving}, 0.0);

  const auto decision = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose, {moving}, 1.0);

  EXPECT_TRUE(decision.active);
  EXPECT_EQ(decision.requested_action, Action::Stop);
  EXPECT_EQ(decision.event, YieldEvent::WaitingMoving);
}

TEST(CrossingYieldCore, ReleasesAfterTargetPassesAndClearHoldCompletes)
{
  CrossingYieldConfig config;
  config.passed_clear_hold = 0.8;
  CrossingYieldSupervisor supervisor(config);
  const Pose2D pose{};
  (void)supervisor.update(
    risk(2, 2.0), pose, {obstacle(2, 2.0, -0.5, 0.0, 0.7)}, 0.0);

  const auto first_clear = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(2, 2.0, 1.2, 0.0, 0.7)}, 1.0);
  EXPECT_TRUE(first_clear.active);

  const auto released = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(2, 2.0, 1.4, 0.0, 0.7)}, 1.81);
  EXPECT_FALSE(released.active);
  EXPECT_EQ(released.requested_action, Action::Pass);
  EXPECT_EQ(released.event, YieldEvent::ReleasedPassed);
}

TEST(CrossingYieldCore, PassedTargetCannotImmediatelyRelatch)
{
  CrossingYieldConfig config;
  config.passed_clear_hold = 0.5;
  config.static_handoff_cooldown = 2.0;
  CrossingYieldSupervisor supervisor(config);
  const Pose2D pose{};
  (void)supervisor.update(
    risk(2, 2.0), pose, {obstacle(2, 2.0, -0.5, 0.0, 0.7)}, 0.0);
  (void)supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(2, 2.0, 1.2, 0.0, 0.7)}, 1.0);
  const auto released = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(2, 2.0, 1.4, 0.0, 0.7)}, 1.51);
  ASSERT_EQ(released.event, YieldEvent::ReleasedPassed);

  const auto during_cooldown = supervisor.update(
    risk(2, 1.8, Action::Slow, 0.6), pose,
    {obstacle(2, 2.0, 1.5, 0.0, 0.7)}, 1.6);

  EXPECT_FALSE(during_cooldown.active);
  EXPECT_EQ(during_cooldown.event, YieldEvent::None);
  EXPECT_EQ(during_cooldown.requested_action, Action::Slow);
  EXPECT_DOUBLE_EQ(during_cooldown.requested_scale, 0.6);
}

TEST(CrossingYieldCore, StationaryTargetIsHandedToNav2AfterContinuousHold)
{
  CrossingYieldConfig config;
  config.stationary_hold = 1.5;
  CrossingYieldSupervisor supervisor(config);
  const Pose2D pose{};
  (void)supervisor.update(
    risk(3, 2.0), pose, {obstacle(3, 2.0, -0.4, 0.0, 0.7)}, 0.0);

  const auto waiting = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(3, 2.0, 0.0, 0.01, 0.01)}, 1.0);
  EXPECT_TRUE(waiting.active);
  EXPECT_EQ(waiting.event, YieldEvent::WaitingStationary);

  const auto released = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(3, 2.0, 0.0, 0.01, 0.01)}, 2.51);
  EXPECT_FALSE(released.active);
  EXPECT_EQ(released.event, YieldEvent::ReleasedStatic);
}

TEST(CrossingYieldCore, StationaryTimerResetsWhenTargetMovesAgain)
{
  CrossingYieldConfig config;
  config.stationary_hold = 1.0;
  CrossingYieldSupervisor supervisor(config);
  const Pose2D pose{};
  (void)supervisor.update(
    risk(5, 2.0), pose, {obstacle(5, 2.0, -0.4, 0.0, 0.7)}, 0.0);
  (void)supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(5, 2.0, 0.0, 0.01, 0.01)}, 0.2);
  (void)supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(5, 2.0, 0.1, 0.0, 0.5)}, 0.8);

  const auto still_waiting = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose,
    {obstacle(5, 2.0, 0.1, 0.01, 0.01)}, 1.4);
  EXPECT_TRUE(still_waiting.active);
  EXPECT_EQ(still_waiting.event, YieldEvent::WaitingStationary);
}

TEST(CrossingYieldCore, MissingTargetNeedsStableClearBeforeRelease)
{
  CrossingYieldConfig config;
  config.target_lost_hold = 1.0;
  CrossingYieldSupervisor supervisor(config);
  const Pose2D pose{};
  (void)supervisor.update(
    risk(8, 2.0), pose, {obstacle(8, 2.0, -0.4, 0.0, 0.7)}, 0.0);

  const auto waiting = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose, {}, 0.5);
  EXPECT_TRUE(waiting.active);
  EXPECT_EQ(waiting.event, YieldEvent::WaitingLost);

  const auto released = supervisor.update(
    risk(kNoTrackId, -1.0, Action::Pass, 1.0), pose, {}, 1.51);
  EXPECT_FALSE(released.active);
  EXPECT_EQ(released.event, YieldEvent::ReleasedLost);
}

TEST(CrossingYieldCore, RejectsInvalidConfiguration)
{
  CrossingYieldConfig config;
  config.stationary_speed = config.minimum_lateral_speed;
  EXPECT_THROW((void)CrossingYieldSupervisor(config), std::invalid_argument);

  config = CrossingYieldConfig{};
  config.entry_ttc = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)CrossingYieldSupervisor(config), std::invalid_argument);
}

}  // namespace
}  // namespace a200_predictive_collision_monitor
