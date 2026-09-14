// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "a200_predictive_collision_monitor/velocity_filter_core.hpp"

namespace a200_predictive_collision_monitor
{
namespace
{

VelocityFilterConfig test_config()
{
  VelocityFilterConfig config;
  config.minimum_stop_hold = 0.50;
  config.stop_clear_hold = 0.80;
  config.slow_clear_hold = 0.40;
  config.scale_release_rate = 1.0;
  return config;
}

TEST(VelocityFilterCore, PassesAtUnityScale)
{
  VelocityFilterController controller(test_config());
  const auto result = controller.update(Action::Pass, 1.0, 10.0);

  EXPECT_EQ(result.applied_action, Action::Pass);
  EXPECT_DOUBLE_EQ(result.applied_scale, 1.0);
  EXPECT_FALSE(result.stop_latched);
}

TEST(VelocityFilterCore, AppliesDangerousScaleImmediately)
{
  VelocityFilterController controller(test_config());
  (void)controller.update(Action::Pass, 1.0, 10.0);
  const auto result = controller.update(Action::Slow, 0.55, 10.1);

  EXPECT_EQ(result.applied_action, Action::Slow);
  EXPECT_DOUBLE_EQ(result.applied_scale, 0.55);
}

TEST(VelocityFilterCore, HoldsSlowAcrossShortClearFlicker)
{
  VelocityFilterController controller(test_config());
  (void)controller.update(Action::Slow, 0.50, 10.0);

  const auto flicker = controller.update(Action::Pass, 1.0, 10.20);
  EXPECT_EQ(flicker.applied_action, Action::Slow);
  EXPECT_DOUBLE_EQ(flicker.applied_scale, 0.50);

  const auto still_held = controller.update(Action::Pass, 1.0, 10.59);
  EXPECT_EQ(still_held.applied_action, Action::Slow);

  const auto released = controller.update(Action::Pass, 1.0, 10.61);
  EXPECT_EQ(released.applied_action, Action::Slow);
  EXPECT_NEAR(released.applied_scale, 0.52, 1.0e-9);
}

TEST(VelocityFilterCore, StopsImmediatelyAndRequiresStableClearance)
{
  VelocityFilterController controller(test_config());
  (void)controller.update(Action::Pass, 1.0, 20.0);
  const auto stopped = controller.update(Action::Stop, 0.0, 20.1);

  EXPECT_EQ(stopped.applied_action, Action::Stop);
  EXPECT_DOUBLE_EQ(stopped.applied_scale, 0.0);
  EXPECT_TRUE(stopped.stop_latched);

  const auto too_early = controller.update(Action::Pass, 1.0, 20.4);
  EXPECT_EQ(too_early.applied_action, Action::Stop);

  const auto clear_timer_started = controller.update(Action::Pass, 1.0, 20.6);
  EXPECT_EQ(clear_timer_started.applied_action, Action::Stop);

  const auto not_stable_long_enough = controller.update(Action::Pass, 1.0, 21.39);
  EXPECT_EQ(not_stable_long_enough.applied_action, Action::Stop);

  const auto released = controller.update(Action::Pass, 1.0, 21.41);
  EXPECT_EQ(released.applied_action, Action::Slow);
  EXPECT_NEAR(released.applied_scale, 0.02, 1.0e-9);
  EXPECT_FALSE(released.stop_latched);
}

TEST(VelocityFilterCore, SlowPredictionDoesNotReleaseStopLatch)
{
  VelocityFilterController controller(test_config());
  (void)controller.update(Action::Stop, 0.0, 30.0);
  const auto result = controller.update(Action::Slow, 0.7, 35.0);

  EXPECT_EQ(result.applied_action, Action::Stop);
  EXPECT_DOUBLE_EQ(result.applied_scale, 0.0);
  EXPECT_TRUE(result.stop_latched);
}

TEST(VelocityFilterCore, MissingRiskDataFailsClosed)
{
  VelocityFilterController controller(test_config());
  (void)controller.update(Action::Pass, 1.0, 40.0);
  const auto result = controller.update(Action::NoData, 1.0, 40.1);

  EXPECT_EQ(result.applied_action, Action::Stop);
  EXPECT_DOUBLE_EQ(result.applied_scale, 0.0);
  EXPECT_TRUE(result.stop_latched);
}

TEST(VelocityFilterCore, RejectsInvalidInputs)
{
  auto config = test_config();
  config.scale_release_rate = 0.0;
  EXPECT_THROW(VelocityFilterController controller(config), std::invalid_argument);

  VelocityFilterController controller(test_config());
  EXPECT_THROW(
    (void)controller.update(
      Action::Pass, 1.0, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

}  // namespace
}  // namespace a200_predictive_collision_monitor
