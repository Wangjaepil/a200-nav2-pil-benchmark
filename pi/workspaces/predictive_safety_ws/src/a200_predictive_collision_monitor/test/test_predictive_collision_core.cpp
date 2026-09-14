// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "a200_predictive_collision_monitor/predictive_collision_core.hpp"

namespace a200_predictive_collision_monitor
{
namespace
{

EvaluatorConfig test_config()
{
  EvaluatorConfig config;
  config.footprint_half_length = 0.50;
  config.footprint_half_width = 0.35;
  config.base_safety_margin = 0.10;
  config.minimum_obstacle_radius = 0.10;
  config.uncertainty_sigma_multiplier = 0.0;
  config.maximum_uncertainty_margin = 0.0;
  config.prediction_horizon = 3.0;
  config.simulation_dt = 0.02;
  config.minimum_dynamic_speed = 0.10;
  config.stop_ttc = 0.75;
  config.slow_ttc = 2.5;
  return config;
}

TrackedObstacle crossing_obstacle()
{
  TrackedObstacle obstacle;
  obstacle.id = 7;
  obstacle.position = Point2D{2.0, -2.0};
  obstacle.velocity = Point2D{0.0, 1.0};
  obstacle.radius = 0.30;
  return obstacle;
}

TEST(PredictiveCollisionCore, DetectsPerpendicularCrossing)
{
  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {crossing_obstacle()}, 0.0);

  EXPECT_TRUE(result.has_dynamic_tracks);
  EXPECT_EQ(result.track_id, 7u);
  EXPECT_EQ(result.model, PredictionModel::ConstantVelocity);
  EXPECT_GT(result.ttc, 1.0);
  EXPECT_LT(result.ttc, 2.0);
  EXPECT_EQ(result.action, Action::Slow);
  EXPECT_GT(result.speed_scale, 0.0);
  EXPECT_LT(result.speed_scale, 1.0);
}

TEST(PredictiveCollisionCore, PassesObstacleMovingAwayFromPath)
{
  auto obstacle = crossing_obstacle();
  obstacle.position = Point2D{2.0, 2.0};
  obstacle.velocity = Point2D{0.0, 1.0};

  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_TRUE(result.has_dynamic_tracks);
  EXPECT_LT(result.ttc, 0.0);
  EXPECT_EQ(result.action, Action::Pass);
  EXPECT_GT(result.min_clearance, 0.0);
}

TEST(PredictiveCollisionCore, IgnoresStationaryTrackInDynamicStage)
{
  auto obstacle = crossing_obstacle();
  obstacle.velocity = Point2D{0.0, 0.0};

  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_FALSE(result.has_dynamic_tracks);
  EXPECT_EQ(result.action, Action::Pass);
  EXPECT_EQ(result.model, PredictionModel::None);
}

TEST(PredictiveCollisionCore, StopsForImmediatePredictedCollision)
{
  auto obstacle = crossing_obstacle();
  obstacle.position = Point2D{0.8, -0.5};

  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_GE(result.ttc, 0.0);
  EXPECT_LE(result.ttc, evaluator.config().stop_ttc);
  EXPECT_EQ(result.action, Action::Stop);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.0);
}

TEST(PredictiveCollisionCore, UsesMoreDangerousImmPrediction)
{
  auto obstacle = crossing_obstacle();
  obstacle.position = Point2D{2.0, 2.0};
  obstacle.velocity = Point2D{0.0, 1.0};  // CV moves safely away.
  obstacle.prediction_dt = 0.1;
  for (int i = 1; i <= 30; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    obstacle.imm_predictions.push_back(Point2D{2.0, 2.0 - t});
  }

  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_EQ(result.model, PredictionModel::Imm);
  EXPECT_GE(result.ttc, 0.0);
  EXPECT_EQ(result.action, Action::Slow);
}

TEST(PredictiveCollisionCore, AccountsForMeasurementAge)
{
  auto obstacle = crossing_obstacle();
  PredictiveCollisionEvaluator evaluator(test_config());

  const auto fresh = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);
  const auto aged = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.40);

  ASSERT_GE(fresh.ttc, 0.0);
  ASSERT_GE(aged.ttc, 0.0);
  EXPECT_LT(aged.ttc, fresh.ttc);
}

TEST(PredictiveCollisionCore, CovarianceMakesPredictionMoreConservative)
{
  auto obstacle = crossing_obstacle();
  obstacle.position = Point2D{2.0, -2.5};

  auto no_uncertainty_config = test_config();
  PredictiveCollisionEvaluator no_uncertainty(no_uncertainty_config);
  const auto baseline = no_uncertainty.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  auto uncertainty_config = test_config();
  uncertainty_config.uncertainty_sigma_multiplier = 1.0;
  uncertainty_config.maximum_uncertainty_margin = 0.50;
  obstacle.position_covariance = {{0.04, 0.0, 0.0, 0.04}};
  obstacle.velocity_covariance = {{0.01, 0.0, 0.0, 0.01}};
  PredictiveCollisionEvaluator with_uncertainty(uncertainty_config);
  const auto conservative = with_uncertainty.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_LT(conservative.min_clearance, baseline.min_clearance);
  if (baseline.ttc >= 0.0 && conservative.ttc >= 0.0) {
    EXPECT_LE(conservative.ttc, baseline.ttc);
  }
}

TEST(PredictiveCollisionCore, RejectsInvalidConfiguration)
{
  auto config = test_config();
  config.slow_ttc = config.stop_ttc;
  EXPECT_THROW(PredictiveCollisionEvaluator evaluator(config), std::invalid_argument);

  config = test_config();
  config.prediction_horizon = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PredictiveCollisionEvaluator evaluator(config), std::invalid_argument);
}

TEST(PredictiveCollisionCore, SkipsNonFiniteTrack)
{
  auto obstacle = crossing_obstacle();
  obstacle.position.x = std::numeric_limits<double>::quiet_NaN();

  PredictiveCollisionEvaluator evaluator(test_config());
  const auto result = evaluator.evaluate(
    Pose2D{0.0, 0.0, 0.0}, Twist2D{1.0, 0.0}, {obstacle}, 0.0);

  EXPECT_FALSE(result.has_dynamic_tracks);
  EXPECT_EQ(result.action, Action::Pass);
}

}  // namespace
}  // namespace a200_predictive_collision_monitor
