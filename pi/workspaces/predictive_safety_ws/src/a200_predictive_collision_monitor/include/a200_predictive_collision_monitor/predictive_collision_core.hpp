// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace a200_predictive_collision_monitor
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct TrackedObstacle
{
  uint32_t id{0};
  Point2D position;
  Point2D velocity;
  double radius{0.0};
  std::array<double, 4> position_covariance{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> velocity_covariance{{0.0, 0.0, 0.0, 0.0}};
  std::vector<Point2D> imm_predictions;
  double prediction_dt{0.0};
};

enum class Action : uint8_t
{
  NoData = 0,
  Pass = 1,
  Slow = 2,
  Stop = 3,
};

enum class PredictionModel : uint8_t
{
  None = 0,
  ConstantVelocity = 1,
  Imm = 2,
};

struct EvaluatorConfig
{
  double footprint_half_length{0.494};
  double footprint_half_width{0.335};
  double base_safety_margin{0.20};
  double minimum_obstacle_radius{0.10};
  double uncertainty_sigma_multiplier{1.0};
  double maximum_uncertainty_margin{0.35};
  double prediction_horizon{3.0};
  double simulation_dt{0.05};
  double minimum_dynamic_speed{0.15};
  double slow_ttc{2.5};
  double stop_ttc{1.0};
};

struct RiskResult
{
  bool has_dynamic_tracks{false};
  Action action{Action::Pass};
  PredictionModel model{PredictionModel::None};
  uint32_t track_id{std::numeric_limits<uint32_t>::max()};
  double ttc{-1.0};
  double time_to_closest_approach{-1.0};
  double min_clearance{-1.0};
  double speed_scale{1.0};
  double obstacle_speed{0.0};
};

class PredictiveCollisionEvaluator
{
public:
  explicit PredictiveCollisionEvaluator(EvaluatorConfig config);

  [[nodiscard]] RiskResult evaluate(
    const Pose2D & robot_pose,
    const Twist2D & candidate_twist,
    const std::vector<TrackedObstacle> & obstacles,
    double obstacle_data_age) const;

  [[nodiscard]] const EvaluatorConfig & config() const noexcept;

private:
  EvaluatorConfig config_;
};

}  // namespace a200_predictive_collision_monitor
