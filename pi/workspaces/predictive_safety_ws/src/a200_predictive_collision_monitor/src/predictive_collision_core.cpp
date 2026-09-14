// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include "a200_predictive_collision_monitor/predictive_collision_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace a200_predictive_collision_monitor
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

struct ModelResult
{
  bool evaluated{false};
  double ttc{-1.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  double time_to_closest_approach{-1.0};
};

Pose2D pose_at_time(const Pose2D & initial, const Twist2D & twist, const double t)
{
  Pose2D pose = initial;
  const double v = twist.linear_x;
  const double w = twist.angular_z;

  if (std::abs(w) < 1.0e-6) {
    pose.x += v * std::cos(initial.yaw) * t;
    pose.y += v * std::sin(initial.yaw) * t;
  } else {
    pose.x += (v / w) * (std::sin(initial.yaw + w * t) - std::sin(initial.yaw));
    pose.y -= (v / w) * (std::cos(initial.yaw + w * t) - std::cos(initial.yaw));
    pose.yaw = initial.yaw + w * t;
  }
  return pose;
}

double maximum_eigenvalue_2x2(const std::array<double, 4> & covariance)
{
  if (!std::all_of(
      covariance.begin(), covariance.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return std::numeric_limits<double>::infinity();
  }

  const double a = std::max(0.0, covariance[0]);
  const double d = std::max(0.0, covariance[3]);
  const double b = 0.5 * (covariance[1] + covariance[2]);
  const double discriminant = std::max(0.0, (a - d) * (a - d) + 4.0 * b * b);
  return std::max(0.0, 0.5 * (a + d + std::sqrt(discriminant)));
}

double uncertainty_margin(
  const TrackedObstacle & obstacle,
  const EvaluatorConfig & config,
  const double time_from_measurement)
{
  const double position_variance = maximum_eigenvalue_2x2(obstacle.position_covariance);
  const double velocity_variance = maximum_eigenvalue_2x2(obstacle.velocity_covariance);
  const double t = std::max(0.0, time_from_measurement);
  const double sigma = std::sqrt(std::max(0.0, position_variance + t * t * velocity_variance));
  return std::min(
    config.maximum_uncertainty_margin,
    config.uncertainty_sigma_multiplier * sigma);
}

double rectangle_circle_clearance(
  const Pose2D & robot_pose,
  const Point2D & obstacle_position,
  const double obstacle_radius,
  const EvaluatorConfig & config)
{
  const double dx = obstacle_position.x - robot_pose.x;
  const double dy = obstacle_position.y - robot_pose.y;
  const double c = std::cos(robot_pose.yaw);
  const double s = std::sin(robot_pose.yaw);

  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const double outside_x = std::max(std::abs(local_x) - config.footprint_half_length, 0.0);
  const double outside_y = std::max(std::abs(local_y) - config.footprint_half_width, 0.0);

  return std::hypot(outside_x, outside_y) - obstacle_radius;
}

Point2D constant_velocity_position(const TrackedObstacle & obstacle, const double t)
{
  return Point2D{
    obstacle.position.x + obstacle.velocity.x * t,
    obstacle.position.y + obstacle.velocity.y * t};
}

bool imm_position_at_time(
  const TrackedObstacle & obstacle,
  const double t,
  Point2D & output)
{
  if (t < 0.0 || obstacle.prediction_dt <= 0.0 || obstacle.imm_predictions.empty()) {
    return false;
  }
  if (t <= kEpsilon) {
    output = obstacle.position;
    return true;
  }

  const double sample_coordinate = t / obstacle.prediction_dt;
  const auto upper_sample = static_cast<std::size_t>(std::ceil(sample_coordinate));
  if (upper_sample == 0 || upper_sample > obstacle.imm_predictions.size()) {
    return false;
  }

  const Point2D lower =
    upper_sample == 1 ? obstacle.position : obstacle.imm_predictions[upper_sample - 2];
  const Point2D upper = obstacle.imm_predictions[upper_sample - 1];
  const double lower_time = static_cast<double>(upper_sample - 1) * obstacle.prediction_dt;
  const double alpha = std::clamp((t - lower_time) / obstacle.prediction_dt, 0.0, 1.0);
  output.x = lower.x + alpha * (upper.x - lower.x);
  output.y = lower.y + alpha * (upper.y - lower.y);
  return std::isfinite(output.x) && std::isfinite(output.y);
}

using PositionFunction = std::function<bool(double, Point2D &)>;

ModelResult evaluate_model(
  const Pose2D & robot_pose,
  const Twist2D & candidate_twist,
  const TrackedObstacle & obstacle,
  const double data_age,
  const EvaluatorConfig & config,
  const PositionFunction & obstacle_position_at)
{
  ModelResult result;
  double previous_clearance = std::numeric_limits<double>::infinity();
  double previous_time = 0.0;
  const auto steps = static_cast<std::size_t>(
    std::ceil(config.prediction_horizon / config.simulation_dt));

  for (std::size_t step = 0; step <= steps; ++step) {
    const double future_time = std::min(
      config.prediction_horizon,
      static_cast<double>(step) * config.simulation_dt);
    const double time_from_measurement = data_age + future_time;
    Point2D obstacle_position;
    if (!obstacle_position_at(time_from_measurement, obstacle_position)) {
      break;
    }

    result.evaluated = true;
    const Pose2D future_robot_pose = pose_at_time(robot_pose, candidate_twist, future_time);
    const double effective_radius =
      std::max(obstacle.radius, config.minimum_obstacle_radius) +
      config.base_safety_margin +
      uncertainty_margin(obstacle, config, time_from_measurement);
    const double clearance = rectangle_circle_clearance(
      future_robot_pose, obstacle_position, effective_radius, config);

    if (clearance < result.min_clearance) {
      result.min_clearance = clearance;
      result.time_to_closest_approach = future_time;
    }

    if (result.ttc < 0.0 && clearance <= 0.0) {
      if (step == 0 || previous_clearance <= 0.0 || !std::isfinite(previous_clearance)) {
        result.ttc = future_time;
      } else {
        const double denominator = previous_clearance - clearance;
        const double fraction = denominator > kEpsilon ?
          std::clamp(previous_clearance / denominator, 0.0, 1.0) : 1.0;
        result.ttc = previous_time + fraction * (future_time - previous_time);
      }
    }

    previous_clearance = clearance;
    previous_time = future_time;
    if (future_time >= config.prediction_horizon) {
      break;
    }
  }
  return result;
}

bool is_more_critical(
  const ModelResult & candidate,
  const ModelResult & current)
{
  if (!candidate.evaluated) {
    return false;
  }
  if (!current.evaluated) {
    return true;
  }

  const bool candidate_collides = candidate.ttc >= 0.0;
  const bool current_collides = current.ttc >= 0.0;
  if (candidate_collides != current_collides) {
    return candidate_collides;
  }
  if (candidate_collides) {
    return candidate.ttc < current.ttc;
  }
  return candidate.min_clearance < current.min_clearance;
}

}  // namespace

PredictiveCollisionEvaluator::PredictiveCollisionEvaluator(EvaluatorConfig config)
: config_(std::move(config))
{
  const std::array<double, 11> scalar_parameters{{
    config_.footprint_half_length,
    config_.footprint_half_width,
    config_.base_safety_margin,
    config_.minimum_obstacle_radius,
    config_.uncertainty_sigma_multiplier,
    config_.maximum_uncertainty_margin,
    config_.prediction_horizon,
    config_.simulation_dt,
    config_.minimum_dynamic_speed,
    config_.slow_ttc,
    config_.stop_ttc,
  }};
  if (!std::all_of(
      scalar_parameters.begin(), scalar_parameters.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("configuration parameters must be finite");
  }

  if (config_.footprint_half_length <= 0.0 || config_.footprint_half_width <= 0.0) {
    throw std::invalid_argument("footprint half dimensions must be positive");
  }
  if (config_.base_safety_margin < 0.0 || config_.minimum_obstacle_radius < 0.0) {
    throw std::invalid_argument("radii and margins must not be negative");
  }
  if (config_.uncertainty_sigma_multiplier < 0.0 ||
    config_.maximum_uncertainty_margin < 0.0)
  {
    throw std::invalid_argument("uncertainty parameters must not be negative");
  }
  if (config_.prediction_horizon <= 0.0 || config_.simulation_dt <= 0.0 ||
    config_.simulation_dt > config_.prediction_horizon)
  {
    throw std::invalid_argument("invalid prediction horizon or simulation step");
  }
  if (config_.minimum_dynamic_speed < 0.0) {
    throw std::invalid_argument("minimum dynamic speed must not be negative");
  }
  if (config_.stop_ttc < 0.0 || config_.slow_ttc <= config_.stop_ttc ||
    config_.slow_ttc > config_.prediction_horizon)
  {
    throw std::invalid_argument("TTC thresholds must satisfy 0 <= stop < slow <= horizon");
  }
}

const EvaluatorConfig & PredictiveCollisionEvaluator::config() const noexcept
{
  return config_;
}

RiskResult PredictiveCollisionEvaluator::evaluate(
  const Pose2D & robot_pose,
  const Twist2D & candidate_twist,
  const std::vector<TrackedObstacle> & obstacles,
  const double obstacle_data_age) const
{
  RiskResult output;
  ModelResult selected;

  for (const auto & obstacle : obstacles) {
    if (!std::isfinite(obstacle.position.x) || !std::isfinite(obstacle.position.y) ||
      !std::isfinite(obstacle.velocity.x) || !std::isfinite(obstacle.velocity.y) ||
      !std::isfinite(obstacle.radius) || obstacle.radius < 0.0)
    {
      continue;
    }
    const double speed = std::hypot(obstacle.velocity.x, obstacle.velocity.y);
    if (!std::isfinite(speed) || speed < config_.minimum_dynamic_speed) {
      continue;
    }
    output.has_dynamic_tracks = true;

    const auto cv_result = evaluate_model(
      robot_pose, candidate_twist, obstacle, obstacle_data_age, config_,
      [&obstacle](const double t, Point2D & point) {
        point = constant_velocity_position(obstacle, t);
        return true;
      });
    if (is_more_critical(cv_result, selected)) {
      selected = cv_result;
      output.model = PredictionModel::ConstantVelocity;
      output.track_id = obstacle.id;
      output.obstacle_speed = speed;
    }

    if (!obstacle.imm_predictions.empty() && obstacle.prediction_dt > 0.0) {
      const auto imm_result = evaluate_model(
        robot_pose, candidate_twist, obstacle, obstacle_data_age, config_,
        [&obstacle](const double t, Point2D & point) {
          return imm_position_at_time(obstacle, t, point);
        });
      if (is_more_critical(imm_result, selected)) {
        selected = imm_result;
        output.model = PredictionModel::Imm;
        output.track_id = obstacle.id;
        output.obstacle_speed = speed;
      }
    }
  }

  if (!output.has_dynamic_tracks || !selected.evaluated) {
    return output;
  }

  output.ttc = selected.ttc;
  output.time_to_closest_approach = selected.time_to_closest_approach;
  output.min_clearance = selected.min_clearance;

  if (output.ttc >= 0.0 && output.ttc <= config_.stop_ttc) {
    output.action = Action::Stop;
    output.speed_scale = 0.0;
  } else if (output.ttc >= 0.0 && output.ttc <= config_.slow_ttc) {
    output.action = Action::Slow;
    output.speed_scale = std::clamp(
      (output.ttc - config_.stop_ttc) / (config_.slow_ttc - config_.stop_ttc),
      0.0, 1.0);
  }
  return output;
}

}  // namespace a200_predictive_collision_monitor
