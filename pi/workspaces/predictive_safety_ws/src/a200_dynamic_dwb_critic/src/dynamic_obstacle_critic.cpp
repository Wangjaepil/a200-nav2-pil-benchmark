#include "a200_dynamic_dwb_critic/dynamic_obstacle_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace a200_dynamic_dwb_critic
{

namespace
{

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double normalizeAngle(const double angle)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;

  double wrapped = std::fmod(angle + pi, two_pi);
  if (wrapped < 0.0) {
    wrapped += two_pi;
  }
  return wrapped - pi;
}

bool samePose(
  const geometry_msgs::msg::Pose2D & first,
  const geometry_msgs::msg::Pose2D & second)
{
  constexpr double xy_epsilon = 1.0e-8;
  constexpr double theta_epsilon = 1.0e-8;

  return
    std::hypot(first.x - second.x, first.y - second.y) <= xy_epsilon &&
    std::fabs(normalizeAngle(first.theta - second.theta)) <= theta_epsilon;
}

}  // namespace

void DynamicObstacleCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("DynamicObstacleCritic failed to lock controller node");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "tracked_obstacles_topic",
    rclcpp::ParameterValue(std::string("/tracked_obstacles")));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "track_timeout_sec", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "future_stamp_tolerance_sec", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minimum_dynamic_speed_mps", rclcpp::ParameterValue(0.15));

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "robot_half_length_m", rclcpp::ParameterValue(0.494));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "robot_half_width_m", rclcpp::ParameterValue(0.335));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minimum_obstacle_radius_m", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "hard_safety_margin_m", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "soft_clearance_m", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "uncertainty_sigma_multiplier", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "maximum_uncertainty_margin_m", rclcpp::ParameterValue(0.35));

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "trajectory_horizon_sec", rclcpp::ParameterValue(1.70));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "ttc_horizon_sec", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "proximity_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "ttc_weight", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "overlap_penalty", rclcpp::ParameterValue(25.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "future_discount_per_sec", rclcpp::ParameterValue(0.15));

  // v0.2-A: long-horizon dynamic reasoning.
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "prediction_horizon_sec", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "prediction_sample_dt_sec", rclcpp::ParameterValue(0.10));

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "vo_weight", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "vo_time_decay_sec", rclcpp::ParameterValue(1.5));

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "cpa_weight", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "cpa_soft_clearance_m", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "cpa_future_discount_per_sec", rclcpp::ParameterValue(0.15));

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "braking_weight", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "braking_reaction_sec", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "braking_decel_mps2", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "braking_time_reserve_sec", rclcpp::ParameterValue(0.30));

  node->get_parameter(prefix + "tracked_obstacles_topic", tracked_obstacles_topic_);
  node->get_parameter(prefix + "track_timeout_sec", track_timeout_sec_);
  node->get_parameter(prefix + "future_stamp_tolerance_sec", future_stamp_tolerance_sec_);
  node->get_parameter(prefix + "minimum_dynamic_speed_mps", minimum_dynamic_speed_mps_);

  node->get_parameter(prefix + "robot_half_length_m", robot_half_length_m_);
  node->get_parameter(prefix + "robot_half_width_m", robot_half_width_m_);
  node->get_parameter(prefix + "minimum_obstacle_radius_m", minimum_obstacle_radius_m_);
  node->get_parameter(prefix + "hard_safety_margin_m", hard_safety_margin_m_);
  node->get_parameter(prefix + "soft_clearance_m", soft_clearance_m_);
  node->get_parameter(
    prefix + "uncertainty_sigma_multiplier", uncertainty_sigma_multiplier_);
  node->get_parameter(
    prefix + "maximum_uncertainty_margin_m", maximum_uncertainty_margin_m_);

  node->get_parameter(prefix + "trajectory_horizon_sec", trajectory_horizon_sec_);
  node->get_parameter(prefix + "ttc_horizon_sec", ttc_horizon_sec_);
  node->get_parameter(prefix + "proximity_weight", proximity_weight_);
  node->get_parameter(prefix + "ttc_weight", ttc_weight_);
  node->get_parameter(prefix + "overlap_penalty", overlap_penalty_);
  node->get_parameter(prefix + "future_discount_per_sec", future_discount_per_sec_);

  node->get_parameter(prefix + "prediction_horizon_sec", prediction_horizon_sec_);
  node->get_parameter(prefix + "prediction_sample_dt_sec", prediction_sample_dt_sec_);

  node->get_parameter(prefix + "vo_weight", vo_weight_);
  node->get_parameter(prefix + "vo_time_decay_sec", vo_time_decay_sec_);

  node->get_parameter(prefix + "cpa_weight", cpa_weight_);
  node->get_parameter(prefix + "cpa_soft_clearance_m", cpa_soft_clearance_m_);
  node->get_parameter(
    prefix + "cpa_future_discount_per_sec", cpa_future_discount_per_sec_);

  node->get_parameter(prefix + "braking_weight", braking_weight_);
  node->get_parameter(prefix + "braking_reaction_sec", braking_reaction_sec_);
  node->get_parameter(prefix + "braking_decel_mps2", braking_decel_mps2_);
  node->get_parameter(
    prefix + "braking_time_reserve_sec", braking_time_reserve_sec_);

  if (
    tracked_obstacles_topic_.empty() ||
    track_timeout_sec_ <= 0.0 ||
    future_stamp_tolerance_sec_ < 0.0 ||
    minimum_dynamic_speed_mps_ < 0.0 ||
    robot_half_length_m_ <= 0.0 ||
    robot_half_width_m_ <= 0.0 ||
    minimum_obstacle_radius_m_ < 0.0 ||
    hard_safety_margin_m_ < 0.0 ||
    soft_clearance_m_ <= 0.0 ||
    uncertainty_sigma_multiplier_ < 0.0 ||
    maximum_uncertainty_margin_m_ < 0.0 ||
    trajectory_horizon_sec_ <= 0.0 ||
    ttc_horizon_sec_ <= 0.0 ||
    proximity_weight_ < 0.0 ||
    ttc_weight_ < 0.0 ||
    overlap_penalty_ < 0.0 ||
    future_discount_per_sec_ < 0.0 ||
    prediction_horizon_sec_ < trajectory_horizon_sec_ ||
    prediction_sample_dt_sec_ <= 0.0 ||
    vo_weight_ < 0.0 ||
    vo_time_decay_sec_ <= 0.0 ||
    cpa_weight_ < 0.0 ||
    cpa_soft_clearance_m_ <= 0.0 ||
    cpa_future_discount_per_sec_ < 0.0 ||
    braking_weight_ < 0.0 ||
    braking_reaction_sec_ < 0.0 ||
    braking_decel_mps2_ <= 0.0 ||
    braking_time_reserve_sec_ < 0.0)
  {
    throw std::invalid_argument("DynamicObstacleCritic received invalid parameters");
  }

  tracks_subscription_ = node->create_subscription<ObstacleArray>(
    tracked_obstacles_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](const ObstacleArray::SharedPtr message) {
      std::lock_guard<std::mutex> lock(tracks_mutex_);
      latest_tracks_ = message;
    });

  RCLCPP_INFO(
    node->get_logger(),
    "DynamicObstacleCritic v0.2-A ready: topic='%s', local_frame='%s', "
    "dynamic>=%.2fm/s, DWB=%.2fs, prediction=%.2fs, "
    "VO=%.2f CPA=%.2f BRAKE=%.2f",
    tracked_obstacles_topic_.c_str(),
    costmap_ros_->getGlobalFrameID().c_str(),
    minimum_dynamic_speed_mps_,
    trajectory_horizon_sec_,
    prediction_horizon_sec_,
    vo_weight_,
    cpa_weight_,
    braking_weight_);
}

bool DynamicObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }

  ObstacleArray::SharedPtr snapshot;
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    snapshot = latest_tracks_;
  }

  cycle_tracks_.reset();
  cycle_track_age_sec_ = 0.0;

  if (!snapshot) {
    return true;
  }

  const std::string local_frame = costmap_ros_->getGlobalFrameID();
  if (snapshot->header.frame_id != local_frame) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "DynamicObstacleCritic ignoring tracks in frame '%s'; local DWB frame is '%s'",
      snapshot->header.frame_id.c_str(), local_frame.c_str());
    return true;
  }

  const rclcpp::Time track_stamp(snapshot->header.stamp);
  const double age_sec = (node->now() - track_stamp).seconds();

  if (
    !std::isfinite(age_sec) ||
    age_sec < -future_stamp_tolerance_sec_ ||
    age_sec > track_timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "DynamicObstacleCritic track snapshot unavailable/stale: age=%.3fs",
      age_sec);
    return true;
  }

  bool has_dynamic_track = false;
  for (const auto & obstacle : snapshot->obstacles) {
    if (isDynamic(obstacle)) {
      has_dynamic_track = true;
      break;
    }
  }

  if (!has_dynamic_track) {
    return true;
  }

  cycle_tracks_ = snapshot;
  cycle_track_age_sec_ = std::max(0.0, age_sec);
  return true;
}

bool DynamicObstacleCritic::isDynamic(const Obstacle & obstacle) const
{
  if (!std::isfinite(obstacle.velocity.x) || !std::isfinite(obstacle.velocity.y)) {
    return false;
  }

  return
    std::hypot(obstacle.velocity.x, obstacle.velocity.y) >=
    minimum_dynamic_speed_mps_;
}

DynamicObstacleCritic::Point2D DynamicObstacleCritic::obstaclePositionAt(
  const Obstacle & obstacle,
  const double future_sec) const
{
  const double t = std::max(0.0, future_sec);
  const Point2D current{obstacle.position.x, obstacle.position.y};

  // Constant velocity is ONLY a fallback when the tracker supplies no usable
  // future positions. Normal v0.2-A operation follows predicted_positions.
  if (
    !std::isfinite(obstacle.prediction_dt) ||
    obstacle.prediction_dt <= 1.0e-6 ||
    obstacle.predicted_positions.empty())
  {
    return Point2D{
      current.x + obstacle.velocity.x * t,
      current.y + obstacle.velocity.y * t};
  }

  const double prediction_dt = obstacle.prediction_dt;

  if (t <= prediction_dt) {
    const double alpha = clamp01(t / prediction_dt);
    const auto & first = obstacle.predicted_positions.front();
    return Point2D{
      current.x + alpha * (first.x - current.x),
      current.y + alpha * (first.y - current.y)};
  }

  const double fractional_index = t / prediction_dt - 1.0;
  const double floored_index = std::floor(std::max(0.0, fractional_index));
  const std::size_t lower_index = static_cast<std::size_t>(floored_index);

  if (lower_index + 1U < obstacle.predicted_positions.size()) {
    const double alpha = clamp01(fractional_index - floored_index);
    const auto & lower = obstacle.predicted_positions[lower_index];
    const auto & upper = obstacle.predicted_positions[lower_index + 1U];

    return Point2D{
      lower.x + alpha * (upper.x - lower.x),
      lower.y + alpha * (upper.y - lower.y)};
  }

  const auto & last = obstacle.predicted_positions.back();
  const double prediction_end_sec =
    prediction_dt * static_cast<double>(obstacle.predicted_positions.size());
  const double extra_sec = std::max(0.0, t - prediction_end_sec);

  // Robustness fallback only. In the intended configuration tracker prediction
  // is 3.0 s and this critic's prediction_horizon_sec is also 3.0 s.
  return Point2D{
    last.x + obstacle.velocity.x * extra_sec,
    last.y + obstacle.velocity.y * extra_sec};
}

DynamicObstacleCritic::Point2D DynamicObstacleCritic::obstacleVelocityAt(
  const Obstacle & obstacle,
  const double future_sec) const
{
  double dt = obstacle.prediction_dt;
  if (!std::isfinite(dt) || dt < 0.05) {
    dt = 0.05;
  }
  dt = std::min(dt, 0.20);

  // Velocity is differentiated from the time-varying prediction. This means
  // direction and speed are allowed to change throughout the horizon.
  const Point2D first = obstaclePositionAt(obstacle, future_sec);
  const Point2D second = obstaclePositionAt(obstacle, future_sec + dt);

  return Point2D{
    (second.x - first.x) / dt,
    (second.y - first.y) / dt};
}

double DynamicObstacleCritic::trajectoryPoseTime(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const std::size_t index) const
{
  if (trajectory.poses.size() <= 1U || index == 0U) {
    return 0.0;
  }

  bool duplicate_final = false;
  if (trajectory.poses.size() >= 3U) {
    duplicate_final = samePose(
      trajectory.poses[trajectory.poses.size() - 1U],
      trajectory.poses[trajectory.poses.size() - 2U]);
  }

  const std::size_t simulated_steps =
    duplicate_final ? trajectory.poses.size() - 2U : trajectory.poses.size() - 1U;

  if (simulated_steps == 0U) {
    return 0.0;
  }

  if (duplicate_final && index >= trajectory.poses.size() - 1U) {
    return trajectory_horizon_sec_;
  }

  const std::size_t clamped_index = std::min(index, simulated_steps);
  return
    trajectory_horizon_sec_ *
    static_cast<double>(clamped_index) /
    static_cast<double>(simulated_steps);
}

DynamicObstacleCritic::Point2D DynamicObstacleCritic::trajectoryVelocityAt(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const std::size_t index) const
{
  if (trajectory.poses.size() <= 1U) {
    return Point2D{};
  }

  std::size_t first_index = index;
  std::size_t second_index = std::min(index + 1U, trajectory.poses.size() - 1U);

  double first_time = trajectoryPoseTime(trajectory, first_index);
  double second_time = trajectoryPoseTime(trajectory, second_index);

  if (second_time - first_time <= 1.0e-6 && index > 0U) {
    first_index = index - 1U;
    second_index = index;
    first_time = trajectoryPoseTime(trajectory, first_index);
    second_time = trajectoryPoseTime(trajectory, second_index);
  }

  const double dt = second_time - first_time;
  if (dt <= 1.0e-6) {
    return Point2D{};
  }

  const auto & first = trajectory.poses[first_index];
  const auto & second = trajectory.poses[second_index];

  return Point2D{
    (second.x - first.x) / dt,
    (second.y - first.y) / dt};
}

DynamicObstacleCritic::Motion2D DynamicObstacleCritic::terminalMotion(
  const dwb_msgs::msg::Trajectory2D & trajectory) const
{
  Motion2D motion;
  if (trajectory.poses.size() <= 1U) {
    return motion;
  }

  std::size_t last_index = trajectory.poses.size() - 1U;
  if (
    trajectory.poses.size() >= 3U &&
    samePose(trajectory.poses[last_index], trajectory.poses[last_index - 1U]))
  {
    --last_index;
  }

  if (last_index == 0U) {
    return motion;
  }

  const std::size_t previous_index = last_index - 1U;
  const double first_time = trajectoryPoseTime(trajectory, previous_index);
  const double second_time = trajectoryPoseTime(trajectory, last_index);
  const double dt = second_time - first_time;
  if (dt <= 1.0e-6) {
    return motion;
  }

  const auto & first = trajectory.poses[previous_index];
  const auto & second = trajectory.poses[last_index];

  const double world_vx = (second.x - first.x) / dt;
  const double world_vy = (second.y - first.y) / dt;

  motion.forward_speed =
    world_vx * std::cos(second.theta) +
    world_vy * std::sin(second.theta);
  motion.angular_speed =
    normalizeAngle(second.theta - first.theta) / dt;

  if (!std::isfinite(motion.forward_speed)) {
    motion.forward_speed = 0.0;
  }
  if (!std::isfinite(motion.angular_speed)) {
    motion.angular_speed = 0.0;
  }
  return motion;
}

geometry_msgs::msg::Pose2D DynamicObstacleCritic::robotPoseAt(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double future_sec) const
{
  geometry_msgs::msg::Pose2D empty;
  if (trajectory.poses.empty()) {
    return empty;
  }

  const double t = std::max(0.0, future_sec);
  if (t <= 0.0 || trajectory.poses.size() == 1U) {
    return trajectory.poses.front();
  }

  if (t <= trajectory_horizon_sec_) {
    for (std::size_t index = 1U; index < trajectory.poses.size(); ++index) {
      const double upper_time = trajectoryPoseTime(trajectory, index);
      if (upper_time + 1.0e-9 < t) {
        continue;
      }

      const std::size_t lower_index = index - 1U;
      const double lower_time = trajectoryPoseTime(trajectory, lower_index);
      const double dt = upper_time - lower_time;
      if (dt <= 1.0e-9) {
        return trajectory.poses[index];
      }

      const double alpha = clamp01((t - lower_time) / dt);
      const auto & first = trajectory.poses[lower_index];
      const auto & second = trajectory.poses[index];

      geometry_msgs::msg::Pose2D result;
      result.x = first.x + alpha * (second.x - first.x);
      result.y = first.y + alpha * (second.y - first.y);
      result.theta = normalizeAngle(
        first.theta + alpha * normalizeAngle(second.theta - first.theta));
      return result;
    }
  }

  std::size_t terminal_index = trajectory.poses.size() - 1U;
  if (
    trajectory.poses.size() >= 3U &&
    samePose(
      trajectory.poses[terminal_index],
      trajectory.poses[terminal_index - 1U]))
  {
    --terminal_index;
  }

  geometry_msgs::msg::Pose2D result = trajectory.poses[terminal_index];
  const Motion2D motion = terminalMotion(trajectory);
  const double extra_sec = std::max(0.0, t - trajectory_horizon_sec_);

  if (std::fabs(motion.angular_speed) < 1.0e-6) {
    result.x += motion.forward_speed * std::cos(result.theta) * extra_sec;
    result.y += motion.forward_speed * std::sin(result.theta) * extra_sec;
    return result;
  }

  const double initial_theta = result.theta;
  const double final_theta = initial_theta + motion.angular_speed * extra_sec;
  const double radius = motion.forward_speed / motion.angular_speed;

  result.x += radius * (std::sin(final_theta) - std::sin(initial_theta));
  result.y -= radius * (std::cos(final_theta) - std::cos(initial_theta));
  result.theta = normalizeAngle(final_theta);
  return result;
}

double DynamicObstacleCritic::uncertaintyMargin(
  const Obstacle & obstacle,
  const double future_sec) const
{
  const double position_variance = std::max(
    std::max(0.0, obstacle.position_covariance[0]),
    std::max(0.0, obstacle.position_covariance[3]));
  const double velocity_variance = std::max(
    std::max(0.0, obstacle.velocity_covariance[0]),
    std::max(0.0, obstacle.velocity_covariance[3]));

  const double position_sigma = std::sqrt(position_variance);
  const double velocity_sigma = std::sqrt(velocity_variance);

  if (!std::isfinite(position_sigma) || !std::isfinite(velocity_sigma)) {
    return maximum_uncertainty_margin_m_;
  }

  const double raw_margin = uncertainty_sigma_multiplier_ * (
    position_sigma + std::max(0.0, future_sec) * velocity_sigma);

  return std::clamp(raw_margin, 0.0, maximum_uncertainty_margin_m_);
}

double DynamicObstacleCritic::rectangleClearance(
  const geometry_msgs::msg::Pose2D & robot_pose,
  const Point2D & obstacle_position,
  const double obstacle_radius,
  const double uncertainty_margin) const
{
  const double dx = obstacle_position.x - robot_pose.x;
  const double dy = obstacle_position.y - robot_pose.y;

  const double cosine = std::cos(robot_pose.theta);
  const double sine = std::sin(robot_pose.theta);

  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;

  const double outside_x =
    std::max(std::fabs(local_x) - robot_half_length_m_, 0.0);
  const double outside_y =
    std::max(std::fabs(local_y) - robot_half_width_m_, 0.0);

  const double obstacle_to_robot_rectangle = std::hypot(outside_x, outside_y);
  const double effective_radius =
    std::max(obstacle_radius, minimum_obstacle_radius_m_) +
    hard_safety_margin_m_ +
    std::max(0.0, uncertainty_margin);

  return obstacle_to_robot_rectangle - effective_radius;
}

double DynamicObstacleCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!cycle_tracks_ || trajectory.poses.size() <= 1U) {
    return 0.0;
  }

  // --------------------------------------------------------------------------
  // v0.1 score: preserved exactly.
  // --------------------------------------------------------------------------
  double maximum_risk = 0.0;
  double risk_sum = 0.0;
  std::size_t risk_samples = 0U;

  for (std::size_t pose_index = 1U; pose_index < trajectory.poses.size(); ++pose_index) {
    const double candidate_time = trajectoryPoseTime(trajectory, pose_index);
    const double obstacle_prediction_time = cycle_track_age_sec_ + candidate_time;

    const auto & robot_pose = trajectory.poses[pose_index];
    const Point2D robot_velocity = trajectoryVelocityAt(trajectory, pose_index);

    for (const auto & obstacle : cycle_tracks_->obstacles) {
      if (!isDynamic(obstacle)) {
        continue;
      }

      const Point2D obstacle_position =
        obstaclePositionAt(obstacle, obstacle_prediction_time);
      const Point2D obstacle_velocity =
        obstacleVelocityAt(obstacle, obstacle_prediction_time);

      if (
        !std::isfinite(obstacle_position.x) ||
        !std::isfinite(obstacle_position.y) ||
        !std::isfinite(obstacle_velocity.x) ||
        !std::isfinite(obstacle_velocity.y))
      {
        continue;
      }

      const double uncertainty_margin =
        uncertaintyMargin(obstacle, obstacle_prediction_time);
      const double clearance = rectangleClearance(
        robot_pose, obstacle_position, obstacle.radius, uncertainty_margin);

      double overlap_risk = 0.0;
      if (clearance <= 0.0) {
        const double penetration_scale =
          std::max(robot_half_width_m_ + minimum_obstacle_radius_m_, 0.10);
        overlap_risk =
          overlap_penalty_ *
          (1.0 + clamp01((-clearance) / penetration_scale));
      }

      double proximity_risk = 0.0;
      if (clearance < soft_clearance_m_) {
        const double normalized =
          clamp01((soft_clearance_m_ - clearance) / soft_clearance_m_);
        proximity_risk = normalized * normalized;
      }

      const double relative_x = obstacle_position.x - robot_pose.x;
      const double relative_y = obstacle_position.y - robot_pose.y;
      const double center_distance = std::hypot(relative_x, relative_y);

      double ttc_risk = 0.0;
      if (center_distance > 1.0e-6) {
        const double relative_velocity_x = obstacle_velocity.x - robot_velocity.x;
        const double relative_velocity_y = obstacle_velocity.y - robot_velocity.y;

        const double closing_speed =
          -(
          relative_x * relative_velocity_x +
          relative_y * relative_velocity_y) /
          center_distance;

        if (closing_speed > 1.0e-3) {
          const double time_to_clearance = clearance / closing_speed;

          if (time_to_clearance < ttc_horizon_sec_) {
            const double normalized =
              clamp01((ttc_horizon_sec_ - time_to_clearance) / ttc_horizon_sec_);
            ttc_risk = normalized * normalized;
          }
        }
      }

      const double future_weight =
        1.0 / (1.0 + future_discount_per_sec_ * candidate_time);

      const double sample_risk = future_weight * (
        overlap_risk +
        proximity_weight_ * proximity_risk +
        ttc_weight_ * ttc_risk);

      maximum_risk = std::max(maximum_risk, sample_risk);
      risk_sum += sample_risk;
      ++risk_samples;
    }
  }

  const double v01_score =
    risk_samples == 0U ?
    0.0 :
    0.75 * maximum_risk +
    0.25 * (risk_sum / static_cast<double>(risk_samples));

  // With all new weights at zero, return the exact v0.1 behavior.
  if (vo_weight_ <= 0.0 && cpa_weight_ <= 0.0 && braking_weight_ <= 0.0) {
    return v01_score;
  }

  // --------------------------------------------------------------------------
  // v0.2-A: generalized finite-horizon VO / sampled CPA / braking reserve.
  //
  // Crucial design choice:
  // We do NOT assume one constant obstacle velocity for the entire maneuver.
  // obstaclePositionAt(t) follows the tracker's predicted_positions and
  // obstacleVelocityAt(t) is differentiated from that time-varying prediction.
  // The constant-velocity path is only a missing-prediction fallback.
  // --------------------------------------------------------------------------
  double maximum_long_horizon_risk = 0.0;
  const Motion2D candidate_terminal_motion = terminalMotion(trajectory);
  const double candidate_speed = std::max(0.0, candidate_terminal_motion.forward_speed);

  for (const auto & obstacle : cycle_tracks_->obstacles) {
    if (!isDynamic(obstacle)) {
      continue;
    }

    double minimum_clearance = std::numeric_limits<double>::infinity();
    double minimum_clearance_time = prediction_horizon_sec_;
    double first_overlap_time = std::numeric_limits<double>::infinity();

    for (
      double candidate_time = prediction_sample_dt_sec_;
      candidate_time <= prediction_horizon_sec_ + 1.0e-9;
      candidate_time += prediction_sample_dt_sec_)
    {
      const double obstacle_prediction_time = cycle_track_age_sec_ + candidate_time;
      const auto robot_pose = robotPoseAt(trajectory, candidate_time);
      const Point2D obstacle_position =
        obstaclePositionAt(obstacle, obstacle_prediction_time);

      if (
        !std::isfinite(robot_pose.x) ||
        !std::isfinite(robot_pose.y) ||
        !std::isfinite(robot_pose.theta) ||
        !std::isfinite(obstacle_position.x) ||
        !std::isfinite(obstacle_position.y))
      {
        continue;
      }

      const double uncertainty_margin =
        uncertaintyMargin(obstacle, obstacle_prediction_time);
      const double clearance = rectangleClearance(
        robot_pose, obstacle_position, obstacle.radius, uncertainty_margin);

      if (clearance < minimum_clearance) {
        minimum_clearance = clearance;
        minimum_clearance_time = candidate_time;
      }

      if (clearance <= 0.0 && !std::isfinite(first_overlap_time)) {
        first_overlap_time = candidate_time;
      }
    }

    if (!std::isfinite(minimum_clearance)) {
      continue;
    }

    // Finite-horizon VO-like risk:
    // A predicted overlap sooner in the horizon is worse. Because overlap is
    // found from the time-indexed predicted trajectory, turns and speed changes
    // in predicted_positions are respected.
    double vo_risk = 0.0;
    if (std::isfinite(first_overlap_time)) {
      vo_risk = std::exp(-first_overlap_time / vo_time_decay_sec_);
    }

    // Sampled CPA risk:
    // Minimum predicted clearance, even without actual overlap, gets a smooth
    // cost. Earlier closest approaches are weighted more strongly.
    double cpa_risk = 0.0;
    if (minimum_clearance < cpa_soft_clearance_m_) {
      const double normalized =
        clamp01((cpa_soft_clearance_m_ - minimum_clearance) / cpa_soft_clearance_m_);
      const double time_weight =
        1.0 / (1.0 + cpa_future_discount_per_sec_ * minimum_clearance_time);
      cpa_risk = normalized * normalized * time_weight;
    }

    // Braking reserve:
    // If the predicted safety-envelope intrusion arrives before the robot could
    // reasonably react + brake + preserve reserve time, penalize the candidate.
    double braking_risk = 0.0;
    if (std::isfinite(first_overlap_time)) {
      const double required_stop_time =
        braking_reaction_sec_ +
        candidate_speed / braking_decel_mps2_ +
        braking_time_reserve_sec_;

      if (required_stop_time > 1.0e-6 && first_overlap_time < required_stop_time) {
        const double normalized =
          clamp01((required_stop_time - first_overlap_time) / required_stop_time);
        braking_risk = normalized * normalized;
      }
    }

    const double obstacle_long_horizon_risk =
      vo_weight_ * vo_risk +
      cpa_weight_ * cpa_risk +
      braking_weight_ * braking_risk;

    maximum_long_horizon_risk =
      std::max(maximum_long_horizon_risk, obstacle_long_horizon_risk);
  }

  return v01_score + maximum_long_horizon_risk;
}

}  // namespace a200_dynamic_dwb_critic

PLUGINLIB_EXPORT_CLASS(
  a200_dynamic_dwb_critic::DynamicObstacleCritic,
  dwb_core::TrajectoryCritic)
