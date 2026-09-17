#ifndef A200_DYNAMIC_DWB_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_
#define A200_DYNAMIC_DWB_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "dwb_core/trajectory_critic.hpp"
#include "prox_mpc_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a200_dynamic_dwb_critic
{

class DynamicObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  using ObstacleArray = prox_mpc_msgs::msg::ObstacleArray;
  using Obstacle = ObstacleArray::_obstacles_type::value_type;

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  struct Motion2D
  {
    double forward_speed{0.0};
    double angular_speed{0.0};
  };

  bool isDynamic(const Obstacle & obstacle) const;

  Point2D obstaclePositionAt(
    const Obstacle & obstacle,
    double future_sec) const;

  Point2D obstacleVelocityAt(
    const Obstacle & obstacle,
    double future_sec) const;

  double trajectoryPoseTime(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    std::size_t index) const;

  Point2D trajectoryVelocityAt(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    std::size_t index) const;

  geometry_msgs::msg::Pose2D robotPoseAt(
    const dwb_msgs::msg::Trajectory2D & trajectory,
    double future_sec) const;

  Motion2D terminalMotion(
    const dwb_msgs::msg::Trajectory2D & trajectory) const;

  double uncertaintyMargin(
    const Obstacle & obstacle,
    double future_sec) const;

  double rectangleClearance(
    const geometry_msgs::msg::Pose2D & robot_pose,
    const Point2D & obstacle_position,
    double obstacle_radius,
    double uncertainty_margin) const;

  std::string tracked_obstacles_topic_{"/tracked_obstacles"};
  double track_timeout_sec_{0.75};
  double future_stamp_tolerance_sec_{0.10};
  double minimum_dynamic_speed_mps_{0.15};

  double robot_half_length_m_{0.494};
  double robot_half_width_m_{0.335};
  double minimum_obstacle_radius_m_{0.10};
  double hard_safety_margin_m_{0.10};
  double soft_clearance_m_{1.20};

  double uncertainty_sigma_multiplier_{1.0};
  double maximum_uncertainty_margin_m_{0.35};

  // v0.1 terms
  double trajectory_horizon_sec_{1.70};
  double ttc_horizon_sec_{4.0};
  double proximity_weight_{1.0};
  double ttc_weight_{1.5};
  double overlap_penalty_{25.0};
  double future_discount_per_sec_{0.15};

  // v0.2-A terms. All default to zero so v0.2-A can exactly reproduce v0.1.
  double prediction_horizon_sec_{3.0};
  double prediction_sample_dt_sec_{0.10};

  double vo_weight_{0.0};
  double vo_time_decay_sec_{1.5};

  double cpa_weight_{0.0};
  double cpa_soft_clearance_m_{1.20};
  double cpa_future_discount_per_sec_{0.15};

  double braking_weight_{0.0};
  double braking_reaction_sec_{0.20};
  double braking_decel_mps2_{1.0};
  double braking_time_reserve_sec_{0.30};

  rclcpp::Subscription<ObstacleArray>::SharedPtr tracks_subscription_;

  std::mutex tracks_mutex_;
  ObstacleArray::SharedPtr latest_tracks_;

  // Frozen in prepare(), then shared by every candidate in the same DWB cycle.
  ObstacleArray::SharedPtr cycle_tracks_;
  double cycle_track_age_sec_{0.0};
};

}  // namespace a200_dynamic_dwb_critic

#endif  // A200_DYNAMIC_DWB_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_
