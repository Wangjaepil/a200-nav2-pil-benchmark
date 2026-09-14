// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <prox_mpc_msgs/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "a200_predictive_collision_monitor/crossing_yield_core.hpp"
#include "a200_predictive_collision_monitor/msg/predictive_collision_state.hpp"
#include "a200_predictive_collision_monitor/predictive_collision_core.hpp"
#include "a200_predictive_collision_monitor/velocity_filter_core.hpp"

namespace a200_predictive_collision_monitor
{

using State = a200_predictive_collision_monitor::msg::PredictiveCollisionState;

class PredictiveCollisionMonitorNode : public rclcpp::Node
{
public:
  PredictiveCollisionMonitorNode()
  : Node("a200_predictive_collision_monitor")
  {
    const std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_raw");
    const std::string obstacles_topic =
      declare_parameter<std::string>("tracked_obstacles_topic", "/tracked_obstacles");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/predictive_collision_state");
    enable_velocity_output_ = declare_parameter<bool>("enable_velocity_output", false);
    const std::string cmd_out_topic =
      declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_raw");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tracked_obstacle_timeout_ =
      declare_parameter<double>("tracked_obstacle_timeout_sec", 0.75);
    future_stamp_tolerance_ =
      declare_parameter<double>("future_stamp_tolerance_sec", 0.10);
    transform_timeout_ = declare_parameter<double>("transform_timeout_sec", 0.10);

    EvaluatorConfig config;
    config.footprint_half_length =
      declare_parameter<double>("footprint_half_length_m", 0.494);
    config.footprint_half_width =
      declare_parameter<double>("footprint_half_width_m", 0.335);
    config.base_safety_margin =
      declare_parameter<double>("base_safety_margin_m", 0.20);
    config.minimum_obstacle_radius =
      declare_parameter<double>("minimum_obstacle_radius_m", 0.10);
    config.uncertainty_sigma_multiplier =
      declare_parameter<double>("uncertainty_sigma_multiplier", 1.0);
    config.maximum_uncertainty_margin =
      declare_parameter<double>("maximum_uncertainty_margin_m", 0.35);
    config.prediction_horizon =
      declare_parameter<double>("prediction_horizon_sec", 3.0);
    config.simulation_dt =
      declare_parameter<double>("simulation_time_step_sec", 0.05);
    config.minimum_dynamic_speed =
      declare_parameter<double>("minimum_dynamic_speed_mps", 0.15);
    config.slow_ttc = declare_parameter<double>("slow_ttc_sec", 2.5);
    config.stop_ttc = declare_parameter<double>("stop_ttc_sec", 1.0);

    crossing_yield_enabled_ = declare_parameter<bool>("crossing_yield_enabled", false);
    CrossingYieldConfig yield_config;
    yield_config.entry_ttc =
      declare_parameter<double>("crossing_yield_entry_ttc_sec", 1.6);
    const double crossing_angle_degrees =
      declare_parameter<double>("crossing_yield_min_angle_deg", 50.0);
    yield_config.minimum_crossing_angle_rad =
      crossing_angle_degrees * 3.14159265358979323846 / 180.0;
    yield_config.minimum_lateral_speed =
      declare_parameter<double>("crossing_yield_min_lateral_speed_mps", 0.25);
    yield_config.stationary_speed =
      declare_parameter<double>("crossing_yield_stationary_speed_mps", 0.12);
    yield_config.stationary_hold =
      declare_parameter<double>("crossing_yield_stationary_hold_sec", 1.5);
    yield_config.passed_clear_hold =
      declare_parameter<double>("crossing_yield_passed_clear_hold_sec", 0.30);
    yield_config.target_lost_hold =
      declare_parameter<double>("crossing_yield_target_lost_hold_sec", 1.0);
    yield_config.static_handoff_cooldown = declare_parameter<double>(
      "crossing_yield_static_handoff_cooldown_sec", 2.0);
    yield_config.passed_clearance_margin = declare_parameter<double>(
      "crossing_yield_passed_clearance_margin_m", 0.05);
    yield_config.footprint_half_width = config.footprint_half_width;
    yield_config.base_safety_margin = config.base_safety_margin;
    yield_config.minimum_obstacle_radius = config.minimum_obstacle_radius;

    VelocityFilterConfig filter_config;
    filter_config.minimum_stop_hold =
      declare_parameter<double>("minimum_stop_hold_sec", 0.50);
    filter_config.stop_clear_hold =
      declare_parameter<double>("stop_clear_hold_sec", 0.20);
    filter_config.slow_clear_hold =
      declare_parameter<double>("slow_clear_hold_sec", 0.30);
    filter_config.scale_release_rate =
      declare_parameter<double>("scale_release_rate_per_sec", 3.0);

    if (
      cmd_topic.empty() || obstacles_topic.empty() || state_topic.empty() ||
      base_frame_.empty())
    {
      throw std::invalid_argument("topic names and base_frame must not be empty");
    }
    if (enable_velocity_output_ && (cmd_out_topic.empty() || cmd_out_topic == cmd_topic)) {
      throw std::invalid_argument(
              "enforced mode requires different non-empty input and output cmd topics");
    }
    if (crossing_yield_enabled_ && yield_config.entry_ttc > config.prediction_horizon) {
      throw std::invalid_argument(
              "crossing_yield_entry_ttc_sec must not exceed prediction_horizon_sec");
    }
    if (!std::isfinite(tracked_obstacle_timeout_) ||
      !std::isfinite(future_stamp_tolerance_) || !std::isfinite(transform_timeout_) ||
      tracked_obstacle_timeout_ <= 0.0 || future_stamp_tolerance_ < 0.0 ||
      transform_timeout_ < 0.0)
    {
      throw std::invalid_argument("invalid timeout parameter");
    }

    evaluator_ = std::make_unique<PredictiveCollisionEvaluator>(config);
    crossing_yield_supervisor_ =
      std::make_unique<CrossingYieldSupervisor>(yield_config);
    velocity_filter_ = std::make_unique<VelocityFilterController>(filter_config);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    state_publisher_ = create_publisher<State>(state_topic, rclcpp::QoS(10).reliable());
    if (enable_velocity_output_) {
      cmd_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_out_topic, rclcpp::QoS(10).reliable());
    }
    tracked_obstacles_subscription_ =
      create_subscription<prox_mpc_msgs::msg::ObstacleArray>(
      obstacles_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](const prox_mpc_msgs::msg::ObstacleArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        latest_obstacles_ = message;
      });
    cmd_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic, rclcpp::QoS(10).reliable(),
      std::bind(&PredictiveCollisionMonitorNode::on_cmd_vel, this, std::placeholders::_1));

    if (enable_velocity_output_) {
      RCLCPP_WARN(
        get_logger(),
        "Velocity enforcement ENABLED: cmd='%s' -> '%s', tracks='%s', "
        "state='%s', horizon=%.2fs, crossing_yield=%s.",
        cmd_topic.c_str(), cmd_out_topic.c_str(), obstacles_topic.c_str(),
        state_topic.c_str(), config.prediction_horizon,
        crossing_yield_enabled_ ? "enabled" : "disabled");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Shadow monitor ready: cmd='%s', tracks='%s', state='%s', horizon=%.2fs. "
        "No velocity publisher exists in this mode.",
        cmd_topic.c_str(), obstacles_topic.c_str(), state_topic.c_str(),
        config.prediction_horizon);
    }
  }

private:
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(sin_yaw, cos_yaw);
  }

  static const char * action_name(const Action action)
  {
    switch (action) {
      case Action::Pass:
        return "PASS";
      case Action::Slow:
        return "SLOW";
      case Action::Stop:
        return "STOP";
      case Action::NoData:
      default:
        return "NO_DATA";
    }
  }

  static const char * yield_event_name(const YieldEvent event)
  {
    switch (event) {
      case YieldEvent::Entered:
        return "yield_crossing_entered";
      case YieldEvent::WaitingMoving:
        return "yield_waiting_for_moving_target";
      case YieldEvent::WaitingStationary:
        return "yield_waiting_for_stationary_confirmation";
      case YieldEvent::WaitingLost:
        return "yield_waiting_for_lost_target_clearance";
      case YieldEvent::ReleasedPassed:
        return "yield_released_target_passed";
      case YieldEvent::ReleasedStatic:
        return "yield_static_handoff_to_nav2";
      case YieldEvent::ReleasedLost:
        return "yield_released_target_lost";
      case YieldEvent::Retargeted:
        return "yield_retargeted_crossing_obstacle";
      case YieldEvent::None:
      default:
        return "";
    }
  }

  CrossingYieldDecision apply_crossing_yield(
    const RiskResult & risk,
    const Pose2D & robot_pose,
    const std::vector<TrackedObstacle> & obstacles,
    const rclcpp::Time & current_time)
  {
    if (!crossing_yield_enabled_) {
      return CrossingYieldDecision{
        risk.action,
        std::clamp(risk.speed_scale, 0.0, 1.0),
        false,
        risk.track_id,
        YieldEvent::None};
    }
    return crossing_yield_supervisor_->update(
      risk, robot_pose, obstacles, current_time.seconds());
  }

  static std::string crossing_yield_reason(
    const std::string & risk_reason,
    const CrossingYieldDecision & policy)
  {
    if (policy.event == YieldEvent::None) {
      return risk_reason;
    }
    return yield_event_name(policy.event);
  }

  static double tracked_obstacle_speed(
    const std::vector<TrackedObstacle> & obstacles,
    const uint32_t target_id,
    const double fallback)
  {
    const auto iterator = std::find_if(
      obstacles.begin(), obstacles.end(),
      [target_id](const TrackedObstacle & obstacle) {return obstacle.id == target_id;});
    if (iterator == obstacles.end() || !std::isfinite(iterator->velocity.x) ||
      !std::isfinite(iterator->velocity.y))
    {
      return fallback;
    }
    return std::hypot(iterator->velocity.x, iterator->velocity.y);
  }

  void publish_filtered_command(
    const geometry_msgs::msg::TwistStamped & command,
    const rclcpp::Time & current_time,
    const double scale)
  {
    if (!enable_velocity_output_) {
      return;
    }

    geometry_msgs::msg::TwistStamped output;
    output.header = command.header;
    output.header.stamp = current_time;
    if (scale > 0.0) {
      output.twist.linear.x = command.twist.linear.x * scale;
      output.twist.angular.z = command.twist.angular.z * scale;
    }
    cmd_publisher_->publish(output);
  }

  VelocityFilterResult apply_velocity_policy(
    const Action requested_action,
    const double requested_scale,
    const rclcpp::Time & current_time,
    const geometry_msgs::msg::TwistStamped & command)
  {
    if (!enable_velocity_output_) {
      return VelocityFilterResult{
        requested_action,
        std::clamp(requested_scale, 0.0, 1.0),
        false};
    }

    const auto decision = velocity_filter_->update(
      requested_action, requested_scale, current_time.seconds());
    publish_filtered_command(command, current_time, decision.applied_scale);
    return decision;
  }

  std::string filtered_reason(
    const std::string & base_reason,
    const Action requested_action,
    const VelocityFilterResult & decision) const
  {
    if (!enable_velocity_output_) {
      return base_reason;
    }
    if (requested_action == Action::NoData) {
      return "filter_fail_closed:" + base_reason;
    }
    if (decision.applied_action == Action::Stop && requested_action != Action::Stop) {
      return "filter_stop_latched:" + base_reason;
    }
    if (decision.applied_action == Action::Slow && requested_action == Action::Pass) {
      return "filter_release_hold_or_ramp:" + base_reason;
    }
    return base_reason;
  }

  void publish_no_data(
    const std::string & reason,
    const geometry_msgs::msg::TwistStamped & command,
    const double age = -1.0)
  {
    const rclcpp::Time current_time = now();
    const auto decision = apply_velocity_policy(
      Action::NoData, 0.0, current_time, command);
    State state;
    state.header.stamp = current_time;
    state.valid = false;
    state.action = static_cast<uint8_t>(decision.applied_action);
    state.prediction_model = State::MODEL_NONE;
    state.track_id = State::NO_TRACK_ID;
    state.ttc_sec = -1.0;
    state.time_to_closest_approach_sec = -1.0;
    state.min_clearance_m = -1.0;
    state.speed_scale = decision.applied_scale;
    state.obstacle_speed_mps = 0.0;
    state.obstacle_data_age_sec = age;
    state.reason = filtered_reason(reason, Action::NoData, decision);
    state_publisher_->publish(state);
    log_transition(
      Action::NoData, Action::NoData, decision.applied_action, State::NO_TRACK_ID,
      -1.0, decision.applied_scale, state.reason);
  }

  void log_transition(
    const Action risk_action,
    const Action policy_action,
    const Action applied_action,
    const uint32_t track_id,
    const double ttc,
    const double scale,
    const std::string & reason)
  {
    if (has_logged_transition_ && risk_action == last_risk_action_ &&
      policy_action == last_policy_action_ && applied_action == last_action_ &&
      track_id == last_track_id_ && reason == last_reason_)
    {
      return;
    }
    has_logged_transition_ = true;
    last_risk_action_ = risk_action;
    last_policy_action_ = policy_action;
    last_action_ = applied_action;
    last_track_id_ = track_id;
    last_reason_ = reason;
    RCLCPP_INFO(
      get_logger(),
      "Decision=%s policy=%s risk=%s track=%u TTC=%.3f scale=%.3f reason=%s",
      action_name(applied_action), action_name(policy_action), action_name(risk_action), track_id,
      ttc, scale, reason.c_str());
  }

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr command)
  {
    prox_mpc_msgs::msg::ObstacleArray::SharedPtr tracks;
    {
      std::lock_guard<std::mutex> lock(obstacles_mutex_);
      tracks = latest_obstacles_;
    }
    if (!tracks) {
      publish_no_data("waiting_for_tracked_obstacles", *command);
      return;
    }

    const rclcpp::Time current_time = now();
    const rclcpp::Time measurement_time(tracks->header.stamp, get_clock()->get_clock_type());
    double data_age = (current_time - measurement_time).seconds();
    if (!std::isfinite(data_age)) {
      publish_no_data("invalid_tracked_obstacle_stamp", *command);
      return;
    }
    if (data_age < -future_stamp_tolerance_) {
      publish_no_data("tracked_obstacle_stamp_is_in_the_future", *command, data_age);
      return;
    }
    data_age = std::max(0.0, data_age);
    if (data_age > tracked_obstacle_timeout_) {
      publish_no_data("tracked_obstacles_stale", *command, data_age);
      return;
    }
    if (tracks->header.frame_id.empty()) {
      publish_no_data("tracked_obstacle_frame_is_empty", *command, data_age);
      return;
    }

    // A fresh empty array proves that the tracker is alive and currently sees
    // no confirmed objects. An already-latched crossing target still needs its
    // lost-target hold, but no TF lookup is needed to update that hold.
    if (tracks->obstacles.empty()) {
      RiskResult risk;
      const auto policy = apply_crossing_yield(
        risk, Pose2D{}, {}, current_time);
      const auto decision = apply_velocity_policy(
        policy.requested_action, policy.requested_scale, current_time, *command);
      State state;
      state.header.stamp = current_time;
      state.header.frame_id = tracks->header.frame_id;
      state.valid = true;
      state.action = static_cast<uint8_t>(decision.applied_action);
      state.prediction_model = State::MODEL_NONE;
      state.track_id = policy.target_id;
      state.ttc_sec = -1.0;
      state.time_to_closest_approach_sec = -1.0;
      state.min_clearance_m = -1.0;
      state.speed_scale = decision.applied_scale;
      state.obstacle_speed_mps = 0.0;
      state.obstacle_data_age_sec = data_age;
      const std::string policy_reason = crossing_yield_reason(
        "no_confirmed_tracks", policy);
      state.reason = filtered_reason(
        policy_reason, policy.requested_action, decision);
      state_publisher_->publish(state);
      log_transition(
        risk.action, policy.requested_action, decision.applied_action,
        policy.target_id,
        -1.0, decision.applied_scale, state.reason);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        tracks->header.frame_id, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s unavailable: %s",
        tracks->header.frame_id.c_str(), base_frame_.c_str(), exception.what());
      publish_no_data("robot_pose_tf_unavailable", *command, data_age);
      return;
    }

    Pose2D robot_pose;
    robot_pose.x = transform.transform.translation.x;
    robot_pose.y = transform.transform.translation.y;
    robot_pose.yaw = yaw_from_quaternion(transform.transform.rotation);

    Twist2D twist;
    twist.linear_x = command->twist.linear.x;
    twist.angular_z = command->twist.angular.z;
    if (!std::isfinite(robot_pose.x) || !std::isfinite(robot_pose.y) ||
      !std::isfinite(robot_pose.yaw) || !std::isfinite(twist.linear_x) ||
      !std::isfinite(twist.angular_z))
    {
      publish_no_data("non_finite_robot_pose_or_command", *command, data_age);
      return;
    }

    std::vector<TrackedObstacle> obstacles;
    obstacles.reserve(tracks->obstacles.size());
    for (const auto & input : tracks->obstacles) {
      TrackedObstacle obstacle;
      obstacle.id = input.id;
      obstacle.position = Point2D{input.position.x, input.position.y};
      obstacle.velocity = Point2D{input.velocity.x, input.velocity.y};
      obstacle.radius = input.radius;
      obstacle.position_covariance = input.position_covariance;
      obstacle.velocity_covariance = input.velocity_covariance;
      obstacle.prediction_dt = input.prediction_dt;
      obstacle.imm_predictions.reserve(input.predicted_positions.size());
      for (const auto & predicted : input.predicted_positions) {
        obstacle.imm_predictions.push_back(Point2D{predicted.x, predicted.y});
      }
      obstacles.push_back(std::move(obstacle));
    }

    const RiskResult result = evaluator_->evaluate(robot_pose, twist, obstacles, data_age);
    const auto policy = apply_crossing_yield(
      result, robot_pose, obstacles, current_time);
    const auto decision = apply_velocity_policy(
      policy.requested_action, policy.requested_scale, current_time, *command);
    State state;
    state.header.stamp = current_time;
    state.header.frame_id = tracks->header.frame_id;
    state.valid = true;
    state.action = static_cast<uint8_t>(decision.applied_action);
    state.prediction_model = static_cast<uint8_t>(result.model);
    state.track_id = policy.target_id;
    state.ttc_sec = result.ttc;
    state.time_to_closest_approach_sec = result.time_to_closest_approach;
    state.min_clearance_m = result.min_clearance;
    state.speed_scale = decision.applied_scale;
    state.obstacle_speed_mps = tracked_obstacle_speed(
      obstacles, policy.target_id, result.obstacle_speed);
    state.obstacle_data_age_sec = data_age;

    std::string base_reason;
    if (!result.has_dynamic_tracks) {
      base_reason = "no_dynamic_tracks";
    } else if (result.action == Action::Stop) {
      base_reason = "predicted_collision_stop";
    } else if (result.action == Action::Slow) {
      base_reason = "predicted_collision_slow";
    } else if (result.ttc >= 0.0) {
      base_reason = "predicted_collision_beyond_slow_threshold";
    } else {
      base_reason = "predicted_path_clear";
    }
    base_reason = crossing_yield_reason(base_reason, policy);
    state.reason = filtered_reason(
      base_reason, policy.requested_action, decision);

    state_publisher_->publish(state);
    log_transition(
      result.action, policy.requested_action, decision.applied_action,
      policy.target_id,
      result.ttc, decision.applied_scale, state.reason);
  }

  std::string base_frame_;
  double tracked_obstacle_timeout_{0.75};
  double future_stamp_tolerance_{0.10};
  double transform_timeout_{0.10};
  bool enable_velocity_output_{false};
  bool crossing_yield_enabled_{false};
  std::unique_ptr<PredictiveCollisionEvaluator> evaluator_;
  std::unique_ptr<CrossingYieldSupervisor> crossing_yield_supervisor_;
  std::unique_ptr<VelocityFilterController> velocity_filter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<prox_mpc_msgs::msg::ObstacleArray>::SharedPtr
    tracked_obstacles_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_subscription_;
  rclcpp::Publisher<State>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_publisher_;
  std::mutex obstacles_mutex_;
  prox_mpc_msgs::msg::ObstacleArray::SharedPtr latest_obstacles_;
  bool has_logged_transition_{false};
  Action last_risk_action_{Action::NoData};
  Action last_policy_action_{Action::NoData};
  Action last_action_{Action::NoData};
  uint32_t last_track_id_{State::NO_TRACK_ID};
  std::string last_reason_;
};

}  // namespace a200_predictive_collision_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<a200_predictive_collision_monitor::PredictiveCollisionMonitorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("a200_predictive_collision_monitor"),
      "Fatal startup/runtime error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
