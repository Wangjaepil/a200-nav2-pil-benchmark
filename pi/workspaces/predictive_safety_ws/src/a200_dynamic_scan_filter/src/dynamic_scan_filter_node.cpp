#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "a200_dynamic_scan_filter/dynamic_scan_filter_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "prox_mpc_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace a200_dynamic_scan_filter
{

class DynamicScanFilterNode : public rclcpp::Node
{
public:
  using ObstacleArray = prox_mpc_msgs::msg::ObstacleArray;
  using Obstacle = ObstacleArray::_obstacles_type::value_type;

  DynamicScanFilterNode()
  : Node("a200_dynamic_scan_filter"),
    motion_gate_(declareMotionGateParams())
  {
    input_scan_topic_ =
      declare_parameter<std::string>("input_scan_topic", "/scan");
    output_scan_topic_ =
      declare_parameter<std::string>("output_scan_topic", "/scan_static_mark");
    tracked_obstacles_topic_ =
      declare_parameter<std::string>("tracked_obstacles_topic", "/tracked_obstacles");

    track_timeout_sec_ =
      declare_parameter<double>("track_timeout_sec", 0.40);
    future_stamp_tolerance_sec_ =
      declare_parameter<double>("future_stamp_tolerance_sec", 0.10);
    transform_timeout_sec_ =
      declare_parameter<double>("transform_timeout_sec", 0.10);
    state_retention_sec_ =
      declare_parameter<double>("state_retention_sec", 1.00);

    minimum_mask_radius_m_ =
      declare_parameter<double>("minimum_mask_radius_m", 0.15);
    mask_padding_m_ =
      declare_parameter<double>("mask_padding_m", 0.05);
    uncertainty_sigma_multiplier_ =
      declare_parameter<double>("uncertainty_sigma_multiplier", 0.50);
    maximum_uncertainty_padding_m_ =
      declare_parameter<double>("maximum_uncertainty_padding_m", 0.10);
    maximum_mask_radius_m_ =
      declare_parameter<double>("maximum_mask_radius_m", 0.80);

    if (
      input_scan_topic_.empty() ||
      output_scan_topic_.empty() ||
      tracked_obstacles_topic_.empty() ||
      track_timeout_sec_ <= 0.0 ||
      future_stamp_tolerance_sec_ < 0.0 ||
      transform_timeout_sec_ < 0.0 ||
      state_retention_sec_ <= 0.0 ||
      minimum_mask_radius_m_ < 0.0 ||
      mask_padding_m_ < 0.0 ||
      uncertainty_sigma_multiplier_ < 0.0 ||
      maximum_uncertainty_padding_m_ < 0.0 ||
      maximum_mask_radius_m_ <= 0.0 ||
      maximum_mask_radius_m_ < minimum_mask_radius_m_)
    {
      throw std::invalid_argument("a200_dynamic_scan_filter received invalid parameters");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_scan_topic_, rclcpp::SensorDataQoS());

    tracks_subscription_ = create_subscription<ObstacleArray>(
      tracked_obstacles_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&DynamicScanFilterNode::tracksCallback, this, std::placeholders::_1));

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DynamicScanFilterNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "DynamicScanFilter v0.4 ready: scan='%s' -> '%s', tracks='%s', "
      "track_timeout=%.2fs, mask_radius=[%.2f, %.2f]m",
      input_scan_topic_.c_str(),
      output_scan_topic_.c_str(),
      tracked_obstacles_topic_.c_str(),
      track_timeout_sec_,
      minimum_mask_radius_m_,
      maximum_mask_radius_m_);
  }

private:
  MotionGateParams declareMotionGateParams()
  {
    MotionGateParams params;
    params.enter_speed_mps =
      declare_parameter<double>("dynamic_enter_speed_mps", 0.20);
    params.exit_speed_mps =
      declare_parameter<double>("dynamic_exit_speed_mps", 0.10);
    params.enter_hold_sec =
      declare_parameter<double>("dynamic_enter_hold_sec", 0.20);
    params.exit_hold_sec =
      declare_parameter<double>("dynamic_exit_hold_sec", 0.60);
    params.enter_min_displacement_m =
      declare_parameter<double>("dynamic_enter_min_displacement_m", 0.03);

    params.history_window_sec =
      declare_parameter<double>("motion_history_window_sec", 1.50);
    params.history_min_span_sec =
      declare_parameter<double>("motion_history_min_span_sec", 1.00);

    const int history_min_samples =
      declare_parameter<int>("motion_history_min_samples", 10);
    const int motion_bin_count =
      declare_parameter<int>("motion_bin_count", 5);

    if (history_min_samples < 3 || motion_bin_count < 3) {
      throw std::invalid_argument(
              "motion_history_min_samples and motion_bin_count must be >= 3");
    }

    params.history_min_samples = static_cast<std::size_t>(history_min_samples);
    params.motion_bin_count = static_cast<std::size_t>(motion_bin_count);

    params.quick_window_sec =
      declare_parameter<double>("quick_motion_window_sec", 0.80);
    params.quick_min_span_sec =
      declare_parameter<double>("quick_motion_min_span_sec", 0.65);

    const int quick_min_samples =
      declare_parameter<int>("quick_motion_min_samples", 7);
    const int quick_vote_window =
      declare_parameter<int>("quick_stationary_vote_window", 3);
    const int quick_votes_required =
      declare_parameter<int>("quick_stationary_votes_required", 2);

    if (quick_min_samples < 3 || quick_vote_window < 2 || quick_votes_required < 1) {
      throw std::invalid_argument("invalid quick motion / vote parameters");
    }

    params.quick_min_samples = static_cast<std::size_t>(quick_min_samples);
    params.quick_vote_window = static_cast<std::size_t>(quick_vote_window);
    params.quick_votes_required = static_cast<std::size_t>(quick_votes_required);
    params.quick_stationary_trend_speed_mps =
      declare_parameter<double>("quick_stationary_trend_speed_mps", 0.08);
    params.quick_stationary_activity_speed_mps =
      declare_parameter<double>("quick_stationary_activity_speed_mps", 0.11);

    params.stationary_trend_speed_mps =
      declare_parameter<double>("stationary_trend_speed_mps", 0.08);
    params.stationary_activity_speed_mps =
      declare_parameter<double>("stationary_activity_speed_mps", 0.15);
    params.stationary_confirm_sec =
      declare_parameter<double>("stationary_confirm_sec", 0.25);

    params.dynamic_trend_speed_mps =
      declare_parameter<double>("dynamic_trend_speed_mps", 0.14);
    params.dynamic_activity_speed_mps =
      declare_parameter<double>("dynamic_activity_speed_mps", 0.16);
    return params;
  }

  static double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return
      static_cast<double>(stamp.sec) +
      1.0e-9 * static_cast<double>(stamp.nanosec);
  }

  void tracksCallback(const ObstacleArray::SharedPtr message)
  {
    const double stamp_sec = stampToSeconds(message->header.stamp);

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_tracks_ = message;

    for (const auto & obstacle : message->obstacles) {
      const double speed =
        std::hypot(obstacle.velocity.x, obstacle.velocity.y);

      const auto id = static_cast<std::uint32_t>(obstacle.id);
      const bool was_masking = motion_gate_.isMasking(id);

      const bool is_masking = motion_gate_.update(
        id,
        speed,
        obstacle.position.x,
        obstacle.position.y,
        stamp_sec);

      if (was_masking != is_masking) {
        const MotionEstimate estimate = motion_gate_.motionEstimate(id);
        const MotionEstimate quick = motion_gate_.quickMotionEstimate(id);
        RCLCPP_INFO(
          get_logger(),
          "DynamicScanFilter track=%u ownership %s -> %s, "
          "tracker_speed=%.3fm/s quick_valid=%s quick_trend=%.3fm/s "
          "quick_activity=%.3fm/s quick_span=%.3fs quick_samples=%zu "
          "long_valid=%s long_trend=%.3fm/s long_activity=%.3fm/s "
          "long_span=%.3fs long_samples=%zu",
          id,
          was_masking ? "DYNAMIC" : "STATIC",
          is_masking ? "DYNAMIC" : "STATIC",
          speed,
          quick.valid ? "true" : "false",
          quick.trend_speed_mps,
          quick.activity_speed_mps,
          quick.history_span_sec,
          quick.sample_count,
          estimate.valid ? "true" : "false",
          estimate.trend_speed_mps,
          estimate.activity_speed_mps,
          estimate.history_span_sec,
          estimate.sample_count);
      }
    }

    motion_gate_.prune(stamp_sec, state_retention_sec_);
  }

  std::pair<double, double> obstaclePositionAt(
    const Obstacle & obstacle,
    const double future_sec) const
  {
    const double t = std::max(0.0, future_sec);
    const double current_x = obstacle.position.x;
    const double current_y = obstacle.position.y;

    if (
      !std::isfinite(obstacle.prediction_dt) ||
      obstacle.prediction_dt <= 1.0e-6 ||
      obstacle.predicted_positions.empty())
    {
      return {
        current_x + obstacle.velocity.x * t,
        current_y + obstacle.velocity.y * t};
    }

    const double prediction_dt = obstacle.prediction_dt;

    if (t <= prediction_dt) {
      const double alpha = std::clamp(t / prediction_dt, 0.0, 1.0);
      const auto & first = obstacle.predicted_positions.front();
      return {
        current_x + alpha * (first.x - current_x),
        current_y + alpha * (first.y - current_y)};
    }

    const double fractional_index = t / prediction_dt - 1.0;
    const double floored_index = std::floor(std::max(0.0, fractional_index));
    const std::size_t lower_index = static_cast<std::size_t>(floored_index);

    if (lower_index + 1U < obstacle.predicted_positions.size()) {
      const double alpha =
        std::clamp(fractional_index - floored_index, 0.0, 1.0);
      const auto & lower = obstacle.predicted_positions[lower_index];
      const auto & upper = obstacle.predicted_positions[lower_index + 1U];

      return {
        lower.x + alpha * (upper.x - lower.x),
        lower.y + alpha * (upper.y - lower.y)};
    }

    const auto & last = obstacle.predicted_positions.back();
    const double prediction_end_sec =
      prediction_dt * static_cast<double>(obstacle.predicted_positions.size());
    const double extra_sec = std::max(0.0, t - prediction_end_sec);

    return {
      last.x + obstacle.velocity.x * extra_sec,
      last.y + obstacle.velocity.y * extra_sec};
  }

  double obstacleMaskRadius(const Obstacle & obstacle) const
  {
    double position_sigma = 0.0;

    const double variance_x = std::max(0.0, obstacle.position_covariance[0]);
    const double variance_y = std::max(0.0, obstacle.position_covariance[3]);
    const double variance = std::max(variance_x, variance_y);

    if (std::isfinite(variance)) {
      position_sigma = std::sqrt(variance);
    } else {
      position_sigma = maximum_uncertainty_padding_m_;
    }

    const double uncertainty_padding = std::clamp(
      uncertainty_sigma_multiplier_ * position_sigma,
      0.0,
      maximum_uncertainty_padding_m_);

    const double raw_radius =
      std::max(static_cast<double>(obstacle.radius), minimum_mask_radius_m_) +
      mask_padding_m_ +
      uncertainty_padding;

    return std::clamp(
      raw_radius,
      minimum_mask_radius_m_,
      maximum_mask_radius_m_);
  }

  void publishPassThrough(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const char * reason)
  {
    scan_publisher_->publish(*scan);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "DynamicScanFilter pass-through: %s", reason);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    ObstacleArray::SharedPtr tracks;
    std::unordered_set<std::uint32_t> active_ids;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      tracks = latest_tracks_;

      if (tracks) {
        for (const auto & obstacle : tracks->obstacles) {
          const auto id = static_cast<std::uint32_t>(obstacle.id);
          if (motion_gate_.isMasking(id)) {
            active_ids.insert(id);
          }
        }
      }
    }

    if (!tracks || active_ids.empty()) {
      publishPassThrough(scan, "no confirmed dynamic tracks");
      return;
    }

    if (tracks->header.frame_id.empty() || scan->header.frame_id.empty()) {
      publishPassThrough(scan, "empty frame id");
      return;
    }

    const rclcpp::Time scan_stamp(scan->header.stamp);
    const rclcpp::Time track_stamp(tracks->header.stamp);
    const double data_age_sec = (scan_stamp - track_stamp).seconds();

    if (
      !std::isfinite(data_age_sec) ||
      data_age_sec < -future_stamp_tolerance_sec_ ||
      data_age_sec > track_timeout_sec_)
    {
      publishPassThrough(scan, "track snapshot stale/future");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        scan->header.frame_id,
        tracks->header.frame_id,
        scan_stamp,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "DynamicScanFilter TF unavailable (%s <- %s): %s. Publishing raw scan.",
        scan->header.frame_id.c_str(),
        tracks->header.frame_id.c_str(),
        exception.what());
      scan_publisher_->publish(*scan);
      return;
    }

    std::vector<CircleMask> masks;
    masks.reserve(active_ids.size());

    const double align_sec = std::max(0.0, data_age_sec);

    for (const auto & obstacle : tracks->obstacles) {
      const auto id = static_cast<std::uint32_t>(obstacle.id);
      if (active_ids.find(id) == active_ids.end()) {
        continue;
      }

      const auto [track_x, track_y] = obstaclePositionAt(obstacle, align_sec);

      if (!std::isfinite(track_x) || !std::isfinite(track_y)) {
        continue;
      }

      geometry_msgs::msg::PointStamped point_in;
      point_in.header.frame_id = tracks->header.frame_id;
      point_in.header.stamp = scan->header.stamp;
      point_in.point.x = track_x;
      point_in.point.y = track_y;
      point_in.point.z = 0.0;

      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, transform);

      const double radius = obstacleMaskRadius(obstacle);

      if (
        std::isfinite(point_out.point.x) &&
        std::isfinite(point_out.point.y) &&
        std::isfinite(radius) &&
        radius > 0.0)
      {
        masks.push_back(CircleMask{
          point_out.point.x,
          point_out.point.y,
          radius});
      }
    }

    if (masks.empty()) {
      publishPassThrough(scan, "no usable dynamic masks");
      return;
    }

    sensor_msgs::msg::LaserScan filtered = *scan;
    const std::size_t masked_beams = maskScanRanges(
      filtered.ranges,
      filtered.angle_min,
      filtered.angle_increment,
      filtered.range_min,
      filtered.range_max,
      masks);

    scan_publisher_->publish(filtered);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "DynamicScanFilter active_tracks=%zu masked_beams=%zu track_age=%.3fs",
      masks.size(), masked_beams, data_age_sec);
  }

  std::string input_scan_topic_;
  std::string output_scan_topic_;
  std::string tracked_obstacles_topic_;

  double track_timeout_sec_{0.40};
  double future_stamp_tolerance_sec_{0.10};
  double transform_timeout_sec_{0.10};
  double state_retention_sec_{1.00};

  double minimum_mask_radius_m_{0.15};
  double mask_padding_m_{0.05};
  double uncertainty_sigma_multiplier_{0.50};
  double maximum_uncertainty_padding_m_{0.10};
  double maximum_mask_radius_m_{0.80};

  MotionGate motion_gate_;

  std::mutex data_mutex_;
  ObstacleArray::SharedPtr latest_tracks_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<ObstacleArray>::SharedPtr tracks_subscription_;
};

}  // namespace a200_dynamic_scan_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<a200_dynamic_scan_filter::DynamicScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
