// Copyright 2026 A200 Navigation Project
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "a200_predictive_collision_monitor/msg/predictive_collision_state.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace a200_predictive_collision_monitor
{

using State = a200_predictive_collision_monitor::msg::PredictiveCollisionState;

/// Triggers one global replan after a crossing target has passed.
///
/// The predictive monitor already publishes the one-shot release event in
/// PredictiveCollisionState.reason. This condition waits for a global-costmap
/// message newer than that event before allowing ComputePathToPose to run.
/// It remains active until the BT receives a different Path, preventing a
/// running ComputePathToPose action from being cancelled on the next BT tick.
class PredictiveReplanCondition : public BT::ConditionNode
{
public:
  PredictiveReplanCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & configuration)
  : BT::ConditionNode(condition_name, configuration)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string state_topic;
    std::string costmap_topic;
    if (!getInput("state_topic", state_topic) || state_topic.empty()) {
      throw BT::RuntimeError("PredictiveReplan requires a non-empty state_topic");
    }
    if (!getInput("costmap_topic", costmap_topic) || costmap_topic.empty()) {
      throw BT::RuntimeError("PredictiveReplan requires a non-empty costmap_topic");
    }
    if (!getInput("event_timeout_sec", event_timeout_sec_) ||
      !std::isfinite(event_timeout_sec_) || event_timeout_sec_ <= 0.0)
    {
      throw BT::RuntimeError("PredictiveReplan requires event_timeout_sec > 0");
    }

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    state_subscription_ = node_->create_subscription<State>(
      state_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&PredictiveReplanCondition::on_state, this, std::placeholders::_1),
      options);

    costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&PredictiveReplanCondition::on_costmap, this, std::placeholders::_1),
      options);

    // Warm up this private callback executor. This is required by some Jazzy
    // rclcpp builds before subscriptions begin delivering reliably.
    callback_group_executor_.spin_all(std::chrono::milliseconds(1));
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_all(std::chrono::milliseconds(2));

    nav_msgs::msg::Path current_path;
    getInput("path", current_path);

    std::lock_guard<std::mutex> lock(mutex_);

    if (trigger_active_) {
      if (current_path != path_before_replan_) {
        trigger_active_ = false;
        RCLCPP_INFO(
          node_->get_logger(),
          "Predictive replan completed; the new global path is active.");
      } else {
        return BT::NodeStatus::SUCCESS;
      }
    }

    if (!event_pending_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_costmap_stamp_nanoseconds_ > event_stamp_nanoseconds_) {
      event_pending_ = false;
      trigger_active_ = true;
      path_before_replan_ = current_path;
      RCLCPP_INFO(
        node_->get_logger(),
        "Crossing target passed; triggering one global replan on a fresh costmap.");
      return BT::NodeStatus::SUCCESS;
    }

    const int64_t age_nanoseconds =
      node_->now().nanoseconds() - event_received_nanoseconds_;
    const double event_age_sec = static_cast<double>(age_nanoseconds) * 1e-9;
    if (event_age_sec < 0.0 || event_age_sec > event_timeout_sec_) {
      event_pending_ = false;
      RCLCPP_WARN(
        node_->get_logger(),
        "Predictive replan event expired without a newer global costmap; "
        "keeping the existing path.");
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Current global path"),
      BT::InputPort<std::string>(
        "state_topic", "/predictive_collision_state",
        "Predictive collision state topic"),
      BT::InputPort<std::string>(
        "costmap_topic", "/global_costmap/costmap",
        "Published global costmap used as the freshness barrier"),
      BT::InputPort<double>(
        "event_timeout_sec", 2.0,
        "Discard the passed-target event if no newer global costmap arrives")
    };
  }

private:
  static bool is_passed_release(const std::string & reason)
  {
    static const std::string suffix = "yield_released_target_passed";
    return reason.size() >= suffix.size() &&
           reason.compare(reason.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  void on_state(const State::SharedPtr message)
  {
    if (!message->valid || !is_passed_release(message->reason)) {
      return;
    }

    const int64_t stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp_nanoseconds <= 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Ignoring passed-target event with a zero timestamp.");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp_nanoseconds <= last_event_stamp_nanoseconds_) {
      return;
    }

    last_event_stamp_nanoseconds_ = stamp_nanoseconds;
    event_stamp_nanoseconds_ = stamp_nanoseconds;
    event_received_nanoseconds_ = node_->now().nanoseconds();
    event_pending_ = true;
  }

  void on_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    const int64_t stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp_nanoseconds > latest_costmap_stamp_nanoseconds_) {
      latest_costmap_stamp_nanoseconds_ = stamp_nanoseconds;
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;

  std::mutex mutex_;
  bool event_pending_{false};
  bool trigger_active_{false};
  int64_t last_event_stamp_nanoseconds_{0};
  int64_t event_stamp_nanoseconds_{0};
  int64_t event_received_nanoseconds_{0};
  int64_t latest_costmap_stamp_nanoseconds_{0};
  double event_timeout_sec_{2.0};
  nav_msgs::msg::Path path_before_replan_;
};

}  // namespace a200_predictive_collision_monitor

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    a200_predictive_collision_monitor::PredictiveReplanCondition>(
    "PredictiveReplan");
}
