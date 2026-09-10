#include <cmath>
#include <memory>

#include "adaptive_escape_bt/adaptive_escape_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace adaptive_escape_bt
{

AdaptiveEscapeAction::AdaptiveEscapeAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void AdaptiveEscapeAction::on_tick()
{
  // Recovery 실행 횟수 기록
  increment_recovery_count();

  // 현재 navigation goal
  if (!getInput("goal", goal_.goal)) {
    throw BT::RuntimeError(
            "AdaptiveEscape missing required input [goal]");
  }

  // path는 Planner failure 시 없을 수도 있으므로 optional처럼 처리
  nav_msgs::msg::Path path;

  if (getInput("path", path)) {
    goal_.path = path;
  } else {
    goal_.path = nav_msgs::msg::Path();
  }

  // 최대 실행시간
  double time_allowance = 6.0;
  getInput("time_allowance", time_allowance);

  if (time_allowance < 0.0) {
    time_allowance = 0.0;
  }

  const auto seconds =
    static_cast<int32_t>(std::floor(time_allowance));

  const auto nanoseconds =
    static_cast<uint32_t>(
      (time_allowance - static_cast<double>(seconds)) * 1e9);

  goal_.time_allowance.sec = seconds;
  goal_.time_allowance.nanosec = nanoseconds;
}


BT::NodeStatus AdaptiveEscapeAction::on_success()
{
  setOutput(
    "error_code_id",
    result_.result->error_code);

  setOutput(
    "error_msg",
    result_.result->error_msg);

  setOutput(
    "selected_maneuver",
    result_.result->selected_maneuver);

  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus AdaptiveEscapeAction::on_aborted()
{
  setOutput(
    "error_code_id",
    result_.result->error_code);

  setOutput(
    "error_msg",
    result_.result->error_msg);

  setOutput(
    "selected_maneuver",
    result_.result->selected_maneuver);

  return BT::NodeStatus::FAILURE;
}

}  // namespace adaptive_escape_bt


BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name,
      const BT::NodeConfiguration & config)
    {
      return std::make_unique<
        adaptive_escape_bt::AdaptiveEscapeAction>(
        name,
        "adaptive_escape",
        config);
    };

  factory.registerBuilder<
    adaptive_escape_bt::AdaptiveEscapeAction>(
    "AdaptiveEscape",
    builder);
}