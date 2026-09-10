#ifndef ADAPTIVE_ESCAPE_BT__ADAPTIVE_ESCAPE_ACTION_HPP_
#define ADAPTIVE_ESCAPE_BT__ADAPTIVE_ESCAPE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "adaptive_escape_msgs/action/adaptive_escape.hpp"

namespace adaptive_escape_bt
{

class AdaptiveEscapeAction
  : public nav2_behavior_tree::BtActionNode<
      adaptive_escape_msgs::action::AdaptiveEscape>
{
public:
  using Action = adaptive_escape_msgs::action::AdaptiveEscape;

  AdaptiveEscapeAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal", "Current navigation goal"),

        BT::InputPort<nav_msgs::msg::Path>(
          "path", "Current global path"),

        BT::InputPort<double>(
          "time_allowance", 6.0,
          "Maximum execution time in seconds"),

        BT::OutputPort<uint16_t>(
          "error_code_id", "AdaptiveEscape error code"),

        BT::OutputPort<std::string>(
          "error_msg", "AdaptiveEscape error message"),

        BT::OutputPort<std::string>(
          "selected_maneuver", "Selected escape maneuver")
      });
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
};

}  // namespace adaptive_escape_bt

#endif