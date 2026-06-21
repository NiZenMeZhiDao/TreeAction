#include "r2_bt/nodes/decorators/for_each_segment.hpp"

#include <behaviortree_cpp/control_node.h>

#include <string>

namespace r2_bt
{

ForEachSegment::ForEachSegment(const std::string& name,
                               const BT::NodeConfig& config)
  : BT::DecoratorNode(name, config)
{
}

BT::PortsList ForEachSegment::providedPorts()
{
  return {
    BT::InputPort<std::string>("stop_type", std::string("EXIT"),
                               "Segment type that triggers loop termination"),
  };
}

/// Recursively call haltChildren() on every ControlNode in the subtree.
/// haltChildren() (= resetChildren()) sets all children to IDLE and halts
/// RUNNING ones.  A single call only reaches direct children, so we must
/// walk the subtree explicitly to reset grandchildren too.
static void deepReset(BT::TreeNode* node)
{
  if (!node) return;
  if (auto* ctrl = dynamic_cast<BT::ControlNode*>(node))
  {
    // Recurse into children first (bottom-up), then halt direct children
    for (size_t i = 0; i < ctrl->childrenCount(); ++i)
      deepReset(ctrl->children()[i]);
    ctrl->haltChildren();
  }
}

BT::NodeStatus ForEachSegment::tick()
{
  const auto child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::FAILURE)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (child_status == BT::NodeStatus::RUNNING)
  {
    first_tick_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // Child returned SUCCESS — a segment completed.
  const auto stop_type =
      getInput<std::string>("stop_type").value_or(std::string("EXIT"));
  const auto seg_type =
      config().blackboard->get<std::string>("segment_type");

  if (seg_type == stop_type)
  {
    deepReset(child_node_);
    first_tick_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // More segments ahead — deep-reset the subtree and loop
  deepReset(child_node_);
  first_tick_ = true;
  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
