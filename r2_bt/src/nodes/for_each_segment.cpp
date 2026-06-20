#include "r2_bt/nodes/decorators/for_each_segment.hpp"

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

BT::NodeStatus ForEachSegment::tick()
{
  const auto child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::FAILURE)
  {
    // Segment failed after all retries — propagate failure
    return BT::NodeStatus::FAILURE;
  }

  if (child_status == BT::NodeStatus::RUNNING)
  {
    first_tick_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // Child returned SUCCESS — a segment completed
  const auto stop_type =
      getInput<std::string>("stop_type").value_or(std::string("EXIT"));
  const auto seg_type =
      config().blackboard->get<std::string>("segment_type");

  if (seg_type == stop_type)
  {
    resetChild();
    first_tick_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // More segments ahead — reset child and continue
  resetChild();
  first_tick_ = true;
  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
