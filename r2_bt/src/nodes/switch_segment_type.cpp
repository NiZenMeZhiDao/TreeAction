#include "r2_bt/nodes/controls/switch_segment_type.hpp"

#include <string>

namespace r2_bt
{

SwitchSegmentType::SwitchSegmentType(const std::string& name,
                                     const BT::NodeConfig& config)
  : BT::ControlNode(name, config)
{
}

BT::PortsList SwitchSegmentType::providedPorts()
{
  return {
    BT::InputPort<std::string>("segment_type",
                               "Segment type string to match against child names"),
  };
}

BT::NodeStatus SwitchSegmentType::tick()
{
  auto segment_type = getInput<std::string>("segment_type");
  if (!segment_type)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto& type = segment_type.value();

  // If a child is still RUNNING, continue ticking it
  if (current_child_idx_ < childrenCount())
  {
    const auto status = children()[current_child_idx_]->status();
    if (status == BT::NodeStatus::RUNNING)
    {
      return children()[current_child_idx_]->executeTick();
    }
  }

  // Search children for a name matching segment_type
  size_t default_idx = childrenCount();  // sentinel for "not found"
  for (size_t i = 0; i < childrenCount(); ++i)
  {
    const auto& child_name = children()[i]->name();
    if (child_name == type)
    {
      current_child_idx_ = i;
      return children()[i]->executeTick();
    }
    if (child_name == "default")
    {
      default_idx = i;
    }
  }

  // No exact match — try the default child
  if (default_idx < childrenCount())
  {
    current_child_idx_ = default_idx;
    return children()[default_idx]->executeTick();
  }

  // Completely unmatched
  return BT::NodeStatus::FAILURE;
}

void SwitchSegmentType::halt()
{
  haltChildren();
  current_child_idx_ = 0;
}

}  // namespace r2_bt
