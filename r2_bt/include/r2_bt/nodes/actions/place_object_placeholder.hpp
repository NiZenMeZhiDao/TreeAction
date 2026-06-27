#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace r2_bt
{

class PlaceObjectPlaceholder : public BT::SyncActionNode
{
public:
  PlaceObjectPlaceholder(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt
