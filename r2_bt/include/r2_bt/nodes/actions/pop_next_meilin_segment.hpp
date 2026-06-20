#pragma once

#include <behaviortree_cpp/action_node.h>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

class PopNextMeilinSegment : public BT::StatefulActionNode
{
public:
  PopNextMeilinSegment(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus tryPop();
};

}  // namespace r2_bt
