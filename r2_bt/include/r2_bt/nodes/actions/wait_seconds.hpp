#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <string>

namespace r2_bt
{

class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double seconds_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace r2_bt
