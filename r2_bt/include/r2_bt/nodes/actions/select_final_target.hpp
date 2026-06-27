#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace r2_bt
{

class SelectFinalTarget : public BT::SyncActionNode
{
public:
  SelectFinalTarget(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  template <typename T>
  bool read_target_value(const std::string& key, T& value, std::string& error) const;
};

}  // namespace r2_bt
