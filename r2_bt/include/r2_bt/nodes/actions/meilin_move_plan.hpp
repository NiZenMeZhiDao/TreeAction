#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace r2_bt
{

class MeilinPlanMove : public BT::SyncActionNode
{
public:
  MeilinPlanMove(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

class MeilinCommitMove : public BT::SyncActionNode
{
public:
  MeilinCommitMove(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt
