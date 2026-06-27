#pragma once

#include <ares_tool_interfaces/srv/tool_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

class AresToolAction : public BT::StatefulActionNode
{
public:
  AresToolAction(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using ServiceT = ares_tool_interfaces::srv::ToolAction;

  rclcpp::Client<ServiceT>::SharedPtr client_;
  rclcpp::Node::SharedPtr node_;
  std::string service_name_;

  bool request_done_ = false;
  bool request_sent_ = false;
  BT::NodeStatus result_status_ = BT::NodeStatus::FAILURE;
  std::string result_message_;
  double timeout_sec_ = 30.0;
  std::chrono::steady_clock::time_point start_time_;
  std::mutex mutex_;
};

}  // namespace r2_bt
